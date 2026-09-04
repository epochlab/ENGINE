// Headless beauty render, for before/after comparison across a code change. Loads a scene exactly as main.cpp does, accumulates N path-traced passes, and writes the Beauty AOV as an 8-bit PNG through the same OCIO display transform the viewer shows.
// Exists because the renderer is a GLFW application: comparing two revisions otherwise means two manual screenshots, which cannot be pixel-differenced and cannot be trusted to share a camera. Everything here is deterministic -- fixed camera from profile.json, fixed runSeed per pass, no interaction -- so two runs over unchanged code produce a byte-identical file, which is what makes a non-zero diff meaningful.
// --compare takes a previously written PNG and reports max/RMS channel deviation against the render just produced, so "did this change the picture, and where" is answered numerically rather than by eye.
// Same standalone-CLI convention as the validate tools: no test framework, non-zero exit on failure.

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <OpenColorIO/OpenColorIO.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <zlib.h>

#include "engine/config/profile_config.h"
#include "engine/config/scene_config.h"
#include "engine/gfx/hdr_image.h"
#include "engine/gfx/ocio_display_transform.h"
#include "engine/scene/camera.h"
#include "engine/scene/embree_accel.h"
#include "engine/scene/environment_map.h"
#include "engine/scene/gltf_loader.h"
#include "engine/scene/material_binding.h"
#include "engine/scene/path_tracer.h"
#include "engine/scene/thread_pool.h"

namespace OCIO = OCIO_NAMESPACE;

namespace {

struct Options {
    std::string scenePath = "scenes/cornell.json";
    std::string outPath;
    std::string comparePath;
    int width = 0;   // 0 = profile.json's window size
    int height = 0;
    int passes = 64;
    float exposureEv = 0.0F;
};

// Big-endian u32 append -- PNG is network byte order throughout.
void appendBe32(std::vector<unsigned char>& out, std::uint32_t value) {
    out.push_back(static_cast<unsigned char>((value >> 24) & 0xFFU));
    out.push_back(static_cast<unsigned char>((value >> 16) & 0xFFU));
    out.push_back(static_cast<unsigned char>((value >> 8) & 0xFFU));
    out.push_back(static_cast<unsigned char>(value & 0xFFU));
}

void appendChunk(std::vector<unsigned char>& out, const char* type,
                  const std::vector<unsigned char>& data) {
    appendBe32(out, static_cast<std::uint32_t>(data.size()));
    const std::size_t crcStart = out.size();
    out.insert(out.end(), type, type + 4);
    out.insert(out.end(), data.begin(), data.end());
    const uLong crc = crc32(crc32(0L, Z_NULL, 0), out.data() + crcStart,
                             static_cast<uInt>(out.size() - crcStart));
    appendBe32(out, static_cast<std::uint32_t>(crc));
}

// Minimal 8-bit RGB PNG writer. zlib arrives transitively with OpenEXR and ships with the platform, so this needs no vendored image library for what is ultimately a debug/reviewing artifact.
bool writePng(const std::string& path, int width, int height,
               const std::vector<unsigned char>& rgb) {
    // Each scanline is prefixed with its filter byte; 0 = None, which compresses adequately here and keeps the encoder trivial.
    std::vector<unsigned char> raw;
    raw.reserve((static_cast<std::size_t>(width) * 3 + 1) * static_cast<std::size_t>(height));
    for (int y = 0; y < height; ++y) {
        raw.push_back(0);
        const std::size_t rowStart = static_cast<std::size_t>(y) * static_cast<std::size_t>(width) * 3;
        raw.insert(raw.end(), rgb.begin() + static_cast<std::ptrdiff_t>(rowStart),
                    rgb.begin() + static_cast<std::ptrdiff_t>(rowStart) +
                        (static_cast<std::ptrdiff_t>(width) * 3));
    }

    uLongf compressedSize = compressBound(static_cast<uLong>(raw.size()));
    std::vector<unsigned char> compressed(compressedSize);
    if (compress2(compressed.data(), &compressedSize, raw.data(), static_cast<uLong>(raw.size()),
                   Z_BEST_COMPRESSION) != Z_OK) {
        std::cerr << "render_beauty: zlib compression failed\n";
        return false;
    }
    compressed.resize(compressedSize);

    std::vector<unsigned char> png = {0x89, 'P', 'N', 'G', '\r', '\n', 0x1A, '\n'};
    std::vector<unsigned char> ihdr;
    appendBe32(ihdr, static_cast<std::uint32_t>(width));
    appendBe32(ihdr, static_cast<std::uint32_t>(height));
    ihdr.push_back(8);  // bit depth
    ihdr.push_back(2);  // colour type 2 = truecolour RGB
    ihdr.push_back(0);  // deflate
    ihdr.push_back(0);  // adaptive filtering
    ihdr.push_back(0);  // no interlace
    appendChunk(png, "IHDR", ihdr);
    appendChunk(png, "IDAT", compressed);
    appendChunk(png, "IEND", {});

    std::ofstream file(path, std::ios::binary);
    if (!file) {
        std::cerr << "render_beauty: could not open " << path << " for writing\n";
        return false;
    }
    file.write(reinterpret_cast<const char*>(png.data()),
                static_cast<std::streamsize>(png.size()));
    return file.good();
}

// Reads back an 8-bit RGB PNG this tool wrote, for --compare. Deliberately narrow: only the exact IHDR shape written above (8-bit, colour type 2, no interlace) and only filter type 0, since the sole producer is writePng. Anything else is rejected rather than half-decoded.
bool readPng(const std::string& path, int& width, int& height, std::vector<unsigned char>& rgb) {
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        std::cerr << "render_beauty: could not read " << path << '\n';
        return false;
    }
    const std::vector<unsigned char> bytes((std::istreambuf_iterator<char>(file)),
                                            std::istreambuf_iterator<char>());
    if (bytes.size() < 8 || bytes[0] != 0x89 || bytes[1] != 'P') {
        std::cerr << "render_beauty: " << path << " is not a PNG\n";
        return false;
    }
    const auto be32 = [&bytes](std::size_t at) {
        return (static_cast<std::uint32_t>(bytes[at]) << 24) |
               (static_cast<std::uint32_t>(bytes[at + 1]) << 16) |
               (static_cast<std::uint32_t>(bytes[at + 2]) << 8) |
               static_cast<std::uint32_t>(bytes[at + 3]);
    };

    std::vector<unsigned char> idat;
    std::size_t at = 8;
    while (at + 8 <= bytes.size()) {
        const std::uint32_t length = be32(at);
        const std::string type(reinterpret_cast<const char*>(&bytes[at + 4]), 4);
        const std::size_t dataAt = at + 8;
        if (dataAt + length > bytes.size()) {
            break;
        }
        if (type == "IHDR") {
            // The 13-byte payload is indexed directly below; a chunk declaring less than that would read past the buffer on a truncated or hostile file.
            if (length < 13) {
                std::cerr << "render_beauty: " << path << " has a malformed IHDR\n";
                return false;
            }
            width = static_cast<int>(be32(dataAt));
            height = static_cast<int>(be32(dataAt + 4));
            if (bytes[dataAt + 8] != 8 || bytes[dataAt + 9] != 2 || bytes[dataAt + 12] != 0) {
                std::cerr << "render_beauty: " << path
                          << " is not the 8-bit non-interlaced RGB this tool writes\n";
                return false;
            }
        } else if (type == "IDAT") {
            idat.insert(idat.end(), bytes.begin() + static_cast<std::ptrdiff_t>(dataAt),
                         bytes.begin() + static_cast<std::ptrdiff_t>(dataAt + length));
        }
        at = dataAt + length + 4;  // + CRC
    }
    if (width <= 0 || height <= 0 || idat.empty()) {
        std::cerr << "render_beauty: " << path << " has no usable image data\n";
        return false;
    }

    const std::size_t stride = (static_cast<std::size_t>(width) * 3) + 1;
    uLongf rawSize = static_cast<uLongf>(stride * static_cast<std::size_t>(height));
    std::vector<unsigned char> raw(rawSize);
    if (uncompress(raw.data(), &rawSize, idat.data(), static_cast<uLong>(idat.size())) != Z_OK ||
        rawSize != stride * static_cast<std::size_t>(height)) {
        std::cerr << "render_beauty: " << path << " failed to inflate\n";
        return false;
    }
    rgb.resize(static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 3);
    for (int y = 0; y < height; ++y) {
        if (raw[static_cast<std::size_t>(y) * stride] != 0) {
            std::cerr << "render_beauty: " << path << " uses a PNG filter this tool cannot read\n";
            return false;
        }
        std::memcpy(&rgb[static_cast<std::size_t>(y) * static_cast<std::size_t>(width) * 3],
                     &raw[(static_cast<std::size_t>(y) * stride) + 1],
                     static_cast<std::size_t>(width) * 3);
    }
    return true;
}

// Deterministic triangular-PDF dither, byte-for-byte the ditherOffset() the display shader applies before the framebuffer's 8-bit quantization (ocio_display_transform.cpp). Reproduced rather than skipped so this output matches what the viewer shows; being a pure function of uv it is identical across runs and cancels in a before/after difference.
glm::vec3 ditherOffset(float u, float v) {
    const auto rand = [](float x, float y) {
        const float s = std::sin((x * 12.9898F) + (y * 78.233F)) * 43758.5453F;
        return s - std::floor(s);
    };
    const float d = (rand(u, v) - rand(u + 0.618F, v + 0.618F)) / 255.0F;
    return {d, d, d};
}

// Scene-referred beauty -> display-referred 8-bit, matching the viewer's pipeline exactly: exposure multiply, the OCIO Display/View transform, then dither and quantize. The OCIO processor is built from the same config/colorspace/display/view constants the display shaders are generated from (ocio_display_transform.h), so this is the same transform evaluated on the CPU rather than a second definition of it.
std::vector<unsigned char> encodeForDisplay(const engine::gfx::HdrImage& beauty, float exposureEv) {
    std::vector<float> rgb(static_cast<std::size_t>(beauty.width) *
                            static_cast<std::size_t>(beauty.height) * 3);
    const float exposure = std::pow(2.0F, exposureEv);
    for (std::size_t i = 0; i < rgb.size() / 3; ++i) {
        rgb[(i * 3) + 0] = beauty.rgba[(i * 4) + 0] * exposure;
        rgb[(i * 3) + 1] = beauty.rgba[(i * 4) + 1] * exposure;
        rgb[(i * 3) + 2] = beauty.rgba[(i * 4) + 2] * exposure;
    }

    const OCIO::ConstConfigRcPtr config =
        OCIO::Config::CreateFromBuiltinConfig(engine::gfx::kOcioConfigName);
    const OCIO::ConstProcessorRcPtr processor =
        config->getProcessor(engine::gfx::kOcioSceneColorSpace, engine::gfx::kOcioSrgbDisplay,
                              engine::gfx::kOcioView, OCIO::TRANSFORM_DIR_FORWARD);
    OCIO::PackedImageDesc desc(rgb.data(), beauty.width, beauty.height, OCIO::CHANNEL_ORDERING_RGB);
    processor->getDefaultCPUProcessor()->apply(desc);

    std::vector<unsigned char> out(rgb.size());
    for (int y = 0; y < beauty.height; ++y) {
        for (int x = 0; x < beauty.width; ++x) {
            const std::size_t i = ((static_cast<std::size_t>(y) *
                                     static_cast<std::size_t>(beauty.width)) +
                                    static_cast<std::size_t>(x)) * 3;
            const glm::vec3 dither =
                ditherOffset((static_cast<float>(x) + 0.5F) / static_cast<float>(beauty.width),
                              (static_cast<float>(y) + 0.5F) / static_cast<float>(beauty.height));
            for (int c = 0; c < 3; ++c) {
                const float value = std::clamp(rgb[i + static_cast<std::size_t>(c)] + dither[c],
                                                0.0F, 1.0F);
                out[i + static_cast<std::size_t>(c)] =
                    static_cast<unsigned char>((value * 255.0F) + 0.5F);
            }
        }
    }
    return out;
}

bool parseArgs(int argc, char** argv, Options& options) {
    for (int i = 1; i < argc; ++i) {
        const auto needsValue = [&](const char* flag) {
            if (i + 1 < argc) {
                return true;
            }
            std::cerr << "render_beauty: " << flag << " requires a value\n";
            return false;
        };
        if (std::strcmp(argv[i], "--scene") == 0) {
            if (!needsValue("--scene")) { return false; }
            options.scenePath = argv[++i];
        } else if (std::strcmp(argv[i], "--out") == 0) {
            if (!needsValue("--out")) { return false; }
            options.outPath = argv[++i];
        } else if (std::strcmp(argv[i], "--compare") == 0) {
            if (!needsValue("--compare")) { return false; }
            options.comparePath = argv[++i];
        } else if (std::strcmp(argv[i], "--passes") == 0) {
            if (!needsValue("--passes")) { return false; }
            options.passes = std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--width") == 0) {
            if (!needsValue("--width")) { return false; }
            options.width = std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--height") == 0) {
            if (!needsValue("--height")) { return false; }
            options.height = std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--exposure") == 0) {
            if (!needsValue("--exposure")) { return false; }
            options.exposureEv = static_cast<float>(std::atof(argv[++i]));
        } else {
            std::cerr << "render_beauty: unknown argument '" << argv[i]
                      << "'\nusage: render_beauty [--scene scenes/x.json] --out out.png "
                         "[--compare ref.png] [--passes N] [--width W] [--height H] [--exposure EV]\n";
            return false;
        }
    }
    if (options.outPath.empty()) {
        std::cerr << "render_beauty: --out is required\n";
        return false;
    }
    if (options.passes < 1) {
        std::cerr << "render_beauty: --passes must be at least 1\n";
        return false;
    }
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    Options options;
    if (!parseArgs(argc, argv, options)) {
        return EXIT_FAILURE;
    }

    const std::string assetRoot = ASSET_ROOT_DIR;
    const std::optional<engine::config::ProfileConfig> profileConfig =
        engine::config::loadProfileConfig(assetRoot + "/config/profile.json");
    const std::optional<engine::config::SceneConfig> sceneConfig =
        engine::config::loadSceneConfig(assetRoot + "/" + options.scenePath);
    if (!profileConfig || !sceneConfig) {
        return EXIT_FAILURE;
    }
    const std::optional<engine::config::MaterialConfig> materialConfig =
        engine::config::loadMaterialConfig(assetRoot + "/" + sceneConfig->materialPath);
    std::optional<engine::gfx::HdrImage> environmentImage =
        engine::gfx::loadExr(assetRoot + "/" + sceneConfig->environment.hdriPath);
    if (!materialConfig || !environmentImage) {
        return EXIT_FAILURE;
    }

    // Scene-level placement, order X,Y,Z -- must stay identical to main.cpp's composition or the comparison renders a different scene than the viewer shows.
    const glm::mat4 rootTransform =
        glm::translate(glm::mat4(1.0F), sceneConfig->model.position) *
        glm::rotate(glm::mat4(1.0F), glm::radians(sceneConfig->model.rotation.z), glm::vec3(0.0F, 0.0F, 1.0F)) *
        glm::rotate(glm::mat4(1.0F), glm::radians(sceneConfig->model.rotation.y), glm::vec3(0.0F, 1.0F, 0.0F)) *
        glm::rotate(glm::mat4(1.0F), glm::radians(sceneConfig->model.rotation.x), glm::vec3(1.0F, 0.0F, 0.0F));
    std::optional<engine::scene::LoadedModel> model = engine::scene::loadGltf(
        assetRoot + "/" + sceneConfig->model.gltfPath, rootTransform,
        sceneConfig->model.texturePath.empty() ? "" : assetRoot + "/" + sceneConfig->model.texturePath);
    if (!model) {
        return EXIT_FAILURE;
    }

    // Resolved the same way as main.cpp's initializeApp: profile.json names a preset, assets/config/camera.json supplies its dimensions.
    const std::optional<std::vector<engine::scene::Camera::FilmBackPreset>> filmBackPresets =
        engine::config::loadFilmBackPresets(assetRoot + "/config/camera.json");
    if (!filmBackPresets) {
        return EXIT_FAILURE;
    }
    const auto filmBackPresetIt =
        std::find_if(filmBackPresets->begin(), filmBackPresets->end(),
                     [&](const engine::scene::Camera::FilmBackPreset& preset) {
                         return preset.name == profileConfig->camera.defaultFilmBackPresetName;
                     });
    if (filmBackPresetIt == filmBackPresets->end()) {
        std::cerr << "render_beauty: profile.json filmBackPreset \""
                   << profileConfig->camera.defaultFilmBackPresetName << "\" not found in camera.json\n";
        return EXIT_FAILURE;
    }

    const engine::config::CameraConfig& cameraConfig = profileConfig->camera;
    const engine::scene::Camera camera(cameraConfig.position, cameraConfig.yawDegrees,
                                        cameraConfig.pitchDegrees, filmBackPresetIt->filmBack,
                                        cameraConfig.focalLengthMm, cameraConfig.nearClip,
                                        cameraConfig.farClip, cameraConfig.aperture,
                                        cameraConfig.shutterSeconds, cameraConfig.iso);

    const int width = options.width > 0 ? options.width : profileConfig->window.width;
    const int height = options.height > 0 ? options.height : profileConfig->window.height;

    const engine::scene::PathTraceSettings baseSettings{
        .samplesPerPixel = 1,  // one sample per pass; convergence comes from accumulating passes below
        .maxBounces = profileConfig->pathTracer.maxBounces,
        .russianRouletteStartBounce = profileConfig->pathTracer.russianRouletteStartBounce,
        .bumpStrength = materialConfig->bumpStrength,
        .roughnessMin = materialConfig->roughnessMin,
        .roughnessMax = materialConfig->roughnessMax,
        .diffuseColour = materialConfig->diffuseColour,
        .ior = materialConfig->ior,
        .transmissionFactor = materialConfig->transmissionFactor,
        .metallicFactor = materialConfig->metallicFactor,
        .roughnessFactor = materialConfig->roughnessFactor,
        .diffuseRoughness = materialConfig->diffuseRoughness,
        .transmissionColor = materialConfig->transmissionColor,
        .transmissionDepth = materialConfig->transmissionDepth,
        .edgeTint = materialConfig->edgeTint,
    };
    const std::optional<std::vector<engine::scene::PathTraceSettings>> perInstanceSettings =
        engine::scene::resolvePerInstanceSettings(baseSettings, model->instances,
                                                   sceneConfig->materialOverrides, assetRoot);
    if (!perInstanceSettings) {
        return EXIT_FAILURE;
    }

    std::optional<engine::scene::EmbreeAccel> accel =
        engine::scene::EmbreeAccel::build(std::move(model->worldTriangles));
    if (!accel) {
        std::cerr << "render_beauty: Embree scene build failed\n";
        return EXIT_FAILURE;
    }
    const engine::scene::EnvironmentMap environmentMap(std::move(*environmentImage));
    engine::scene::ThreadPool threadPool;

    // Mean of `passes` independent single-sample passes, each with its own runSeed -- the same accumulation PathTraceDriver performs, done synchronously. Seeds are the pass index, so the whole render is reproducible.
    engine::scene::PathTraceResult result = engine::scene::makePathTraceResult(width, height);
    engine::gfx::HdrImage accumulated = engine::gfx::HdrImage{
        width, height, std::vector<float>(static_cast<std::size_t>(width) *
                                           static_cast<std::size_t>(height) * 4, 0.0F)};
    const std::atomic<std::uint64_t> generation{1};
    for (int pass = 0; pass < options.passes; ++pass) {
        engine::scene::renderPathTraced(camera, *accel, model->shadingTriangles, model->instances,
                                         environmentMap, width, height, /*envRotationRadians=*/0.0F,
                                         /*showSky=*/true, /*envExposure=*/1.0F, baseSettings,
                                         *perInstanceSettings, static_cast<std::uint32_t>(pass),
                                         generation, /*requestedGeneration=*/1U, threadPool, result);
        for (std::size_t i = 0; i < accumulated.rgba.size(); ++i) {
            accumulated.rgba[i] += result.beauty.rgba[i];
        }
    }
    for (float& v : accumulated.rgba) {
        v /= static_cast<float>(options.passes);
    }

    const std::vector<unsigned char> encoded = encodeForDisplay(accumulated, options.exposureEv);
    if (!writePng(options.outPath, width, height, encoded)) {
        return EXIT_FAILURE;
    }
    std::cout << "render_beauty: wrote " << options.outPath << " (" << width << "x" << height << ", "
              << options.passes << " passes)\n";

    if (!options.comparePath.empty()) {
        int refWidth = 0;
        int refHeight = 0;
        std::vector<unsigned char> reference;
        if (!readPng(options.comparePath, refWidth, refHeight, reference)) {
            return EXIT_FAILURE;
        }
        if (refWidth != width || refHeight != height) {
            std::cerr << "render_beauty: --compare image is " << refWidth << "x" << refHeight
                      << ", this render is " << width << "x" << height << "\n";
            return EXIT_FAILURE;
        }
        int maxDelta = 0;
        double squaredSum = 0.0;
        // Signed mean alongside RMS: the direction of an energy change, not just its magnitude. Near-zero mean against non-zero RMS means light moved rather than appeared or vanished.
        double signedSum = 0.0;
        std::size_t differing = 0;
        for (std::size_t i = 0; i < encoded.size(); ++i) {
            const int signedDelta = static_cast<int>(encoded[i]) - static_cast<int>(reference[i]);
            const int delta = std::abs(signedDelta);
            maxDelta = std::max(maxDelta, delta);
            squaredSum += static_cast<double>(delta) * delta;
            signedSum += signedDelta;
            differing += delta != 0 ? 1 : 0;
        }
        const double rms = std::sqrt(squaredSum / static_cast<double>(encoded.size()));
        const double meanSigned = signedSum / static_cast<double>(encoded.size());
        std::cout << "render_beauty: vs " << options.comparePath << " -- max channel delta "
                  << maxDelta << "/255, RMS " << rms << ", mean signed " << meanSigned << ", "
                  << differing << "/" << encoded.size() << " channels differ\n";
    }
    return EXIT_SUCCESS;
}
