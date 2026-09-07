// Headless beauty render, for before/after comparison across a code change. Loads a scene exactly as main.cpp does, accumulates N path-traced passes, and writes one path-traced AOV (--aov, Beauty by default) as an 8-bit PNG through the same display encoding the viewer shows it under.
// Exists because the renderer is a GLFW application: comparing two revisions otherwise means two manual screenshots, which cannot be pixel-differenced and cannot be trusted to share a camera. Everything here is deterministic -- fixed camera from profile.json, a fixed scramble seed for the whole render with the sample index advancing per pass, no interaction -- so two runs over unchanged code produce a byte-identical file, which is what makes a non-zero diff meaningful.
// --compare takes a previously written PNG and reports max/RMS channel deviation against the render just produced, so "did this change the picture, and where" is answered numerically rather than by eye.
// Same standalone-CLI convention as the validate tools: no test framework, non-zero exit on failure.

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cctype>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

#include <OpenColorIO/OpenColorIO.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <zlib.h>

#include "engine/config/profile_config.h"
#include "engine/config/scene_config.h"
#include "engine/debug/aov.h"
#include "engine/debug/power_spectrum.h"
#include "engine/debug/render_stats.h"
#include "engine/gfx/hdr_image.h"
#include "engine/gfx/ocio_display_transform.h"
#include "engine/scene/camera.h"
#include "engine/scene/embree_accel.h"
#include "engine/scene/environment_map.h"
#include "engine/scene/gltf_loader.h"
#include "engine/scene/light.h"
#include "engine/scene/material_binding.h"
#include "engine/scene/path_tracer.h"
#include "engine/scene/thread_pool.h"

namespace OCIO = OCIO_NAMESPACE;

namespace {

// Which PathTraceResult image an AOV reads, null for the AOVs renderPathTraced does not produce. Keyed off AovId rather than a private name list so --aov and the viewer's dropdown name the same 27 AOVs by the same names; which buffer each one displays is main.cpp's selectPathTracedImage and is not restated here.
using PathTracedLane = engine::gfx::HdrImage engine::scene::PathTraceResult::*;

PathTracedLane pathTracedLane(engine::debug::AovId aov) {
    using engine::debug::AovId;
    using Result = engine::scene::PathTraceResult;
    switch (aov) {
        case AovId::Beauty:           return &Result::beauty;
        case AovId::BounceCount:      return &Result::bounceHeatmap;
        case AovId::AO:               return &Result::ao;
        case AovId::Shadow:           return &Result::shadow;
        case AovId::DirectDiffuse:    return &Result::directDiffuse;
        case AovId::IndirectDiffuse:  return &Result::indirectDiffuse;
        case AovId::DirectSpecular:   return &Result::directSpecular;
        case AovId::IndirectSpecular: return &Result::indirectSpecular;
        case AovId::Refraction:       return &Result::refraction;
        default:                      return nullptr;
    }
}

struct Options {
    std::string scenePath = "scenes/cornell.json";
    std::string outPath;
    std::string comparePath;
    // Linear-light companions to --out/--compare, for measuring convergence rather than inspecting an image. The PNG
    // path cannot do that job: it is display-transformed and 8-bit, so it compresses highlights and clamps everything
    // above display range, and a sampling change's effect on exactly those bright high-variance regions reads as zero.
    std::string outExrPath;
    std::string compareExrPath;
    // Reports how --compare-exr's error distributes over spatial frequency rather than only how large it is. A sampling
    // change that rearranges error without reducing it is invisible to RMSE by construction, so RMSE alone cannot
    // confirm or refute one.
    bool errorSpectrum = false;
    int width = 0;   // 0 = profile.json's window size
    int height = 0;
    int passes = 64;
    // The scramble seed is one realization of the randomization, not a property of the sampler: two seeds give two
    // independent error images with the same expected RMSE. Exposed because a claim that two samplers converge equally
    // well needs the spread across seeds to say what "equally" means, and because a reference sharing a seed with the
    // render measured against it also shares that render's exact samples, cancelling part of the error being measured.
    std::uint32_t scrambleSeed = 1;
    float exposureEv = 0.0F;
    // -1 = use the scene's own authored environment.lightEnabled default; 0/1 override it -- lets a
    // headless capture of the classic (env-off) Cornell variant not need a second scene.json.
    int envLight = -1;
    // Resolved by --aov. Defaulting to Beauty keeps every existing invocation -- and the bit-identity gate built on them -- unchanged.
    PathTracedLane lane = &engine::scene::PathTraceResult::beauty;
    std::string aovName = "Beauty";
};

// Case- and separator-insensitive match against kAovNames, whose entries are HUD labels ("Bounce Count", "Indirect Specular"): the CLI takes bounce-count, bounce_count or bouncecount for the same AOV rather than introducing a second vocabulary to keep in sync.
std::string normalizeAovName(const std::string& name) {
    std::string out;
    for (const char c : name) {
        if (c == ' ' || c == '-' || c == '_') {
            continue;
        }
        out.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
    }
    return out;
}

// Rejects by category so the message says why an AOV is unavailable rather than that it is unknown: the primary-hit AOVs live in rasterizer.h's RasterGBuffer and HSV/Luminance/Sobel/Gabor are GPU filters over Beauty, and this tool runs neither the rasterizer nor a GL context.
bool resolveAov(const std::string& requested, Options& options) {
    const std::string wanted = normalizeAovName(requested);
    for (int i = 0; i < static_cast<int>(engine::debug::AovId::Count); ++i) {
        if (normalizeAovName(engine::debug::kAovNames[i]) != wanted) {
            continue;
        }
        const PathTracedLane lane = pathTracedLane(static_cast<engine::debug::AovId>(i));
        if (lane == nullptr) {
            std::cerr << "render_beauty: AOV \"" << engine::debug::kAovNames[i]
                      << "\" is not path-traced -- it is a rasterizer G-buffer AOV or a post-filter "
                         "over Beauty, neither of which this tool runs\n";
            return false;
        }
        options.lane = lane;
        options.aovName = engine::debug::kAovNames[i];
        return true;
    }
    std::cerr << "render_beauty: unknown AOV \"" << requested << "\"; path-traced AOVs are:";
    for (int i = 0; i < static_cast<int>(engine::debug::AovId::Count); ++i) {
        if (pathTracedLane(static_cast<engine::debug::AovId>(i)) != nullptr) {
            std::cerr << ' ' << normalizeAovName(engine::debug::kAovNames[i]);
        }
    }
    std::cerr << '\n';
    return false;
}

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

// The viewer's sRGB display path, evaluated on the CPU. Built from the same config/colorspace/display/view constants the display shaders are generated from (ocio_display_transform.h), so this is the same transform rather than a second definition of it. In place, RGB triples.
void applyOcioDisplayTransform(std::vector<float>& rgb, int width, int height) {
    const OCIO::ConstConfigRcPtr config =
        OCIO::Config::CreateFromBuiltinConfig(engine::gfx::kOcioConfigName);
    const OCIO::ConstProcessorRcPtr processor =
        config->getProcessor(engine::gfx::kOcioSceneColorSpace, engine::gfx::kOcioSrgbDisplay,
                              engine::gfx::kOcioView, OCIO::TRANSFORM_DIR_FORWARD);
    OCIO::PackedImageDesc desc(rgb.data(), width, height, OCIO::CHANNEL_ORDERING_RGB);
    processor->getDefaultCPUProcessor()->apply(desc);
}

// Scene-referred image -> display-referred 8-bit, matching the viewer's pipeline exactly: exposure multiply, the display curve, then dither and quantize.
// applyDisplayTransform mirrors presentFrame's `isBeauty ? userLut : Raw`: only Beauty is scene-referred radiance, and putting a data AOV like AO or Shadow through a display curve would distort values that are already display-ready. Raw is the OCIO-free branch, exactly what buildRawFragmentSource does -- exposure, then dither and quantize.
std::vector<unsigned char> encodeForDisplay(const engine::gfx::HdrImage& image, float exposureEv,
                                             bool applyDisplayTransform) {
    std::vector<float> rgb(static_cast<std::size_t>(image.width) *
                            static_cast<std::size_t>(image.height) * 3);
    const float exposure = std::pow(2.0F, exposureEv);
    for (std::size_t i = 0; i < rgb.size() / 3; ++i) {
        rgb[(i * 3) + 0] = image.rgba[(i * 4) + 0] * exposure;
        rgb[(i * 3) + 1] = image.rgba[(i * 4) + 1] * exposure;
        rgb[(i * 3) + 2] = image.rgba[(i * 4) + 2] * exposure;
    }

    if (applyDisplayTransform) {
        applyOcioDisplayTransform(rgb, image.width, image.height);
    }

    std::vector<unsigned char> out(rgb.size());
    for (int y = 0; y < image.height; ++y) {
        for (int x = 0; x < image.width; ++x) {
            const std::size_t i = ((static_cast<std::size_t>(y) *
                                     static_cast<std::size_t>(image.width)) +
                                    static_cast<std::size_t>(x)) * 3;
            const glm::vec3 dither =
                ditherOffset((static_cast<float>(x) + 0.5F) / static_cast<float>(image.width),
                              (static_cast<float>(y) + 0.5F) / static_cast<float>(image.height));
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

// How the error against the reference distributes over spatial frequency, as each octave band's share of total power.
// RMSE already reports the total; what a blue-noise sampler claims to change is the ARRANGEMENT, which is invisible to
// any single number and is exactly this distribution. Normalised by the total so two renders with different error
// magnitudes are still comparable as distributions -- the claim under test is that the total is unchanged and only its
// placement moved, and those are two separate readings.
// Luminance rather than per-channel: a scalar field is what has a spectrum, and error visibility is a luminance effect.
void reportErrorSpectrum(const engine::gfx::HdrImage& image, const engine::gfx::HdrImage& reference) {
    const auto pixels = static_cast<std::size_t>(image.width) * static_cast<std::size_t>(image.height);
    std::vector<double> luminanceError(pixels);
    double squaredSum = 0.0;
    for (std::size_t p = 0; p < pixels; ++p) {
        const std::size_t i = p * 4;
        const double e = (0.2126 * (image.rgba[i] - reference.rgba[i])) +
                          (0.7152 * (image.rgba[i + 1] - reference.rgba[i + 1])) +
                          (0.0722 * (image.rgba[i + 2] - reference.rgba[i + 2]));
        luminanceError[p] = e;
        squaredSum += e * e;
    }

    const std::array<double, engine::debug::kSpectrumBands> bands =
        engine::debug::octaveBandPower(luminanceError, image.width, image.height);
    // Bands are far from equal in width, so a raw share says nothing on its own -- what matters is the share relative to
    // what white noise would put there. Printed alongside, so a band reads directly as blue (below 1.0) or red (above).
    const std::array<double, engine::debug::kSpectrumBands> white =
        engine::debug::whiteNoiseBandShare(image.width, image.height);
    const double total = std::accumulate(bands.begin(), bands.end(), 0.0);
    std::cout << "render_beauty: error spectrum -- luminance RMS "
              << std::sqrt(squaredSum / static_cast<double>(pixels))
              << ", octave bands (low to high), share of total power and ratio to white noise\n";
    for (int band = engine::debug::kSpectrumBands - 1; band >= 0; --band) {
        const double high = 0.5 / std::exp2(band);
        const double share = bands[static_cast<std::size_t>(band)] / total;
        // A band can hold no lattice points at all below 256 px on an axis, and dividing by its share would print nan.
        const double whiteShare = white[static_cast<std::size_t>(band)];
        std::cout << "  " << std::setw(8) << (high * 0.5) << " - " << std::setw(8) << high << " c/px   "
                  << std::setw(8) << (100.0 * share) << " %";
        if (whiteShare > 0.0) {
            std::cout << "   " << std::setw(7) << (share / whiteShare) << "x white";
        }
        std::cout << "\n";
    }
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
        } else if (std::strcmp(argv[i], "--out-exr") == 0) {
            if (!needsValue("--out-exr")) { return false; }
            options.outExrPath = argv[++i];
        } else if (std::strcmp(argv[i], "--compare-exr") == 0) {
            if (!needsValue("--compare-exr")) { return false; }
            options.compareExrPath = argv[++i];
        } else if (std::strcmp(argv[i], "--error-spectrum") == 0) {
            options.errorSpectrum = true;
        } else if (std::strcmp(argv[i], "--seed") == 0) {
            if (!needsValue("--seed")) { return false; }
            options.scrambleSeed = static_cast<std::uint32_t>(std::strtoul(argv[++i], nullptr, 10));
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
        } else if (std::strcmp(argv[i], "--aov") == 0) {
            if (!needsValue("--aov")) { return false; }
            if (!resolveAov(argv[++i], options)) { return false; }
        } else if (std::strcmp(argv[i], "--env-light") == 0) {
            if (!needsValue("--env-light")) { return false; }
            options.envLight = std::atoi(argv[++i]) != 0 ? 1 : 0;
        } else {
            std::cerr << "render_beauty: unknown argument '" << argv[i]
                      << "'\nusage: render_beauty [--scene scenes/x.json] --out out.png [--out-exr out.exr] [--compare-exr ref.exr] "
                         "[--compare ref.png] [--error-spectrum] [--seed N] [--passes N] [--width W] [--height H] "
                         "[--exposure EV] [--aov name]\n";
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
    if (options.errorSpectrum && options.compareExrPath.empty()) {
        std::cerr << "render_beauty: --error-spectrum needs --compare-exr to have an error to analyse\n";
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
    std::vector<int> instanceLightIndex(model->instances.size(), -1);
    const std::vector<engine::scene::QuadLight> quadLights =
        engine::scene::buildQuadLights(sceneConfig->lights, rootTransform);
    engine::scene::appendQuadLights(*model, quadLights, instanceLightIndex);

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
        .aoMaxDistance = profileConfig->pathTracer.aoMaxDistance,
        .bumpStrength = materialConfig->bumpStrength,
        .roughnessMin = materialConfig->roughnessMin,
        .roughnessMax = materialConfig->roughnessMax,
        .diffuseColour = materialConfig->diffuseColour,
        .ior = materialConfig->ior,
        .abbe = materialConfig->abbe,
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

    // --env-light overrides the scene's own authored default (-1 = no override).
    const bool envLightEnabled =
        options.envLight >= 0 ? options.envLight != 0 : sceneConfig->environment.lightEnabled;
    const engine::scene::LightSet lights(envLightEnabled ? &environmentMap : nullptr,
                                         /*envRotationRadians=*/0.0F, /*envExposure=*/1.0F, quadLights);

    // Mean of `passes` single-sample passes -- the same accumulation PathTraceDriver performs, done synchronously. Each pass advances the sampler's sequence index rather than re-randomizing it, so the accumulated samples stratify against each other exactly as they do in the viewer; the scramble seed is held fixed for the whole render (--seed, default 1), which is what makes two runs over unchanged code byte-identical. Every lane is traced regardless of which one --aov selects: they share the sample set and the reconstruction filter, so producing one alone would not be cheaper.
    engine::scene::PathTraceResult result = engine::scene::makePathTraceResult(width, height);
    engine::gfx::HdrImage accumulated = engine::gfx::HdrImage{
        width, height, std::vector<float>(static_cast<std::size_t>(width) *
                                           static_cast<std::size_t>(height) * 4, 0.0F)};
    const std::atomic<std::uint64_t> generation{1};
    // Reported, not discarded: ray counts are the one figure that says whether a change altered what the integrator
    // actually did, as opposed to only which values it sampled. Deterministic here where the viewer's are not, since
    // this tool renders a fixed pass count with no cancellation.
    engine::debug::PassStats stats;
    for (int pass = 0; pass < options.passes; ++pass) {
        engine::scene::renderPathTraced(camera, *accel, model->shadingTriangles, model->instances,
                                         instanceLightIndex, lights, width, height,
                                         /*showSky=*/true, baseSettings, *perInstanceSettings,
                                         options.scrambleSeed, /*sampleBase=*/pass, /*sampleCount=*/options.passes, generation,
                                         /*requestedGeneration=*/1U, threadPool, stats, result);
        for (std::size_t i = 0; i < accumulated.rgba.size(); ++i) {
            accumulated.rgba[i] += (result.*options.lane).rgba[i];
        }
    }
    for (float& v : accumulated.rgba) {
        v /= static_cast<float>(options.passes);
    }

    const engine::debug::RayCounts rays = stats.rays();
    std::cout << "render_beauty: rays over " << options.passes << " passes -- primary " << rays.primary << ", bounce "
              << rays.bounce << ", ao " << rays.ao << ", shadow " << rays.shadow << ", total " << rays.total() << "\n";

    if (!options.outExrPath.empty()) {
        if (!engine::gfx::writeExr(options.outExrPath, accumulated)) {
            return EXIT_FAILURE;
        }
        std::cout << "render_beauty: wrote " << options.outExrPath << " (linear " << options.aovName << ", "
                  << width << "x" << height << ", " << options.passes << " passes)\n";
    }

    if (!options.compareExrPath.empty()) {
        const std::optional<engine::gfx::HdrImage> reference = engine::gfx::loadExr(options.compareExrPath);
        if (!reference.has_value()) {
            return EXIT_FAILURE;
        }
        if (reference->width != width || reference->height != height) {
            std::cerr << "render_beauty: --compare-exr image is " << reference->width << "x"
                      << reference->height << ", this render is " << width << "x" << height << "\n";
            return EXIT_FAILURE;
        }
        // Two metrics, because neither alone characterises a render's error. Absolute RMSE is dominated by the brightest
        // pixels, so it tracks the highlights a sampling change moves most; relative MSE (Rousselle et al. 2011, the
        // standard metric in the denoising/sampling literature) divides by the reference's own intensity, so a dim
        // corner's noise counts as much as a bright one's. The epsilon is the conventional guard against dividing by a
        // black pixel, not a tuned parameter.
        constexpr double kRelativeEpsilon = 1e-2;
        double squaredSum = 0.0;
        double relativeSum = 0.0;
        std::size_t counted = 0;
        for (std::size_t i = 0; i < accumulated.rgba.size(); ++i) {
            if (i % 4 == 3) {
                continue;  // alpha carries no radiance
            }
            const double delta = static_cast<double>(accumulated.rgba[i]) - static_cast<double>(reference->rgba[i]);
            const double ref = reference->rgba[i];
            squaredSum += delta * delta;
            relativeSum += (delta * delta) / ((ref * ref) + kRelativeEpsilon);
            ++counted;
        }
        const double rmse = std::sqrt(squaredSum / static_cast<double>(counted));
        const double relativeMse = relativeSum / static_cast<double>(counted);
        std::cout << "render_beauty: vs " << options.compareExrPath << " -- linear RMSE " << rmse
                  << ", relMSE " << relativeMse << "\n";
        if (options.errorSpectrum) {
            reportErrorSpectrum(accumulated, *reference);
        }
    }

    const bool isBeauty = options.lane == &engine::scene::PathTraceResult::beauty;
    const std::vector<unsigned char> encoded =
        encodeForDisplay(accumulated, options.exposureEv, isBeauty);
    if (!writePng(options.outPath, width, height, encoded)) {
        return EXIT_FAILURE;
    }
    std::cout << "render_beauty: wrote " << options.outPath << " (" << options.aovName << ", " << width
              << "x" << height << ", " << options.passes << " passes)\n";

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
