#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

// GLEW before GLFW — see gl_debug.cpp for why.
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_inverse.hpp>

#include "engine/config/profile_config.h"
#include "engine/config/scene_config.h"
#include "engine/debug/frame_stats.h"
#include "engine/debug/gpu_timer.h"
#include "engine/debug/histogram.h"
#include "engine/debug/hud_overlay.h"
#include "engine/debug/memory_tracker.h"
#include "engine/debug/scene_stats.h"
#include "engine/debug/system_info.h"
#include "engine/gfx/cubemap_texture.h"
#include "engine/gfx/env_prefilter_pass.h"
#include "engine/gfx/gl_debug.h"
#include "engine/gfx/hdr_framebuffer.h"
#include "engine/gfx/hdr_image.h"
#include "engine/gfx/ocio_display_transform.h"
#include "engine/gfx/post_process_pass.h"
#include "engine/gfx/shader_program.h"
#include "engine/platform/window.h"
#include "engine/scene/bvh.h"
#include "engine/scene/camera.h"
#include "engine/scene/debug_camera_controller.h"
#include "engine/scene/frustum.h"
#include "engine/scene/gltf_loader.h"
#include "engine/scene/sh_irradiance.h"

namespace {

void glfwErrorCallback(int error, const char* description) {
    std::cerr << "GLFW error " << error << ": " << description << '\n';
}

// Transforms a local-space AABB's 8 corners by `transform` and returns the resulting world-space AABB -- for frustum culling and the World position debug AOV.
std::pair<glm::vec3, glm::vec3> worldSpaceBounds(const glm::vec3& localMin,
                                                  const glm::vec3& localMax,
                                                  const glm::mat4& transform) {
    glm::vec3 worldMin(std::numeric_limits<float>::max());
    glm::vec3 worldMax(std::numeric_limits<float>::lowest());
    for (int i = 0; i < 8; ++i) {
        const glm::vec3 corner((i & 1) != 0 ? localMax.x : localMin.x,
                                (i & 2) != 0 ? localMax.y : localMin.y,
                                (i & 4) != 0 ? localMax.z : localMin.z);
        const glm::vec3 worldCorner = glm::vec3(transform * glm::vec4(corner, 1.0F));
        worldMin = glm::min(worldMin, worldCorner);
        worldMax = glm::max(worldMax, worldCorner);
    }
    return {worldMin, worldMax};
}

// Deterministic per-index false color for the Object/Material ID debug AOV (golden-ratio fractional hash -- cheap, well-spread across ids).
glm::vec3 falseColorForId(int id) {
    const auto f = static_cast<float>(id);
    return {std::fmod(f * 0.6180339887F, 1.0F), std::fmod((f * 0.3247179572F) + 0.5F, 1.0F),
            std::fmod((f * 0.1231234F) + 0.25F, 1.0F)};
}

const char* lutName(engine::gfx::OcioDisplayTransform::Lut lut) {
    using Lut = engine::gfx::OcioDisplayTransform::Lut;
    return lut == Lut::SRGB ? "sRGB" : lut == Lut::Rec709 ? "Rec709" : "Raw";
}

// Static Gabor kernel weights: 4 orientations (0/45/90/135deg) x 5x5 taps, precomputed once here rather than in the shader -- these never change at runtime, so re-deriving sin/cos/exp per-fragment on the GPU would be pure redundant work. Consumed by edge_filter.frag's Gabor branch; tap order (dy outer, dx inner, both -2..2) must match its sampling loop.
std::array<float, 100> buildGaborKernel() {
    constexpr float kSigma = 1.4F;
    constexpr float kLambda = 4.0F;
    constexpr float kGamma = 0.5F;
    constexpr std::array<float, 4> kOrientationsDeg = {0.0F, 45.0F, 90.0F, 135.0F};

    std::array<float, 100> kernel{};
    for (int o = 0; o < 4; ++o) {
        const float theta = glm::radians(kOrientationsDeg[static_cast<std::size_t>(o)]);
        int tapIndex = 0;
        for (int dy = -2; dy <= 2; ++dy) {
            for (int dx = -2; dx <= 2; ++dx) {
                const auto x = static_cast<float>(dx);
                const auto y = static_cast<float>(dy);
                const float xp = (x * std::cos(theta)) + (y * std::sin(theta));
                const float yp = (-x * std::sin(theta)) + (y * std::cos(theta));
                const float envelope = std::exp(
                    -((xp * xp) + (kGamma * kGamma * yp * yp)) / (2.0F * kSigma * kSigma));
                // Odd/quadrature carrier (sin, not cos) -- edge-sensitive, not bar/ridge-sensitive.
                const float carrier = std::sin(2.0F * glm::pi<float>() * xp / kLambda);
                kernel[(static_cast<std::size_t>(o) * 25) + static_cast<std::size_t>(tapIndex)] =
                    envelope * carrier;
                ++tapIndex;
            }
        }
    }
    return kernel;
}

// Everything the render loop touches every frame, plus the GPU resources and one-time-computed state (cached uniform locations, IBL bake, BVH) that must stay alive for the run's duration. A pure aggregate (no user-declared constructors) so initializeApp can return it by value via designated initializers -- each RAII member's own move constructor (already verified elsewhere to correctly transfer GL handles/tracked byte counts) handles the actual transfer.
struct AppResources {
    engine::gfx::HdrFramebuffer hdrFbo;
    engine::gfx::ShaderProgram sceneShader;
    engine::gfx::ShaderProgram edgeFilterShader;
    engine::gfx::ShaderProgram skyShader;
    engine::gfx::OcioDisplayTransform ocioTransform;
    engine::gfx::Texture environmentTexture;
    engine::gfx::PrefilteredEnvironment prefilteredEnv;
    engine::scene::LoadedModel stumpModel;
    std::vector<std::pair<glm::vec3, glm::vec3>> instanceWorldBounds;
    int totalTriangles;
    int totalPoints;

    engine::gfx::PostProcessPass postProcess;
    engine::debug::HudOverlay hud;
    engine::debug::FrameStats frameStats;
    engine::debug::GpuTimer geomTimer;
    engine::debug::GpuTimer postTimer;
    engine::debug::Histogram histogram;
    engine::scene::DebugCameraController debugCamera;
    engine::debug::GpuInfo gpuInfo;

    // No RAII wrapper (see main()'s cleanup) -- matches the single attribute-less VAO this project already hand-manages this way.
    unsigned int skyVao;

    int uModelLoc;
    int uViewLoc;
    int uProjectionLoc;
    int uNormalMatrixLoc;
    int uBaseColorFactorLoc;
    int uMetallicFactorLoc;
    int uRoughnessFactorLoc;
    int uBoundsMinLoc;
    int uBoundsMaxLoc;
    int uObjectIdColorLoc;
    int uCameraPosLoc;
    int uAovLoc;
    int uChannelViewLoc;
    int uEnvRotationRadiansLoc;
    int uFilterModeLoc;
    int uSkyInvViewProjLoc;
    int uSkyCameraPosLoc;
    int uSkyEnvRotationRadiansLoc;

    // HUD-editable UI/run state.
    int aov;
    int channelView;
    engine::gfx::OcioDisplayTransform::Lut userLut;
    engine::debug::FramingOverlayState framingState;
    bool showSky;
    int envRotationDegrees;

    // Orbit-pick and RAM-sampling state carried frame to frame.
    bool orbitPickRequested;
    double lastCursorX;
    double lastCursorY;
    std::size_t ramBytes;
    std::size_t systemAvailableBytes;
    std::uint64_t systemTotalBytes;
    std::chrono::steady_clock::time_point lastRamSample;
    std::chrono::steady_clock::time_point lastFrameTime;
};

struct RequiredShaders {
    engine::gfx::ShaderProgram sceneShader;
    engine::gfx::ShaderProgram edgeFilterShader;
    engine::gfx::ShaderProgram equirectToCubemapShader;
    engine::gfx::ShaderProgram prefilterShader;
    engine::gfx::ShaderProgram skyShader;
    engine::gfx::OcioDisplayTransform ocioTransform;
};

// All shader/OCIO loading in one place so initializeApp has one all-or-nothing check, matching how it already treats model/environment loading as a single startup gate.
std::optional<RequiredShaders> loadShaders() {
    std::optional<engine::gfx::ShaderProgram> sceneShader = engine::gfx::ShaderProgram::loadFromFiles(
        ASSET_ROOT_DIR "/shaders/pbr.vert", ASSET_ROOT_DIR "/shaders/pbr.frag");
    std::optional<engine::gfx::ShaderProgram> edgeFilterShader =
        engine::gfx::ShaderProgram::loadFromFiles(ASSET_ROOT_DIR "/shaders/fullscreen_triangle.vert",
                                                   ASSET_ROOT_DIR "/shaders/edge_filter.frag");
    // IBL preprocessing shaders (env_prefilter_pass.h) -- one-time startup use only, never touched again once the prefiltered cubemap is built.
    std::optional<engine::gfx::ShaderProgram> equirectToCubemapShader =
        engine::gfx::ShaderProgram::loadFromFiles(
            ASSET_ROOT_DIR "/shaders/fullscreen_triangle.vert",
            ASSET_ROOT_DIR "/shaders/equirect_to_cubemap.frag");
    std::optional<engine::gfx::ShaderProgram> prefilterShader = engine::gfx::ShaderProgram::loadFromFiles(
        ASSET_ROOT_DIR "/shaders/fullscreen_triangle.vert",
        ASSET_ROOT_DIR "/shaders/prefilter_specular.frag");
    // Background pass sampling the raw equirect map directly -- see sky.frag; toggled at runtime via the HUD, off by default.
    std::optional<engine::gfx::ShaderProgram> skyShader = engine::gfx::ShaderProgram::loadFromFiles(
        ASSET_ROOT_DIR "/shaders/fullscreen_triangle.vert", ASSET_ROOT_DIR "/shaders/sky.frag");
    std::optional<engine::gfx::OcioDisplayTransform> ocioTransform =
        engine::gfx::OcioDisplayTransform::create();

    if (!sceneShader || !edgeFilterShader || !equirectToCubemapShader || !prefilterShader ||
        !skyShader || !ocioTransform) {
        return std::nullopt;
    }
    return RequiredShaders{
        std::move(*sceneShader),  std::move(*edgeFilterShader),
        std::move(*equirectToCubemapShader), std::move(*prefilterShader),
        std::move(*skyShader),    std::move(*ocioTransform),
    };
}

// Punctual lights (Phase 4), sourced from scene.json -- set once here, same as the single fixed light this replaces; no runtime light-editing UI exists yet. A directional's color of pi cancels the shader's Lambertian /pi so a directly-lit surface's brightness matches its albedo (unchanged from that convention).
void uploadLights(const engine::gfx::ShaderProgram& sceneShader,
                   const std::vector<engine::config::Light>& lights) {
    const int lightCount = static_cast<int>(lights.size()) < engine::config::kMaxLights
                                ? static_cast<int>(lights.size())
                                : engine::config::kMaxLights;
    GL_CALL(glUniform1i(sceneShader.uniformLocation("uLightCount"), lightCount));
    for (int i = 0; i < lightCount; ++i) {
        const engine::config::Light& light = lights[static_cast<std::size_t>(i)];
        const std::string index = "[" + std::to_string(i) + "]";
        GL_CALL(glUniform1i(sceneShader.uniformLocation("uLightType" + index),
                            light.type == engine::config::Light::Type::Directional ? 0 : 1));
        GL_CALL(glUniform3fv(sceneShader.uniformLocation("uLightPositionOrDir" + index), 1,
                             &light.directionOrPosition[0]));
        GL_CALL(glUniform3fv(sceneShader.uniformLocation("uLightColor" + index), 1,
                             &light.color[0]));
        GL_CALL(glUniform1f(sceneShader.uniformLocation("uLightRange" + index), light.range));
    }
}

struct IblResult {
    engine::gfx::Texture environmentTexture;
    engine::gfx::PrefilteredEnvironment prefilteredEnv;
};

// IBL (Phase 4): equirect env map -> prefiltered specular cubemap (Karis 2013 split-sum) + SH-9 diffuse irradiance (Ramamoorthi & Hanrahan 2001). Both are one-time startup preprocessing; texture unit 6 is otherwise unused, so the cubemap is bound once here rather than every frame.
IblResult buildAndUploadIbl(const engine::gfx::HdrImage& environmentImage,
                            const engine::gfx::ShaderProgram& sceneShader,
                            const engine::gfx::ShaderProgram& equirectToCubemapShader,
                            const engine::gfx::ShaderProgram& prefilterShader) {
    GL_CALL(glEnable(GL_TEXTURE_CUBE_MAP_SEAMLESS));
    const auto envStart = std::chrono::steady_clock::now();
    // GL_REPEAT, not CLAMP_TO_EDGE: this is an equirectangular panorama, which needs horizontal wraparound at the phi=+/-180deg seam (u=0/u=1) for both correct bilinear sampling there and correct glGenerateMipmap downsampling across that seam -- under CLAMP_TO_EDGE the mip chain bakes in a faint discontinuity at that column, static in texture space but rotated into view at nonzero uEnvRotationRadians. createFromFloatPixels applies one wrap mode to both axes, so this also repeats vertically: a real but sub-texel cost (bilinear filtering exactly at the zenith/nadir blends across to the opposite pole's row), accepted rather than worth a per-axis wrap parameter on the shared Texture API for this one caller.
    engine::gfx::Texture environmentTexture = engine::gfx::Texture::createFromFloatPixels(
        environmentImage.width, environmentImage.height, environmentImage.rgba.data(), GL_REPEAT);
    std::cout << "environmentTexture upload: "
              << std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                             envStart)
                     .count()
              << " ms\n"
              << std::flush;
    const auto prefilterStart = std::chrono::steady_clock::now();
    engine::gfx::PrefilteredEnvironment prefilteredEnv = engine::gfx::buildPrefilteredEnvironment(
        environmentTexture, equirectToCubemapShader, prefilterShader);
    std::cout << "buildPrefilteredEnvironment: "
              << std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                             prefilterStart)
                     .count()
              << " ms\n"
              << std::flush;
    sceneShader.use();
    GL_CALL(glUniform1i(sceneShader.uniformLocation("uPrefilteredSpecular"), 6));
    GL_CALL(glUniform1f(sceneShader.uniformLocation("uPrefilteredSpecularMaxLod"),
                        static_cast<float>(prefilteredEnv.specularMipCount - 1)));
    prefilteredEnv.specular.bind(6);

    const auto shStart = std::chrono::steady_clock::now();
    const std::array<glm::vec3, 9> shIrradiance = engine::scene::projectIrradianceSH9(environmentImage);
    GL_CALL(glUniform3fv(sceneShader.uniformLocation("uShIrradiance"), 9, &shIrradiance[0][0]));
    std::cout << "projectIrradianceSH9: "
              << std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                             shStart)
                     .count()
              << " ms, DC term = (" << shIrradiance[0].r << ", " << shIrradiance[0].g << ", "
              << shIrradiance[0].b << ")\n"
              << std::flush;

    return IblResult{std::move(environmentTexture), std::move(prefilteredEnv)};
}

struct SceneUniformLocations {
    int uModelLoc;
    int uViewLoc;
    int uProjectionLoc;
    int uNormalMatrixLoc;
    int uBaseColorFactorLoc;
    int uMetallicFactorLoc;
    int uRoughnessFactorLoc;
    int uBoundsMinLoc;
    int uBoundsMaxLoc;
    int uObjectIdColorLoc;
    int uCameraPosLoc;
    int uAovLoc;
    int uChannelViewLoc;
    int uEnvRotationRadiansLoc;
};

SceneUniformLocations cacheSceneUniformLocations(const engine::gfx::ShaderProgram& sceneShader) {
    return SceneUniformLocations{
        sceneShader.uniformLocation("uModel"),
        sceneShader.uniformLocation("uView"),
        sceneShader.uniformLocation("uProjection"),
        sceneShader.uniformLocation("uNormalMatrix"),
        sceneShader.uniformLocation("uBaseColorFactor"),
        sceneShader.uniformLocation("uMetallicFactor"),
        sceneShader.uniformLocation("uRoughnessFactor"),
        sceneShader.uniformLocation("uBoundsMin"),
        sceneShader.uniformLocation("uBoundsMax"),
        sceneShader.uniformLocation("uObjectIdColor"),
        sceneShader.uniformLocation("uCameraPos"),
        sceneShader.uniformLocation("uAov"),
        sceneShader.uniformLocation("uChannelView"),
        sceneShader.uniformLocation("uEnvRotationRadians"),
    };
}

// Sobel/Gabor's second pass (see edge_filter.frag): uHdrColor's texture unit and the Gabor kernel weights are both fixed for the whole run, set once here. Returns uFilterMode's cached location.
int setupEdgeFilterShader(const engine::gfx::ShaderProgram& edgeFilterShader) {
    edgeFilterShader.use();
    GL_CALL(glUniform1i(edgeFilterShader.uniformLocation("uHdrColor"), 0));
    const std::array<float, 100> gaborKernel = buildGaborKernel();
    GL_CALL(glUniform1fv(edgeFilterShader.uniformLocation("uGaborKernel"), 100, gaborKernel.data()));
    return edgeFilterShader.uniformLocation("uFilterMode");
}

struct SkyUniformLocations {
    int uInvViewProjLoc;
    int uCameraPosLoc;
    int uEnvRotationRadiansLoc;
};

// Sky background pass (sky.frag): samples the raw equirect map on texture unit 0, same as edgeFilterShader -- the geometry pass rebinds unit 0 to each instance's base colour texture every frame, but that happens after the sky is drawn each frame, so there's no conflict.
SkyUniformLocations setupSkyShader(const engine::gfx::ShaderProgram& skyShader) {
    skyShader.use();
    GL_CALL(glUniform1i(skyShader.uniformLocation("uEquirect"), 0));
    return SkyUniformLocations{
        skyShader.uniformLocation("uInvViewProj"),
        skyShader.uniformLocation("uCameraPos"),
        skyShader.uniformLocation("uEnvRotationRadians"),
    };
}

// Attribute-less VAO for the sky's fullscreen-triangle draw (gl_VertexID trick, see fullscreen_triangle.vert) -- Apple's core-profile driver requires some VAO bound for any draw call even with zero vertex attributes, same rationale as PostProcessPass's own VAO. Not PostProcessPass itself: that draws into the default framebuffer, not the HDR FBO the sky needs.
unsigned int createSkyVao() {
    unsigned int skyVao = 0;
    GL_CALL(glGenVertexArrays(1, &skyVao));
    return skyVao;
}

// Instance transforms and mesh bounds are fixed after load (mesh.h: "not updated if the mesh is ever mutated... it isn't, today") -- computed once here, not per frame.
std::vector<std::pair<glm::vec3, glm::vec3>> computeInstanceWorldBounds(
    const std::vector<engine::scene::MeshInstance>& instances) {
    std::vector<std::pair<glm::vec3, glm::vec3>> bounds;
    bounds.reserve(instances.size());
    for (const engine::scene::MeshInstance& instance : instances) {
        bounds.push_back(
            worldSpaceBounds(instance.mesh.boundsMin(), instance.mesh.boundsMax(), instance.transform));
    }
    return bounds;
}

// All one-time startup work: camera/model/framebuffer/shader/environment loading (nullopt on any failure -- matches the shader/model/OCIO all-or-nothing gate this replaces), one-time uniform assignment, IBL preprocessing, BVH build, and cached uniform-location lookups. Doesn't wire input callbacks -- those capture a stable AppResources& and must be set up by the caller only after this returns (see main()), since a callback capturing a reference into an AppResources that's still about to be moved into its final std::optional storage would dangle.
std::optional<AppResources> initializeApp(const engine::config::SceneConfig& sceneConfig,
                                           const engine::config::ProfileConfig& profileConfig,
                                           engine::platform::Window& window) {
    std::cout << "GL_KHR_debug available: " << std::boolalpha << engine::gfx::khrDebugAvailable()
              << '\n';
    std::cout << "GL_ARB_timer_query available: " << std::boolalpha
              << engine::debug::gpuTimerQueryAvailable() << '\n';
    const engine::debug::GpuInfo gpuInfo = engine::debug::queryGpuInfo();

    // exposure()/ev100() are logged but not render-path-consumed yet: no scene-referred exposure multiply exists, only OCIO's display-encode step (Phase 4+ real lighting is the natural point to seed it from here).
    engine::scene::DebugCameraController debugCamera(
        profileConfig.position, profileConfig.yawDegrees, profileConfig.pitchDegrees,
        profileConfig.filmBack, profileConfig.focalLengthMm, profileConfig.nearClip,
        profileConfig.farClip, profileConfig.aperture, profileConfig.shutterSeconds,
        profileConfig.iso, profileConfig.flySpeedMetersPerSecond,
        profileConfig.orbitSensitivityDegPerPixel);
    {
        const engine::scene::Camera initialCamera = debugCamera.snapshot();
        const glm::vec3 camPos = initialCamera.position();
        std::cout << "Camera: position=(" << camPos.x << ", " << camPos.y << ", " << camPos.z
                  << ") verticalFov=" << glm::degrees(initialCamera.verticalFovRadians())
                  << " deg ev100=" << initialCamera.ev100()
                  << " exposure=" << initialCamera.exposure() << '\n';
    }

    // tier1 LOD (36.5k triangles): fast iteration for shader work.
    const auto loadStart = std::chrono::steady_clock::now();
    std::optional<engine::scene::LoadedModel> stumpModel =
        engine::scene::loadGltf(std::string(ASSET_ROOT_DIR) + "/" + sceneConfig.gltfPath);
    const double loadMs =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - loadStart)
            .count();
    int totalTriangles = 0;
    if (stumpModel) {
        for (const engine::scene::MeshInstance& instance : stumpModel->instances) {
            totalTriangles += instance.mesh.triangleCount();
        }
        std::cout << "loadGltf: " << stumpModel->instances.size() << " instance(s), "
                  << totalTriangles << " triangles, " << loadMs << " ms\n"
                  << std::flush;
    }
    // "Points": total vertex-index count, i.e. 3 per triangle, not the unique vertex buffer size -- always exactly 3x triangles for this triangle-only renderer, so it's derived rather than tracked separately.
    const int totalPoints = totalTriangles * 3;

    const auto [fbWidth, fbHeight] = window.framebufferSize();
    engine::gfx::HdrFramebuffer hdrFbo(fbWidth, fbHeight);

    std::optional<RequiredShaders> shaders = loadShaders();
    // Decoded once here (not via Texture::createFromExr) since both the GPU upload below and projectIrradianceSH9 need the same CPU pixel data -- see hdr_image.h.
    std::optional<engine::gfx::HdrImage> environmentImage =
        engine::gfx::loadExr(std::string(ASSET_ROOT_DIR) + "/textures/republiqueHDR_2k.exr");

    if (!shaders || !stumpModel || !environmentImage) {
        std::cerr << "main: shader compile/link, model load, or environment map load failed, "
                     "aborting startup\n";
        return std::nullopt;
    }

    engine::gfx::PostProcessPass postProcess;
    engine::debug::HudOverlay hud(window.nativeHandle());
    engine::debug::FrameStats frameStats;
    engine::debug::GpuTimer geomTimer;
    engine::debug::GpuTimer postTimer;
    engine::debug::Histogram histogram;

    // One-time texture-unit assignment (units 0-5, see below). Explicit even though unit 0 is GL's implicit default for an unset sampler uniform — relying on that default silently breaks the moment the shader gains a second sampler. OcioDisplayTransform sets its own uHdrColor uniform the same way at construction.
    shaders->sceneShader.use();
    GL_CALL(glUniform1i(shaders->sceneShader.uniformLocation("uBaseColor"), 0));
    GL_CALL(glUniform1i(shaders->sceneShader.uniformLocation("uRoughness"), 1));
    GL_CALL(glUniform1i(shaders->sceneShader.uniformLocation("uAo"), 2));
    GL_CALL(glUniform1i(shaders->sceneShader.uniformLocation("uNormal"), 3));
    GL_CALL(glUniform1i(shaders->sceneShader.uniformLocation("uBump"), 4));
    GL_CALL(glUniform1i(shaders->sceneShader.uniformLocation("uSpecular"), 5));
    uploadLights(shaders->sceneShader, sceneConfig.lights);

    IblResult ibl = buildAndUploadIbl(*environmentImage, shaders->sceneShader,
                                       shaders->equirectToCubemapShader, shaders->prefilterShader);

    // BVH (Phase 4): built once from the loaded scene's world-space triangles, Phase 5 infrastructure -- not wired into rendering this phase ("no secondary rays" holds). Correctness is exercised by tools/bvh_validate.cpp, not by anything here; this log line is the only thing that observes it this phase.
    const auto bvhBuildStart = std::chrono::steady_clock::now();
    const engine::scene::Bvh sceneBvh =
        engine::scene::Bvh::build(std::move(stumpModel->worldTriangles));
    const double bvhBuildMs =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                    bvhBuildStart)
            .count();
    std::cout << "Bvh::build: " << sceneBvh.triangleCount() << " triangles, "
              << sceneBvh.nodeCount() << " nodes, " << bvhBuildMs << " ms\n"
              << std::flush;

    // Depth AOV's linearization near/far -- fixed for the whole run, never mutated by the debug camera (see profile_config.h).
    GL_CALL(glUniform1f(shaders->sceneShader.uniformLocation("uNearClip"), profileConfig.nearClip));
    GL_CALL(glUniform1f(shaders->sceneShader.uniformLocation("uFarClip"), profileConfig.farClip));
    const SceneUniformLocations sceneLocs = cacheSceneUniformLocations(shaders->sceneShader);
    const int uFilterModeLoc = setupEdgeFilterShader(shaders->edgeFilterShader);
    const SkyUniformLocations skyLocs = setupSkyShader(shaders->skyShader);
    const unsigned int skyVao = createSkyVao();
    std::vector<std::pair<glm::vec3, glm::vec3>> instanceWorldBounds =
        computeInstanceWorldBounds(stumpModel->instances);

    return AppResources{
        .hdrFbo = std::move(hdrFbo),
        .sceneShader = std::move(shaders->sceneShader),
        .edgeFilterShader = std::move(shaders->edgeFilterShader),
        .skyShader = std::move(shaders->skyShader),
        .ocioTransform = std::move(shaders->ocioTransform),
        .environmentTexture = std::move(ibl.environmentTexture),
        .prefilteredEnv = std::move(ibl.prefilteredEnv),
        .stumpModel = std::move(*stumpModel),
        .instanceWorldBounds = std::move(instanceWorldBounds),
        .totalTriangles = totalTriangles,
        .totalPoints = totalPoints,
        .postProcess = std::move(postProcess),
        .hud = std::move(hud),
        .frameStats = std::move(frameStats),
        .geomTimer = std::move(geomTimer),
        .postTimer = std::move(postTimer),
        .histogram = std::move(histogram),
        .debugCamera = std::move(debugCamera),
        .gpuInfo = gpuInfo,
        .skyVao = skyVao,
        .uModelLoc = sceneLocs.uModelLoc,
        .uViewLoc = sceneLocs.uViewLoc,
        .uProjectionLoc = sceneLocs.uProjectionLoc,
        .uNormalMatrixLoc = sceneLocs.uNormalMatrixLoc,
        .uBaseColorFactorLoc = sceneLocs.uBaseColorFactorLoc,
        .uMetallicFactorLoc = sceneLocs.uMetallicFactorLoc,
        .uRoughnessFactorLoc = sceneLocs.uRoughnessFactorLoc,
        .uBoundsMinLoc = sceneLocs.uBoundsMinLoc,
        .uBoundsMaxLoc = sceneLocs.uBoundsMaxLoc,
        .uObjectIdColorLoc = sceneLocs.uObjectIdColorLoc,
        .uCameraPosLoc = sceneLocs.uCameraPosLoc,
        .uAovLoc = sceneLocs.uAovLoc,
        .uChannelViewLoc = sceneLocs.uChannelViewLoc,
        .uEnvRotationRadiansLoc = sceneLocs.uEnvRotationRadiansLoc,
        .uFilterModeLoc = uFilterModeLoc,
        .uSkyInvViewProjLoc = skyLocs.uInvViewProjLoc,
        .uSkyCameraPosLoc = skyLocs.uCameraPosLoc,
        .uSkyEnvRotationRadiansLoc = skyLocs.uEnvRotationRadiansLoc,
        // aov selects which debug buffer pbr.frag outputs (see its uAov comment for the index order); channelView isolates one R/G/B channel of whatever aov currently shows. userLut is the LUT 'L' cycles through -- kept separate from OcioDisplayTransform's active LUT because non-Beauty AOVs force Raw (see the LUT-select comment in renderFrame) and must not clobber the user's actual choice. Both aov and userLut start from scene.json rather than a fixed literal.
        .aov = sceneConfig.initialAov,
        .channelView = 0,
        .userLut = sceneConfig.initialLut,
        .framingState = engine::debug::FramingOverlayState{},
        // "Show/Hide Background" HDRI-section checkbox -- off by default, preserving today's black background; only takes visible effect for the Beauty AOV (aov == 0), see renderFrame.
        .showSky = false,
        // HDR environment's Y-axis (world up) rotation, degrees [0,359] -- affects sky background, SH diffuse, and prefiltered specular together (see pbr.frag/sky.frag's uEnvRotationRadians), all rotated at query time rather than re-baked.
        .envRotationDegrees = 0,
        .orbitPickRequested = false,
        .lastCursorX = 0.0,
        .lastCursorY = 0.0,
        // task_info() is a real syscall; the HUD is read by human eyes, not per-frame logic, so re-sampling RAM 4x/sec instead of every frame drops one source of frame-time jitter for free.
        .ramBytes = engine::debug::residentSetBytes(),
        .systemAvailableBytes = engine::debug::availableSystemBytes(),
        // Fixed for the machine, unlike the other two -- queried once here rather than resampled alongside them.
        .systemTotalBytes = engine::debug::totalSystemBytes(),
        .lastRamSample = std::chrono::steady_clock::now(),
        .lastFrameTime = std::chrono::steady_clock::now(),
    };
}

// Debug-only: 'L' cycles the viewer LUT (sRGB -> Rec709 -> Raw -> sRGB -> ...), Raw being a genuine no-display-encode passthrough for direct encoded-vs-unencoded comparison. '1'/'2'/'3' toggle isolating a channel of the active AOV (pressing the active one again turns it back off) -- moved off R/G/B in Phase 3 so 'R' is free to reset the debug camera. 'K' toggles the centre-crosshair framing overlay. No general input-mapping system for these few keys is needed: WASD/QE turned out to need continuous per-frame state (Window::isKeyDown) rather than this edge-triggered callback, so this single slot still covers everything that's actually event-shaped.
//
// Wired up here, not inside initializeApp: every callback captures a reference into app, which must already be at its final, stable address (main()'s local, unwrapped from the optional initializeApp returned) -- capturing a reference during initializeApp would dangle the moment that AppResources is moved into its optional's storage.
void wireCallbacks(engine::platform::Window& window, AppResources& app) {
    window.setResizeCallback([&app](int width, int height) { app.hdrFbo.resize(width, height); });

    window.setKeyCallback([&app](int key, int action) {
        if (action != GLFW_PRESS) {
            return;
        }
        using Lut = engine::gfx::OcioDisplayTransform::Lut;
        if (key == GLFW_KEY_L) {
            app.userLut = app.userLut == Lut::SRGB     ? Lut::Rec709
                          : app.userLut == Lut::Rec709 ? Lut::Raw
                                                        : Lut::SRGB;
            std::cout << "OcioDisplayTransform: active LUT = " << lutName(app.userLut) << '\n';
        } else if (key == GLFW_KEY_1) {
            app.channelView = app.channelView == 1 ? 0 : 1;
        } else if (key == GLFW_KEY_2) {
            app.channelView = app.channelView == 2 ? 0 : 2;
        } else if (key == GLFW_KEY_3) {
            app.channelView = app.channelView == 3 ? 0 : 3;
        } else if (key == GLFW_KEY_R) {
            app.debugCamera.resetToDefault();
        } else if (key == GLFW_KEY_K) {
            app.framingState.crosshair = !app.framingState.crosshair;
        }
    });

    // LMB begins/ends an orbit -- gated on the HUD not wanting the click (dragging a HUD widget shouldn't also tumble the camera underneath it). The release always ends an in-progress orbit regardless of where the cursor ended up, so a drag that finishes over the HUD still releases cleanly.
    window.setMouseButtonCallback([&app, &window](int button, int action) {
        if (button != GLFW_MOUSE_BUTTON_LEFT) {
            return;
        }
        if (action == GLFW_PRESS) {
            if (!app.hud.wantsCaptureMouse()) {
                app.orbitPickRequested = true;
            }
        } else if (action == GLFW_RELEASE && app.debugCamera.isOrbiting()) {
            app.debugCamera.endOrbit();
            window.setCursorLocked(false);
        }
    });
}

engine::scene::Camera updateCamera(engine::platform::Window& window, AppResources& app,
                                    float dtSeconds) {
    if (app.debugCamera.isOrbiting()) {
        const auto [cursorX, cursorY] = window.cursorPosition();
        app.debugCamera.applyOrbitDelta(static_cast<float>(cursorX - app.lastCursorX),
                                         static_cast<float>(cursorY - app.lastCursorY));
        app.lastCursorX = cursorX;
        app.lastCursorY = cursorY;
    } else {
        app.debugCamera.applyFlyInput(window, dtSeconds);
    }
    return app.debugCamera.snapshot();
}

// Sky background: drawn with depth test/write both off, before the depth-tested geometry pass, so geometry simply overwrites it wherever it covers a pixel -- only takes visible effect for Beauty (aov == 0); every other AOV keeps its black background, since a sky colour has no valid value in those debug buffers (e.g. Alpha's coverage-mask convention).
void drawSky(AppResources& app, const glm::mat4& viewProjection, const glm::vec3& cameraPos) {
    if (!app.showSky || app.aov != 0) {
        return;
    }
    // Depth writes explicitly off, not just depth test: fullscreen_triangle.vert's fixed clip-space z (0.0) would otherwise land in the depth buffer ahead of real geometry.
    GL_CALL(glDepthMask(GL_FALSE));
    const glm::mat4 invViewProj = glm::inverse(viewProjection);
    app.skyShader.use();
    GL_CALL(glUniformMatrix4fv(app.uSkyInvViewProjLoc, 1, GL_FALSE, &invViewProj[0][0]));
    GL_CALL(glUniform3fv(app.uSkyCameraPosLoc, 1, &cameraPos[0]));
    GL_CALL(glUniform1f(app.uSkyEnvRotationRadiansLoc,
                        glm::radians(static_cast<float>(app.envRotationDegrees))));
    app.environmentTexture.bind(0);
    GL_CALL(glBindVertexArray(app.skyVao));
    GL_CALL(glDrawArrays(GL_TRIANGLES, 0, 3));
    GL_CALL(glDepthMask(GL_TRUE));
}

struct GeometryDrawStats {
    int instancesDrawn;
    int instancesCulled;
    long long trianglesDrawn;
};

GeometryDrawStats drawSceneGeometry(AppResources& app, const glm::mat4& viewProjection) {
    GeometryDrawStats stats{0, 0, 0};
    int instanceId = 0;
    for (const engine::scene::MeshInstance& instance : app.stumpModel.instances) {
        const auto& [worldMin, worldMax] =
            app.instanceWorldBounds[static_cast<std::size_t>(instanceId)];
        const bool visible =
            engine::scene::frustumIntersectsAabb(viewProjection, worldMin, worldMax);
        if (!visible) {
            ++stats.instancesCulled;
            ++instanceId;
            continue;
        }
        ++stats.instancesDrawn;
        stats.trianglesDrawn += instance.mesh.triangleCount();

        GL_CALL(glUniformMatrix4fv(app.uModelLoc, 1, GL_FALSE, &instance.transform[0][0]));
        const glm::mat3 normalMatrix = glm::inverseTranspose(glm::mat3(instance.transform));
        GL_CALL(glUniformMatrix3fv(app.uNormalMatrixLoc, 1, GL_FALSE, &normalMatrix[0][0]));
        const glm::vec3 baseColorFactor(instance.material.baseColorFactor);
        GL_CALL(glUniform3fv(app.uBaseColorFactorLoc, 1, &baseColorFactor[0]));
        GL_CALL(glUniform1f(app.uMetallicFactorLoc, instance.material.metallicFactor));
        GL_CALL(glUniform1f(app.uRoughnessFactorLoc, instance.material.roughnessFactor));
        GL_CALL(glUniform3fv(app.uBoundsMinLoc, 1, &worldMin[0]));
        GL_CALL(glUniform3fv(app.uBoundsMaxLoc, 1, &worldMax[0]));
        const glm::vec3 objectIdColor = falseColorForId(instanceId);
        GL_CALL(glUniform3fv(app.uObjectIdColorLoc, 1, &objectIdColor[0]));
        instance.material.baseColorTexture.bind(0);
        instance.material.roughnessTexture.bind(1);
        instance.material.aoTexture.bind(2);
        instance.material.normalTexture.bind(3);
        instance.material.bumpTexture.bind(4);
        instance.material.specularTexture.bind(5);
        instance.mesh.draw();
        ++instanceId;
    }
    return stats;
}

// Resolved here, not in the mouse callback: this is the first point after this frame's scene draw where hdrFbo's depth buffer holds this frame's contents (the callback fires during pollEvents(), before the draw, which would read last frame's).
void resolveOrbitPick(engine::platform::Window& window, AppResources& app,
                       const engine::scene::Camera& camera, const glm::mat4& viewProjection,
                       int winWidth, int winHeight) {
    if (!app.orbitPickRequested) {
        return;
    }
    app.orbitPickRequested = false;
    const float depth = app.hdrFbo.sampleDepth(winWidth / 2, winHeight / 2);
    glm::vec3 pivot(0.0F);
    if (depth < 0.9999F) {
        const glm::vec4 clip(0.0F, 0.0F, (2.0F * depth) - 1.0F, 1.0F);
        glm::vec4 world = glm::inverse(viewProjection) * clip;
        world /= world.w;
        pivot = glm::vec3(world);
    } else {
        pivot = camera.position() + (3.0F * camera.forward());
    }
    app.debugCamera.beginOrbit(pivot);
    window.setCursorLocked(true);
    const auto [cursorX, cursorY] = window.cursorPosition();
    app.lastCursorX = cursorX;
    app.lastCursorY = cursorY;
}

// Debug AOVs (aov != 0) are already display-oriented (most clamped to [0,1]; unclamped ones like Luminance just hard-clip to white past 1, same as Beauty already does in Raw mode), not scene-referred radiance -- force the Raw passthrough so the sRGB/Rec709 display curve doesn't distort them, restoring the user's chosen LUT for Beauty. Still set even for Sobel/Gabor below, which don't draw through ocioTransform at all, so the HUD's LUT-name readout stays accurate.
void presentFrame(AppResources& app, int winWidth, int winHeight) {
    app.ocioTransform.setActiveLut(app.aov == 0 ? app.userLut
                                                 : engine::gfx::OcioDisplayTransform::Lut::Raw);
    if (app.aov == 5 || app.aov == 6) {
        // Sobel/Gabor: hdrFbo's color texture holds the Luminance AOV (pbr.frag's aov==4/5/6 branch) -- run the edge-filter second pass over it instead of the OCIO display transform.
        app.edgeFilterShader.use();
        GL_CALL(glUniform1i(app.uFilterModeLoc, app.aov == 6 ? 1 : 0));
        app.postProcess.draw(app.hdrFbo.colorTexture(), app.edgeFilterShader,
                              {winWidth, winHeight});
    } else {
        app.ocioTransform.bind();
        app.postProcess.draw(app.hdrFbo.colorTexture(), app.ocioTransform.activeShader(),
                              {winWidth, winHeight});
    }
}

void updateHud(AppResources& app, const engine::scene::Camera& camera,
               const GeometryDrawStats& drawStats, int winWidth, int winHeight) {
    const engine::debug::SceneStats sceneStats{
        static_cast<int>(app.stumpModel.instances.size()),
        drawStats.instancesDrawn,
        drawStats.instancesCulled,
        app.totalTriangles,
        drawStats.trianglesDrawn,
        app.totalPoints,
        winWidth,
        winHeight,
    };
    const engine::debug::HudFrameData hudFrameData{
        app.gpuInfo,
        app.frameStats,
        app.geomTimer.millisecondsElapsed(),
        app.postTimer.millisecondsElapsed(),
        app.ramBytes,
        engine::debug::gpuAllocatedBytes(),
        app.systemAvailableBytes,
        app.systemTotalBytes,
        app.channelView,
        lutName(app.ocioTransform.activeLut()),
        sceneStats,
        camera,
        app.debugCamera.yawDegrees(),
        app.debugCamera.pitchDegrees(),
        app.debugCamera.isOrbiting(),
        app.histogram,
    };
    // Round-tripped through a local so the HUD's Lens slider can bind a plain float&, same as aov -- DebugCameraController is the authoritative owner, read before draw() and written back after.
    float focalLengthMm = app.debugCamera.focalLengthMm();
    app.hud.draw(hudFrameData, app.aov, focalLengthMm, app.showSky, app.envRotationDegrees,
                 app.framingState);
    app.debugCamera.setFocalLengthMm(focalLengthMm);
    app.hud.render();
}

// One frame: poll -> bind HDR FBO -> clear -> draw scene -> post-process blit (exposure + OCIO display transform) to the default framebuffer -> swap.
void renderFrame(engine::platform::Window& window, AppResources& app) {
    window.pollEvents();
    app.hud.beginFrame();
    app.frameStats.tick();

    const auto frameNow = std::chrono::steady_clock::now();
    const float dtSeconds = std::chrono::duration<float>(frameNow - app.lastFrameTime).count();
    app.lastFrameTime = frameNow;

    const engine::scene::Camera camera = updateCamera(window, app, dtSeconds);
    const auto [winWidth, winHeight] = window.framebufferSize();
    const float aspect = static_cast<float>(winWidth) / static_cast<float>(winHeight);
    const glm::mat4 view = camera.viewMatrix();
    const glm::mat4 projection = camera.projectionMatrix(aspect);
    const glm::mat4 viewProjection = projection * view;
    const glm::vec3 cameraPos = camera.position();

    app.geomTimer.begin();
    app.hdrFbo.bind();
    glClearColor(0.0F, 0.0F, 0.0F, 1.0F);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    drawSky(app, viewProjection, cameraPos);

    // Scoped to just this scene draw: the post-process pass below presents a fullscreen triangle to the default framebuffer at a fixed NDC z, so leaving depth test on for it would make correctness depend on whatever the driver leaves in that buffer's depth contents across frames, not on anything this code controls.
    glEnable(GL_DEPTH_TEST);
    app.sceneShader.use();
    GL_CALL(glUniformMatrix4fv(app.uViewLoc, 1, GL_FALSE, &view[0][0]));
    GL_CALL(glUniformMatrix4fv(app.uProjectionLoc, 1, GL_FALSE, &projection[0][0]));
    GL_CALL(glUniform3fv(app.uCameraPosLoc, 1, &cameraPos[0]));
    GL_CALL(glUniform1i(app.uAovLoc, app.aov));
    GL_CALL(glUniform1i(app.uChannelViewLoc, app.channelView));
    GL_CALL(glUniform1f(app.uEnvRotationRadiansLoc,
                        glm::radians(static_cast<float>(app.envRotationDegrees))));

    const GeometryDrawStats drawStats = drawSceneGeometry(app, viewProjection);
    glDisable(GL_DEPTH_TEST);
    app.geomTimer.end();

    resolveOrbitPick(window, app, camera, viewProjection, winWidth, winHeight);

    app.postTimer.begin();
    presentFrame(app, winWidth, winHeight);
    app.postTimer.end();

    // Captured after the composited image lands in the default framebuffer, before the HUD draws on top of it.
    app.histogram.update(winWidth, winHeight);

    const auto now = std::chrono::steady_clock::now();
    if (now - app.lastRamSample >= std::chrono::milliseconds(250)) {
        app.ramBytes = engine::debug::residentSetBytes();
        app.systemAvailableBytes = engine::debug::availableSystemBytes();
        app.lastRamSample = now;
    }

    updateHud(app, camera, drawStats, winWidth, winHeight);

    window.swapBuffers();
}

}  // namespace

int main() {
    glfwSetErrorCallback(&glfwErrorCallback);

    if (glfwInit() != GLFW_TRUE) {
        std::cerr << "main: glfwInit failed\n";
        return EXIT_FAILURE;
    }

    int exitCode = EXIT_SUCCESS;
    {
        // Config is pure file I/O with no GL dependency, but scene.json's window size must be known before Window is constructed, so it's loaded first, before any GLFW/GL object exists. Both files hard-fail identically on missing or malformed: this is user-editable input where a load failure is a real, expected-to-happen event, not an internal invariant, so it's surfaced immediately rather than defaulted around -- matching the shader/model/OCIO all-or-nothing gate inside initializeApp.
        std::optional<engine::config::SceneConfig> sceneConfig =
            engine::config::loadSceneConfig(ASSET_ROOT_DIR "/config/scene.json");
        std::optional<engine::config::ProfileConfig> profileConfig =
            engine::config::loadProfileConfig(ASSET_ROOT_DIR "/config/profile.json");

        if (!sceneConfig || !profileConfig) {
            std::cerr << "main: scene/profile config load failed, aborting startup\n";
            exitCode = EXIT_FAILURE;
        } else {
            // Window construction creates the GL 4.1 core/fwd-compat context and makes it current; fatal failure inside it exits the process directly (see window.cpp) since nothing recoverable exists yet.
            engine::platform::Window window(sceneConfig->windowWidth, sceneConfig->windowHeight,
                                             "ENGINE");

            glewExperimental = GL_TRUE;
            const GLenum glewStatus = glewInit();
            // GLEW's init is known to leave a spurious error even on success; drain it here so it's never misattributed to a later GL_CALL.
            while (glGetError() != GL_NO_ERROR) {
            }

            if (glewStatus != GLEW_OK) {
                std::cerr << "main: glewInit failed: "
                          << reinterpret_cast<const char*>(glewGetErrorString(glewStatus)) << '\n';
                exitCode = EXIT_FAILURE;
            } else {
                // With heavy scene content, an uncapped CPU submits draw calls faster than the GPU can drain them, growing the driver's command queue unboundedly. Keep vsync on; disable it only for a deliberate, short-lived uncapped-FPS measurement.
                glfwSwapInterval(1);

                std::optional<AppResources> app =
                    initializeApp(*sceneConfig, *profileConfig, window);
                if (!app) {
                    exitCode = EXIT_FAILURE;
                } else {
                    wireCallbacks(window, *app);

                    while (!window.shouldClose()) {
                        renderFrame(window, *app);
                    }

                    // skyVao has no RAII wrapper (see AppResources) -- free it explicitly at the same scope exit point its lifetime is tied to.
                    GL_CALL(glDeleteVertexArrays(1, &app->skyVao));
                }
            }
        }
    }  // Window destroyed here, while GLFW is still initialized.

    glfwTerminate();
    return exitCode;
}
