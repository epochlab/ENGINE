#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>

// GLEW before GLFW: see gl_debug.cpp for why.
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <OpenColorIO/OpenColorIO.h>

#include "engine/config/profile_config.h"
#include "engine/config/scene_config.h"
#include "engine/debug/aov.h"
#include "engine/debug/colormap.h"
#include "engine/debug/frame_stats.h"
#include "engine/debug/gpu_timer.h"
#include "engine/debug/histogram.h"
#include "engine/debug/hud_overlay.h"
#include "engine/debug/memory_tracker.h"
#include "engine/debug/scene_stats.h"
#include "engine/debug/system_info.h"
#include "engine/gfx/gl_debug.h"
#include "engine/gfx/hdr_image.h"
#include "engine/gfx/ocio_display_transform.h"
#include "engine/gfx/post_process_pass.h"
#include "engine/gfx/shader_program.h"
#include "engine/gfx/texture.h"
#include "engine/platform/window.h"
#include "engine/scene/camera.h"
#include "engine/scene/debug_camera_controller.h"
#include "engine/scene/embree_accel.h"
#include "engine/scene/environment_map.h"
#include "engine/scene/false_color.h"
#include "engine/scene/gltf_loader.h"
#include "engine/scene/material_binding.h"
#include "engine/scene/path_trace_driver.h"
#include "engine/scene/path_tracer.h"
#include "engine/scene/rasterizer.h"
#include "engine/scene/thread_pool.h"

namespace OCIO = OCIO_NAMESPACE;

namespace {

void glfwErrorCallback(int error, const char* description) {
    std::cerr << "GLFW error " << error << ": " << description << '\n';
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

// True for AOVs needing light-transport data (Beauty, transport-component AOVs, post-filter AOVs reading Beauty) -- false for the 15 primary-hit-only AOVs the rasterizer covers (rasterizer.h). Avoids restarting PathTraceDriver's accumulation for an AOV that will never show it.
bool aovNeedsLightTransport(engine::debug::AovId aov) {
    using engine::debug::AovId;
    switch (aov) {
        case AovId::Beauty:
        case AovId::HSV:
        case AovId::Luminance:
        case AovId::Sobel:
        case AovId::Gabor:
        case AovId::BounceCount:
        case AovId::Shadow:
        case AovId::DirectDiffuse:
        case AovId::IndirectDiffuse:
        case AovId::DirectSpecular:
        case AovId::IndirectSpecular:
        case AovId::Refraction:
            return true;
        default:
            return false;
    }
}

// Snapshot of every input renderPathTraced's result actually depends on except the resolution it renders at -- compared frame to frame (see requestPathTraceIfTriggerChanged) to decide whether to hand PathTraceDriver a fresh request. envRotationDegrees defaults to the sentinel -1 (never a real value, since the HUD clamps it to [0,359]) specifically so the very first comparison always mismatches, giving the path-traced view a live result from the first rendered frame with no separate startup-trace call needed. needsLightTransport is folded in (not just checked ad hoc) so switching the AOV dropdown into a light-transport AOV registers as a trigger change even with a static camera -- otherwise Beauty would show a stale result until the next camera move.
struct PathTraceInputState {
    glm::vec3 cameraPosition{0.0F};
    float cameraYawDegrees = 0.0F;
    float cameraPitchDegrees = 0.0F;
    float focalLengthMm = 0.0F;
    int envRotationDegrees = -1;
    bool showSky = false;
    float envExposureStops = 0.0F;
    int fbWidth = 0;
    int fbHeight = 0;
    bool needsLightTransport = true;
    // The AOV itself, not just needsLightTransport: switching Normal -> Albedo moves between two rasterizer-backed AOVs, changing neither the camera nor needsLightTransport, and must still re-run the rasterizer now that it no longer runs unconditionally.
    int aov = -1;

    bool operator==(const PathTraceInputState&) const = default;
};

// The inputs plus the scale they are currently being rendered at. Split in two because renderScale is *derived* from whether the inputs changed (requestPathTraceIfTriggerChanged): folding it into one struct would make the settle-time promotion to full resolution read as fresh interaction on the next frame, re-arming the timer it just satisfied and pinning the renderer at the interactive scale forever. The outer comparison is still what dispatches, so promoting the scale re-requests through the existing generation mechanism with no separate path.
struct PathTraceTriggerState {
    PathTraceInputState input;
    float renderScale = 0.0F;  // sentinel, never a real value: profile_config.h bounds it to (0,1]

    bool operator==(const PathTraceTriggerState&) const = default;
};

// Seconds of no input change before the renderer promotes itself back to full renderScale -- long enough that the momentary gaps between mouse-drag events during an orbit do not each trigger a full-resolution restart, short enough to feel immediate when the camera actually stops.
constexpr double kInteractiveSettleSeconds = 0.25;

// max(1) so a non-empty framebuffer never scales to a zero-pixel render target; a genuinely empty one (minimized window) stays 0 and is skipped by the caller's own guard, exactly as before.
int scaledExtent(int framebufferExtent, float scale) {
    if (framebufferExtent <= 0) {
        return 0;
    }
    return std::max(1, static_cast<int>(std::lround(static_cast<float>(framebufferExtent) * scale)));
}

// Everything the render loop touches every frame, plus the one-time-computed state (cached uniform locations, Embree scene) that must stay alive for the run's duration. A pure aggregate (no user-declared constructors) so initializeApp can return it by value via designated initializers -- each RAII member's own move constructor (already verified elsewhere to correctly transfer GL handles/tracked byte counts) handles the actual transfer.
struct AppResources {
    engine::gfx::ShaderProgram edgeFilterShader;
    engine::gfx::ShaderProgram hsvDisplayShader;
    engine::gfx::OcioDisplayTransform ocioTransform;
    engine::scene::EmbreeAccel sceneAccel;     // path tracer scene intersection
    // stumpModel.shadingTriangles indexes sceneAccel's triangles 1:1, no separate field needed.
    engine::scene::EnvironmentMap environmentMap;
    engine::scene::LoadedModel stumpModel;
    int totalTriangles;
    int totalPoints;

    engine::gfx::PostProcessPass postProcess;
    engine::debug::HudOverlay hud;
    engine::debug::FrameStats frameStats;
    engine::debug::GpuTimer postTimer;
    engine::debug::Histogram histogram;
    // Companion to histogram, computed separately (see updateOverRangeStats) since Histogram bins the post-display-transform, post-8-bit-clamp framebuffer and cannot tell 1.01 from 100.0 -- both saturate bin 255 identically. Gated at the same capture interval, not scanned every frame.
    int overRangeFrameCounter = 0;
    float overRangeFraction = 0.0F;
    float overRangePeakMultiple = 0.0F;
    engine::scene::DebugCameraController debugCamera;
    engine::debug::GpuInfo gpuInfo;

    int uFilterModeLoc;
    int uEdgeChannelViewLoc;
    int uEdgeExposureLoc;
    int uHsvChannelViewLoc;
    int uHsvExposureLoc;
    int uEdgeInvertLoc;
    int uHsvInvertLoc;

    // HUD-editable UI/run state.
    int aov;
    int channelView;
    engine::gfx::OcioDisplayTransform::Lut userLut;
    engine::debug::FramingOverlayState framingState;
    bool showSky;
    int envRotationDegrees;
    float envExposureStops;  // stops, not a multiplier; requestPathTrace does exp2()
    bool invert;   // 1.0 - colour, applied to the final display-referred image -- the 'I' debug toggle
    bool showHud;  // 'H' toggle; gates HudOverlay::draw only -- beginFrame/render stay unconditional so ImGui's frame pairing is never broken
    // Chromatic aberration strength (0 = off), radial UV offset passed to OcioDisplayTransform::setAberration -- HUD slider only.
    float aberrationStrength;

    // Async path-traced view, selected via the `aov` field (engine::debug::AovId, the HUD's AOV dropdown). pathTraceDriver runs continuously on its own background thread once constructed (main() constructs it after initializeApp() returns -- see path_trace_driver.h's constructor precondition on reference stability); requestTrace() is called only from renderFrame's requestPathTraceIfTriggerChanged, whenever lastPathTraceTrigger detects the camera/scene state renderPathTraced depends on has changed -- no manual trigger. unique_ptr, not a by-value optional: PathTraceDriver holds reference members and an owned std::jthread/std::mutex, so it's neither copyable nor movable -- a by-value optional<T> member would make that non-movability propagate to AppResources itself (optional<T>'s move ctor is only available when T's is), which would break initializeApp's return-by-value/RVO pattern every other member here relies on. A unique_ptr's own move just transfers ownership of the pointee's address, never touching PathTraceDriver's reference members, so AppResources stays movable and PathTraceDriver itself is never relocated in memory once constructed.
    engine::scene::PathTraceSettings pathTraceSettings;
    // Per-instance material fields, parallel-indexed with stumpModel.instances (ShadingTriangle::instanceIndex resolves into this) -- pathTraceSettings's 11 material fields copied per instance, overridden from sceneConfig.materialOverrides by MeshInstance::name where present. Renderer-only fields (samplesPerPixel/maxBounces/RR) are never read from these entries; see resolveBsdfParams/buildShadingFrame call sites in path_tracer.cpp/rasterizer.cpp.
    std::vector<engine::scene::PathTraceSettings> perInstanceSettings;
    int maxSamples;  // accumulated-pass cap for PathTraceDriver; 0 = unbounded
    std::unique_ptr<engine::scene::PathTraceDriver> pathTraceDriver;
    std::optional<engine::gfx::Texture> pathTraceDisplayTexture;
    int pathTraceDisplayedAov;  // which AovId pathTraceDisplayTexture currently holds, -1 = none yet
    // Max raw Depth value seen in the last rebuilt pathTraceDisplayTexture -- see ensurePathTraceDisplayTexture; only meaningful/updated when aov==Depth.
    float pathTraceDisplayedDepthMax;
    std::uint64_t pathTraceDisplayedGeneration;  // which RasterGBuffer generation the texture holds; 0 when it was built from a PathTraceResult instead
    // Strong ref (kept alive, not just an identity pointer) to whichever published object -- PathTraceResult or RasterGBuffer -- pathTraceDisplayTexture currently reflects; see ensurePathTraceDisplayTexture.
    std::shared_ptr<const void> pathTraceDisplayedOwner;
    PathTraceTriggerState lastPathTraceTrigger;  // sentinel-initialized, see its own doc comment
    // Render resolution as a fraction of the framebuffer: renderScale once settled, interactiveRenderScale while any input is changing (profile_config.h). lastInputChange is the timer the promotion between them is measured against.
    float renderScale;
    float interactiveRenderScale;
    std::chrono::steady_clock::time_point lastInputChange;

    // Synchronous per-frame CPU rasterizer for the 15 primary-hit-only G-buffer AOVs (rasterizer.h) -- their only producer, decoupled from PathTraceDriver's async convergence loop. unique_ptr for the same reason as pathTraceDriver: ThreadPool's copy/move are deleted (owns worker threads), so a by-value member would break AppResources's movability.
    std::unique_ptr<engine::scene::ThreadPool> rasterThreadPool;
    // Allocated once and rendered into in place (rasterizer.h), never republished -- its `generation` field, not its address, is what tells one render from the next. Refreshed synchronously in requestPathTraceIfTriggerChanged whenever a rasterizer-backed AOV is selected and an input changed; generation stays 0 while only light-transport AOVs are ever shown, because then it never runs at all.
    std::shared_ptr<engine::scene::RasterGBuffer> rasterGBuffer;

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
    engine::gfx::ShaderProgram edgeFilterShader;
    engine::gfx::ShaderProgram hsvDisplayShader;
    engine::gfx::OcioDisplayTransform ocioTransform;
};

// All shader/OCIO loading in one place so initializeApp has one all-or-nothing check, matching how it already treats model/environment loading as a single startup gate.
std::optional<RequiredShaders> loadShaders() {
    // HSV/Sobel/Gabor AOV display passes -- see hsv_display.frag/edge_filter.frag; both run over the path tracer's Beauty image via the shared fullscreen-triangle post-process pass.
    std::optional<engine::gfx::ShaderProgram> edgeFilterShader =
        engine::gfx::ShaderProgram::loadFromFiles(ASSET_ROOT_DIR "/shaders/fullscreen_triangle.vert",
                                                   ASSET_ROOT_DIR "/shaders/edge_filter.frag");
    std::optional<engine::gfx::ShaderProgram> hsvDisplayShader = engine::gfx::ShaderProgram::loadFromFiles(
        ASSET_ROOT_DIR "/shaders/fullscreen_triangle.vert", ASSET_ROOT_DIR "/shaders/hsv_display.frag");
    std::optional<engine::gfx::OcioDisplayTransform> ocioTransform =
        engine::gfx::OcioDisplayTransform::create();

    if (!edgeFilterShader || !hsvDisplayShader || !ocioTransform) {
        return std::nullopt;
    }
    return RequiredShaders{
        std::move(*edgeFilterShader),
        std::move(*hsvDisplayShader),
        std::move(*ocioTransform),
    };
}

// The runtime-changing edge-filter uniforms, cached once rather than re-queried per frame.
struct EdgeFilterUniforms {
    int filterMode;
    int channelView;
    int exposure;
    int invert;
};

// Sobel/Gabor's second pass (see edge_filter.frag): uHdrColor's texture unit and the Gabor kernel weights are both fixed for the whole run, set once here.
EdgeFilterUniforms setupEdgeFilterShader(const engine::gfx::ShaderProgram& edgeFilterShader) {
    edgeFilterShader.use();
    GL_CALL(glUniform1i(edgeFilterShader.uniformLocation("uHdrColor"), 0));
    const std::array<float, 100> gaborKernel = buildGaborKernel();
    GL_CALL(glUniform1fv(edgeFilterShader.uniformLocation("uGaborKernel"), 100, gaborKernel.data()));
    return EdgeFilterUniforms{edgeFilterShader.uniformLocation("uFilterMode"),
                               edgeFilterShader.uniformLocation("uChannelView"),
                               edgeFilterShader.uniformLocation("uExposure"),
                               edgeFilterShader.uniformLocation("uInvert")};
}

// The runtime-changing hsv-display uniforms, cached once rather than re-queried per frame.
struct HsvDisplayUniforms {
    int channelView;
    int exposure;
    int invert;
};

// hsv_display.frag's uHdrColor texture unit is fixed for the whole run, same convention as setupEdgeFilterShader above.
HsvDisplayUniforms setupHsvDisplayShader(const engine::gfx::ShaderProgram& hsvDisplayShader) {
    hsvDisplayShader.use();
    GL_CALL(glUniform1i(hsvDisplayShader.uniformLocation("uHdrColor"), 0));
    return HsvDisplayUniforms{hsvDisplayShader.uniformLocation("uChannelView"),
                               hsvDisplayShader.uniformLocation("uExposure"),
                               hsvDisplayShader.uniformLocation("uInvert")};
}

// All one-time startup work: camera/model/shader/environment loading (nullopt on any failure -- matches the shader/model/OCIO all-or-nothing gate this replaces), Embree scene build, and cached uniform-location lookups for the shared edge-filter/HSV shaders. Doesn't wire input callbacks -- those capture a stable AppResources& and must be set up by the caller only after this returns (see main()), since a callback capturing a reference into an AppResources that's still about to be moved into its final std::optional storage would dangle.
std::optional<AppResources> initializeApp(const engine::config::SceneConfig& sceneConfig,
                                           const engine::config::ProfileConfig& profileConfig,
                                           engine::platform::Window& window) {
    std::cout << "GL_KHR_debug available: " << std::boolalpha << engine::gfx::khrDebugAvailable()
              << '\n';
    std::cout << "GL_ARB_timer_query available: " << std::boolalpha
              << engine::debug::gpuTimerQueryAvailable() << '\n';
    const engine::debug::GpuInfo gpuInfo = engine::debug::queryGpuInfo();

    // ev100() is logged but not render-path-consumed directly: DebugCameraController::relativeExposureEv() derives the display-stage multiplier from it (OcioDisplayTransform), not this log line.
    engine::scene::DebugCameraController debugCamera(
        profileConfig.camera.position, profileConfig.camera.yawDegrees,
        profileConfig.camera.pitchDegrees, profileConfig.camera.filmBack,
        profileConfig.camera.focalLengthMm, profileConfig.camera.nearClip,
        profileConfig.camera.farClip, profileConfig.camera.aperture,
        profileConfig.camera.shutterSeconds, profileConfig.camera.iso,
        profileConfig.controls.flySpeedMetersPerSecond,
        profileConfig.controls.orbitSensitivityDegPerPixel);
    {
        const engine::scene::Camera initialCamera = debugCamera.snapshot();
        const glm::vec3 camPos = initialCamera.position();
        std::cout << "Camera: position=(" << camPos.x << ", " << camPos.y << ", " << camPos.z
                  << ") verticalFov=" << glm::degrees(initialCamera.verticalFovRadians())
                  << " deg ev100=" << initialCamera.ev100() << '\n';
    }

    // Scene-level placement (scene.json model.position/model.rotation), order X,Y,Z.
    const glm::mat4 sceneTransform =
        glm::translate(glm::mat4(1.0F), sceneConfig.model.position) *
        glm::rotate(glm::mat4(1.0F), glm::radians(sceneConfig.model.rotation.z), glm::vec3(0.0F, 0.0F, 1.0F)) *
        glm::rotate(glm::mat4(1.0F), glm::radians(sceneConfig.model.rotation.y), glm::vec3(0.0F, 1.0F, 0.0F)) *
        glm::rotate(glm::mat4(1.0F), glm::radians(sceneConfig.model.rotation.x), glm::vec3(1.0F, 0.0F, 0.0F));

    const auto loadStart = std::chrono::steady_clock::now();
    std::optional<engine::scene::LoadedModel> stumpModel = engine::scene::loadGltf(
        std::string(ASSET_ROOT_DIR) + "/" + sceneConfig.model.gltfPath, sceneTransform,
        std::string(ASSET_ROOT_DIR) + "/" + sceneConfig.model.texturePath);
    const double loadMs =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - loadStart)
            .count();
    int totalTriangles = 0;
    if (stumpModel) {
        totalTriangles = static_cast<int>(stumpModel->worldTriangles.size());
        std::cout << "loadGltf: " << stumpModel->instances.size() << " instance(s), "
                  << totalTriangles << " triangles, " << loadMs << " ms\n"
                  << std::flush;
    }
    // "Points": total vertex-index count, i.e. 3 per triangle -- derived rather than tracked separately.
    const int totalPoints = totalTriangles * 3;

    std::optional<RequiredShaders> shaders = loadShaders();
    // Decoded once here (not via a texture-upload helper): the path tracer is the only consumer, sampling this CPU HdrImage directly, with no GPU upload step in between.
    std::optional<engine::gfx::HdrImage> environmentImage =
        engine::gfx::loadExr(std::string(ASSET_ROOT_DIR) + "/" + sceneConfig.environment.hdriPath);
    std::optional<engine::config::MaterialConfig> materialConfig = engine::config::loadMaterialConfig(
        std::string(ASSET_ROOT_DIR) + "/" + sceneConfig.materialPath);

    if (!shaders || !stumpModel || !environmentImage || !materialConfig) {
        std::cerr << "main: shader compile/link, model load, environment map load, or material "
                     "load failed, aborting startup\n";
        return std::nullopt;
    }

    engine::gfx::PostProcessPass postProcess;
    engine::debug::HudOverlay hud(window.nativeHandle());
    engine::debug::FrameStats frameStats;
    engine::debug::GpuTimer postTimer;
    engine::debug::Histogram histogram;

    engine::scene::EnvironmentMap environmentMap(std::move(*environmentImage));

    const auto accelBuildStart = std::chrono::steady_clock::now();
    std::optional<engine::scene::EmbreeAccel> sceneAccel =
        engine::scene::EmbreeAccel::build(std::move(stumpModel->worldTriangles));
    if (!sceneAccel) {
        std::cerr << "main: Embree scene build failed, aborting startup\n";
        return std::nullopt;
    }
    const double accelBuildMs =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                    accelBuildStart)
            .count();
    std::cout << "EmbreeAccel::build: " << sceneAccel->triangleCount() << " triangles, "
              << accelBuildMs << " ms\n"
              << std::flush;

    const EdgeFilterUniforms edgeFilterUniforms = setupEdgeFilterShader(shaders->edgeFilterShader);
    const HsvDisplayUniforms hsvUniforms = setupHsvDisplayShader(shaders->hsvDisplayShader);

    const engine::scene::PathTraceSettings basePathTraceSettings{
        .samplesPerPixel = profileConfig.pathTracer.samplesPerPixel,
        .maxBounces = profileConfig.pathTracer.maxBounces,
        .russianRouletteStartBounce = profileConfig.pathTracer.russianRouletteStartBounce,
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

    std::optional<std::vector<engine::scene::PathTraceSettings>> perInstanceSettings =
        engine::scene::resolvePerInstanceSettings(basePathTraceSettings, stumpModel->instances,
                                                   sceneConfig.materialOverrides, ASSET_ROOT_DIR);
    if (!perInstanceSettings) {
        std::cerr << "main: material override resolution failed, aborting startup\n";
        return std::nullopt;
    }

    return AppResources{
        .edgeFilterShader = std::move(shaders->edgeFilterShader),
        .hsvDisplayShader = std::move(shaders->hsvDisplayShader),
        .ocioTransform = std::move(shaders->ocioTransform),
        .sceneAccel = std::move(*sceneAccel),
        .environmentMap = std::move(environmentMap),
        .stumpModel = std::move(*stumpModel),
        .totalTriangles = totalTriangles,
        .totalPoints = totalPoints,
        .postProcess = std::move(postProcess),
        .hud = std::move(hud),
        .frameStats = std::move(frameStats),
        .postTimer = std::move(postTimer),
        .histogram = std::move(histogram),
        .debugCamera = std::move(debugCamera),
        .gpuInfo = gpuInfo,
        .uFilterModeLoc = edgeFilterUniforms.filterMode,
        .uEdgeChannelViewLoc = edgeFilterUniforms.channelView,
        .uEdgeExposureLoc = edgeFilterUniforms.exposure,
        .uHsvChannelViewLoc = hsvUniforms.channelView,
        .uHsvExposureLoc = hsvUniforms.exposure,
        .uEdgeInvertLoc = edgeFilterUniforms.invert,
        .uHsvInvertLoc = hsvUniforms.invert,
        // aov selects which AOV the path tracer's snapshot supplies (see selectPathTracedImage); channelView isolates one R/G/B channel of whatever aov currently shows. userLut is the LUT 'L' cycles through -- kept separate from OcioDisplayTransform's active LUT because non-Beauty AOVs force Raw (see the LUT-select comment in presentFrame) and must not clobber the user's actual choice. Both aov and userLut start from profile.json rather than a fixed literal.
        .aov = profileConfig.render.defaultAov,
        .channelView = 0,
        .userLut = profileConfig.render.defaultLut,
        .framingState = engine::debug::FramingOverlayState{},
        // "Show/Hide Background" HDRI-section checkbox -- off by default; only takes visible effect for the Beauty AOV, see presentFrame.
        .showSky = false,
        // HDR environment's Y-axis (world up) rotation, degrees [0,359] -- affects both the background and the environment's contribution to lighting (see environment_map.h), rotated at query time rather than re-baked.
        .envRotationDegrees = 0,
        // HDRI Exposure slider, stops. requestPathTrace does exp2() -> path_tracer.cpp miss-ray sampleDirection.
        .envExposureStops = 0.0F,
        .invert = false,
        .showHud = true,
        .aberrationStrength = 0.0F,
        .pathTraceSettings = basePathTraceSettings,
        .perInstanceSettings = std::move(*perInstanceSettings),
        .maxSamples = profileConfig.pathTracer.maxSamples,
        // Constructed in main() right after initializeApp() returns -- see path_trace_driver.h's constructor precondition (its reference members must bind to sceneAccel/environmentMap/stumpModel at their final, permanent address, which this designated-initializer expression, still local-variable-based and one AppResources move away from that address, cannot yet guarantee).
        .pathTraceDriver = nullptr,
        .pathTraceDisplayTexture = std::nullopt,
        .pathTraceDisplayedAov = -1,
        .pathTraceDisplayedDepthMax = 0.0F,
        .pathTraceDisplayedGeneration = 0,
        .pathTraceDisplayedOwner = nullptr,
        .lastPathTraceTrigger = PathTraceTriggerState{},
        .renderScale = profileConfig.render.renderScale,
        .interactiveRenderScale = profileConfig.render.interactiveRenderScale,
        .lastInputChange = std::chrono::steady_clock::time_point{},
        .rasterThreadPool = std::make_unique<engine::scene::ThreadPool>(),
        .rasterGBuffer = std::make_shared<engine::scene::RasterGBuffer>(),
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

// Debug-only: 'L' cycles the viewer LUT (sRGB -> Rec709 -> Raw -> sRGB -> ...), Raw being a genuine no-display-encode passthrough for direct encoded-vs-unencoded comparison. 'R'/'G'/'B' toggle isolating a channel of the active AOV (pressing the active one again turns it back off) -- reset moved to '0' to free these back up. 'I' inverts the final display-referred colour. 'H' toggles the HUD. 'ESC' quits. No general input-mapping system for these few keys is needed: WASD/QE need continuous per-frame state (Window::isKeyDown) rather than this edge-triggered callback, so this single slot still covers everything that's actually event-shaped. Wired up here, not inside initializeApp: every callback captures a reference into app, which must already be at its final, stable address (main()'s local, unwrapped from the optional initializeApp returned) -- capturing a reference during initializeApp would dangle the moment that AppResources is moved into its optional's storage.
void wireCallbacks(engine::platform::Window& window, AppResources& app) {
    window.setKeyCallback([&app, &window](int key, int action) {
        if (action != GLFW_PRESS) {
            return;
        }
        using Lut = engine::gfx::OcioDisplayTransform::Lut;
        if (key == GLFW_KEY_L) {
            app.userLut = app.userLut == Lut::SRGB     ? Lut::Rec709
                          : app.userLut == Lut::Rec709 ? Lut::Raw
                                                        : Lut::SRGB;
            std::cout << "OcioDisplayTransform: active LUT = " << lutName(app.userLut) << '\n';
        } else if (key == GLFW_KEY_R) {
            app.channelView = app.channelView == 1 ? 0 : 1;
        } else if (key == GLFW_KEY_G) {
            app.channelView = app.channelView == 2 ? 0 : 2;
        } else if (key == GLFW_KEY_B) {
            app.channelView = app.channelView == 3 ? 0 : 3;
        } else if (key == GLFW_KEY_0) {
            app.debugCamera.resetToDefault();
        } else if (key == GLFW_KEY_I) {
            app.invert = !app.invert;
        } else if (key == GLFW_KEY_H) {
            app.showHud = !app.showHud;
        } else if (key == GLFW_KEY_ESCAPE) {
            window.setShouldClose(true);
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

// Direct texel read, no bilinear -- gbuffer AOVs are per-pixel snapshots.
glm::vec3 sampleTexel(const engine::gfx::HdrImage& image, int x, int y) {
    const std::size_t idx = ((static_cast<std::size_t>(y) * static_cast<std::size_t>(image.width)) +
                              static_cast<std::size_t>(x)) *
                             4;
    return {image.rgba[idx + 0], image.rgba[idx + 1], image.rgba[idx + 2]};
}

// Cached CPU OCIO processors for the Beauty pixel probe -- built once, on first use, not per probe update: OCIO::Config::CreateFromBuiltinConfig/getProcessor parse the builtin config's colorspace graph, the same one-time cost OcioDisplayTransform::create() already pays for the GPU shaders at startup. Same four constants (ocio_display_transform.h) as that GPU build and as tools/render_beauty.cpp's CPU reference encode, so this reproduces the exact viewer transform rather than a second definition of it.
const OCIO::ConstCPUProcessorRcPtr& ocioCpuProcessor(engine::gfx::OcioDisplayTransform::Lut lut) {
    static const OCIO::ConstConfigRcPtr config =
        OCIO::Config::CreateFromBuiltinConfig(engine::gfx::kOcioConfigName);
    static const OCIO::ConstCPUProcessorRcPtr srgbProcessor =
        config
            ->getProcessor(engine::gfx::kOcioSceneColorSpace, engine::gfx::kOcioSrgbDisplay,
                            engine::gfx::kOcioView, OCIO::TRANSFORM_DIR_FORWARD)
            ->getDefaultCPUProcessor();
    static const OCIO::ConstCPUProcessorRcPtr rec709Processor =
        config
            ->getProcessor(engine::gfx::kOcioSceneColorSpace, engine::gfx::kOcioRec709Display,
                            engine::gfx::kOcioView, OCIO::TRANSFORM_DIR_FORWARD)
            ->getDefaultCPUProcessor();
    return lut == engine::gfx::OcioDisplayTransform::Lut::Rec709 ? rec709Processor : srgbProcessor;
}

// Beauty pixel probe's display transform, evaluated for one texel on the CPU instead of the whole framebuffer on the GPU: channel isolation -> exposure -> OCIO display curve (skipped in Raw mode, matching buildRawFragmentSource's passthrough) -> invert -- the same order OcioDisplayTransform's fragment shaders run (ocio_display_transform.cpp), minus the aberration UV-shift and dither, which are display artifacts rather than scene data and are intentionally not reproduced here. No clamp, no 8-bit quantize -- the point is a ground-truth readout of what's on screen, not a copy of its framebuffer precision loss.
glm::vec3 applyBeautyDisplayTransform(glm::vec3 hdrColor, const AppResources& app) {
    if (app.channelView == 1) {
        hdrColor = glm::vec3(hdrColor.r);
    } else if (app.channelView == 2) {
        hdrColor = glm::vec3(hdrColor.g);
    } else if (app.channelView == 3) {
        hdrColor = glm::vec3(hdrColor.b);
    }
    glm::vec3 displayColor = hdrColor * std::pow(2.0F, app.debugCamera.relativeExposureEv());
    if (app.userLut != engine::gfx::OcioDisplayTransform::Lut::Raw) {
        std::array<float, 3> pixel{displayColor.r, displayColor.g, displayColor.b};
        ocioCpuProcessor(app.userLut)->applyRGB(pixel.data());
        displayColor = {pixel[0], pixel[1], pixel[2]};
    }
    if (app.invert) {
        displayColor = glm::vec3(1.0F) - displayColor;
    }
    return displayColor;
}

// Orbit pivot picked with a single Embree ray down the view centre. Previously this read worldPos/alpha at the centre texel of the rasterizer's G-buffer, which is what forced that rasterization to run unconditionally -- two million pixels shaded to read one. A ray is also the more accurate instrument: no fill rule, no z-precision, no dependence on the resolution the G-buffer happened to be rendered at. Falls back to a fixed forward-offset pivot when the centre ray misses all geometry.
void resolveOrbitPick(engine::platform::Window& window, AppResources& app,
                       const engine::scene::Camera& camera) {
    if (!app.orbitPickRequested) {
        return;
    }
    app.orbitPickRequested = false;

    // primaryRay at ndc (0,0) reduces to exactly the camera's forward axis -- both ndc terms vanish -- so the centre ray needs no aspect ratio and no projection.
    const engine::scene::Ray ray{camera.position(), camera.forward(), camera.nearClip(),
                                  camera.farClip()};
    const std::optional<engine::scene::Hit> hit = app.sceneAccel.intersect(ray);
    const glm::vec3 pivot =
        hit ? ray.origin + (hit->t * ray.dir) : camera.position() + (3.0F * camera.forward());

    app.debugCamera.beginOrbit(pivot);
    window.setCursorLocked(true);
    const auto [cursorX, cursorY] = window.cursorPosition();
    app.lastCursorX = cursorX;
    app.lastCursorY = cursorY;
}

// Bundles the HdrImage a given AOV should display with a type-erased strong ref to whichever published object -- the driver's PathTraceResult or the synchronous RasterGBuffer -- actually owns it. That ref both keeps the owning object alive and gives ensurePathTraceDisplayTexture an ABA-safe cache-key identity: a raw pointer to it could, in principle, be freed and have a later unrelated shared_ptr allocation reuse the same address; holding a real shared_ptr can't.
struct PathTracedAovSource {
    const engine::gfx::HdrImage* image = nullptr;
    std::shared_ptr<const void> owner;
    // RasterGBuffer's render counter for the 15 rasterizer-backed AOVs, 0 for the path-traced ones. The rasterizer's buffer is now reused in place, so its address is constant and `owner` alone can no longer tell one render from the next; a PathTraceResult is still a fresh object per pass and needs no counter.
    std::uint64_t generation = 0;
};

// Returns a default (null image) if the specific source an AOV needs hasn't published yet -- callers show black instead. The 15 primary-hit-only AOVs read rasterGBuffer (refreshed synchronously whenever one of them is selected, requestPathTraceIfTriggerChanged); Beauty and the light-transport AOVs read the driver's asynchronously published PathTraceResult. Extended as RasterGBuffer/PathTraceResult grow more buffers.
PathTracedAovSource selectPathTracedImage(
    const std::shared_ptr<const engine::scene::PathTraceResult>& snapshot,
    const std::shared_ptr<engine::scene::RasterGBuffer>& rasterGBuffer,
    engine::debug::AovId aov) {
    // One construction site for all 15 rasterizer-backed AOVs, so the generation stamp cannot be omitted at one of them. The buffer is allocated for the process's life now, so a null check no longer distinguishes "no render yet" -- generation 0 does.
    const auto fromRaster = [&rasterGBuffer](const engine::gfx::HdrImage& image) {
        return rasterGBuffer->generation == 0
                   ? PathTracedAovSource{}
                   : PathTracedAovSource{&image, rasterGBuffer, rasterGBuffer->generation};
    };
    const auto fromSnapshot = [&snapshot](const engine::gfx::HdrImage& image) {
        return PathTracedAovSource{&image, snapshot};
    };
    switch (aov) {
        case engine::debug::AovId::Beauty:
            return snapshot ? fromSnapshot(snapshot->beauty) : PathTracedAovSource{};
        case engine::debug::AovId::IOR:
            return fromRaster(rasterGBuffer->iorAov);
        case engine::debug::AovId::BounceCount:
            return snapshot ? fromSnapshot(snapshot->bounceHeatmap) : PathTracedAovSource{};
        case engine::debug::AovId::Depth:
            return fromRaster(rasterGBuffer->depth);
        case engine::debug::AovId::WorldPos:
            return fromRaster(rasterGBuffer->worldPos);
        case engine::debug::AovId::UV:
            return fromRaster(rasterGBuffer->uv);
        case engine::debug::AovId::Normal:
            return fromRaster(rasterGBuffer->normal);
        case engine::debug::AovId::GeomNormal:
            return fromRaster(rasterGBuffer->geomNormal);
        case engine::debug::AovId::Albedo:
            return fromRaster(rasterGBuffer->albedo);
        case engine::debug::AovId::Metallic:
            return fromRaster(rasterGBuffer->metallic);
        case engine::debug::AovId::Roughness:
            return fromRaster(rasterGBuffer->roughness);
        case engine::debug::AovId::Tangent:
            return fromRaster(rasterGBuffer->tangent);
        case engine::debug::AovId::ObjectID:
            return fromRaster(rasterGBuffer->objectId);
        case engine::debug::AovId::Alpha:
            return fromRaster(rasterGBuffer->alpha);
        case engine::debug::AovId::Fresnel:
            return fromRaster(rasterGBuffer->fresnel);
        case engine::debug::AovId::AO:
            return fromRaster(rasterGBuffer->ao);
        case engine::debug::AovId::Shadow:
            return snapshot ? fromSnapshot(snapshot->shadow) : PathTracedAovSource{};
        case engine::debug::AovId::Wireframe:
            return fromRaster(rasterGBuffer->wireframe);
        case engine::debug::AovId::DirectDiffuse:
            return snapshot ? fromSnapshot(snapshot->directDiffuse) : PathTracedAovSource{};
        case engine::debug::AovId::IndirectDiffuse:
            return snapshot ? fromSnapshot(snapshot->indirectDiffuse) : PathTracedAovSource{};
        case engine::debug::AovId::DirectSpecular:
            return snapshot ? fromSnapshot(snapshot->directSpecular) : PathTracedAovSource{};
        case engine::debug::AovId::IndirectSpecular:
            return snapshot ? fromSnapshot(snapshot->indirectSpecular) : PathTracedAovSource{};
        case engine::debug::AovId::Refraction:
            return snapshot ? fromSnapshot(snapshot->refraction) : PathTracedAovSource{};
        default:
            return {};
    }
}

// Bottom-right HUD probe: the post-filter AOVs (HSV/Luminance/Sobel/Gabor, which have no independent buffer of their own, see presentFrame's isPostFilterAov) are GPU-only shader filters over Beauty with no CPU-side equivalent to sample, so they read back the literal composited, OCIO-display-transformed pixel from framebuffer 0, since that is the value being shown.
// Every other AOV samples its own raw HdrImage texel directly (sampleTexel, full float precision, no 8-bit-quantization involved) so the readout is in that AOV's native units (metres for Depth, bounce count for BounceCount, etc.) regardless of how it's displayed. Beauty samples its raw texel the same way but then runs it through applyBeautyDisplayTransform -- the viewer's own exposure/LUT/invert pipeline evaluated for one pixel on the CPU -- so the probe matches what's on screen without framebuffer 0's clamp/quantize (the histogram's over-range stats, updateOverRangeStats, hit the identical clamp and are fixed the same way: off the pre-quantization float value).
// cursorPosition()/windowSize() are screen points; the framebuffer path scales by framebufferSize() (not windowSize() directly, wrong by DPI factor on Retina) and flips Y (GL's origin is bottom-left, cursor's is top-left).
// The raw-texel path instead scales by the sampled image's own resolution (robust to a resize race, same precedent as resolveOrbitPick) and needs no flip: HdrImage row 0 is documented top, already matching cursor space's top-left origin.
engine::debug::PixelProbeSample samplePixelProbe(
    const engine::platform::Window& window,
    const std::shared_ptr<const engine::scene::PathTraceResult>& pathTraceSnapshot,
    const AppResources& app, engine::debug::AovId aovId) {
    const auto [windowWidth, windowHeight] = window.windowSize();
    if (windowWidth <= 0 || windowHeight <= 0) {
        return {};
    }
    const auto [cursorX, cursorY] = window.cursorPosition();
    if (cursorX < 0.0 || cursorY < 0.0 || cursorX >= windowWidth || cursorY >= windowHeight) {
        return {};
    }

    const bool isPostFilterAov =
        aovId == engine::debug::AovId::HSV || aovId == engine::debug::AovId::Luminance ||
        aovId == engine::debug::AovId::Sobel || aovId == engine::debug::AovId::Gabor;
    if (!isPostFilterAov) {
        const PathTracedAovSource source =
            selectPathTracedImage(pathTraceSnapshot, app.rasterGBuffer, aovId);
        if (source.image == nullptr) {
            return {};
        }
        const int imgX = std::min(source.image->width - 1,
                                   static_cast<int>(cursorX / windowWidth * source.image->width));
        const int imgY = std::min(source.image->height - 1,
                                   static_cast<int>(cursorY / windowHeight * source.image->height));
        const glm::vec3 texel = sampleTexel(*source.image, imgX, imgY);
        const glm::vec3 color =
            aovId == engine::debug::AovId::Beauty ? applyBeautyDisplayTransform(texel, app) : texel;
        return {true, glm::vec4(color, 1.0F)};
    }

    const auto [fbWidth, fbHeight] = window.framebufferSize();
    if (fbWidth <= 0 || fbHeight <= 0) {
        return {};
    }
    const int fbX = std::min(fbWidth - 1, static_cast<int>(cursorX / windowWidth * fbWidth));
    const int fbY = std::min(fbHeight - 1,
                              fbHeight - 1 - static_cast<int>(cursorY / windowHeight * fbHeight));

    std::array<unsigned char, 4> pixel{};
    GL_CALL(glBindFramebuffer(GL_READ_FRAMEBUFFER, 0));
    GL_CALL(glReadPixels(fbX, fbY, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, pixel.data()));
    return {true, glm::vec4(pixel[0], pixel[1], pixel[2], pixel[3]) / 255.0F};
}

// Re-uploads pathTraceDisplayTexture only when the selected AOV or the published object that owns the image actually changed: re-sending 33MB over PCIe every frame just to redisplay texels the GPU already holds would violate this codebase's no-work-per-frame-without-a-reason convention.
// The upload itself no longer destroys and recreates the texture object (Texture::upload).
// The driver publishes a fresh result object every completed pass, though, so while it's actively converging this does rebuild the texture up to once per rendered frame; that per-frame cap (not a lower one) is deliberate, it is what makes newly-accumulated samples visible at all.
// Channel view is deliberately absent from that key: it is a shader uniform now, so isolating a channel changes nothing about the texels and must not force a rebuild.
// owner: a strong ref to whichever published object actually owns `image` (see PathTracedAovSource), comparing shared_ptr identity, not a raw pointer, since a raw pointer to a previous frame's already-freed result could in principle have its address reused by a later allocation (ABA); holding a real shared_ptr in app.pathTraceDisplayedOwner rules that out.
// For Depth specifically, also rescans `image` for its own max value into app.pathTraceDisplayedDepthMax on every rebuild: presentFrame uses that as an auto-ranging display-exposure bound instead of Camera::farClip(), since farClip is a conservative ray tMax bound, not a proxy for the actual visible scene's depth extent.
void ensurePathTraceDisplayTexture(AppResources& app, const std::shared_ptr<const void>& owner,
                                    const engine::gfx::HdrImage& image, std::uint64_t generation) {
    if (app.pathTraceDisplayTexture.has_value() && app.pathTraceDisplayedAov == app.aov &&
        app.pathTraceDisplayedOwner == owner && app.pathTraceDisplayedGeneration == generation) {
        return;
    }
    if (app.aov == static_cast<int>(engine::debug::AovId::Depth)) {
        float maxDepth = 0.0F;
        for (int i = 0; i < image.width * image.height; ++i) {
            maxDepth = std::max(maxDepth, image.rgba[static_cast<std::size_t>(i) * 4]);
        }
        app.pathTraceDisplayedDepthMax = maxDepth;
    }
    // BounceCount is a mean-termination-depth scalar (R==G==B, see path_tracer.cpp's writeTexel call), not a colour, mapped through Turbo here, on the CPU, before upload, rather than as a display-shader uniform.
    // This function already only runs once per rebuilt pass (see the cache-key check above), so the map costs nothing extra per frame and needs no new uniform/texture unit.
    if (app.aov == static_cast<int>(engine::debug::AovId::BounceCount)) {
        const float maxBounceCount = static_cast<float>(app.pathTraceSettings.maxBounces) + 1.0F;
        engine::gfx::HdrImage mapped;
        mapped.width = image.width;
        mapped.height = image.height;
        mapped.rgba.resize(image.rgba.size());
        for (int i = 0; i < image.width * image.height; ++i) {
            const std::size_t idx = static_cast<std::size_t>(i) * 4;
            const float t = image.rgba[idx] / maxBounceCount;
            const glm::vec3 mappedColor = engine::debug::turbo(t);
            mapped.rgba[idx + 0] = mappedColor.r;
            mapped.rgba[idx + 1] = mappedColor.g;
            mapped.rgba[idx + 2] = mappedColor.b;
            mapped.rgba[idx + 3] = image.rgba[idx + 3];
        }
        if (app.pathTraceDisplayTexture.has_value()) {
            app.pathTraceDisplayTexture->upload(mapped.width, mapped.height, mapped.rgba.data());
        } else {
            app.pathTraceDisplayTexture = engine::gfx::Texture::createFromFloatPixels(
                mapped.width, mapped.height, mapped.rgba.data());
        }
        app.pathTraceDisplayedAov = app.aov;
        app.pathTraceDisplayedOwner = owner;
        app.pathTraceDisplayedGeneration = generation;
        return;
    }
    // Uploaded straight from the HdrImage: no row-reversed scratch copy, and no texture object churn -- fullscreen_triangle.vert now resolves the row-order convention, and Texture::upload reallocates only if the render resolution actually changed.
    if (app.pathTraceDisplayTexture.has_value()) {
        app.pathTraceDisplayTexture->upload(image.width, image.height, image.rgba.data());
    } else {
        app.pathTraceDisplayTexture =
            engine::gfx::Texture::createFromFloatPixels(image.width, image.height, image.rgba.data());
    }
    app.pathTraceDisplayedAov = app.aov;
    app.pathTraceDisplayedOwner = owner;
    app.pathTraceDisplayedGeneration = generation;
}

// Nothing to show yet (no path-trace pass has published) or the selected AOV has no buffer -- clears the default framebuffer instead of leaving stale contents on screen.
void clearToBlack(int winWidth, int winHeight) {
    GL_CALL(glBindFramebuffer(GL_FRAMEBUFFER, 0));
    GL_CALL(glViewport(0, 0, winWidth, winHeight));
    GL_CALL(glClearColor(0.0F, 0.0F, 0.0F, 1.0F));
    GL_CALL(glClear(GL_COLOR_BUFFER_BIT));
}

// Blits the path-traced buffer for the selected AOV through the shared OCIO/post-process path -- Beauty uses the user's LUT, everything else forces Raw (not scene-referred radiance, a display curve would distort them). Depth additionally gets an exposure-based normalization since its raw range exceeds the default framebuffer's fixed-point [0,1] clamp -- see the exposureEv branch below. BounceCount is already mapped into displayable [0,1] RGB (Turbo) by ensurePathTraceDisplayTexture, so it takes the unscaled passthrough case here, same as everything else.
void presentFrame(AppResources& app,
                   const std::shared_ptr<const engine::scene::PathTraceResult>& pathTraceSnapshot,
                   int winWidth, int winHeight) {
    const auto aovId = static_cast<engine::debug::AovId>(app.aov);

    const bool isPostFilterAov =
        aovId == engine::debug::AovId::HSV || aovId == engine::debug::AovId::Luminance ||
        aovId == engine::debug::AovId::Sobel || aovId == engine::debug::AovId::Gabor;
    if (isPostFilterAov) {
        // These are 2D image filters of the beauty image, not independent per-AOV buffers -- always read path-traced Beauty regardless of which of the four is selected. Needs a completed path-trace pass (unlike the rasterizer-backed AOVs below) since Beauty itself is light-transport output.
        if (!pathTraceSnapshot) {
            clearToBlack(winWidth, winHeight);
            return;
        }
        const bool isHsv = aovId == engine::debug::AovId::HSV;
        ensurePathTraceDisplayTexture(app, pathTraceSnapshot, pathTraceSnapshot->beauty, 0);
        app.ocioTransform.setActiveLut(engine::gfx::OcioDisplayTransform::Lut::Raw);
        // Same multiplier Beauty itself displays at (OcioDisplayTransform::bind) -- these four AOVs previously bypassed OCIO entirely and stayed frozen at unity gain regardless of the exposure slider.
        const float exposure = std::pow(2.0F, app.debugCamera.relativeExposureEv());
        if (isHsv) {
            app.hsvDisplayShader.use();
            GL_CALL(glUniform1i(app.uHsvChannelViewLoc, app.channelView));
            GL_CALL(glUniform1f(app.uHsvExposureLoc, exposure));
            GL_CALL(glUniform1i(app.uHsvInvertLoc, app.invert ? 1 : 0));
            app.postProcess.draw(app.pathTraceDisplayTexture->id(), app.hsvDisplayShader,
                                  {winWidth, winHeight});
        } else {
            app.edgeFilterShader.use();
            const int filterMode = aovId == engine::debug::AovId::Gabor ? 1
                                    : aovId == engine::debug::AovId::Sobel ? 0
                                                                            : 2;  // Luminance passthrough
            GL_CALL(glUniform1i(app.uFilterModeLoc, filterMode));
            GL_CALL(glUniform1i(app.uEdgeChannelViewLoc, app.channelView));
            GL_CALL(glUniform1f(app.uEdgeExposureLoc, exposure));
            GL_CALL(glUniform1i(app.uEdgeInvertLoc, app.invert ? 1 : 0));
            app.postProcess.draw(app.pathTraceDisplayTexture->id(), app.edgeFilterShader,
                                  {winWidth, winHeight});
        }
        return;
    }

    const PathTracedAovSource pathTracedSource =
        selectPathTracedImage(pathTraceSnapshot, app.rasterGBuffer, aovId);
    if (pathTracedSource.image != nullptr) {
        ensurePathTraceDisplayTexture(app, pathTracedSource.owner, *pathTracedSource.image,
                                       pathTracedSource.generation);
        const bool isBeauty = aovId == engine::debug::AovId::Beauty;
        app.ocioTransform.setActiveLut(isBeauty ? app.userLut
                                                 : engine::gfx::OcioDisplayTransform::Lut::Raw);
        // Beauty: photographic exposure. Depth: auto-ranged to the actual max depth visible in the current buffer (see ensurePathTraceDisplayTexture); farClip is a conservative ray tMax bound, not a proxy for the scene's real depth extent, and normalizing by it left real scenes (a small fraction of farClip) reading as black.
        // This exists because the default framebuffer is fixed-point and clamps any raw value >=1 to white otherwise. Everything else (including BounceCount, already colormapped into [0,1] RGB): unscaled passthrough.
        float exposureEv = 0.0F;
        if (isBeauty) {
            exposureEv = app.debugCamera.relativeExposureEv();
        } else if (aovId == engine::debug::AovId::Depth) {
            exposureEv = -std::log2(std::max(app.pathTraceDisplayedDepthMax, 1e-4F));
        }
        app.ocioTransform.setExposureEv(exposureEv);
        app.ocioTransform.setChannelView(app.channelView);
        app.ocioTransform.setInvert(app.invert);
        // Beauty only -- an artistic lens effect over the rendered image, not meaningful on a raw data AOV like Normal/Depth/Albedo.
        app.ocioTransform.setAberration(isBeauty ? app.aberrationStrength : 0.0F);
        app.ocioTransform.bind();
        app.postProcess.draw(app.pathTraceDisplayTexture->id(), app.ocioTransform.activeShader(),
                              {winWidth, winHeight});
        return;
    }

    clearToBlack(winWidth, winHeight);
}

// Non-blocking: hands a fresh request to the background PathTraceDriver, which restarts progressive accumulation at this camera pose/window size (superseding whatever it was accumulating before) and converges over subsequent passes on its own thread. Called only from renderFrame, whenever PathTraceTriggerState detects camera/scene state renderPathTraced depends on has changed.
void requestPathTrace(AppResources& app, const engine::scene::Camera& camera, int winWidth,
                      int winHeight) {
    app.pathTraceDriver->requestTrace(engine::scene::PathTraceDriver::Request{
        camera, winWidth, winHeight, glm::radians(static_cast<float>(app.envRotationDegrees)),
        app.showSky, std::exp2(app.envExposureStops), app.pathTraceSettings, app.maxSamples});
}

// Called once per rendered frame. Re-traces on any input that would actually change the image, not a fixed timer, so the path-traced view stays live without retracing every frame the camera happens to sit still.
// Because DebugCameraController's fly/orbit controls update every frame a key/mouse-drag is held, a fresh (progressive-accumulation-reset) request fires on almost every frame during camera interaction; accepted, since async execution (PathTraceDriver) keeps that from blocking the UI, it just converges more slowly while the camera moves, matching how every interactive path tracer (Cycles' viewport, Brigade) behaves.
// Only actually fires while the selected AOV needs light transport (aovNeedsLightTransport): restarting full Embree+BSDF accumulation every frame for an AOV nobody can see (Wireframe, Depth, ...) would just burn CPU competing with the rasterizer's own thread pool for no visible benefit; any accumulation already in flight from before the switch still finishes on its own.
// Both the path trace and the rasterization run at renderScale/interactiveRenderScale of the framebuffer rather than at the framebuffer itself (profile_config.h), dropping to the interactive scale on any input change and promoting back kInteractiveSettleSeconds after the last one.
// The promotion needs no separate code path: it changes the trigger, and a changed trigger is already what dispatches.
// The display blit upscales for free: glViewport targets the framebuffer and the display texture samples GL_LINEAR, so nothing downstream is aware of the resolution the image arrived at.
// Also refreshes app.rasterGBuffer synchronously on the same trigger, on the calling (render) thread; unlike the path-traced request, this blocks briefly rather than handing off to a background driver, since the point of the rasterizer is a same-frame update for its 15 AOVs (rasterizer.h).
// Gated on one of those AOVs being selected, symmetrically with the path-trace request: it is not cheap (a full-screen shade, measured at ~150ms per call at 2048x1152).
// The three things that once justified running it unconditionally no longer need it: orbit-pick now casts its own ray, the pixel probe only reaches these buffers for an AOV that is displaying them, and an AOV switch is itself a trigger change, so switching into a rasterizer AOV rasterizes on that same frame.
void requestPathTraceIfTriggerChanged(AppResources& app, const engine::scene::Camera& camera,
                                       int fbWidth, int fbHeight,
                                       std::chrono::steady_clock::time_point now) {
    const bool needsLightTransport = aovNeedsLightTransport(static_cast<engine::debug::AovId>(app.aov));
    const PathTraceInputState input{
        camera.position(),       app.debugCamera.yawDegrees(), app.debugCamera.pitchDegrees(),
        app.debugCamera.focalLengthMm(), app.envRotationDegrees, app.showSky, app.envExposureStops,
        fbWidth,                 fbHeight,                     needsLightTransport,
        app.aov};

    // Interaction is a change in anything the image depends on other than the resolution it renders at -- compared against the inputs alone, so the scale promotion below cannot re-arm the timer that produced it.
    if (input != app.lastPathTraceTrigger.input) {
        app.lastInputChange = now;
    }
    const bool settled =
        std::chrono::duration<double>(now - app.lastInputChange).count() >= kInteractiveSettleSeconds;
    const PathTraceTriggerState current{input, settled ? app.renderScale : app.interactiveRenderScale};
    if (current == app.lastPathTraceTrigger) {
        return;
    }

    const int renderWidth = scaledExtent(fbWidth, current.renderScale);
    const int renderHeight = scaledExtent(fbHeight, current.renderScale);
    // Park the driver whenever the selected AOV is one it does not produce. Without this it keeps accumulating passes of an image no longer on screen, on every core, for as long as a rasterizer AOV stays selected -- competing with the rasterizer the render thread is running synchronously right here.
    app.pathTraceDriver->setSuspended(!needsLightTransport);
    if (needsLightTransport) {
        requestPathTrace(app, camera, renderWidth, renderHeight);
    }
    // The complement of needsLightTransport is exactly the rasterizer's 15 AOVs: aovNeedsLightTransport covers 12 of AovId::Count's 27 and selectPathTracedImage routes the other 15 here, so the two sets partition the enum and no AOV needs neither producer. On Beauty -- the default -- the rasterizer now does not run at all, where before it rasterized the full framebuffer on the render thread every frame of camera interaction to produce 15 images nobody was looking at.
    if (!needsLightTransport && renderWidth > 0 && renderHeight > 0) {
        engine::scene::renderRasterGBuffer(camera, app.sceneAccel, app.stumpModel.shadingTriangles,
                                            app.stumpModel.instances, app.perInstanceSettings,
                                            renderWidth, renderHeight, *app.rasterThreadPool,
                                            *app.rasterGBuffer);
    }
    app.lastPathTraceTrigger = current;
}

// Fraction of texels that would clip at the display encode, plus the peak such value as a multiple of display range -- e.g. "3.2%, peak 47.8x". Computed from the pre-display-transform HdrImage (full float, in hand already) rather than the composited framebuffer Histogram reads, since B1's colorimetric-only display transform means anything above 1.0 clips with no tone-mapped rolloff to cushion it, and the on-screen histogram alone cannot distinguish "just barely over" from "wildly over" -- both pin bin 255 identically. Gated at Histogram's own capture interval rather than scanned every frame, matching this codebase's no-work-per-frame-without-a-reason convention (a multi-megapixel float scan is not free).
void updateOverRangeStats(AppResources& app,
                           const std::shared_ptr<const engine::scene::PathTraceResult>& pathTraceSnapshot) {
    ++app.overRangeFrameCounter;
    if (app.overRangeFrameCounter % engine::debug::Histogram::kCaptureIntervalFrames != 0) {
        return;
    }
    if (!pathTraceSnapshot) {
        app.overRangeFraction = 0.0F;
        app.overRangePeakMultiple = 0.0F;
        return;
    }
    const engine::gfx::HdrImage& beauty = pathTraceSnapshot->beauty;
    const float exposure = std::pow(2.0F, app.debugCamera.relativeExposureEv());
    const int texelCount = beauty.width * beauty.height;
    int overCount = 0;
    float peak = 0.0F;
    for (int i = 0; i < texelCount; ++i) {
        const std::size_t idx = static_cast<std::size_t>(i) * 4;
        const float maxChannel = std::max({beauty.rgba[idx + 0], beauty.rgba[idx + 1], beauty.rgba[idx + 2]}) *
                                  exposure;
        overCount += maxChannel > 1.0F ? 1 : 0;
        peak = std::max(peak, maxChannel);
    }
    app.overRangeFraction = texelCount > 0 ? static_cast<float>(overCount) / static_cast<float>(texelCount) : 0.0F;
    app.overRangePeakMultiple = peak;
}

void updateHud(AppResources& app, const engine::platform::Window& window,
               const engine::scene::Camera& camera,
               const std::shared_ptr<const engine::scene::PathTraceResult>& pathTraceSnapshot,
               int winWidth,
               int winHeight) {
    const int accumulatedSamples =
        app.pathTraceDriver != nullptr ? app.pathTraceDriver->accumulatedSamples() : 0;
    const engine::debug::PathTracedStatus pathTracedStatus{
        accumulatedSamples > 0,
        app.pathTraceDriver != nullptr ? app.pathTraceDriver->lastPassSeconds() : 0.0,
        accumulatedSamples, app.maxSamples};
    const engine::debug::SceneStats sceneStats{
        static_cast<int>(app.stumpModel.instances.size()),
        app.totalTriangles,
        app.totalPoints,
        winWidth,
        winHeight,
    };
    const engine::debug::HudFrameData hudFrameData{
        app.gpuInfo,
        app.frameStats,
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
        pathTracedStatus,
        app.overRangeFraction,
        app.overRangePeakMultiple,
    };
    // Round-tripped through locals so the HUD's sliders can bind plain float&s, same as aov -- DebugCameraController is the authoritative owner, read before draw() and written back after.
    float focalLengthMm = app.debugCamera.focalLengthMm();
    float aperture = app.debugCamera.aperture();
    float shutterSeconds = app.debugCamera.shutterSeconds();
    float iso = app.debugCamera.iso();
    const engine::debug::PixelProbeSample pixelProbe = samplePixelProbe(
        window, pathTraceSnapshot, app, static_cast<engine::debug::AovId>(app.aov));
    if (app.showHud) {
        app.hud.draw(hudFrameData, app.aov, focalLengthMm, aperture, shutterSeconds, iso, app.showSky,
                     app.envRotationDegrees, app.envExposureStops, app.aberrationStrength,
                     app.framingState, pixelProbe);
    }
    app.debugCamera.setFocalLengthMm(focalLengthMm);
    app.debugCamera.setAperture(aperture);
    app.debugCamera.setShutterSeconds(shutterSeconds);
    app.debugCamera.setIso(iso);
    app.hud.render();
}

// One frame: poll -> update camera -> request a fresh path trace if input changed -> orbit-pick from the path tracer's own G-buffer -> post-process blit to the default framebuffer -> swap.
void renderFrame(engine::platform::Window& window, AppResources& app) {
    window.pollEvents();
    app.hud.beginFrame();
    app.frameStats.tick();

    const auto frameNow = std::chrono::steady_clock::now();
    const float dtSeconds = std::chrono::duration<float>(frameNow - app.lastFrameTime).count();
    app.lastFrameTime = frameNow;

    const engine::scene::Camera camera = updateCamera(window, app, dtSeconds);
    const auto [winWidth, winHeight] = window.framebufferSize();

    requestPathTraceIfTriggerChanged(app, camera, winWidth, winHeight, frameNow);

    // Held for the rest of this frame so the images behind it stay valid even if the driver publishes a newer result mid-frame -- a strong ref, not a raw fetch. Null until the first pass of the app's life completes.
    const std::shared_ptr<const engine::scene::PathTraceResult> pathTraceSnapshot =
        app.pathTraceDriver != nullptr ? app.pathTraceDriver->latestResult() : nullptr;

    resolveOrbitPick(window, app, camera);

    app.postTimer.begin();
    presentFrame(app, pathTraceSnapshot, winWidth, winHeight);
    app.postTimer.end();

    // Captured after the composited image lands in the default framebuffer, before the HUD draws on top of it.
    app.histogram.update(winWidth, winHeight);
    updateOverRangeStats(app, pathTraceSnapshot);

    const auto now = std::chrono::steady_clock::now();
    if (now - app.lastRamSample >= std::chrono::milliseconds(250)) {
        app.ramBytes = engine::debug::residentSetBytes();
        app.systemAvailableBytes = engine::debug::availableSystemBytes();
        app.lastRamSample = now;
    }

    updateHud(app, window, camera, pathTraceSnapshot, winWidth, winHeight);

    window.swapBuffers();
}

struct Options {
    std::string scenePath = ASSET_ROOT_DIR "/scenes/cornell.json";
};

// Returns nullopt on an unrecognized flag or a missing value -- argv is a system
// boundary, so a bad value is surfaced rather than defaulted around.
std::optional<Options> parseOptions(int argc, char** argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--scene") == 0) {
            if (i + 1 >= argc) {
                std::cerr << "engine: --scene expects a value\n";
                return std::nullopt;
            }
            options.scenePath = argv[++i];
        } else {
            std::cerr << "engine: unknown flag " << argv[i]
                       << "\n  usage: engine [--scene path/to/scene.json]\n";
            return std::nullopt;
        }
    }
    return options;
}

}  // namespace

int main(int argc, char** argv) {
    const std::optional<Options> options = parseOptions(argc, argv);
    if (!options) {
        return EXIT_FAILURE;
    }

    glfwSetErrorCallback(&glfwErrorCallback);

    if (glfwInit() != GLFW_TRUE) {
        std::cerr << "main: glfwInit failed\n";
        return EXIT_FAILURE;
    }

    int exitCode = EXIT_SUCCESS;
    {
        // Config is pure file I/O with no GL dependency, but profile.json's window size must be known before Window is constructed, so it's loaded first, before any GLFW/GL object exists. Both files hard-fail identically on missing or malformed: this is user-editable input where a load failure is a real, expected-to-happen event, not an internal invariant, so it's surfaced immediately rather than defaulted around -- matching the shader/model/OCIO all-or-nothing gate inside initializeApp.
        std::optional<engine::config::SceneConfig> sceneConfig =
            engine::config::loadSceneConfig(options->scenePath);
        std::optional<engine::config::ProfileConfig> profileConfig =
            engine::config::loadProfileConfig(ASSET_ROOT_DIR "/config/profile.json");

        if (!sceneConfig || !profileConfig) {
            std::cerr << "main: scene/profile config load failed, aborting startup\n";
            exitCode = EXIT_FAILURE;
        } else {
            // Window construction creates the GL 4.1 core/fwd-compat context and makes it current; fatal failure inside it exits the process directly (see window.cpp) since nothing recoverable exists yet.
            engine::platform::Window window(profileConfig->window.width, profileConfig->window.height,
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
                    // Constructed here, not as part of AppResources's designated-initializer list: app (this std::optional<AppResources> local) is where sceneAccel/environmentMap/stumpModel/perInstanceSettings first reach their final, permanent address (initializeApp's own return-type conversion to std::optional<AppResources> move-constructs once en route), so this is the first point at which PathTraceDriver's reference members can safely bind to them -- see path_trace_driver.h's constructor comment.
                    app->pathTraceDriver = std::make_unique<engine::scene::PathTraceDriver>(
                        app->sceneAccel, app->stumpModel.shadingTriangles, app->stumpModel.instances,
                        app->environmentMap, app->perInstanceSettings);

                    wireCallbacks(window, *app);

                    while (!window.shouldClose()) {
                        renderFrame(window, *app);
                    }
                }
            }
        }
    }  // Window destroyed here, while GLFW is still initialized.

    glfwTerminate();
    return exitCode;
}
