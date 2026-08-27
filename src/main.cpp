#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

// GLEW before GLFW — see gl_debug.cpp for why.
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "engine/config/material_config.h"
#include "engine/config/profile_config.h"
#include "engine/config/scene_config.h"
#include "engine/debug/aov.h"
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
#include "engine/scene/path_trace_driver.h"
#include "engine/scene/path_tracer.h"

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

// Snapshot of every input renderPathTraced's result actually depends on -- compared frame to frame (see requestPathTraceIfTriggerChanged) to decide whether to hand PathTraceDriver a fresh request. envRotationDegrees defaults to the sentinel -1 (never a real value, since the HUD clamps it to [0,359]) specifically so the very first comparison always mismatches, giving the path-traced view a live result from the first rendered frame with no separate startup-trace call needed.
struct PathTraceTriggerState {
    glm::vec3 cameraPosition{0.0F};
    float cameraYawDegrees = 0.0F;
    float cameraPitchDegrees = 0.0F;
    float focalLengthMm = 0.0F;
    int envRotationDegrees = -1;
    bool showSky = false;
    float envExposureStops = 0.0F;
    int winWidth = 0;
    int winHeight = 0;

    bool operator==(const PathTraceTriggerState&) const = default;
};

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
    engine::scene::DebugCameraController debugCamera;
    engine::debug::GpuInfo gpuInfo;

    int uFilterModeLoc;
    int uHsvChannelViewLoc;

    // HUD-editable UI/run state.
    int aov;
    int channelView;
    engine::gfx::OcioDisplayTransform::Lut userLut;
    engine::debug::FramingOverlayState framingState;
    bool showSky;
    int envRotationDegrees;
    float envExposureStops;  // stops, not a multiplier; requestPathTrace does exp2()

    // Async path-traced view, selected via the `aov` field (engine::debug::AovId, the HUD's AOV dropdown). pathTraceDriver runs continuously on its own background thread once constructed (main() constructs it after initializeApp() returns -- see path_trace_driver.h's constructor precondition on reference stability); requestTrace() is called only from renderFrame's requestPathTraceIfTriggerChanged, whenever lastPathTraceTrigger detects the camera/scene state renderPathTraced depends on has changed -- no manual trigger. unique_ptr, not a by-value optional: PathTraceDriver holds reference members and an owned std::jthread/std::mutex, so it's neither copyable nor movable -- a by-value optional<T> member would make that non-movability propagate to AppResources itself (optional<T>'s move ctor is only available when T's is), which would break initializeApp's return-by-value/RVO pattern every other member here relies on. A unique_ptr's own move just transfers ownership of the pointee's address, never touching PathTraceDriver's reference members, so AppResources stays movable and PathTraceDriver itself is never relocated in memory once constructed.
    engine::scene::PathTraceSettings pathTraceSettings;
    int maxSamples;  // accumulated-pass cap for PathTraceDriver; 0 = unbounded
    std::unique_ptr<engine::scene::PathTraceDriver> pathTraceDriver;
    std::optional<engine::gfx::Texture> pathTraceDisplayTexture;
    int pathTraceDisplayedAov;  // which AovId pathTraceDisplayTexture currently holds, -1 = none yet
    int pathTraceDisplayedChannelView;  // which channelView pathTraceDisplayTexture was isolated for, -1 = none yet
    // Strong ref (kept alive, not just an identity pointer) to whichever PathTraceSnapshot object (gbuffer/dynamic) pathTraceDisplayTexture currently reflects -- see ensurePathTraceDisplayTexture.
    std::shared_ptr<const void> pathTraceDisplayedOwner;
    // Max raw Depth value seen in the last rebuilt pathTraceDisplayTexture -- see ensurePathTraceDisplayTexture; only meaningful/updated when aov==Depth.
    float pathTraceDisplayedDepthMax;
    PathTraceTriggerState lastPathTraceTrigger;  // sentinel-initialized, see its own doc comment

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

// Sobel/Gabor's second pass (see edge_filter.frag): uHdrColor's texture unit and the Gabor kernel weights are both fixed for the whole run, set once here. Returns uFilterMode's cached location.
int setupEdgeFilterShader(const engine::gfx::ShaderProgram& edgeFilterShader) {
    edgeFilterShader.use();
    GL_CALL(glUniform1i(edgeFilterShader.uniformLocation("uHdrColor"), 0));
    const std::array<float, 100> gaborKernel = buildGaborKernel();
    GL_CALL(glUniform1fv(edgeFilterShader.uniformLocation("uGaborKernel"), 100, gaborKernel.data()));
    return edgeFilterShader.uniformLocation("uFilterMode");
}

// hsv_display.frag's uHdrColor texture unit is fixed for the whole run, same convention as setupEdgeFilterShader above. Returns uChannelView's cached location.
int setupHsvDisplayShader(const engine::gfx::ShaderProgram& hsvDisplayShader) {
    hsvDisplayShader.use();
    GL_CALL(glUniform1i(hsvDisplayShader.uniformLocation("uHdrColor"), 0));
    return hsvDisplayShader.uniformLocation("uChannelView");
}

// All one-time startup work: camera/model/shader/environment loading (nullopt on any failure -- matches the shader/model/OCIO all-or-nothing gate this replaces), Embree scene build, and cached uniform-location lookups for the shared edge-filter/HSV shaders. Doesn't wire input callbacks -- those capture a stable AppResources& and must be set up by the caller only after this returns (see main()), since a callback capturing a reference into an AppResources that's still about to be moved into its final std::optional storage would dangle.
std::optional<AppResources> initializeApp(const engine::config::SceneConfig& sceneConfig,
                                           const engine::config::ProfileConfig& profileConfig,
                                           const engine::config::MaterialConfig& materialConfig,
                                           engine::platform::Window& window) {
    std::cout << "GL_KHR_debug available: " << std::boolalpha << engine::gfx::khrDebugAvailable()
              << '\n';
    std::cout << "GL_ARB_timer_query available: " << std::boolalpha
              << engine::debug::gpuTimerQueryAvailable() << '\n';
    const engine::debug::GpuInfo gpuInfo = engine::debug::queryGpuInfo();

    // exposure()/ev100() are logged but not render-path-consumed yet: no scene-referred exposure multiply exists, only OCIO's display-encode step.
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

    // Scene-level placement (scene.json position/rotationDegrees), order X,Y,Z.
    const glm::mat4 sceneTransform =
        glm::translate(glm::mat4(1.0F), sceneConfig.position) *
        glm::rotate(glm::mat4(1.0F), glm::radians(sceneConfig.rotationDegrees.z), glm::vec3(0.0F, 0.0F, 1.0F)) *
        glm::rotate(glm::mat4(1.0F), glm::radians(sceneConfig.rotationDegrees.y), glm::vec3(0.0F, 1.0F, 0.0F)) *
        glm::rotate(glm::mat4(1.0F), glm::radians(sceneConfig.rotationDegrees.x), glm::vec3(1.0F, 0.0F, 0.0F));

    const auto loadStart = std::chrono::steady_clock::now();
    std::optional<engine::scene::LoadedModel> stumpModel = engine::scene::loadGltf(
        std::string(ASSET_ROOT_DIR) + "/" + sceneConfig.gltfPath, sceneTransform,
        std::string(ASSET_ROOT_DIR) + "/" + sceneConfig.texturePath);
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
        engine::gfx::loadExr(std::string(ASSET_ROOT_DIR) + "/" + profileConfig.hdriPath);

    if (!shaders || !stumpModel || !environmentImage) {
        std::cerr << "main: shader compile/link, model load, or environment map load failed, "
                     "aborting startup\n";
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

    const int uFilterModeLoc = setupEdgeFilterShader(shaders->edgeFilterShader);
    const int uHsvChannelViewLoc = setupHsvDisplayShader(shaders->hsvDisplayShader);

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
        .uFilterModeLoc = uFilterModeLoc,
        .uHsvChannelViewLoc = uHsvChannelViewLoc,
        // aov selects which AOV the path tracer's snapshot supplies (see selectPathTracedImage); channelView isolates one R/G/B channel of whatever aov currently shows. userLut is the LUT 'L' cycles through -- kept separate from OcioDisplayTransform's active LUT because non-Beauty AOVs force Raw (see the LUT-select comment in presentFrame) and must not clobber the user's actual choice. Both aov and userLut start from profile.json rather than a fixed literal.
        .aov = profileConfig.defaultAov,
        .channelView = 0,
        .userLut = profileConfig.defaultLut,
        .framingState = engine::debug::FramingOverlayState{},
        // "Show/Hide Background" HDRI-section checkbox -- off by default; only takes visible effect for the Beauty AOV, see presentFrame.
        .showSky = false,
        // HDR environment's Y-axis (world up) rotation, degrees [0,359] -- affects both the background and the environment's contribution to lighting (see environment_map.h), rotated at query time rather than re-baked.
        .envRotationDegrees = 0,
        // HDRI Exposure slider, stops. requestPathTrace does exp2() -> path_tracer.cpp miss-ray sampleDirection.
        .envExposureStops = 0.0F,
        .pathTraceSettings =
            engine::scene::PathTraceSettings{
                .samplesPerPixel = profileConfig.samplesPerPixel,
                .maxBounces = profileConfig.maxBounces,
                .russianRouletteStartBounce = profileConfig.russianRouletteStartBounce,
                .bumpStrength = materialConfig.bumpStrength,
                .roughnessMin = materialConfig.roughnessMin,
                .roughnessMax = materialConfig.roughnessMax,
            },
        .maxSamples = profileConfig.maxSamples,
        // Constructed in main() right after initializeApp() returns -- see path_trace_driver.h's constructor precondition (its reference members must bind to sceneAccel/environmentMap/stumpModel at their final, permanent address, which this designated-initializer expression, still local-variable-based and one AppResources move away from that address, cannot yet guarantee).
        .pathTraceDriver = nullptr,
        .pathTraceDisplayTexture = std::nullopt,
        .pathTraceDisplayedAov = -1,
        .pathTraceDisplayedChannelView = -1,
        .pathTraceDisplayedOwner = nullptr,
        .pathTraceDisplayedDepthMax = 0.0F,
        .lastPathTraceTrigger = PathTraceTriggerState{},
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

// Debug-only: 'L' cycles the viewer LUT (sRGB -> Rec709 -> Raw -> sRGB -> ...), Raw being a genuine no-display-encode passthrough for direct encoded-vs-unencoded comparison. 'R'/'G'/'B' toggle isolating a channel of the active AOV (pressing the active one again turns it back off) -- reset moved to '0' to free these back up. 'K' toggles the centre-crosshair framing overlay. No general input-mapping system for these few keys is needed: WASD/QE need continuous per-frame state (Window::isKeyDown) rather than this edge-triggered callback, so this single slot still covers everything that's actually event-shaped. Wired up here, not inside initializeApp: every callback captures a reference into app, which must already be at its final, stable address (main()'s local, unwrapped from the optional initializeApp returned) -- capturing a reference during initializeApp would dangle the moment that AppResources is moved into its optional's storage.
void wireCallbacks(engine::platform::Window& window, AppResources& app) {
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
        } else if (key == GLFW_KEY_R) {
            app.channelView = app.channelView == 1 ? 0 : 1;
        } else if (key == GLFW_KEY_G) {
            app.channelView = app.channelView == 2 ? 0 : 2;
        } else if (key == GLFW_KEY_B) {
            app.channelView = app.channelView == 3 ? 0 : 3;
        } else if (key == GLFW_KEY_0) {
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

// Direct texel read, no bilinear -- gbuffer AOVs are per-pixel snapshots.
glm::vec3 sampleTexel(const engine::gfx::HdrImage& image, int x, int y) {
    const std::size_t idx = ((static_cast<std::size_t>(y) * static_cast<std::size_t>(image.width)) +
                              static_cast<std::size_t>(x)) *
                             4;
    return {image.rgba[idx + 0], image.rgba[idx + 1], image.rgba[idx + 2]};
}

// Orbit pivot picked from the path tracer's own G-buffer (worldPos/alpha at its centre pixel) -- sampled at the gbuffer image's own resolution, not the live window size, in case of a resize race. Falls back to a fixed forward-offset pivot when no pass has published yet or the centre pixel missed all geometry.
void resolveOrbitPick(engine::platform::Window& window, AppResources& app,
                       const engine::scene::Camera& camera,
                       const engine::scene::PathTraceSnapshot& pathTraceSnapshot) {
    if (!app.orbitPickRequested) {
        return;
    }
    app.orbitPickRequested = false;

    glm::vec3 pivot = camera.position() + (3.0F * camera.forward());
    if (pathTraceSnapshot.gbuffer) {
        const engine::gfx::HdrImage& worldPos = pathTraceSnapshot.gbuffer->worldPos;
        const int cx = worldPos.width / 2;
        const int cy = worldPos.height / 2;
        if (sampleTexel(pathTraceSnapshot.gbuffer->alpha, cx, cy).x > 0.5F) {
            pivot = sampleTexel(worldPos, cx, cy);
        }
    }

    app.debugCamera.beginOrbit(pivot);
    window.setCursorLocked(true);
    const auto [cursorX, cursorY] = window.cursorPosition();
    app.lastCursorX = cursorX;
    app.lastCursorY = cursorY;
}

// HdrImage's row 0 is documented as the image's top (EXR/glTF convention, hdr_image.h); GL texture v=0 samples the first uploaded row, and fullscreen_triangle.vert's vUv places v=0 at the bottom of the window. Row-reversing here -- only for this fixed-blit display path, never for material or environment HdrImages sampled via mesh UVs / the equirect formula (environment_map.cpp), which already agree with row-0-top by construction -- makes the uploaded buffer's first row the image's bottom row, matching every other texture this same blit displays. channelView isolates one R/G/B channel (broadcast to grey) -- folded into this same per-pixel copy rather than a separate shader pass, since the path-traced display texture is a direct CPU-image upload with no shader pass of its own to apply it in.
std::vector<float> flipRowsForDisplay(const engine::gfx::HdrImage& image, int channelView) {
    std::vector<float> flipped(image.rgba.size());
    const std::size_t rowFloats = static_cast<std::size_t>(image.width) * 4;
    for (int y = 0; y < image.height; ++y) {
        const std::size_t src = static_cast<std::size_t>(y) * rowFloats;
        const std::size_t dst = static_cast<std::size_t>(image.height - 1 - y) * rowFloats;
        for (std::size_t i = 0; i < rowFloats; ++i) {
            flipped[dst + i] = image.rgba[src + i];
        }
        if (channelView >= 1 && channelView <= 3) {
            for (std::size_t px = 0; px < rowFloats; px += 4) {
                const float isolated = flipped[dst + px + static_cast<std::size_t>(channelView - 1)];
                flipped[dst + px + 0] = isolated;
                flipped[dst + px + 1] = isolated;
                flipped[dst + px + 2] = isolated;
            }
        }
    }
    return flipped;
}

// Bundles the HdrImage a given AOV should display with a type-erased strong ref to whichever of PathTraceSnapshot's two objects (gbuffer/dynamic) actually owns it. That ref both keeps the owning object alive and gives ensurePathTraceDisplayTexture an ABA-safe cache-key identity: a raw pointer to it could, in principle, be freed and have a later unrelated shared_ptr allocation reuse the same address; holding a real shared_ptr can't.
struct PathTracedAovSource {
    const engine::gfx::HdrImage* image = nullptr;
    std::shared_ptr<const void> owner;
};

// Returns a default (null image) while either half of the snapshot hasn't published its first pass yet -- callers show black instead. Extended as PathTraceGBuffer/PathTraceDynamic grow more buffers.
PathTracedAovSource selectPathTracedImage(const engine::scene::PathTraceSnapshot& snapshot,
                                           engine::debug::AovId aov) {
    if (!snapshot.gbuffer || !snapshot.dynamic) {
        return {};
    }
    switch (aov) {
        case engine::debug::AovId::Beauty:
            return {&snapshot.dynamic->beauty, snapshot.dynamic};
        case engine::debug::AovId::IOR:
            return {&snapshot.gbuffer->iorAov, snapshot.gbuffer};
        case engine::debug::AovId::BounceCount:
            return {&snapshot.dynamic->bounceHeatmap, snapshot.dynamic};
        case engine::debug::AovId::Depth:
            return {&snapshot.gbuffer->depth, snapshot.gbuffer};
        case engine::debug::AovId::WorldPos:
            return {&snapshot.gbuffer->worldPos, snapshot.gbuffer};
        case engine::debug::AovId::UV:
            return {&snapshot.gbuffer->uv, snapshot.gbuffer};
        case engine::debug::AovId::Normal:
            return {&snapshot.gbuffer->normal, snapshot.gbuffer};
        case engine::debug::AovId::GeomNormal:
            return {&snapshot.gbuffer->geomNormal, snapshot.gbuffer};
        case engine::debug::AovId::Albedo:
            return {&snapshot.gbuffer->albedo, snapshot.gbuffer};
        case engine::debug::AovId::Metallic:
            return {&snapshot.gbuffer->metallic, snapshot.gbuffer};
        case engine::debug::AovId::Roughness:
            return {&snapshot.gbuffer->roughness, snapshot.gbuffer};
        case engine::debug::AovId::Tangent:
            return {&snapshot.gbuffer->tangent, snapshot.gbuffer};
        case engine::debug::AovId::ObjectID:
            return {&snapshot.gbuffer->objectId, snapshot.gbuffer};
        case engine::debug::AovId::Alpha:
            return {&snapshot.gbuffer->alpha, snapshot.gbuffer};
        case engine::debug::AovId::Fresnel:
            return {&snapshot.gbuffer->fresnel, snapshot.gbuffer};
        case engine::debug::AovId::AO:
            return {&snapshot.gbuffer->ao, snapshot.gbuffer};
        case engine::debug::AovId::Shadow:
            return {&snapshot.gbuffer->shadow, snapshot.gbuffer};
        case engine::debug::AovId::Wireframe:
            return {&snapshot.gbuffer->wireframe, snapshot.gbuffer};
        case engine::debug::AovId::BoundingBox:
            return {&snapshot.gbuffer->boundingBox, snapshot.gbuffer};
        case engine::debug::AovId::DirectDiffuse:
            return {&snapshot.dynamic->directDiffuse, snapshot.dynamic};
        case engine::debug::AovId::IndirectDiffuse:
            return {&snapshot.dynamic->indirectDiffuse, snapshot.dynamic};
        case engine::debug::AovId::DirectSpecular:
            return {&snapshot.dynamic->directSpecular, snapshot.dynamic};
        case engine::debug::AovId::IndirectSpecular:
            return {&snapshot.dynamic->indirectSpecular, snapshot.dynamic};
        case engine::debug::AovId::Refraction:
            return {&snapshot.dynamic->refraction, snapshot.dynamic};
        default:
            return {};
    }
}

// Bottom-right HUD probe: Beauty and the post-filter AOVs (HSV/Luminance/Sobel/Gabor, which have no
// independent buffer of their own -- see presentFrame's isPostFilterAov) read back the literal
// composited, OCIO-display-transformed pixel from framebuffer 0, since that IS the value being shown.
// Every other AOV instead samples its own raw HdrImage texel directly (sampleTexel, full float
// precision, no display-exposure/8-bit-quantization involved) so the readout is in that AOV's native
// units (metres for Depth, bounce count for BounceCount, etc.) regardless of how it's displayed.
// cursorPosition()/windowSize() are screen points; the framebuffer path scales by framebufferSize()
// (not windowSize() directly -- wrong by DPI factor on Retina) and flips Y (GL's origin is
// bottom-left, cursor's is top-left). The raw-texel path instead scales by the sampled image's own
// resolution (robust to a resize race, same precedent as resolveOrbitPick) and needs no flip --
// HdrImage row 0 is documented top, already matching cursor space's top-left origin.
engine::debug::PixelProbeSample samplePixelProbe(const engine::platform::Window& window,
                                                  const engine::scene::PathTraceSnapshot& pathTraceSnapshot,
                                                  engine::debug::AovId aovId) {
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
    if (aovId != engine::debug::AovId::Beauty && !isPostFilterAov) {
        const PathTracedAovSource source = selectPathTracedImage(pathTraceSnapshot, aovId);
        if (source.image == nullptr) {
            return {};
        }
        const int imgX = std::min(source.image->width - 1,
                                   static_cast<int>(cursorX / windowWidth * source.image->width));
        const int imgY = std::min(source.image->height - 1,
                                   static_cast<int>(cursorY / windowHeight * source.image->height));
        return {true, glm::vec4(sampleTexel(*source.image, imgX, imgY), 1.0F)};
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

// Rebuilds pathTraceDisplayTexture only when the selected AOV, the channel-view isolation, or the driver's published result object actually changed -- recreating a GL texture (alloc + upload + mipmap) every frame just to redisplay the same image would violate this codebase's no-allocation-after-init convention for no reason. The driver publishes a fresh result object every completed pass, though, so while it's actively converging this does rebuild the texture up to once per rendered frame -- that per-frame cap (not a lower one) is deliberate: it is what makes newly-accumulated samples visible at all. channelViewToBake: normally app.channelView, isolating an R/G/B channel of `image` itself before upload -- except the HSV AOV, which passes 0 (no isolation here) and applies channel view to its own H/S/V output instead (see hsv_display.frag's uChannelView): isolating a source RGB channel first would broadcast it to grey, and grey always converts to H=0/S=0, destroying the very thing that AOV exists to show. owner: a strong ref to whichever PathTraceSnapshot object actually owns `image` (see PathTracedAovSource) -- comparing shared_ptr identity, not a raw pointer, since a raw pointer to a previous frame's already-freed result could in principle have its address reused by a later allocation (ABA); holding a real shared_ptr in app.pathTraceDisplayedOwner rules that out. For Depth specifically, also rescans `image` for its own max value into app.pathTraceDisplayedDepthMax on every rebuild -- presentFrame uses that as an auto-ranging display-exposure bound instead of Camera::farClip(), since farClip is a conservative ray tMax bound, not a proxy for the actual visible scene's depth extent.
void ensurePathTraceDisplayTexture(AppResources& app, const std::shared_ptr<const void>& owner,
                                    const engine::gfx::HdrImage& image, int channelViewToBake) {
    if (app.pathTraceDisplayTexture.has_value() && app.pathTraceDisplayedAov == app.aov &&
        app.pathTraceDisplayedChannelView == channelViewToBake &&
        app.pathTraceDisplayedOwner == owner) {
        return;
    }
    if (app.aov == static_cast<int>(engine::debug::AovId::Depth)) {
        float maxDepth = 0.0F;
        for (int i = 0; i < image.width * image.height; ++i) {
            maxDepth = std::max(maxDepth, image.rgba[static_cast<std::size_t>(i) * 4]);
        }
        app.pathTraceDisplayedDepthMax = maxDepth;
    }
    const std::vector<float> flipped = flipRowsForDisplay(image, channelViewToBake);
    app.pathTraceDisplayTexture =
        engine::gfx::Texture::createFromFloatPixels(image.width, image.height, flipped.data());
    app.pathTraceDisplayedAov = app.aov;
    app.pathTraceDisplayedChannelView = channelViewToBake;
    app.pathTraceDisplayedOwner = owner;
}

// Nothing to show yet (no path-trace pass has published) or the selected AOV has no buffer -- clears the default framebuffer instead of leaving stale contents on screen.
void clearToBlack(int winWidth, int winHeight) {
    GL_CALL(glBindFramebuffer(GL_FRAMEBUFFER, 0));
    GL_CALL(glViewport(0, 0, winWidth, winHeight));
    GL_CALL(glClearColor(0.0F, 0.0F, 0.0F, 1.0F));
    GL_CALL(glClear(GL_COLOR_BUFFER_BIT));
}

// Blits the path-traced buffer for the selected AOV through the shared OCIO/post-process path -- Beauty uses the user's LUT, everything else forces Raw (not scene-referred radiance, a display curve would distort them). Depth/BounceCount additionally get an exposure-based normalization since their raw range exceeds the default framebuffer's fixed-point [0,1] clamp -- see the exposureEv branch below.
void presentFrame(AppResources& app, const engine::scene::PathTraceSnapshot& pathTraceSnapshot,
                   int winWidth, int winHeight) {
    const auto aovId = static_cast<engine::debug::AovId>(app.aov);
    const bool hasPathTraceResult = pathTraceSnapshot.gbuffer && pathTraceSnapshot.dynamic;
    if (!hasPathTraceResult) {
        clearToBlack(winWidth, winHeight);
        return;
    }

    const bool isPostFilterAov =
        aovId == engine::debug::AovId::HSV || aovId == engine::debug::AovId::Luminance ||
        aovId == engine::debug::AovId::Sobel || aovId == engine::debug::AovId::Gabor;
    if (isPostFilterAov) {
        // These are 2D image filters of the beauty image, not independent per-AOV buffers -- always read path-traced Beauty regardless of which of the four is selected.
        const bool isHsv = aovId == engine::debug::AovId::HSV;
        ensurePathTraceDisplayTexture(app, pathTraceSnapshot.dynamic, pathTraceSnapshot.dynamic->beauty,
                                       isHsv ? 0 : app.channelView);
        app.ocioTransform.setActiveLut(engine::gfx::OcioDisplayTransform::Lut::Raw);
        if (isHsv) {
            app.hsvDisplayShader.use();
            GL_CALL(glUniform1i(app.uHsvChannelViewLoc, app.channelView));
            app.postProcess.draw(app.pathTraceDisplayTexture->id(), app.hsvDisplayShader,
                                  {winWidth, winHeight});
        } else {
            app.edgeFilterShader.use();
            const int filterMode = aovId == engine::debug::AovId::Gabor ? 1
                                    : aovId == engine::debug::AovId::Sobel ? 0
                                                                            : 2;  // Luminance passthrough
            GL_CALL(glUniform1i(app.uFilterModeLoc, filterMode));
            app.postProcess.draw(app.pathTraceDisplayTexture->id(), app.edgeFilterShader,
                                  {winWidth, winHeight});
        }
        return;
    }

    const PathTracedAovSource pathTracedSource = selectPathTracedImage(pathTraceSnapshot, aovId);
    if (pathTracedSource.image != nullptr) {
        ensurePathTraceDisplayTexture(app, pathTracedSource.owner, *pathTracedSource.image,
                                       app.channelView);
        const bool isBeauty = aovId == engine::debug::AovId::Beauty;
        app.ocioTransform.setActiveLut(isBeauty ? app.userLut
                                                 : engine::gfx::OcioDisplayTransform::Lut::Raw);
        // Beauty: photographic exposure. Depth: auto-ranged to the actual max depth visible in the
        // current buffer (see ensurePathTraceDisplayTexture) -- farClip is a conservative ray tMax
        // bound, not a proxy for the scene's real depth extent, and normalizing by it left real scenes
        // (a small fraction of farClip) reading as black. BounceCount: normalized by its own provable
        // upper bound (maxBounces+1), a realistically tight range so this stays a static divide. Both
        // exist because the default framebuffer is fixed-point and clamps any raw value >= 1 to white
        // otherwise. Everything else: unscaled passthrough.
        float exposureEv = 0.0F;
        if (isBeauty) {
            exposureEv = app.debugCamera.relativeExposureEv();
        } else if (aovId == engine::debug::AovId::Depth) {
            exposureEv = -std::log2(std::max(app.pathTraceDisplayedDepthMax, 1e-4F));
        } else if (aovId == engine::debug::AovId::BounceCount) {
            exposureEv = -std::log2(static_cast<float>(app.pathTraceSettings.maxBounces) + 1.0F);
        }
        app.ocioTransform.setExposureEv(exposureEv);
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

// Called once per rendered frame. Re-traces on any input that would actually change the image -- not a fixed timer -- so the path-traced view stays live without retracing every frame the camera happens to sit still. Because DebugCameraController's fly/orbit controls update every frame a key/mouse-drag is held, this does mean a fresh (progressive-accumulation-reset) request fires on almost every frame for the duration of any camera interaction -- accepted: async execution (PathTraceDriver) keeps that from blocking the UI, it just converges more slowly while the camera is moving, matching how every interactive path tracer (Cycles' viewport, Brigade) behaves.
void requestPathTraceIfTriggerChanged(AppResources& app, const engine::scene::Camera& camera,
                                       int winWidth, int winHeight) {
    const PathTraceTriggerState current{
        camera.position(),       app.debugCamera.yawDegrees(), app.debugCamera.pitchDegrees(),
        app.debugCamera.focalLengthMm(), app.envRotationDegrees, app.showSky, app.envExposureStops,
        winWidth,                winHeight};
    if (current == app.lastPathTraceTrigger) {
        return;
    }
    requestPathTrace(app, camera, winWidth, winHeight);
    app.lastPathTraceTrigger = current;
}

void updateHud(AppResources& app, const engine::platform::Window& window,
               const engine::scene::Camera& camera,
               const engine::scene::PathTraceSnapshot& pathTraceSnapshot, int winWidth,
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
    };
    // Round-tripped through locals so the HUD's sliders can bind plain float&s, same as aov -- DebugCameraController is the authoritative owner, read before draw() and written back after.
    float focalLengthMm = app.debugCamera.focalLengthMm();
    float aperture = app.debugCamera.aperture();
    float shutterSeconds = app.debugCamera.shutterSeconds();
    float iso = app.debugCamera.iso();
    const engine::debug::PixelProbeSample pixelProbe = samplePixelProbe(
        window, pathTraceSnapshot, static_cast<engine::debug::AovId>(app.aov));
    app.hud.draw(hudFrameData, app.aov, focalLengthMm, aperture, shutterSeconds, iso, app.showSky,
                 app.envRotationDegrees, app.envExposureStops, app.framingState, pixelProbe);
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

    requestPathTraceIfTriggerChanged(app, camera, winWidth, winHeight);

    // Held for the rest of this frame so the objects behind it stay valid even if the driver publishes a newer result mid-frame -- shared_ptrs, not a raw fetch. gbuffer/dynamic are two separately-published objects (see PathTraceSnapshot's doc comment); either being null means no pass has completed yet.
    const engine::scene::PathTraceSnapshot pathTraceSnapshot =
        app.pathTraceDriver != nullptr ? app.pathTraceDriver->latestResult()
                                        : engine::scene::PathTraceSnapshot{};

    resolveOrbitPick(window, app, camera, pathTraceSnapshot);

    app.postTimer.begin();
    presentFrame(app, pathTraceSnapshot, winWidth, winHeight);
    app.postTimer.end();

    // Captured after the composited image lands in the default framebuffer, before the HUD draws on top of it.
    app.histogram.update(winWidth, winHeight);

    const auto now = std::chrono::steady_clock::now();
    if (now - app.lastRamSample >= std::chrono::milliseconds(250)) {
        app.ramBytes = engine::debug::residentSetBytes();
        app.systemAvailableBytes = engine::debug::availableSystemBytes();
        app.lastRamSample = now;
    }

    updateHud(app, window, camera, pathTraceSnapshot, winWidth, winHeight);

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
        // Config is pure file I/O with no GL dependency, but profile.json's window size must be known before Window is constructed, so it's loaded first, before any GLFW/GL object exists. All three files hard-fail identically on missing or malformed: this is user-editable input where a load failure is a real, expected-to-happen event, not an internal invariant, so it's surfaced immediately rather than defaulted around -- matching the shader/model/OCIO all-or-nothing gate inside initializeApp.
        std::optional<engine::config::SceneConfig> sceneConfig =
            engine::config::loadSceneConfig(ASSET_ROOT_DIR "/config/scene.json");
        std::optional<engine::config::ProfileConfig> profileConfig =
            engine::config::loadProfileConfig(ASSET_ROOT_DIR "/config/profile.json");
        std::optional<engine::config::MaterialConfig> materialConfig =
            sceneConfig.has_value()
                ? engine::config::loadMaterialConfig(std::string(ASSET_ROOT_DIR) + "/" +
                                                      sceneConfig->materialPath)
                : std::nullopt;

        if (!sceneConfig || !profileConfig || !materialConfig) {
            std::cerr << "main: scene/profile/material config load failed, aborting startup\n";
            exitCode = EXIT_FAILURE;
        } else {
            // Window construction creates the GL 4.1 core/fwd-compat context and makes it current; fatal failure inside it exits the process directly (see window.cpp) since nothing recoverable exists yet.
            engine::platform::Window window(profileConfig->windowWidth, profileConfig->windowHeight,
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
                    initializeApp(*sceneConfig, *profileConfig, *materialConfig, window);
                if (!app) {
                    exitCode = EXIT_FAILURE;
                } else {
                    // Constructed here, not as part of AppResources's designated-initializer list: app (this std::optional<AppResources> local) is where sceneAccel/environmentMap/stumpModel first reach their final, permanent address (initializeApp's own return-type conversion to std::optional<AppResources> move-constructs once en route), so this is the first point at which PathTraceDriver's reference members can safely bind to them -- see path_trace_driver.h's constructor comment.
                    app->pathTraceDriver = std::make_unique<engine::scene::PathTraceDriver>(
                        app->sceneAccel, app->stumpModel.shadingTriangles, app->stumpModel.instances,
                        app->environmentMap);

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
