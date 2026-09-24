#include <algorithm>
#include <array>
#include <chrono>
#include <filesystem>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

// GLEW before GLFW: see gl_debug.cpp for why.
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <OpenColorIO/OpenColorIO.h>

#include "pathtracer/config/profile_config.h"
#include "pathtracer/config/scene_config.h"
#include "pathtracer/debug/aov.h"
#include "pathtracer/debug/aov_routing.h"
#include "pathtracer/debug/aov_filters.h"
#include "pathtracer/debug/bench_log.h"
#include "pathtracer/debug/colormap.h"
#include "pathtracer/debug/frame_stats.h"
#include "pathtracer/debug/gpu_timer.h"
#include "pathtracer/debug/histogram.h"
#include "pathtracer/debug/perf_dashboard.h"
#include "pathtracer/debug/render_stats.h"
#include "pathtracer/debug/spec_report.h"
#include "pathtracer/debug/hud_overlay.h"
#include "pathtracer/debug/memory_tracker.h"
#include "pathtracer/debug/scene_stats.h"
#include "pathtracer/debug/system_info.h"
#include "pathtracer/gfx/gl_debug.h"
#include "pathtracer/gfx/hdr_image.h"
#include "pathtracer/gfx/ocio_display_transform.h"
#include "pathtracer/gfx/post_process_pass.h"
#include "pathtracer/gfx/shader_program.h"
#include "pathtracer/gfx/texture.h"
#include "pathtracer/platform/display_link.h"
#include "pathtracer/platform/window.h"
#include "pathtracer/scene/camera.h"
#include "pathtracer/scene/debug_camera_controller.h"
#include "pathtracer/scene/embree_accel.h"
#include "pathtracer/scene/environment_map.h"
#include "pathtracer/scene/false_color.h"
#include "pathtracer/scene/gltf_loader.h"
#include "pathtracer/scene/light.h"
#include "pathtracer/scene/material_binding.h"
#include "pathtracer/scene/path_trace_driver.h"
#include "pathtracer/scene/path_tracer.h"
#include "pathtracer/scene/rasterizer.h"
#include "pathtracer/scene/thread_pool.h"

namespace OCIO = OCIO_NAMESPACE;

namespace {

void glfwErrorCallback(int error, const char* description) {
    std::cerr << "GLFW error " << error << ": " << description << '\n';
}

const char* lutName(pathtracer::gfx::OcioDisplayTransform::Lut lut) {
    using Lut = pathtracer::gfx::OcioDisplayTransform::Lut;
    return lut == Lut::SRGB ? "sRGB" : lut == Lut::Rec709 ? "Rec709" : "Raw";
}

// Camera and framebuffer geometry: the whole of what renderRasterGBuffer's output depends on, and the leading part of
// what renderPathTraced's does. Factored rather than duplicated so the two producers compare the same fields.
struct ViewInputState {
    glm::vec3 cameraPosition{0.0F};
    float cameraYawDegrees = 0.0F;
    float cameraPitchDegrees = 0.0F;
    float focalLengthMm = 0.0F;
    // The only FilmBack component that feeds the render (Camera::verticalFovRadians()). widthMm is display-only, so
    // tracking it here would retrace for a change with no visible effect.
    float filmBackHeightMm = 0.0F;
    int fbWidth = 0;
    int fbHeight = 0;

    bool operator==(const ViewInputState&) const = default;
};

// Snapshot of every input renderPathTraced's result depends on except the resolution, compared frame to frame to
// decide whether to hand PathTraceDriver a fresh request. The selected AOV is deliberately not here -- one pass
// writes every lane. See DERIVATIONS.md "Retrace trigger state".
struct PathTraceInputState {
    ViewInputState view;
    int envRotationDegrees = 0;
    bool showSky = false;
    bool envLightEnabled = true;
    float envExposureStops = 0.0F;

    bool operator==(const PathTraceInputState&) const = default;
};

// The inputs plus the scale they are currently rendered at. Split in two because renderScale is derived from whether
// the inputs changed: folding it in would make the settle-time promotion read as fresh interaction next frame.
struct PathTraceTriggerState {
    PathTraceInputState input;
    float renderScale = 0.0F;  // sentinel, never a real value: profile_config.h bounds it to (0,1]

    bool operator==(const PathTraceTriggerState&) const = default;
};

// The rasterizer's own last-rendered state, compared the same way and at the same scale. Separate because the two
// producers refresh independently: an environment change must retrace without re-rasterizing a G-buffer that does
// not depend on it.
struct RasterTriggerState {
    ViewInputState view;
    float renderScale = 0.0F;  // same sentinel, same bound

    bool operator==(const RasterTriggerState&) const = default;
};

// Seconds of no input change before the renderer promotes itself back to full renderScale: long enough that the gaps
// between mouse-drag events during an orbit do not each restart it, short enough to feel immediate when it stops.
constexpr double kInteractiveSettleSeconds = 0.25;

// max(1) so a non-empty framebuffer never scales to a zero-pixel target; a genuinely empty one stays 0 and is skipped
// by the caller's own guard.
int scaledExtent(int framebufferExtent, float scale) {
    if (framebufferExtent <= 0) {
        return 0;
    }
    return std::max(1, static_cast<int>(std::lround(static_cast<float>(framebufferExtent) * scale)));
}

// Everything the render loop touches every frame, plus one-time state that must outlive the run. A pure aggregate, so
// initializeApp returns it by value. -bench appends raw frame and pass columns at exit; stage 0 is the never-measured
// warm-up, so what survives is one fixed workload, the settled full-resolution convergence.
struct BenchCapture {
    std::string logPath;
    std::vector<std::string> argv;
    std::vector<int> aovs;  // the -bench-aovs schedule; empty = single-stage, and `stage` then never leaves 0
    std::size_t stage = 0;  // index into aovs of the AOV currently selected
    std::chrono::steady_clock::time_point stageStart;
    std::vector<float> stageWallMs;  // one entry per measured stage, so stages 1..aovs.size()-1
    std::uint64_t generation = 0;  // requestTrace's generation for the accumulation being captured
    std::vector<pathtracer::debug::PassRecord> passes;
    std::vector<pathtracer::debug::FrameStageTimes> frames;
    std::vector<float> frameMs;
    std::vector<float> presentGpuMs;
    // Per frame, not in config: it is a measurement of the display link's period, so it lands on a different double
    // run to run, and config's contract is exact equality between comparable records.
    std::vector<float> refreshHz;
    std::vector<float> uploadMs;  // one entry per display-texture upload, not per frame
    bool complete = false;

    void restart(std::uint64_t requestGeneration) {
        generation = requestGeneration;
        if (stage > 0) {
            return;  // past the warm-up every restart is part of the measured schedule and its columns are kept
        }
        passes.clear();
        frames.clear();
        frameMs.clear();
        presentGpuMs.clear();
        refreshHz.clear();
        uploadMs.clear();
    }
};

struct AppResources {
    pathtracer::gfx::ShaderProgram edgeFilterShader;
    pathtracer::gfx::ShaderProgram hsvDisplayShader;
    pathtracer::gfx::OcioDisplayTransform ocioTransform;
    pathtracer::scene::EmbreeAccel sceneAccel;     // path tracer scene intersection
    // stumpModel.shadingTriangles indexes sceneAccel's triangles 1:1, no separate field needed.
    pathtracer::scene::EnvironmentMap environmentMap;
    pathtracer::scene::LoadedModel stumpModel;
    // Parallel to stumpModel.instances: -1 for ordinary geometry, else the index into quadLights this instance's two
    // triangles emit as. The scene's own geometry is all -1; appendQuadLights adds one entry per authored light.
    std::vector<int> instanceLightIndex;
    std::vector<pathtracer::scene::QuadLight> quadLights;
    int totalTriangles;
    int totalPoints;

    pathtracer::gfx::PostProcessPass postProcess;
    pathtracer::debug::HudOverlay hud;
    pathtracer::debug::FrameStats frameStats;
    pathtracer::debug::GpuTimer postTimer;
    pathtracer::debug::Histogram histogram;
    // Zeroed at the top of every frame -- see FrameStageTimes.
    pathtracer::debug::FrameStageTimes stages;
    pathtracer::debug::PerfDashboard dashboard;
    // Companion to histogram, computed separately: Histogram bins the post-display-transform, post-8-bit-clamp
    // framebuffer and cannot tell 1.01 from 100.0, both saturating bin 255 identically.
    float overRangeFraction = 0.0F;
    float overRangePeakMultiple = 0.0F;
    pathtracer::scene::DebugCameraController debugCamera;
    pathtracer::debug::GpuInfo gpuInfo;

    int uFilterModeLoc;
    int uEdgeChannelViewLoc;
    int uEdgeExposureLoc;
    int uHsvChannelViewLoc;
    int uHsvExposureLoc;
    int uEdgeInvertLoc;
    int uHsvInvertLoc;

    // HUD-editable UI/run state.
    int aov;
    // Film-back preset catalogue (assets/config/camera.json), loaded once at startup. filmBackPresetNames is
    // index-parallel .c_str() pointers into it, built once for ImGui::Combo, the same shape as aov/kAovNames.
    std::vector<pathtracer::scene::Camera::FilmBackPreset> filmBackPresets;
    std::vector<const char*> filmBackPresetNames;
    int filmBackPresetIndex;
    int channelView;
    pathtracer::gfx::OcioDisplayTransform::Lut userLut;
    pathtracer::debug::FramingOverlayState framingState;
    bool showSky;
    // Whether the environment is in LightSet at all (NEE, MIS, miss radiance), distinct from showSky, which gates
    // only the camera ray's own miss. Off is what makes the classic Goral 1984 Cornell reachable interactively.
    // Initialised from the scene's own environment.lightEnabled.
    bool envLightEnabled;
    int envRotationDegrees;
    float envExposureStops;  // stops, not a multiplier; requestPathTrace does exp2()
    bool invert;   // 1.0 - colour, applied to the final display-referred image -- the 'I' debug toggle
    bool statsEnabled;  // -stats: the live terminal dashboard. The instrumentation behind it always runs; this only gates the drawing.
    // 'H' toggle; gates HudOverlay::draw only -- beginFrame/render stay unconditional so ImGui's frame pairing holds.
    bool showHud;
    bool vsync;  // profile.json: pace each frame to the vblank, or run uncapped
    // Chromatic aberration strength (0 = off), radial UV offset passed to OcioDisplayTransform::setAberration -- HUD slider only.
    float aberrationStrength;
    // The rasterizer runs only on a trigger change into one of its AOVs, so its cost is not a per-frame stage: kept
    // across frames and reported as a last-actual-cost plus duty cycle rather than averaged away.
    float lastRasterMs;

    // Async path-traced view, selected by the `aov` field. The driver runs continuously on its own thread once
    // constructed, and requestTrace() is called only from requestPathTraceIfTriggerChanged -- no manual trigger.
    // unique_ptr, not a by-value optional: see DERIVATIONS.md "Retrace trigger state" for why movability matters.
    pathtracer::scene::PathTraceSettings pathTraceSettings;
    // Per-instance material fields, parallel-indexed with stumpModel.instances, overridden from
    // sceneConfig.materialOverrides by MeshInstance::name where present. Renderer-only fields stay scene-wide.
    std::vector<pathtracer::scene::PathTraceSettings> perInstanceSettings;
    // World-space AABB per instance, parallel-indexed with stumpModel.instances. Static geometry, so computed once
    // at startup and read every rasterizer call for the Wireframe AOV's per-object box edges.
    std::vector<pathtracer::scene::AabbBounds> instanceBounds;
    int maxSamples;  // accumulated-pass cap for PathTraceDriver; 0 = unbounded
    pathtracer::gfx::ScalarType displayFormat;  // pathTraceDisplayTexture's component type (profile.json displayBitDepth)
    pathtracer::gfx::ScalarType textureType;    // scene textures' storage (profile.json textureBitDepth), recorded in -bench configs
    std::unique_ptr<pathtracer::scene::PathTraceDriver> pathTraceDriver;
    std::optional<pathtracer::gfx::Texture> pathTraceDisplayTexture;
    // Which image pathTraceDisplayTexture holds -- the image, not the AovId that selected it, so every AOV reading
    // the same buffer shares one upload. An interior pointer into pathTraceDisplayedOwner, valid while that ref is.
    const pathtracer::gfx::HdrImage* pathTraceDisplayedImage;  // nullptr = nothing uploaded yet
    // Max raw Depth in the last rebuilt pathTraceDisplayTexture; only meaningful when aov==Depth.
    float pathTraceDisplayedDepthMax;
    // Which RasterGBuffer generation the texture holds; 0 when it was built from a PathTraceResult instead.
    std::uint64_t pathTraceDisplayedGeneration;
    // Strong ref -- kept alive, not just an identity pointer -- to whichever published object pathTraceDisplayTexture
    // currently reflects.
    std::shared_ptr<const void> pathTraceDisplayedOwner;
    PathTraceTriggerState lastPathTraceTrigger;  // sentinel-initialized, see its own doc comment
    RasterTriggerState lastRasterTrigger;        // the same, for the rasterizer's independent refresh
    // Render resolution as a fraction of the framebuffer: renderScale once settled, interactiveRenderScale while any
    // input is changing. lastInputChange is the timer the promotion between them is measured against.
    float renderScale;
    float interactiveRenderScale;
    std::chrono::steady_clock::time_point lastInputChange;

    // Synchronous CPU rasterizer for the 14 primary-hit-only G-buffer AOVs, their only producer, decoupled from the
    // driver's async loop. unique_ptr for the same reason as pathTraceDriver: ThreadPool owns worker threads and is
    // neither copyable nor movable.
    std::unique_ptr<pathtracer::scene::ThreadPool> rasterThreadPool;
    // Allocated once and rendered into in place, never republished: its `generation` field, not its address, tells
    // one render from the next. Refreshed synchronously whenever a rasterizer-backed AOV is selected and stale.
    std::shared_ptr<pathtracer::scene::RasterGBuffer> rasterGBuffer;
    // Scene file's basename, for the dashboard's one-line SCENE row -- the full path is in the spec block, and the row has no space for it.
    std::string sceneName;

    // Orbit-pick and RAM-sampling state carried frame to frame.
    bool orbitPickRequested;
    double lastCursorX;
    double lastCursorY;
    std::size_t ramBytes;
    std::size_t bvhBytes;  // Embree's own device accounting, fixed after startup -- see embreeAllocatedBytes
    std::size_t systemAvailableBytes;
    std::uint64_t systemTotalBytes;
    std::chrono::steady_clock::time_point lastRamSample;
    std::chrono::steady_clock::time_point lastFrameTime;
    std::optional<BenchCapture> bench;  // engaged by -bench
    GLsync frameFence;  // the last presented frame's GPU completion, waited on before the next frame begins
    double refreshHz;   // 1 / DisplayLink::refreshPeriodSeconds, refreshed each frame so it follows the window across displays
};

struct RequiredShaders {
    pathtracer::gfx::ShaderProgram edgeFilterShader;
    pathtracer::gfx::ShaderProgram hsvDisplayShader;
    pathtracer::gfx::OcioDisplayTransform ocioTransform;
};

// All shader and OCIO loading in one place so initializeApp has one all-or-nothing check, matching how it already
// treats model and environment loading.
std::optional<RequiredShaders> loadShaders() {
    // HSV/Sobel/Gabor AOV display passes (hsv_display.frag, edge_filter.frag); both run over the path tracer's Beauty
    // image through the shared fullscreen-triangle post-process pass.
    std::optional<pathtracer::gfx::ShaderProgram> edgeFilterShader =
        pathtracer::gfx::ShaderProgram::loadFromFiles(ASSET_ROOT_DIR "/shaders/fullscreen_triangle.vert",
                                                   ASSET_ROOT_DIR "/shaders/edge_filter.frag");
    std::optional<pathtracer::gfx::ShaderProgram> hsvDisplayShader = pathtracer::gfx::ShaderProgram::loadFromFiles(
        ASSET_ROOT_DIR "/shaders/fullscreen_triangle.vert", ASSET_ROOT_DIR "/shaders/hsv_display.frag");
    std::optional<pathtracer::gfx::OcioDisplayTransform> ocioTransform =
        pathtracer::gfx::OcioDisplayTransform::create();

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

// Sobel/Gabor's second pass (edge_filter.frag): uHdrColor's texture unit and the Gabor kernel weights are both fixed
// for the whole run, so they are set once here.
EdgeFilterUniforms setupEdgeFilterShader(const pathtracer::gfx::ShaderProgram& edgeFilterShader) {
    edgeFilterShader.use();
    GL_CALL(glUniform1i(edgeFilterShader.uniformLocation("uHdrColor"), 0));
    const std::array<float, pathtracer::debug::kGaborKernelSize> gaborKernel =
        pathtracer::debug::buildGaborKernel();
    GL_CALL(glUniform1fv(edgeFilterShader.uniformLocation("uGaborKernel"),
                          static_cast<GLsizei>(gaborKernel.size()), gaborKernel.data()));
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
HsvDisplayUniforms setupHsvDisplayShader(const pathtracer::gfx::ShaderProgram& hsvDisplayShader) {
    hsvDisplayShader.use();
    GL_CALL(glUniform1i(hsvDisplayShader.uniformLocation("uHdrColor"), 0));
    return HsvDisplayUniforms{hsvDisplayShader.uniformLocation("uChannelView"),
                               hsvDisplayShader.uniformLocation("uExposure"),
                               hsvDisplayShader.uniformLocation("uInvert")};
}

// All one-time startup work: camera, model, shader and environment loading (nullopt on any failure), the Embree
// scene build, and cached uniform-location lookups. Does not wire input callbacks, which need the returned object.
std::optional<AppResources> initializeApp(const pathtracer::config::SceneConfig& sceneConfig,
                                           const pathtracer::config::ProfileConfig& profileConfig,
                                           const pathtracer::platform::Window& window,
                                           const std::string& scenePath, bool statsEnabled,
                                           double refreshHz) {
    const pathtracer::debug::GpuInfo gpuInfo = pathtracer::debug::queryGpuInfo();

    // Loaded separately from profile.json, not gated on window size, then resolved by name against
    // profileConfig.camera.defaultFilmBackPresetName -- mirroring loadMaterialConfig's standalone-JSON precedent.
    std::optional<std::vector<pathtracer::scene::Camera::FilmBackPreset>> filmBackPresets =
        pathtracer::config::loadFilmBackPresets(ASSET_ROOT_DIR "/config/camera.json");
    if (!filmBackPresets) {
        std::cerr << "main: film back preset load failed, aborting startup\n";
        return std::nullopt;
    }
    const auto filmBackPresetIt =
        std::find_if(filmBackPresets->begin(), filmBackPresets->end(),
                     [&](const pathtracer::scene::Camera::FilmBackPreset& preset) {
                         return preset.name == profileConfig.camera.defaultFilmBackPresetName;
                     });
    if (filmBackPresetIt == filmBackPresets->end()) {
        std::cerr << "main: profile.json filmBackPreset \"" << profileConfig.camera.defaultFilmBackPresetName
                   << "\" not found in camera.json\n";
        return std::nullopt;
    }
    const int filmBackPresetIndex =
        static_cast<int>(std::distance(filmBackPresets->begin(), filmBackPresetIt));
    // Computed before filmBackPresets is moved into AppResources: vector's move transfers buffer ownership, so
    // element addresses and these c_str() pointers stay valid, but building this from a moved-from vector would not.
    std::vector<const char*> filmBackPresetNames;
    filmBackPresetNames.reserve(filmBackPresets->size());
    for (const pathtracer::scene::Camera::FilmBackPreset& preset : *filmBackPresets) {
        filmBackPresetNames.push_back(preset.name.c_str());
    }

    // ev100() is logged, not consumed by the render path: relativeExposureEv() derives the display-stage multiplier
    // from it, not this log line.
    pathtracer::scene::DebugCameraController debugCamera(
        profileConfig.camera.position, profileConfig.camera.yawDegrees,
        profileConfig.camera.pitchDegrees, filmBackPresetIt->filmBack,
        profileConfig.camera.focalLengthMm, profileConfig.camera.nearClip,
        profileConfig.camera.farClip, profileConfig.camera.aperture,
        profileConfig.camera.shutterSeconds, profileConfig.camera.iso,
        profileConfig.controls.flySpeedMetersPerSecond,
        profileConfig.controls.orbitSensitivityDegPerPixel);
    // Scene-level placement (scene.json model.position/model.rotation), order X,Y,Z.
    const glm::mat4 sceneTransform =
        glm::translate(glm::mat4(1.0F), sceneConfig.model.position) *
        glm::rotate(glm::mat4(1.0F), glm::radians(sceneConfig.model.rotation.z), glm::vec3(0.0F, 0.0F, 1.0F)) *
        glm::rotate(glm::mat4(1.0F), glm::radians(sceneConfig.model.rotation.y), glm::vec3(0.0F, 1.0F, 0.0F)) *
        glm::rotate(glm::mat4(1.0F), glm::radians(sceneConfig.model.rotation.x), glm::vec3(1.0F, 0.0F, 0.0F));

    const auto loadStart = std::chrono::steady_clock::now();
    std::optional<pathtracer::scene::LoadedModel> stumpModel = pathtracer::scene::loadGltf(
        std::string(ASSET_ROOT_DIR) + "/" + sceneConfig.model.gltfPath, profileConfig.render.textureType, sceneTransform,
        std::string(ASSET_ROOT_DIR) + "/" + sceneConfig.model.texturePath);
    const double loadMs =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - loadStart)
            .count();
    int totalTriangles = 0;
    std::vector<int> instanceLightIndex;
    std::vector<pathtracer::scene::QuadLight> quadLights;
    if (stumpModel) {
        instanceLightIndex.assign(stumpModel->instances.size(), -1);
        quadLights = pathtracer::scene::buildQuadLights(sceneConfig.lights, sceneTransform);
        pathtracer::scene::appendQuadLights(*stumpModel, quadLights, instanceLightIndex);

        totalTriangles = static_cast<int>(stumpModel->worldTriangles.size());
    }
    // "Points": total vertex-index count, i.e. 3 per triangle -- derived rather than tracked separately.
    const int totalPoints = totalTriangles * 3;

    std::optional<RequiredShaders> shaders = loadShaders();
    // Decoded once here, not through a texture-upload helper: the path tracer is the only consumer and samples this
    // CPU ImageTexture directly, with no GPU upload in between.
    std::optional<pathtracer::gfx::ImageTexture> environmentImage = pathtracer::gfx::loadImageTexture(
        std::string(ASSET_ROOT_DIR) + "/" + sceneConfig.environment.hdriPath, profileConfig.render.textureType);
    std::optional<pathtracer::config::MaterialConfig> materialConfig = pathtracer::config::loadMaterialConfig(
        std::string(ASSET_ROOT_DIR) + "/" + sceneConfig.materialPath);

    if (!shaders || !stumpModel || !environmentImage || !materialConfig) {
        std::cerr << "main: shader compile/link, model load, environment map load, or material "
                     "load failed, aborting startup\n";
        return std::nullopt;
    }

    pathtracer::gfx::PostProcessPass postProcess;
    pathtracer::debug::HudOverlay hud(window.nativeHandle());
    pathtracer::debug::FrameStats frameStats;
    pathtracer::debug::GpuTimer postTimer;
    pathtracer::debug::Histogram histogram;

    pathtracer::scene::EnvironmentMap environmentMap(std::move(*environmentImage));

    const auto accelBuildStart = std::chrono::steady_clock::now();
    std::optional<pathtracer::scene::EmbreeAccel> sceneAccel =
        pathtracer::scene::EmbreeAccel::build(std::move(stumpModel->worldTriangles));
    if (!sceneAccel) {
        std::cerr << "main: Embree scene build failed, aborting startup\n";
        return std::nullopt;
    }
    const double accelBuildMs =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                    accelBuildStart)
            .count();

    const EdgeFilterUniforms edgeFilterUniforms = setupEdgeFilterShader(shaders->edgeFilterShader);
    const HsvDisplayUniforms hsvUniforms = setupHsvDisplayShader(shaders->hsvDisplayShader);

    const pathtracer::scene::PathTraceSettings basePathTraceSettings{
        .samplesPerPixel = profileConfig.pathTracer.samplesPerPixel,
        .maxBounces = profileConfig.pathTracer.maxBounces,
        .russianRouletteStartBounce = profileConfig.pathTracer.russianRouletteStartBounce,
        .aoMaxDistance = profileConfig.pathTracer.aoMaxDistance,
        .lookaheadDistance = profileConfig.pathTracer.lookaheadDistance,
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

    std::optional<std::vector<pathtracer::scene::PathTraceSettings>> perInstanceSettings =
        pathtracer::scene::resolvePerInstanceSettings(basePathTraceSettings, stumpModel->instances,
                                                   sceneConfig.materialOverrides, ASSET_ROOT_DIR);
    if (!perInstanceSettings) {
        std::cerr << "main: material override resolution failed, aborting startup\n";
        return std::nullopt;
    }

    // After appendQuadLights, so the light panels' own instances are bounded too.
    std::vector<pathtracer::scene::AabbBounds> instanceBounds = pathtracer::scene::computeInstanceBounds(
        stumpModel->shadingTriangles, static_cast<int>(stumpModel->instances.size()));

    // Printed at the end of startup rather than at the six points these values become known: every number is real by
    // now, and one contiguous block survives being piped to a log where six scattered lines would not.
    const pathtracer::debug::EngineSpec spec{
        scenePath.c_str(),
        sceneConfig.environment.hdriPath.c_str(),
        pathtracer::debug::kAovNames[profileConfig.render.defaultAov],
        profileConfig.window.width,
        profileConfig.window.height,
        profileConfig.render.renderScale,
        profileConfig.render.interactiveRenderScale,
        basePathTraceSettings.samplesPerPixel,
        basePathTraceSettings.maxBounces,
        basePathTraceSettings.russianRouletteStartBounce,
        profileConfig.pathTracer.maxSamples,
        basePathTraceSettings.aoMaxDistance,
        // Both pools take ThreadPool's default; PathTraceDriver owns its own and is not constructed until main() has
        // AppResources at its final address, so neither can be queried from here.
        pathtracer::scene::ThreadPool::defaultThreadCount(),
        pathtracer::scene::ThreadPool::defaultThreadCount(),
        pathtracer::scene::kPathTraceTileSize,
        static_cast<int>(stumpModel->instances.size()),
        static_cast<int>(quadLights.size()),
        totalTriangles,
        loadMs,
        accelBuildMs,
        pathtracer::scene::embreeAllocatedBytes(),
        pathtracer::gfx::khrDebugAvailable(),
        pathtracer::debug::gpuTimerQueryAvailable(),
        refreshHz,
    };
    pathtracer::debug::printSpec(spec, gpuInfo);

    return AppResources{
        .edgeFilterShader = std::move(shaders->edgeFilterShader),
        .hsvDisplayShader = std::move(shaders->hsvDisplayShader),
        .ocioTransform = std::move(shaders->ocioTransform),
        .sceneAccel = std::move(*sceneAccel),
        .environmentMap = std::move(environmentMap),
        .stumpModel = std::move(*stumpModel),
        .instanceLightIndex = std::move(instanceLightIndex),
        .quadLights = std::move(quadLights),
        .totalTriangles = totalTriangles,
        .totalPoints = totalPoints,
        .postProcess = std::move(postProcess),
        .hud = std::move(hud),
        .frameStats = std::move(frameStats),
        .postTimer = std::move(postTimer),
        .histogram = std::move(histogram),
        .stages = {},
        .dashboard = {},
        .debugCamera = std::move(debugCamera),
        .gpuInfo = gpuInfo,
        .uFilterModeLoc = edgeFilterUniforms.filterMode,
        .uEdgeChannelViewLoc = edgeFilterUniforms.channelView,
        .uEdgeExposureLoc = edgeFilterUniforms.exposure,
        .uHsvChannelViewLoc = hsvUniforms.channelView,
        .uHsvExposureLoc = hsvUniforms.exposure,
        .uEdgeInvertLoc = edgeFilterUniforms.invert,
        .uHsvInvertLoc = hsvUniforms.invert,
        // aov selects which AOV the path tracer's snapshot supplies; channelView isolates one R/G/B channel of it.
        // userLut is the LUT 'L' cycles, kept separate from OcioDisplayTransform's active LUT because non-Beauty AOVs
        // force Raw and must not overwrite the choice.
        .aov = profileConfig.render.defaultAov,
        .filmBackPresets = std::move(*filmBackPresets),
        .filmBackPresetNames = std::move(filmBackPresetNames),
        .filmBackPresetIndex = filmBackPresetIndex,
        .channelView = 0,
        .userLut = profileConfig.render.defaultLut,
        .framingState = pathtracer::debug::FramingOverlayState{},
        // "Show/Hide Background" HDRI-section checkbox -- off by default; only takes visible effect for the Beauty AOV, see presentFrame.
        .showSky = false,
        // The scene's own authored default, not a separate runtime one -- the HUD checkbox edits this in place.
        .envLightEnabled = sceneConfig.environment.lightEnabled,
        // HDR environment's Y-axis rotation, degrees [0,359] -- affects both the background and the environment's
        // contribution to lighting, rotated at query time rather than re-baked.
        .envRotationDegrees = 0,
        // HDRI Exposure slider, stops. requestPathTrace does exp2() -> path_tracer.cpp miss-ray sampleDirection.
        .envExposureStops = 0.0F,
        .invert = false,
        .statsEnabled = statsEnabled,
        .showHud = true,
        .vsync = profileConfig.render.vsync,
        .aberrationStrength = 0.0F,
        .lastRasterMs = 0.0F,
        .pathTraceSettings = basePathTraceSettings,
        .perInstanceSettings = std::move(*perInstanceSettings),
        .instanceBounds = std::move(instanceBounds),
        .maxSamples = profileConfig.pathTracer.maxSamples,
        .displayFormat = profileConfig.render.displayFormat,
        .textureType = profileConfig.render.textureType,
        // Constructed in main() right after initializeApp() returns: its reference members must bind to the scene
        // objects at their final address, which a designated-initializer expression cannot guarantee.
        .pathTraceDriver = nullptr,
        .pathTraceDisplayTexture = std::nullopt,
        .pathTraceDisplayedImage = nullptr,
        .pathTraceDisplayedDepthMax = 0.0F,
        .pathTraceDisplayedGeneration = 0,
        .pathTraceDisplayedOwner = nullptr,
        .lastPathTraceTrigger = PathTraceTriggerState{},
        .lastRasterTrigger = RasterTriggerState{},
        .renderScale = profileConfig.render.renderScale,
        .interactiveRenderScale = profileConfig.render.interactiveRenderScale,
        .lastInputChange = std::chrono::steady_clock::time_point{},
        .rasterThreadPool = std::make_unique<pathtracer::scene::ThreadPool>(),
        .rasterGBuffer = std::make_shared<pathtracer::scene::RasterGBuffer>(),
        .sceneName = std::filesystem::path(scenePath).filename().string(),
        .orbitPickRequested = false,
        .lastCursorX = 0.0,
        .lastCursorY = 0.0,
        // task_info() is a real syscall, and the HUD is read by human eyes rather than per-frame logic, so sampling
        // RAM 4x/sec instead of every frame drops one source of frame-time jitter for free.
        .ramBytes = pathtracer::debug::residentSetBytes(),
        // Embree has finished building by now, so its device counter is final.
        .bvhBytes = pathtracer::scene::embreeAllocatedBytes(),
        .systemAvailableBytes = pathtracer::debug::availableSystemBytes(),
        // Fixed for the machine, unlike the other two -- queried once here rather than resampled alongside them.
        .systemTotalBytes = pathtracer::debug::totalSystemBytes(),
        .lastRamSample = std::chrono::steady_clock::now(),
        .lastFrameTime = std::chrono::steady_clock::now(),
        .bench = std::nullopt,
        .frameFence = nullptr,
        .refreshHz = refreshHz,
    };
}

// Debug-only: 'L' cycles the viewer LUT (sRGB -> Rec709 -> Raw), Raw being a genuine no-display-encode passthrough.
// 'R'/'G'/'B' toggle isolating a channel of the active AOV, pressing the active one again turning it off.
void wireCallbacks(pathtracer::platform::Window& window, AppResources& app) {
    window.setKeyCallback([&app, &window](int key, int action) {
        if (action != GLFW_PRESS) {
            return;
        }
        using Lut = pathtracer::gfx::OcioDisplayTransform::Lut;
        if (key == GLFW_KEY_L) {
            app.userLut = app.userLut == Lut::SRGB     ? Lut::Rec709
                          : app.userLut == Lut::Rec709 ? Lut::Raw
                                                        : Lut::SRGB;
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

    // LMB begins and ends an orbit, gated on the HUD not wanting the click. The release always ends an orbit in
    // progress wherever the cursor ended up, so a drag finishing over the HUD still releases it.
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

pathtracer::scene::Camera updateCamera(const pathtracer::platform::Window& window, AppResources& app,
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
glm::vec3 sampleTexel(const pathtracer::gfx::HdrImage& image, int x, int y) {
    const std::size_t idx = ((static_cast<std::size_t>(y) * static_cast<std::size_t>(image.width)) +
                              static_cast<std::size_t>(x)) *
                             4;
    return {image.rgba[idx + 0], image.rgba[idx + 1], image.rgba[idx + 2]};
}

// Cached CPU OCIO processors for the Beauty pixel probe, built once on first use rather than per probe update:
// CreateFromBuiltinConfig/getProcessor parse the builtin config's colorspace graph, the same one-time cost the GPU
// shaders already pay at startup.
const OCIO::ConstCPUProcessorRcPtr& ocioCpuProcessor(pathtracer::gfx::OcioDisplayTransform::Lut lut) {
    static const OCIO::ConstConfigRcPtr config =
        OCIO::Config::CreateFromBuiltinConfig(pathtracer::gfx::kOcioConfigName);
    static const OCIO::ConstCPUProcessorRcPtr srgbProcessor =
        config
            ->getProcessor(pathtracer::gfx::kOcioSceneColorSpace, pathtracer::gfx::kOcioSrgbDisplay,
                            pathtracer::gfx::kOcioView, OCIO::TRANSFORM_DIR_FORWARD)
            ->getDefaultCPUProcessor();
    static const OCIO::ConstCPUProcessorRcPtr rec709Processor =
        config
            ->getProcessor(pathtracer::gfx::kOcioSceneColorSpace, pathtracer::gfx::kOcioRec709Display,
                            pathtracer::gfx::kOcioView, OCIO::TRANSFORM_DIR_FORWARD)
            ->getDefaultCPUProcessor();
    return lut == pathtracer::gfx::OcioDisplayTransform::Lut::Rec709 ? rec709Processor : srgbProcessor;
}

// Beauty pixel probe's display transform, evaluated for one texel on the CPU rather than the whole framebuffer on the
// GPU: channel isolation, exposure, the OCIO display curve (skipped in Raw mode), then invert -- the same order the
// fragment shaders use.
glm::vec3 applyBeautyDisplayTransform(glm::vec3 hdrColor, const AppResources& app) {
    if (app.channelView == 1) {
        hdrColor = glm::vec3(hdrColor.r);
    } else if (app.channelView == 2) {
        hdrColor = glm::vec3(hdrColor.g);
    } else if (app.channelView == 3) {
        hdrColor = glm::vec3(hdrColor.b);
    }
    glm::vec3 displayColor = hdrColor * std::pow(2.0F, app.debugCamera.relativeExposureEv());
    if (app.userLut != pathtracer::gfx::OcioDisplayTransform::Lut::Raw) {
        std::array<float, 3> pixel{displayColor.r, displayColor.g, displayColor.b};
        ocioCpuProcessor(app.userLut)->applyRGB(pixel.data());
        displayColor = {pixel[0], pixel[1], pixel[2]};
    }
    if (app.invert) {
        displayColor = glm::vec3(1.0F) - displayColor;
    }
    return displayColor;
}

// Orbit pivot picked with a single Embree ray down the view centre. Reading worldPos/alpha at the centre texel of the
// G-buffer instead is what forced that rasterization to run unconditionally -- two million pixels shaded to read one.
void resolveOrbitPick(pathtracer::platform::Window& window, AppResources& app,
                       const pathtracer::scene::Camera& camera) {
    if (!app.orbitPickRequested) {
        return;
    }
    app.orbitPickRequested = false;

    // primaryRay at ndc (0,0) reduces to exactly the camera's forward axis, both ndc terms vanishing, so the centre
    // ray needs no aspect ratio and no projection.
    const pathtracer::scene::Ray ray{camera.position(), camera.forward(), camera.nearClip(),
                                  camera.farClip()};
    const std::optional<pathtracer::scene::Hit> hit = app.sceneAccel.intersect(ray);
    const glm::vec3 pivot =
        hit ? ray.origin + (hit->t * ray.dir) : camera.position() + (3.0F * camera.forward());

    app.debugCamera.beginOrbit(pivot);
    window.setCursorLocked(true);
    const auto [cursorX, cursorY] = window.cursorPosition();
    app.lastCursorX = cursorX;
    app.lastCursorY = cursorY;
}

// Bundles the HdrImage an AOV should display with a type-erased strong ref to whichever published object owns it.
// That ref keeps the owner alive and gives ensurePathTraceDisplayTexture an ABA-safe cache key.
struct PathTracedAovSource {
    const pathtracer::gfx::HdrImage* image = nullptr;
    std::shared_ptr<const void> owner;
    // RasterGBuffer's render counter for the 14 rasterizer-backed AOVs, 0 for path-traced ones. That buffer is reused
    // in place, so its address is constant and `owner` alone can no longer tell one render from the next.
    std::uint64_t generation = 0;
};

// Returns a null image if the source an AOV needs has not published yet, so callers show black. Routed through
// aov_routing.h's lane tables rather than a switch restating them: that switch was a third copy of the mapping, so a
// new AOV had to be added in two places or the viewer silently showed black.
PathTracedAovSource selectPathTracedImage(
    const std::shared_ptr<const pathtracer::scene::PathTraceResult>& snapshot,
    const std::shared_ptr<pathtracer::scene::RasterGBuffer>& rasterGBuffer,
    pathtracer::debug::AovId aov) {
    if (const pathtracer::debug::GBufferLane lane = pathtracer::debug::gbufferLane(aov)) {
        // The buffer is allocated for the process's life now, so a null check no longer distinguishes "no render yet" -- generation 0 does.
        return rasterGBuffer->generation == 0
                   ? PathTracedAovSource{}
                   : PathTracedAovSource{&(*rasterGBuffer.*lane), rasterGBuffer, rasterGBuffer->generation};
    }
    if (const pathtracer::debug::PathTracedLane lane = pathtracer::debug::pathTracedLane(aov)) {
        return snapshot ? PathTracedAovSource{&(*snapshot.*lane), snapshot} : PathTracedAovSource{};
    }
    return {};
}

// Bottom-right HUD probe. The post-filter AOVs are GPU-only shader filters over Beauty with no CPU equivalent, so
// they read back the composited framebuffer texel; every other AOV samples its own raw HdrImage texel in native
// units. The framebuffer path scales by framebufferSize() and flips Y; the raw path needs neither.
pathtracer::debug::PixelProbeSample samplePixelProbe(
    const pathtracer::platform::Window& window,
    const std::shared_ptr<const pathtracer::scene::PathTraceResult>& pathTraceSnapshot,
    const AppResources& app, pathtracer::debug::AovId aovId, float& probeMs) {
    // Timed here rather than at the call site so updateHud stays one screen. For the post-filter AOVs this reads one
    // pixel back from framebuffer 0, a genuine synchronous GPU stall -- the one place the render thread blocks.
    const pathtracer::debug::ScopedCpuTimer probeTimer(probeMs);
    const auto [windowWidth, windowHeight] = window.windowSize();
    if (windowWidth <= 0 || windowHeight <= 0) {
        return {};
    }
    const auto [cursorX, cursorY] = window.cursorPosition();
    if (cursorX < 0.0 || cursorY < 0.0 || cursorX >= windowWidth || cursorY >= windowHeight) {
        return {};
    }

    const bool isPostFilterAov =
        aovId == pathtracer::debug::AovId::HSV || aovId == pathtracer::debug::AovId::Luminance ||
        aovId == pathtracer::debug::AovId::Sobel || aovId == pathtracer::debug::AovId::Gabor;
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
            aovId == pathtracer::debug::AovId::Beauty ? applyBeautyDisplayTransform(texel, app) : texel;
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

// Re-uploads pathTraceDisplayTexture only when the published object owning the image changed -- re-sending 33MB every
// frame for texels the GPU already holds would be work without a reason. Keyed on owner identity, not a raw pointer,
// since a freed address could be reused; channel view is excluded, being a uniform that changes no texels.
void ensurePathTraceDisplayTexture(AppResources& app, const std::shared_ptr<const void>& owner,
                                    const pathtracer::gfx::HdrImage& image, std::uint64_t generation) {
    if (app.pathTraceDisplayTexture.has_value() && app.pathTraceDisplayedImage == &image &&
        app.pathTraceDisplayedOwner == owner && app.pathTraceDisplayedGeneration == generation) {
        return;
    }
    // Started after the cache-key check, never before: on a cache hit this does nothing and must report 0, not the
    // cost of the last real upload.
    const pathtracer::debug::ScopedCpuTimer uploadTimer(app.stages.uploadMs);
    app.stages.uploaded = true;
    if (app.aov == static_cast<int>(pathtracer::debug::AovId::Depth)) {
        float maxDepth = 0.0F;
        for (int i = 0; i < image.width * image.height; ++i) {
            maxDepth = std::max(maxDepth, image.rgba[static_cast<std::size_t>(i) * 4]);
        }
        app.pathTraceDisplayedDepthMax = maxDepth;
    }
    // BounceCount is a mean-termination-depth scalar (R==G==B), not a colour, mapped through Turbo on the CPU before
    // upload rather than as a display-shader uniform. This runs once per rebuilt pass, so it costs nothing per frame.
    if (app.aov == static_cast<int>(pathtracer::debug::AovId::BounceCount)) {
        const float maxBounceCount = static_cast<float>(app.pathTraceSettings.maxBounces) + 1.0F;
        pathtracer::gfx::HdrImage mapped;
        mapped.width = image.width;
        mapped.height = image.height;
        mapped.rgba.resize(image.rgba.size());
        for (int i = 0; i < image.width * image.height; ++i) {
            const std::size_t idx = static_cast<std::size_t>(i) * 4;
            const float t = image.rgba[idx] / maxBounceCount;
            const glm::vec3 mappedColor = pathtracer::debug::turbo(t);
            mapped.rgba[idx + 0] = mappedColor.r;
            mapped.rgba[idx + 1] = mappedColor.g;
            mapped.rgba[idx + 2] = mappedColor.b;
            mapped.rgba[idx + 3] = image.rgba[idx + 3];
        }
        if (app.pathTraceDisplayTexture.has_value()) {
            app.pathTraceDisplayTexture->upload(mapped.width, mapped.height, mapped.rgba.data());
        } else {
            app.pathTraceDisplayTexture = pathtracer::gfx::Texture::createFromFloatPixels(
                mapped.width, mapped.height, mapped.rgba.data(), app.displayFormat);
        }
        app.pathTraceDisplayedImage = &image;
        app.pathTraceDisplayedOwner = owner;
        app.pathTraceDisplayedGeneration = generation;
        return;
    }
    // Uploaded straight from the HdrImage: no row-reversed scratch copy and no texture churn. The vertex shader
    // resolves the row-order convention, and Texture::upload reallocates only if the resolution actually changed.
    if (app.pathTraceDisplayTexture.has_value()) {
        app.pathTraceDisplayTexture->upload(image.width, image.height, image.rgba.data());
    } else {
        app.pathTraceDisplayTexture =
            pathtracer::gfx::Texture::createFromFloatPixels(image.width, image.height, image.rgba.data(),
                                                         app.displayFormat);
    }
    app.pathTraceDisplayedImage = &image;
    app.pathTraceDisplayedOwner = owner;
    app.pathTraceDisplayedGeneration = generation;
}

// Nothing to show yet, or the selected AOV has no buffer: clear the default framebuffer rather than leave stale
// contents on screen.
void clearToBlack(int winWidth, int winHeight) {
    GL_CALL(glBindFramebuffer(GL_FRAMEBUFFER, 0));
    GL_CALL(glViewport(0, 0, winWidth, winHeight));
    GL_CALL(glClearColor(0.0F, 0.0F, 0.0F, 1.0F));
    GL_CALL(glClear(GL_COLOR_BUFFER_BIT));
}

// Blits the selected AOV through the shared OCIO/post-process path. Beauty uses the user's LUT; everything else
// forces Raw, not being scene-referred radiance that a display curve could only distort. Depth additionally gets an
// exposure-based normalization against its own scanned max.
void presentFrame(AppResources& app,
                   const std::shared_ptr<const pathtracer::scene::PathTraceResult>& pathTraceSnapshot,
                   int winWidth, int winHeight) {
    const auto aovId = static_cast<pathtracer::debug::AovId>(app.aov);

    const bool isPostFilterAov =
        aovId == pathtracer::debug::AovId::HSV || aovId == pathtracer::debug::AovId::Luminance ||
        aovId == pathtracer::debug::AovId::Sobel || aovId == pathtracer::debug::AovId::Gabor;
    if (isPostFilterAov) {
        // 2D image filters of the beauty image, not independent per-AOV buffers: all four read path-traced Beauty
        // whichever is selected, so they need a completed pass, unlike the rasterizer-backed AOVs.
        if (!pathTraceSnapshot) {
            clearToBlack(winWidth, winHeight);
            return;
        }
        const bool isHsv = aovId == pathtracer::debug::AovId::HSV;
        ensurePathTraceDisplayTexture(app, pathTraceSnapshot, pathTraceSnapshot->beauty, 0);
        app.ocioTransform.setActiveLut(pathtracer::gfx::OcioDisplayTransform::Lut::Raw);
        // The same multiplier Beauty displays at. These four previously bypassed OCIO entirely and stayed frozen at
        // unity gain regardless of the exposure slider.
        const float exposure = std::pow(2.0F, app.debugCamera.relativeExposureEv());
        if (isHsv) {
            app.hsvDisplayShader.use();
            GL_CALL(glUniform1i(app.uHsvChannelViewLoc, app.channelView));
            GL_CALL(glUniform1f(app.uHsvExposureLoc, exposure));
            GL_CALL(glUniform1i(app.uHsvInvertLoc, app.invert ? 1 : 0));
            // Engaged by the ensurePathTraceDisplayTexture call above, which either uploads into the existing
            // texture or creates one on every path; the analyser cannot carry that through the call.
            // NOLINTNEXTLINE(bugprone-unchecked-optional-access)
            app.postProcess.draw(app.pathTraceDisplayTexture->id(), app.hsvDisplayShader,
                                  {winWidth, winHeight});
        } else {
            app.edgeFilterShader.use();
            const int filterMode = aovId == pathtracer::debug::AovId::Gabor ? 1
                                    : aovId == pathtracer::debug::AovId::Sobel ? 0
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
        const bool isBeauty = aovId == pathtracer::debug::AovId::Beauty;
        app.ocioTransform.setActiveLut(isBeauty ? app.userLut
                                                 : pathtracer::gfx::OcioDisplayTransform::Lut::Raw);
        // Beauty gets photographic exposure; Depth is auto-ranged to the max depth actually in the buffer, farClip
        // being a conservative ray tMax rather than a proxy for scene extent. Needed because the default framebuffer
        // is fixed-point and clamps anything >=1 to white. Everything else passes through unscaled.
        float exposureEv = 0.0F;
        if (isBeauty) {
            exposureEv = app.debugCamera.relativeExposureEv();
        } else if (aovId == pathtracer::debug::AovId::Depth) {
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

// Non-blocking: hands a fresh request to the background PathTraceDriver, which restarts progressive accumulation at
// this pose and size, superseding whatever it was accumulating, and converges on its own thread.
std::uint64_t requestPathTrace(AppResources& app, const pathtracer::scene::Camera& camera, int winWidth,
                               int winHeight) {
    return app.pathTraceDriver->requestTrace(pathtracer::scene::PathTraceDriver::Request{
        camera, winWidth, winHeight, glm::radians(static_cast<float>(app.envRotationDegrees)),
        app.showSky, app.envLightEnabled, std::exp2(app.envExposureStops), app.pathTraceSettings,
        app.maxSamples});
}

// Once per rendered frame. Re-traces on any input that would change the image rather than on a timer, each producer
// on its own inputs. Both run at a fraction of the framebuffer, the display blit upscaling for free. The rasterizer
// runs synchronously -- ~150ms at 2048x1152 -- so it stays gated on one of its own AOVs being selected.
void requestPathTraceIfTriggerChanged(AppResources& app, const pathtracer::scene::Camera& camera,
                                       int fbWidth, int fbHeight,
                                       std::chrono::steady_clock::time_point now) {
    const ViewInputState view{camera.position(),
                               app.debugCamera.yawDegrees(),
                               app.debugCamera.pitchDegrees(),
                               app.debugCamera.focalLengthMm(),
                               app.debugCamera.filmBack().heightMm,
                               fbWidth,
                               fbHeight};
    const PathTraceInputState input{view, app.envRotationDegrees, app.showSky, app.envLightEnabled,
                                     app.envExposureStops};

    // Interaction is a change in anything the image depends on other than its resolution, compared against the inputs
    // alone so the scale promotion cannot re-arm the timer that produced it.
    if (input != app.lastPathTraceTrigger.input) {
        app.lastInputChange = now;
    }
    const bool settled =
        std::chrono::duration<double>(now - app.lastInputChange).count() >= kInteractiveSettleSeconds;
    const float renderScale = settled ? app.renderScale : app.interactiveRenderScale;
    const int renderWidth = scaledExtent(fbWidth, renderScale);
    const int renderHeight = scaledExtent(fbHeight, renderScale);
    const bool needsLightTransport = aovNeedsLightTransport(static_cast<pathtracer::debug::AovId>(app.aov));

    // Park the driver whenever the selected AOV is one it does not produce. Otherwise it keeps accumulating passes of
    // an image no longer on screen, on every core, competing with the rasterizer the render thread runs.
    app.pathTraceDriver->setSuspended(!needsLightTransport);

    const PathTraceTriggerState pathTrace{input, renderScale};
    if (pathTrace != app.lastPathTraceTrigger) {
        const std::uint64_t generation = requestPathTrace(app, camera, renderWidth, renderHeight);
        if (app.bench) {
            app.bench->restart(generation);
        }
        app.lastPathTraceTrigger = pathTrace;
    }

    const RasterTriggerState raster{view, renderScale};
    if (needsLightTransport || raster == app.lastRasterTrigger || renderWidth <= 0 || renderHeight <= 0) {
        return;
    }
    {
        const pathtracer::debug::ScopedCpuTimer rasterTimer(app.stages.rasterMs);
        pathtracer::scene::renderRasterGBuffer(camera, app.stumpModel.shadingTriangles,
                                            app.stumpModel.instances, app.perInstanceSettings,
                                            app.instanceBounds,
                                            renderWidth, renderHeight, *app.rasterThreadPool,
                                            *app.rasterGBuffer);
    }
    app.lastRasterTrigger = raster;
}

// Fraction of texels that would clip at the display encode, plus that peak as a multiple of display range, read from
// the pre-display-transform beauty rather than the framebuffer Histogram reads. The per-texel walk lives on the
// driver thread, leaving exposure to be applied here: exactly for the peak, via the published CDF for the fraction.
void updateOverRangeStats(AppResources& app,
                           const std::shared_ptr<const pathtracer::scene::PathTraceResult>& pathTraceSnapshot) {
    const pathtracer::debug::ScopedCpuTimer overRangeTimer(app.stages.overRangeMs);
    if (!pathTraceSnapshot) {
        app.overRangeFraction = 0.0F;
        app.overRangePeakMultiple = 0.0F;
        return;
    }
    const pathtracer::scene::OverRangeStats& stats = pathTraceSnapshot->overRange;
    const float exposure = std::pow(2.0F, app.debugCamera.relativeExposureEv());
    app.overRangePeakMultiple = exposure * stats.rawPeak;
    // Texels binned strictly above the one holding 1/exposure: an exact count above that bin's upper edge, so the
    // only approximation is the threshold, quantised to at most 1/1024 of a stop.
    const auto bin = static_cast<std::size_t>(pathtracer::scene::overRangeBin(1.0F / exposure));
    const std::uint32_t texelCount = stats.aboveBin.front();
    app.overRangeFraction =
        texelCount > 0 ? static_cast<float>(stats.aboveBin[bin + 1]) / static_cast<float>(texelCount)
                        : 0.0F;
}

void updateHud(AppResources& app, const pathtracer::platform::Window& window,
               const pathtracer::scene::Camera& camera,
               const std::shared_ptr<const pathtracer::scene::PathTraceResult>& pathTraceSnapshot,
               int winWidth,
               int winHeight) {
    const pathtracer::debug::PathTracedStatus pathTracedStatus{
        pathTraceSnapshot != nullptr,
        // PassRecord is the single source of truth for pass timing now; the HUD wants seconds, the record carries milliseconds.
        app.pathTraceDriver != nullptr ? app.pathTraceDriver->lastPassRecord().passMs / 1000.0 : 0.0,
        pathTraceSnapshot != nullptr ? pathTraceSnapshot->samples : 0, app.maxSamples};
    const pathtracer::debug::SceneStats sceneStats{
        static_cast<int>(app.stumpModel.instances.size()),
        app.totalTriangles,
        app.totalPoints,
        winWidth,
        winHeight,
    };
    const pathtracer::debug::HudFrameData hudFrameData{
        app.gpuInfo,
        app.refreshHz,
        app.frameStats,
        app.postTimer.millisecondsElapsed(),
        app.ramBytes,
        pathtracer::debug::gpuAllocatedBytes(),
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
        app.vsync,
    };
    // Round-tripped through locals so the HUD's sliders can bind plain float&s. DebugCameraController is the
    // authoritative owner: read before draw(), written back after.
    float focalLengthMm = app.debugCamera.focalLengthMm();
    float aperture = app.debugCamera.aperture();
    float shutterSeconds = app.debugCamera.shutterSeconds();
    float iso = app.debugCamera.iso();
    int filmBackPresetIndex = app.filmBackPresetIndex;
    const pathtracer::debug::PixelProbeSample pixelProbe =
        samplePixelProbe(window, pathTraceSnapshot, app,
                          static_cast<pathtracer::debug::AovId>(app.aov), app.stages.probeMs);
    const pathtracer::debug::ScopedCpuTimer hudTimer(app.stages.hudMs);
    if (app.showHud) {
        app.hud.draw(hudFrameData, app.aov, focalLengthMm, aperture, shutterSeconds, iso,
                     filmBackPresetIndex, app.filmBackPresetNames, app.showSky, app.envLightEnabled,
                     app.envRotationDegrees, app.envExposureStops, app.aberrationStrength,
                     app.framingState, pixelProbe);
    }
    app.debugCamera.setFocalLengthMm(focalLengthMm);
    app.debugCamera.setAperture(aperture);
    app.debugCamera.setShutterSeconds(shutterSeconds);
    app.debugCamera.setIso(iso);
    app.filmBackPresetIndex = filmBackPresetIndex;
    app.debugCamera.setFilmBack(
        app.filmBackPresets[static_cast<std::size_t>(filmBackPresetIndex)].filmBack);
    {
        const pathtracer::debug::ScopedCpuTimer hudRenderTimer(app.stages.hudRenderMs);
        app.hud.render();
    }
}

// The per-frame read-back steps, after the composited image lands in the default framebuffer and before the HUD draws
// over it: histogram capture, the over-range readout, and the rate-limited RAM resample. Only the histogram actually
// reads the framebuffer.
void sampleDisplayedFrame(AppResources& app,
                           const std::shared_ptr<const pathtracer::scene::PathTraceResult>& pathTraceSnapshot,
                           int winWidth, int winHeight) {
    {
        const pathtracer::debug::ScopedCpuTimer histogramTimer(app.stages.histogramMs);
        app.histogram.update(winWidth, winHeight);
    }
    updateOverRangeStats(app, pathTraceSnapshot);

    // task_info() is a real syscall; nothing per-frame reads these, so 4Hz keeps one source of frame-time jitter out of the loop.
    const auto now = std::chrono::steady_clock::now();
    if (now - app.lastRamSample >= std::chrono::milliseconds(250)) {
        app.ramBytes = pathtracer::debug::residentSetBytes();
        app.systemAvailableBytes = pathtracer::debug::availableSystemBytes();
        app.lastRamSample = now;
    }
}

// Assembles one DashboardFrame and hands it to the dashboard, which decides for itself whether this frame is a
// redraw. Split out of renderFrame so that stays a readable sequence of stages.
void updateDashboard(AppResources& app,
                     const std::shared_ptr<const pathtracer::scene::PathTraceResult>& pathTraceSnapshot, float frameMs,
                     int winWidth, int winHeight) {
    const pathtracer::debug::PassRecord pass =
        app.pathTraceDriver != nullptr ? app.pathTraceDriver->lastPassRecord()
                                        : pathtracer::debug::PassRecord{};
    const bool interactive = app.lastPathTraceTrigger.renderScale == app.interactiveRenderScale;
    const pathtracer::debug::DashboardFrame frame{
        app.frameStats,
        app.stages,
        pass,
        frameMs,
        app.postTimer.millisecondsElapsed(),
        pathTraceSnapshot != nullptr ? pathTraceSnapshot->samples : 0,
        app.maxSamples,
        !aovNeedsLightTransport(static_cast<pathtracer::debug::AovId>(app.aov)),
        app.ramBytes,
        pathtracer::debug::gpuAllocatedBytes(),
        app.bvhBytes,
        app.systemAvailableBytes,
        app.systemTotalBytes,
        pathtracer::debug::kAovNames[app.aov],
        app.sceneName.c_str(),
        static_cast<int>(app.stumpModel.instances.size()),
        static_cast<int>(app.quadLights.size()),
        static_cast<int>(app.totalTriangles),
        winWidth,
        winHeight,
        pass.width,
        pass.height,
        app.lastPathTraceTrigger.renderScale,
        interactive,
        app.refreshHz,
        app.vsync,
    };
    app.dashboard.update(frame);
}

// True once the selected AOV's producer has nothing left to do for it. A rasterizer AOV is finished as soon as a
// G-buffer exists, the rasterizer running synchronously on the frame it is selected, so there is no convergence to
// wait for. A light-transport AOV is finished once the accumulation reaches its target pass count.
bool stageComplete(const AppResources& app, const BenchCapture& bench) {
    if (!aovNeedsLightTransport(static_cast<pathtracer::debug::AovId>(app.aov))) {
        return app.rasterGBuffer->generation != 0;
    }
    return !bench.passes.empty() && bench.passes.back().generation == bench.generation &&
           bench.passes.back().passIndex == app.maxSamples;
}

// Appends this frame's stage times and any newly finished pass of the captured accumulation, then advances the
// -bench-aovs schedule. `pass` was read before this frame's snapshot, since the driver publishes a record before the
// frame that observes it.
void captureBenchFrame(pathtracer::platform::Window& window, AppResources& app, BenchCapture& bench,
                       const pathtracer::debug::PassRecord& pass,
                       const std::shared_ptr<const pathtracer::scene::PathTraceResult>& snapshot, float frameMs,
                       std::chrono::steady_clock::time_point now) {
    const bool ours = pass.generation == bench.generation && !pass.cancelled;
    // Keyed on the generation too, not the index alone: consecutive generations both number passes from 1, so an
    // index-only comparison would drop the first pass of every restart after one cancelled at the same index.
    const bool unseen = bench.passes.empty() || bench.passes.back().generation != pass.generation ||
                        bench.passes.back().passIndex != pass.passIndex;
    if (ours && unseen) {
        bench.passes.push_back(pass);
    }
    bench.frames.push_back(app.stages);
    bench.frameMs.push_back(frameMs);
    bench.presentGpuMs.push_back(app.postTimer.millisecondsElapsed());
    bench.refreshHz.push_back(static_cast<float>(app.refreshHz));
    // Only uploads the displayed AOV's own producer issued: a superseded request's in-flight pass can still publish
    // after a restart, and a rasterizer AOV's upload is never the driver's at all.
    const bool ourUpload = !aovNeedsLightTransport(static_cast<pathtracer::debug::AovId>(app.aov)) ||
                           (snapshot != nullptr && snapshot->generation == bench.generation);
    if (app.stages.uploaded && ourUpload) {
        bench.uploadMs.push_back(app.stages.uploadMs);
    }

    if (!stageComplete(app, bench)) {
        return;
    }
    if (bench.stage > 0) {
        bench.stageWallMs.push_back(std::chrono::duration<float, std::milli>(now - bench.stageStart).count());
    }
    // An empty schedule leaves stage at 0 and ends here, which is the single-stage capture unchanged.
    if (bench.stage + 1 >= bench.aovs.size()) {
        bench.complete = true;
        window.setShouldClose(true);
        return;
    }
    bench.stageStart = now;
    ++bench.stage;
    app.aov = bench.aovs[bench.stage];
}

// Everything the captured workload's cost depends on; two engine records are comparable iff these are equal.
nlohmann::json benchConfig(const AppResources& app, const BenchCapture& bench) {
    const pathtracer::scene::Camera camera = app.debugCamera.snapshot();
    std::vector<std::string> schedule;
    schedule.reserve(bench.aovs.size());
    for (const int aov : bench.aovs) {
        schedule.emplace_back(pathtracer::debug::kAovNames[aov]);
    }
    nlohmann::json config = {{"scene", app.sceneName},
            {"width", bench.passes.back().width},
            {"height", bench.passes.back().height},
            {"max_samples", app.maxSamples},
            {"spp_per_pass", app.pathTraceSettings.samplesPerPixel},
            {"max_bounces", app.pathTraceSettings.maxBounces},
            {"rr_start_bounce", app.pathTraceSettings.russianRouletteStartBounce},
            {"ao_max_distance", app.pathTraceSettings.aoMaxDistance},
            {"aov", pathtracer::debug::kAovNames[app.aov]},
            {"camera", {{"position", {camera.position().x, camera.position().y, camera.position().z}},
                        {"yaw", app.debugCamera.yawDegrees()},
                        {"pitch", app.debugCamera.pitchDegrees()},
                        {"focal_mm", app.debugCamera.focalLengthMm()},
                        {"film_height_mm", app.debugCamera.filmBack().heightMm}}},
            {"env", {{"rotation_deg", app.envRotationDegrees}, {"exposure_stops", app.envExposureStops},
                     {"light", app.envLightEnabled}, {"show_sky", app.showSky}}},
            {"hud", app.showHud},
            {"vsync", app.vsync},
            {"display_type", pathtracer::gfx::scalarTypeName(app.displayFormat)},
            {"texture_type", pathtracer::gfx::scalarTypeName(app.textureType)}};
    // Only with -bench-aovs, so a single-stage record stays comparable with every one logged before this existed.
    // The whole switch sequence is the workload, and bench_compare refuses to pair records whose configs differ.
    if (!schedule.empty()) {
        config["aov_schedule"] = schedule;
    }
    return config;
}

// Raw columns, one entry per event of their own: per captured frame for render-thread stages, per upload for
// upload_ms, per pass for driver phases.
nlohmann::json benchSamples(const BenchCapture& bench) {
    const auto frameColumn = [&](float pathtracer::debug::FrameStageTimes::*stage) {
        std::vector<float> column;
        column.reserve(bench.frames.size());
        for (const pathtracer::debug::FrameStageTimes& frame : bench.frames) {
            column.push_back(frame.*stage);
        }
        return column;
    };
    const auto passColumn = [&](double pathtracer::debug::PassRecord::*phase) {
        std::vector<double> column;
        column.reserve(bench.passes.size());
        for (const pathtracer::debug::PassRecord& pass : bench.passes) {
            column.push_back(pass.*phase);
        }
        return column;
    };
    using Stages = pathtracer::debug::FrameStageTimes;
    using Pass = pathtracer::debug::PassRecord;
    nlohmann::json samples = {{"frame_ms", bench.frameMs},
            {"fence_ms", frameColumn(&Stages::fenceMs)},
            {"pace_ms", frameColumn(&Stages::paceMs)},
            {"poll_ms", frameColumn(&Stages::pollMs)},
            {"camera_ms", frameColumn(&Stages::cameraMs)},
            {"raster_ms", frameColumn(&Stages::rasterMs)},
            {"upload_ms", bench.uploadMs},
            {"present_ms", frameColumn(&Stages::presentMs)},
            {"present_gpu_ms", bench.presentGpuMs},
            {"refresh_hz", bench.refreshHz},
            {"histogram_ms", frameColumn(&Stages::histogramMs)},
            {"over_range_ms", frameColumn(&Stages::overRangeMs)},
            {"probe_ms", frameColumn(&Stages::probeMs)},
            {"hud_ms", frameColumn(&Stages::hudMs)},
            {"hud_render_ms", frameColumn(&Stages::hudRenderMs)},
            {"swap_ms", frameColumn(&Stages::swapMs)},
            {"pass_trace_ms", passColumn(&Pass::traceMs)},
            {"pass_accumulate_ms", passColumn(&Pass::accumulateMs)},
            {"pass_over_range_ms", passColumn(&Pass::overRangeMs)},
            {"pass_publish_ms", passColumn(&Pass::publishMs)},
            {"pass_ms", passColumn(&Pass::passMs)}};
    // Absent without -bench-aovs, so a single-stage record keeps exactly the columns it has always had.
    if (!bench.stageWallMs.empty()) {
        samples["stage_wall_ms"] = bench.stageWallMs;
    }
    return samples;
}

// Writes the captured accumulation as one benchmark-log record. Refuses an incomplete capture -- closed early, or a
// pass whose record was overwritten before a frame read it -- rather than logging a workload that differs from its
// config.
bool finishBench(const AppResources& app, const BenchCapture& bench) {
    if (!bench.complete) {
        std::cerr << "pathtracer: -bench closed before the schedule finished; nothing logged\n";
        return false;
    }
    // Contiguous from 1 within each generation rather than across the column: a schedule spans several accumulations
    // and each restart numbers its own passes from 1. A gap still means a PassRecord was overwritten unread.
    std::uint64_t generation = 0;
    int expected = 0;
    for (const pathtracer::debug::PassRecord& pass : bench.passes) {
        if (pass.generation != generation) {
            generation = pass.generation;
            expected = 0;
        }
        if (pass.passIndex != ++expected) {
            std::cerr << "pathtracer: -bench missed a pass record of generation " << generation << " (two passes finished within one frame); nothing logged\n";
            return false;
        }
    }
    pathtracer::debug::RayCounts rays;
    for (const pathtracer::debug::PassRecord& pass : bench.passes) {
        rays.add(pass.rays);
    }
    const pathtracer::debug::BenchRecord record{
        .tool = "pathtracer",
        .argv = bench.argv,
        .config = benchConfig(app, bench),
        .samples = benchSamples(bench),
        .work = {{"rays", {{"primary", rays.primary}, {"bounce", rays.bounce}, {"ao", rays.ao}, {"shadow", rays.shadow}}},
                 {"crc32", pathtracer::debug::floatCrc32(app.pathTraceDriver->latestResult()->beauty.rgba)}},
    };
    return pathtracer::debug::appendBenchRecord(bench.logPath, record);
}

// Bounds GPU work to one frame in flight: at swap interval 0 flushBuffer waits on neither the vblank nor the GPU,
// only on a synchronous WindowServer query, so nothing else stops the command queue outgrowing the GPU.
void waitForPreviousFrame(AppResources& app) {
    if (app.frameFence == nullptr) {
        return;
    }
    GL_CALL(glClientWaitSync(app.frameFence, GL_SYNC_FLUSH_COMMANDS_BIT, std::numeric_limits<GLuint64>::max()));
    GL_CALL(glDeleteSync(app.frameFence));
    app.frameFence = nullptr;
}

// One frame: vblank -> previous frame's GPU completion -> poll -> update camera -> request a fresh path trace if
// input changed -> orbit-pick -> post-process blit to the default framebuffer -> swap.
void renderFrame(pathtracer::platform::Window& window, pathtracer::platform::DisplayLink& displayLink, AppResources& app) {
    // Every stage zeroed first: one that does not run this frame must read 0, or the dashboard reports the last time
    // it did run as if it were still happening.
    app.stages = {};
    {
        // Before the poll, so input is sampled right after the vblank this frame latches against. Timed either way,
        // so an uncapped frame logs its pace as 0 rather than as absent.
        const pathtracer::debug::ScopedCpuTimer paceTimer(app.stages.paceMs);
        if (app.vsync) {
            displayLink.waitForNextVblank();
        }
    }
    {
        // After the vblank wait, which the GPU drains the previous frame during, so this is non-zero only for a GPU-bound frame.
        const pathtracer::debug::ScopedCpuTimer fenceTimer(app.stages.fenceMs);
        waitForPreviousFrame(app);
    }
    app.refreshHz = 1.0 / displayLink.refreshPeriodSeconds();
    {
        const pathtracer::debug::ScopedCpuTimer pollTimer(app.stages.pollMs);
        window.pollEvents();
        app.hud.beginFrame();
    }
    app.frameStats.tick();

    const auto frameNow = std::chrono::steady_clock::now();
    const float dtSeconds = std::chrono::duration<float>(frameNow - app.lastFrameTime).count();
    app.lastFrameTime = frameNow;

    const pathtracer::scene::Camera camera = [&] {
        const pathtracer::debug::ScopedCpuTimer cameraTimer(app.stages.cameraMs);
        return updateCamera(window, app, dtSeconds);
    }();
    const auto [winWidth, winHeight] = window.framebufferSize();

    requestPathTraceIfTriggerChanged(app, camera, winWidth, winHeight, frameNow);

    // Read before the snapshot so a final record guarantees the snapshot holds the final image -- see captureBenchFrame.
    const pathtracer::debug::PassRecord benchPass =
        app.bench ? app.pathTraceDriver->lastPassRecord() : pathtracer::debug::PassRecord{};
    // Held for the rest of this frame so the images stay valid even if the driver publishes mid-frame -- a strong
    // ref, not a raw fetch. Null until the first pass of the app's life completes.
    const std::shared_ptr<const pathtracer::scene::PathTraceResult> pathTraceSnapshot =
        app.pathTraceDriver != nullptr ? app.pathTraceDriver->latestResult() : nullptr;

    resolveOrbitPick(window, app, camera);

    app.postTimer.begin();
    {
        // Inclusive of the display-texture upload inside it; the blit's own cost is the difference, which the
        // dashboard subtracts rather than measuring twice.
        const pathtracer::debug::ScopedCpuTimer presentTimer(app.stages.presentMs);
        presentFrame(app, pathTraceSnapshot, winWidth, winHeight);
    }
    app.postTimer.end();

    // Captured after the composited image lands in the default framebuffer, before the HUD draws on top of it.
    sampleDisplayedFrame(app, pathTraceSnapshot, winWidth, winHeight);

    updateHud(app, window, camera, pathTraceSnapshot, winWidth, winHeight);

    {
        const pathtracer::debug::ScopedCpuTimer swapTimer(app.stages.swapMs);
        window.swapBuffers();
    }
    GL_CALL(app.frameFence = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0));

    if (app.statsEnabled) {
        // After swapBuffers, so the dashboard's write(2) lands in the frame's slack rather than ahead of the present.
        // It times its own draw internally: a timer here could never be observed, the stages being zeroed first.
        updateDashboard(app, pathTraceSnapshot, dtSeconds * 1000.0F, winWidth, winHeight);
    }
    if (app.bench) {
        captureBenchFrame(window, app, *app.bench, benchPass, pathTraceSnapshot, dtSeconds * 1000.0F, frameNow);
    }
}

struct Options {
    std::string scenePath = ASSET_ROOT_DIR "/scenes/cornell.json";
    // Off by default: the live dashboard redraws in place, right for a session a human watches and wrong for anything
    // scripted. The instrumentation is always compiled in, so -stats measures the binary that ships.
    bool stats = false;
    // -bench PATH: run one accumulation to profile.json's maxSamples, append it to this benchmark log (bench_log.h), exit.
    std::string benchLogPath;
    // -bench-aovs A,B,C: AovId sequence the run walks, one stage per entry. Empty = today's single-stage capture.
    std::vector<int> benchAovs;
};

// Resolves a comma-separated AOV list against kAovNames, so -bench-aovs and the HUD dropdown name the same 28 AOVs
// identically. nullopt on an unknown name, which parseOptions surfaces rather than defaulting around.
std::optional<std::vector<int>> parseAovList(const char* list) {
    std::vector<int> aovs;
    const std::string text(list);
    for (std::size_t begin = 0; begin <= text.size();) {
        const std::size_t comma = text.find(',', begin);
        const std::string name = text.substr(begin, comma - begin);
        const auto* match = std::find_if(std::begin(pathtracer::debug::kAovNames), std::end(pathtracer::debug::kAovNames),
                                          [&name](const char* candidate) { return name == candidate; });
        if (match == std::end(pathtracer::debug::kAovNames)) {
            std::cerr << "pathtracer: -bench-aovs has no AOV named '" << name << "'; valid names are";
            for (const char* candidate : pathtracer::debug::kAovNames) {
                std::cerr << " '" << candidate << "'";
            }
            std::cerr << '\n';
            return std::nullopt;
        }
        aovs.push_back(static_cast<int>(match - std::begin(pathtracer::debug::kAovNames)));
        if (comma == std::string::npos) {
            break;
        }
        begin = comma + 1;
    }
    return aovs;
}

// Returns nullopt on an unrecognized flag or a missing value -- argv is a system
// boundary, so a bad value is surfaced rather than defaulted around.
std::optional<Options> parseOptions(int argc, char** argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "-scene") == 0) {
            if (i + 1 >= argc) {
                std::cerr << "pathtracer: -scene expects a value\n";
                return std::nullopt;
            }
            options.scenePath = argv[++i];
        } else if (std::strcmp(argv[i], "-stats") == 0) {
            options.stats = true;
        } else if (std::strcmp(argv[i], "-bench") == 0) {
            if (i + 1 >= argc) {
                std::cerr << "pathtracer: -bench expects a value\n";
                return std::nullopt;
            }
            options.benchLogPath = argv[++i];
        } else if (std::strcmp(argv[i], "-bench-aovs") == 0) {
            if (i + 1 >= argc) {
                std::cerr << "pathtracer: -bench-aovs expects a value\n";
                return std::nullopt;
            }
            std::optional<std::vector<int>> aovs = parseAovList(argv[++i]);
            if (!aovs) {
                return std::nullopt;
            }
            options.benchAovs = std::move(*aovs);
        } else {
            std::cerr << "pathtracer: unknown flag " << argv[i]
                       << "\n  usage: pathtracer [-scene path/to/scene.json] [-stats] [-bench log.jsonl] [-bench-aovs Beauty,Normal,...]\n";
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
        // Config is pure file I/O, but profile.json's window size must be known before Window is constructed, so it
        // loads first, before any GL object exists. Both files hard-fail identically on missing or malformed input.
        std::optional<pathtracer::config::SceneConfig> sceneConfig =
            pathtracer::config::loadSceneConfig(options->scenePath);
        std::optional<pathtracer::config::ProfileConfig> profileConfig =
            pathtracer::config::loadProfileConfig(ASSET_ROOT_DIR "/config/profile.json");

        if (!sceneConfig || !profileConfig) {
            std::cerr << "main: scene/profile config load failed, aborting startup\n";
            exitCode = EXIT_FAILURE;
        } else if (options->benchLogPath.empty() && !options->benchAovs.empty()) {
            std::cerr << "main: -bench-aovs is the schedule -bench walks; it does nothing on its own\n";
            exitCode = EXIT_FAILURE;
        } else if (!options->benchLogPath.empty() &&
                   (profileConfig->pathTracer.maxSamples <= 0 ||
                    !aovNeedsLightTransport(static_cast<pathtracer::debug::AovId>(
                        options->benchAovs.empty() ? profileConfig->render.defaultAov
                                                   : options->benchAovs.front())))) {
            // An unbounded accumulation never ends and a rasterizer AOV parks the driver, so neither is a benchmark
            // workload. The first stage is the warm-up every later one is measured from, so it has to converge.
            std::cerr << "main: -bench needs profile.json maxSamples > 0 and a path-traced first AOV\n";
            exitCode = EXIT_FAILURE;
        } else {
            // Window construction creates the GL 4.1 core forward-compatible context and makes it current; a fatal
            // failure inside it exits the process directly, nothing recoverable existing yet.
            pathtracer::platform::Window window(profileConfig->window.width, profileConfig->window.height,
                                             "PATHTRACER");

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
                // Off: NSGL's interval lets two swaps through per refresh on current macOS, and while it is non-zero
                // GLFW paces an occluded window with a fixed 60 Hz usleep. DisplayLink paces to the real vblank.
                glfwSwapInterval(0);
                pathtracer::platform::DisplayLink displayLink(window);

                std::optional<AppResources> app =
                    initializeApp(*sceneConfig, *profileConfig, window, options->scenePath,
                                   options->stats, 1.0 / displayLink.refreshPeriodSeconds());
                if (!app) {
                    exitCode = EXIT_FAILURE;
                } else {
                    // Constructed here, not in the designated-initializer list: this local is where the scene objects
                    // first reach their final address, which the driver's reference members require.
                    app->pathTraceDriver = std::make_unique<pathtracer::scene::PathTraceDriver>(
                        app->sceneAccel, app->stumpModel.shadingTriangles, app->stumpModel.instances,
                        app->instanceLightIndex, app->environmentMap, app->quadLights,
                        app->perInstanceSettings);

                    wireCallbacks(window, *app);
                    if (!options->benchLogPath.empty()) {
                        app->bench.emplace();
                        app->bench->logPath = options->benchLogPath;
                        app->bench->argv.assign(argv, argv + argc);
                        app->bench->aovs = options->benchAovs;
                        if (!app->bench->aovs.empty()) {
                            app->aov = app->bench->aovs.front();
                        }
                    }

                    while (!window.shouldClose()) {
                        renderFrame(window, displayLink, *app);
                    }
                    GL_CALL(glDeleteSync(app->frameFence));
                    if (app->bench && !finishBench(*app, *app->bench)) {
                        exitCode = EXIT_FAILURE;
                    }
                }
            }
        }
    }  // Window destroyed here, while GLFW is still initialized.

    glfwTerminate();
    return exitCode;
}
