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
#include "engine/gfx/gl_debug.h"
#include "engine/gfx/hdr_framebuffer.h"
#include "engine/gfx/ocio_display_transform.h"
#include "engine/gfx/post_process_pass.h"
#include "engine/gfx/shader_program.h"
#include "engine/platform/window.h"
#include "engine/scene/camera.h"
#include "engine/scene/debug_camera_controller.h"
#include "engine/scene/frustum.h"
#include "engine/scene/gltf_loader.h"

namespace {

void glfwErrorCallback(int error, const char* description) {
    std::cerr << "GLFW error " << error << ": " << description << '\n';
}

// Transforms a local-space AABB's 8 corners by `transform` and returns the
// resulting world-space AABB -- for frustum culling and the World position
// debug AOV.
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

// Deterministic per-index false color for the Object/Material ID debug
// AOV (golden-ratio fractional hash -- cheap, well-spread across ids).
glm::vec3 falseColorForId(int id) {
    const auto f = static_cast<float>(id);
    return {std::fmod(f * 0.6180339887F, 1.0F), std::fmod((f * 0.3247179572F) + 0.5F, 1.0F),
            std::fmod((f * 0.1231234F) + 0.25F, 1.0F)};
}

const char* lutName(engine::gfx::OcioDisplayTransform::Lut lut) {
    using Lut = engine::gfx::OcioDisplayTransform::Lut;
    return lut == Lut::SRGB ? "sRGB" : lut == Lut::Rec709 ? "Rec709" : "Raw";
}

// Static Gabor kernel weights: 4 orientations (0/45/90/135deg) x 5x5
// taps, precomputed once here rather than in the shader -- these never
// change at runtime, so re-deriving sin/cos/exp per-fragment on the GPU
// would be pure redundant work. Consumed by edge_filter.frag's Gabor
// branch; tap order (dy outer, dx inner, both -2..2) must match its
// sampling loop.
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
                // Odd/quadrature carrier (sin, not cos) -- edge-sensitive,
                // not bar/ridge-sensitive.
                const float carrier = std::sin(2.0F * glm::pi<float>() * xp / kLambda);
                kernel[static_cast<std::size_t>((o * 25) + tapIndex)] = envelope * carrier;
                ++tapIndex;
            }
        }
    }
    return kernel;
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
        // Config is pure file I/O with no GL dependency, but scene.json's
        // window size must be known before Window is constructed, so it's
        // loaded first, before any GLFW/GL object exists. Both files
        // hard-fail identically on missing or malformed: this is
        // user-editable input where a load failure is a real, expected-
        // to-happen event, not an internal invariant, so it's surfaced
        // immediately rather than defaulted around -- matching the
        // shader/model/OCIO all-or-nothing gate a few lines below.
        std::optional<engine::config::SceneConfig> sceneConfig =
            engine::config::loadSceneConfig(ASSET_ROOT_DIR "/config/scene.json");
        std::optional<engine::config::ProfileConfig> profileConfig =
            engine::config::loadProfileConfig(ASSET_ROOT_DIR "/config/profile.json");

        if (!sceneConfig || !profileConfig) {
            std::cerr << "main: scene/profile config load failed, aborting startup\n";
            exitCode = EXIT_FAILURE;
        } else {
            // Window construction creates the GL 4.1 core/fwd-compat context
            // and makes it current; fatal failure inside it exits the process
            // directly (see window.cpp) since nothing recoverable exists yet.
            engine::platform::Window window(sceneConfig->windowWidth, sceneConfig->windowHeight,
                                             "ENGINE");

            glewExperimental = GL_TRUE;
            const GLenum glewStatus = glewInit();
            // GLEW's init is known to leave a spurious error even on success;
            // drain it here so it's never misattributed to a later GL_CALL.
            while (glGetError() != GL_NO_ERROR) {
            }

            if (glewStatus != GLEW_OK) {
                std::cerr << "main: glewInit failed: "
                          << reinterpret_cast<const char*>(glewGetErrorString(glewStatus)) << '\n';
                exitCode = EXIT_FAILURE;
            } else {
                // With heavy scene content, an uncapped CPU submits draw calls
                // faster than the GPU can drain them, growing the driver's
                // command queue unboundedly. Keep vsync on; disable it only
                // for a deliberate, short-lived uncapped-FPS measurement.
                glfwSwapInterval(1);

                std::cout << "GL_KHR_debug available: " << std::boolalpha
                          << engine::gfx::khrDebugAvailable() << '\n';
                std::cout << "GL_ARB_timer_query available: " << std::boolalpha
                          << engine::debug::gpuTimerQueryAvailable() << '\n';

                const engine::debug::GpuInfo gpuInfo = engine::debug::queryGpuInfo();

                // exposure()/ev100() are logged but not render-path-consumed
                // yet: no scene-referred exposure multiply exists, only OCIO's
                // display-encode step (Phase 4+ real lighting is the natural
                // point to seed it from here).
                engine::scene::DebugCameraController debugCamera(
                    profileConfig->position, profileConfig->yawDegrees,
                    profileConfig->pitchDegrees, profileConfig->filmBack,
                    profileConfig->focalLengthMm, profileConfig->nearClip, profileConfig->farClip,
                    profileConfig->aperture, profileConfig->shutterSeconds, profileConfig->iso,
                    profileConfig->flySpeedMetersPerSecond,
                    profileConfig->orbitSensitivityDegPerPixel);
                {
                    const engine::scene::Camera initialCamera = debugCamera.snapshot();
                    const glm::vec3 camPos = initialCamera.position();
                    std::cout << "Camera: position=(" << camPos.x << ", " << camPos.y << ", "
                              << camPos.z << ") verticalFov="
                              << glm::degrees(initialCamera.verticalFovRadians())
                              << " deg ev100=" << initialCamera.ev100()
                              << " exposure=" << initialCamera.exposure() << '\n';
                }

                // tier1 LOD (36.5k triangles): fast iteration for shader work.
                const auto loadStart = std::chrono::steady_clock::now();
                std::optional<engine::scene::LoadedModel> stumpModel = engine::scene::loadGltf(
                    std::string(ASSET_ROOT_DIR) + "/" + sceneConfig->gltfPath);
                const double loadMs = std::chrono::duration<double, std::milli>(
                                           std::chrono::steady_clock::now() - loadStart)
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
                // "Points" (KODAK's term for total vertex-index count,
                // i.e. 3 per triangle, not the unique vertex buffer size)
                // -- always exactly 3x triangles for this triangle-only
                // renderer, so it's derived rather than tracked
                // separately.
                const int totalPoints = totalTriangles * 3;

                const auto [fbWidth, fbHeight] = window.framebufferSize();
                engine::gfx::HdrFramebuffer hdrFbo(fbWidth, fbHeight);

                std::optional<engine::gfx::ShaderProgram> sceneShader =
                    engine::gfx::ShaderProgram::loadFromFiles(ASSET_ROOT_DIR "/shaders/pbr.vert",
                                                               ASSET_ROOT_DIR "/shaders/pbr.frag");
                std::optional<engine::gfx::ShaderProgram> edgeFilterShader =
                    engine::gfx::ShaderProgram::loadFromFiles(
                        ASSET_ROOT_DIR "/shaders/fullscreen_triangle.vert",
                        ASSET_ROOT_DIR "/shaders/edge_filter.frag");
                std::optional<engine::gfx::OcioDisplayTransform> ocioTransform =
                    engine::gfx::OcioDisplayTransform::create();

                if (!sceneShader || !edgeFilterShader || !ocioTransform || !stumpModel) {
                    std::cerr
                        << "main: shader compile/link or model load failed, aborting startup\n";
                    exitCode = EXIT_FAILURE;
                } else {
                    const engine::gfx::PostProcessPass postProcess;
                    engine::debug::HudOverlay hud(window.nativeHandle());
                    engine::debug::FrameStats frameStats;
                    engine::debug::GpuTimer geomTimer;
                    engine::debug::GpuTimer postTimer;
                    engine::debug::Histogram histogram;

                    // One-time texture-unit assignment (units 0-5, see below).
                    // Explicit even though unit 0 is GL's implicit default for
                    // an unset sampler uniform — relying on that default
                    // silently breaks the moment the shader gains a second
                    // sampler. OcioDisplayTransform sets its own uHdrColor
                    // uniform the same way at construction.
                    sceneShader->use();
                    GL_CALL(glUniform1i(sceneShader->uniformLocation("uBaseColor"), 0));
                    GL_CALL(glUniform1i(sceneShader->uniformLocation("uRoughness"), 1));
                    GL_CALL(glUniform1i(sceneShader->uniformLocation("uAo"), 2));
                    GL_CALL(glUniform1i(sceneShader->uniformLocation("uNormal"), 3));
                    GL_CALL(glUniform1i(sceneShader->uniformLocation("uBump"), 4));
                    GL_CALL(glUniform1i(sceneShader->uniformLocation("uSpecular"), 5));
                    // One fixed test light, sourced from scene.json -- no
                    // punctual-light system until Phase 4. Direction points
                    // toward the light; color is pi so a directly-lit
                    // Lambertian surface's brightness matches its albedo
                    // (cancels the shader's /pi).
                    GL_CALL(glUniform3f(sceneShader->uniformLocation("uLightDir"),
                                        sceneConfig->lightDirection.x,
                                        sceneConfig->lightDirection.y,
                                        sceneConfig->lightDirection.z));
                    GL_CALL(glUniform3fv(sceneShader->uniformLocation("uLightColor"), 1,
                                         &sceneConfig->lightColor[0]));
                    // Depth AOV's linearization near/far -- fixed for the
                    // whole run, never mutated by the debug camera (see
                    // profile_config.h).
                    GL_CALL(glUniform1f(sceneShader->uniformLocation("uNearClip"),
                                        profileConfig->nearClip));
                    GL_CALL(glUniform1f(sceneShader->uniformLocation("uFarClip"),
                                        profileConfig->farClip));
                    const int uModelLoc = sceneShader->uniformLocation("uModel");
                    const int uViewLoc = sceneShader->uniformLocation("uView");
                    const int uProjectionLoc = sceneShader->uniformLocation("uProjection");
                    const int uNormalMatrixLoc = sceneShader->uniformLocation("uNormalMatrix");
                    const int uBaseColorFactorLoc =
                        sceneShader->uniformLocation("uBaseColorFactor");
                    const int uMetallicFactorLoc = sceneShader->uniformLocation("uMetallicFactor");
                    const int uRoughnessFactorLoc =
                        sceneShader->uniformLocation("uRoughnessFactor");
                    const int uBoundsMinLoc = sceneShader->uniformLocation("uBoundsMin");
                    const int uBoundsMaxLoc = sceneShader->uniformLocation("uBoundsMax");
                    const int uObjectIdColorLoc = sceneShader->uniformLocation("uObjectIdColor");
                    const int uCameraPosLoc = sceneShader->uniformLocation("uCameraPos");
                    const int uAovLoc = sceneShader->uniformLocation("uAov");
                    const int uChannelViewLoc = sceneShader->uniformLocation("uChannelView");

                    // Sobel/Gabor's second pass (see edge_filter.frag):
                    // uHdrColor's texture unit and the Gabor kernel weights
                    // are both fixed for the whole run, set once here.
                    edgeFilterShader->use();
                    GL_CALL(glUniform1i(edgeFilterShader->uniformLocation("uHdrColor"), 0));
                    const std::array<float, 100> gaborKernel = buildGaborKernel();
                    GL_CALL(glUniform1fv(edgeFilterShader->uniformLocation("uGaborKernel"), 100,
                                         gaborKernel.data()));
                    const int uFilterModeLoc = edgeFilterShader->uniformLocation("uFilterMode");

                    window.setResizeCallback(
                        [&hdrFbo](int width, int height) { hdrFbo.resize(width, height); });

                    // Instance transforms and mesh bounds are fixed after load
                    // (mesh.h: "not updated if the mesh is ever mutated... it
                    // isn't, today") -- computed once here, not per frame.
                    std::vector<std::pair<glm::vec3, glm::vec3>> instanceWorldBounds;
                    instanceWorldBounds.reserve(stumpModel->instances.size());
                    for (const engine::scene::MeshInstance& instance : stumpModel->instances) {
                        instanceWorldBounds.push_back(worldSpaceBounds(
                            instance.mesh.boundsMin(), instance.mesh.boundsMax(),
                            instance.transform));
                    }
                    // aov selects which debug buffer pbr.frag outputs (see its
                    // uAov comment for the index order); channelView isolates
                    // one R/G/B channel of whatever aov currently shows.
                    // userLut is the LUT 'L' cycles through -- kept separate
                    // from OcioDisplayTransform's active LUT because non-Beauty
                    // AOVs force Raw (see the LUT-select comment in the render
                    // loop below) and must not clobber the user's actual choice.
                    // Both aov and userLut start from scene.json rather than a
                    // fixed literal.
                    int aov = sceneConfig->initialAov;
                    int channelView = 0;
                    auto userLut = sceneConfig->initialLut;
                    engine::debug::FramingOverlayState framingState;

                    // Set by the mouse-button callback on an LMB press;
                    // resolved right after this frame's scene draw (not
                    // inline in the callback, which fires during
                    // pollEvents() before the draw -- reading depth there
                    // would read last frame's contents).
                    bool orbitPickRequested = false;
                    double lastCursorX = 0.0;
                    double lastCursorY = 0.0;

                    // Debug-only: 'L' cycles the viewer LUT (sRGB -> Rec709 ->
                    // Raw -> sRGB -> ...), Raw being a genuine no-display-encode
                    // passthrough for direct encoded-vs-unencoded comparison.
                    // '1'/'2'/'3' toggle isolating a channel of the active AOV
                    // (pressing the active one again turns it back off) --
                    // moved off R/G/B in Phase 3 so 'R' is free to reset the
                    // debug camera. 'K' toggles the centre-crosshair framing
                    // overlay. No general input-mapping system for these few
                    // keys is needed: WASD/QE turned out to need continuous
                    // per-frame state (Window::isKeyDown) rather than this
                    // edge-triggered callback, so this single slot still
                    // covers everything that's actually event-shaped.
                    window.setKeyCallback([&userLut, &channelView, &debugCamera,
                                            &framingState](int key, int action) {
                        if (action != GLFW_PRESS) {
                            return;
                        }
                        using Lut = engine::gfx::OcioDisplayTransform::Lut;
                        if (key == GLFW_KEY_L) {
                            userLut = userLut == Lut::SRGB     ? Lut::Rec709
                                      : userLut == Lut::Rec709 ? Lut::Raw
                                                                : Lut::SRGB;
                            std::cout << "OcioDisplayTransform: active LUT = " << lutName(userLut)
                                       << '\n';
                        } else if (key == GLFW_KEY_1) {
                            channelView = channelView == 1 ? 0 : 1;
                        } else if (key == GLFW_KEY_2) {
                            channelView = channelView == 2 ? 0 : 2;
                        } else if (key == GLFW_KEY_3) {
                            channelView = channelView == 3 ? 0 : 3;
                        } else if (key == GLFW_KEY_R) {
                            debugCamera.resetToDefault();
                        } else if (key == GLFW_KEY_K) {
                            framingState.crosshair = !framingState.crosshair;
                        }
                    });

                    // LMB begins/ends an orbit -- gated on the HUD not
                    // wanting the click (dragging a HUD widget shouldn't
                    // also tumble the camera underneath it). The release
                    // always ends an in-progress orbit regardless of where
                    // the cursor ended up, so a drag that finishes over the
                    // HUD still releases cleanly.
                    window.setMouseButtonCallback([&](int button, int action) {
                        if (button != GLFW_MOUSE_BUTTON_LEFT) {
                            return;
                        }
                        if (action == GLFW_PRESS) {
                            if (!hud.wantsCaptureMouse()) {
                                orbitPickRequested = true;
                            }
                        } else if (action == GLFW_RELEASE && debugCamera.isOrbiting()) {
                            debugCamera.endOrbit();
                            window.setCursorLocked(false);
                        }
                    });

                    // poll -> bind HDR FBO -> clear -> draw scene -> post-process
                    // blit (exposure + OCIO display transform) to the default
                    // framebuffer -> swap.
                    // task_info() is a real syscall; the HUD is read by human
                    // eyes, not per-frame logic, so re-sampling RAM 4x/sec
                    // instead of every frame drops one source of frame-time
                    // jitter for free.
                    std::size_t ramBytes = engine::debug::residentSetBytes();
                    std::chrono::steady_clock::time_point lastRamSample =
                        std::chrono::steady_clock::now();
                    std::chrono::steady_clock::time_point lastFrameTime =
                        std::chrono::steady_clock::now();
                    while (!window.shouldClose()) {
                        window.pollEvents();
                        hud.beginFrame();
                        frameStats.tick();

                        const auto frameNow = std::chrono::steady_clock::now();
                        const float dtSeconds =
                            std::chrono::duration<float>(frameNow - lastFrameTime).count();
                        lastFrameTime = frameNow;

                        if (debugCamera.isOrbiting()) {
                            const auto [cursorX, cursorY] = window.cursorPosition();
                            debugCamera.applyOrbitDelta(
                                static_cast<float>(cursorX - lastCursorX),
                                static_cast<float>(cursorY - lastCursorY));
                            lastCursorX = cursorX;
                            lastCursorY = cursorY;
                        } else {
                            debugCamera.applyFlyInput(window, dtSeconds);
                        }
                        const engine::scene::Camera camera = debugCamera.snapshot();

                        const auto [winWidth, winHeight] = window.framebufferSize();
                        const float aspect =
                            static_cast<float>(winWidth) / static_cast<float>(winHeight);
                        const glm::mat4 view = camera.viewMatrix();
                        const glm::mat4 projection = camera.projectionMatrix(aspect);
                        const glm::mat4 viewProjection = projection * view;

                        geomTimer.begin();
                        hdrFbo.bind();
                        glClearColor(0.0F, 0.0F, 0.0F, 1.0F);
                        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

                        // Scoped to just this scene draw: the post-process
                        // pass below presents a fullscreen triangle to the
                        // default framebuffer at a fixed NDC z, so leaving
                        // depth test on for it would make correctness depend
                        // on whatever the driver leaves in that buffer's
                        // depth contents across frames, not on anything this
                        // code controls.
                        glEnable(GL_DEPTH_TEST);
                        sceneShader->use();
                        GL_CALL(glUniformMatrix4fv(uViewLoc, 1, GL_FALSE, &view[0][0]));
                        GL_CALL(
                            glUniformMatrix4fv(uProjectionLoc, 1, GL_FALSE, &projection[0][0]));
                        const glm::vec3 cameraPos = camera.position();
                        GL_CALL(glUniform3fv(uCameraPosLoc, 1, &cameraPos[0]));
                        GL_CALL(glUniform1i(uAovLoc, aov));
                        GL_CALL(glUniform1i(uChannelViewLoc, channelView));

                        int instancesDrawnThisFrame = 0;
                        int instancesCulledThisFrame = 0;
                        long long trianglesDrawnThisFrame = 0;
                        int instanceId = 0;
                        for (const engine::scene::MeshInstance& instance :
                             stumpModel->instances) {
                            const auto& [worldMin, worldMax] = instanceWorldBounds[instanceId];
                            const bool visible = engine::scene::frustumIntersectsAabb(
                                viewProjection, worldMin, worldMax);
                            if (!visible) {
                                ++instancesCulledThisFrame;
                                ++instanceId;
                                continue;
                            }
                            ++instancesDrawnThisFrame;
                            trianglesDrawnThisFrame += instance.mesh.triangleCount();

                            GL_CALL(glUniformMatrix4fv(uModelLoc, 1, GL_FALSE,
                                                        &instance.transform[0][0]));
                            const glm::mat3 normalMatrix =
                                glm::inverseTranspose(glm::mat3(instance.transform));
                            GL_CALL(glUniformMatrix3fv(uNormalMatrixLoc, 1, GL_FALSE,
                                                        &normalMatrix[0][0]));
                            const glm::vec3 baseColorFactor(instance.material.baseColorFactor);
                            GL_CALL(
                                glUniform3fv(uBaseColorFactorLoc, 1, &baseColorFactor[0]));
                            GL_CALL(glUniform1f(uMetallicFactorLoc,
                                                instance.material.metallicFactor));
                            GL_CALL(glUniform1f(uRoughnessFactorLoc,
                                                instance.material.roughnessFactor));
                            GL_CALL(glUniform3fv(uBoundsMinLoc, 1, &worldMin[0]));
                            GL_CALL(glUniform3fv(uBoundsMaxLoc, 1, &worldMax[0]));
                            const glm::vec3 objectIdColor = falseColorForId(instanceId);
                            GL_CALL(glUniform3fv(uObjectIdColorLoc, 1, &objectIdColor[0]));
                            instance.material.baseColorTexture.bind(0);
                            instance.material.roughnessTexture.bind(1);
                            instance.material.aoTexture.bind(2);
                            instance.material.normalTexture.bind(3);
                            instance.material.bumpTexture.bind(4);
                            instance.material.specularTexture.bind(5);
                            instance.mesh.draw();
                            ++instanceId;
                        }
                        glDisable(GL_DEPTH_TEST);
                        geomTimer.end();

                        // Resolved here, not in the mouse callback: this is
                        // the first point after this frame's scene draw
                        // where hdrFbo's depth buffer holds this frame's
                        // contents (the callback fires during pollEvents(),
                        // before the draw, which would read last frame's).
                        if (orbitPickRequested) {
                            orbitPickRequested = false;
                            const float depth =
                                hdrFbo.sampleDepth(winWidth / 2, winHeight / 2);
                            glm::vec3 pivot(0.0F);
                            if (depth < 0.9999F) {
                                const glm::vec4 clip(0.0F, 0.0F, (2.0F * depth) - 1.0F, 1.0F);
                                glm::vec4 world = glm::inverse(viewProjection) * clip;
                                world /= world.w;
                                pivot = glm::vec3(world);
                            } else {
                                pivot = camera.position() + (3.0F * camera.forward());
                            }
                            debugCamera.beginOrbit(pivot);
                            window.setCursorLocked(true);
                            const auto [cursorX, cursorY] = window.cursorPosition();
                            lastCursorX = cursorX;
                            lastCursorY = cursorY;
                        }

                        postTimer.begin();
                        // Debug AOVs (aov != 0) are already display-oriented
                        // (most clamped to [0,1]; unclamped ones like
                        // Luminance just hard-clip to white past 1, same as
                        // Beauty already does in Raw mode), not scene-
                        // referred radiance -- force the Raw passthrough so
                        // the sRGB/Rec709 display curve doesn't distort them,
                        // restoring the user's chosen LUT for Beauty. Still
                        // set even for Sobel/Gabor below, which don't draw
                        // through ocioTransform at all, so the HUD's
                        // LUT-name readout stays accurate.
                        ocioTransform->setActiveLut(
                            aov == 0 ? userLut : engine::gfx::OcioDisplayTransform::Lut::Raw);
                        if (aov == 5 || aov == 6) {
                            // Sobel/Gabor: hdrFbo's color texture holds the
                            // Luminance AOV (pbr.frag's aov==4/5/6 branch)
                            // -- run the edge-filter second pass over it
                            // instead of the OCIO display transform.
                            edgeFilterShader->use();
                            GL_CALL(glUniform1i(uFilterModeLoc, aov == 6 ? 1 : 0));
                            postProcess.draw(hdrFbo.colorTexture(), *edgeFilterShader,
                                              {winWidth, winHeight});
                        } else {
                            ocioTransform->bind();
                            postProcess.draw(hdrFbo.colorTexture(), ocioTransform->activeShader(),
                                              {winWidth, winHeight});
                        }
                        postTimer.end();

                        // After the composited image lands in the default
                        // framebuffer, before the HUD draws on top of it --
                        // matches epochlab/KODAK's capture point exactly.
                        histogram.update(winWidth, winHeight);

                        const auto now = std::chrono::steady_clock::now();
                        if (now - lastRamSample >= std::chrono::milliseconds(250)) {
                            ramBytes = engine::debug::residentSetBytes();
                            lastRamSample = now;
                        }

                        const engine::debug::SceneStats sceneStats{
                            static_cast<int>(stumpModel->instances.size()),
                            instancesDrawnThisFrame,
                            instancesCulledThisFrame,
                            totalTriangles,
                            trianglesDrawnThisFrame,
                            totalPoints,
                            winWidth,
                            winHeight,
                        };
                        const engine::debug::HudFrameData hudFrameData{
                            gpuInfo,
                            frameStats,
                            geomTimer.millisecondsElapsed(),
                            postTimer.millisecondsElapsed(),
                            ramBytes,
                            engine::debug::gpuAllocatedBytes(),
                            channelView,
                            lutName(ocioTransform->activeLut()),
                            sceneStats,
                            camera,
                            debugCamera.yawDegrees(),
                            debugCamera.pitchDegrees(),
                            debugCamera.isOrbiting(),
                            histogram,
                        };
                        // Round-tripped through a local so the HUD's Lens
                        // slider can bind a plain float&, same as aov --
                        // DebugCameraController is the authoritative owner,
                        // read before draw() and written back after.
                        float focalLengthMm = debugCamera.focalLengthMm();
                        hud.draw(hudFrameData, aov, focalLengthMm, framingState);
                        debugCamera.setFocalLengthMm(focalLengthMm);
                        hud.render();

                        window.swapBuffers();
                    }
                }
            }
        }
    }  // Window destroyed here, while GLFW is still initialized.

    glfwTerminate();
    return exitCode;
}
