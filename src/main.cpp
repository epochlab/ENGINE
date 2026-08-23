#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <optional>
#include <utility>

// GLEW before GLFW — see gl_debug.cpp for why.
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_inverse.hpp>

#include "engine/debug/frame_stats.h"
#include "engine/debug/gpu_timer.h"
#include "engine/debug/hud_overlay.h"
#include "engine/debug/memory_tracker.h"
#include "engine/debug/system_info.h"
#include "engine/gfx/gl_debug.h"
#include "engine/gfx/hdr_framebuffer.h"
#include "engine/gfx/ocio_display_transform.h"
#include "engine/gfx/post_process_pass.h"
#include "engine/gfx/shader_program.h"
#include "engine/platform/window.h"
#include "engine/scene/camera.h"
#include "engine/scene/frustum.h"
#include "engine/scene/gltf_loader.h"

namespace {

void glfwErrorCallback(int error, const char* description) {
    std::cerr << "GLFW error " << error << ": " << description << '\n';
}

// Transforms a local-space AABB's 8 corners by `transform` and returns the
// resulting world-space AABB -- for the World position debug AOV.
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

}  // namespace

int main() {
    glfwSetErrorCallback(&glfwErrorCallback);

    if (glfwInit() != GLFW_TRUE) {
        std::cerr << "main: glfwInit failed\n";
        return EXIT_FAILURE;
    }

    int exitCode = EXIT_SUCCESS;
    {
        // Window construction creates the GL 4.1 core/fwd-compat context
        // and makes it current; fatal failure inside it exits the process
        // directly (see window.cpp) since nothing recoverable exists yet.
        engine::platform::Window window(1024, 576, "ENGINE");

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
            const engine::scene::Camera camera(
                glm::vec3(0.0F, 0.0F, 3.0F), 0.0F, 0.0F,
                engine::scene::Camera::FilmBack{36.0F, 24.0F}, 50.0F, 0.1F, 100.0F,
                /*aperture=*/2.8F, /*shutterSeconds=*/1.0F / 125.0F, /*iso=*/100.0F);
            const glm::vec3 camPos = camera.position();
            std::cout << "Camera: position=(" << camPos.x << ", " << camPos.y << ", " << camPos.z
                      << ") verticalFov=" << glm::degrees(camera.verticalFovRadians())
                      << " deg ev100=" << camera.ev100() << " exposure=" << camera.exposure()
                      << '\n';

            // tier1 LOD (36.5k triangles): fast iteration for shader work.
            const auto loadStart = std::chrono::steady_clock::now();
            std::optional<engine::scene::LoadedModel> stumpModel = engine::scene::loadGltf(
                ASSET_ROOT_DIR "/geometry/broken_stump_rkswd_raw/rkswd_tier_1.gltf");
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

            const auto [fbWidth, fbHeight] = window.framebufferSize();
            engine::gfx::HdrFramebuffer hdrFbo(fbWidth, fbHeight);

            std::optional<engine::gfx::ShaderProgram> sceneShader =
                engine::gfx::ShaderProgram::loadFromFiles(ASSET_ROOT_DIR "/shaders/pbr.vert",
                                                           ASSET_ROOT_DIR "/shaders/pbr.frag");
            std::optional<engine::gfx::OcioDisplayTransform> ocioTransform =
                engine::gfx::OcioDisplayTransform::create();

            if (!sceneShader || !ocioTransform || !stumpModel) {
                std::cerr << "main: shader compile/link or model load failed, aborting startup\n";
                exitCode = EXIT_FAILURE;
            } else {
                const engine::gfx::PostProcessPass postProcess;
                engine::debug::HudOverlay hud(window.nativeHandle());
                engine::debug::FrameStats frameStats;
                engine::debug::GpuTimer geomTimer;
                engine::debug::GpuTimer postTimer;

                // One-time texture-unit assignment: uBaseColor/uNormal/
                // uRoughness/uBump/uSpecular/uAo sample from
                // GL_TEXTURE0-5. Explicit even though unit 0 is GL's
                // implicit default for an unset sampler uniform — relying
                // on that default silently breaks the moment the shader
                // gains a second sampler.
                // OcioDisplayTransform sets its own uHdrColor uniform the
                // same way at construction.
                sceneShader->use();
                GL_CALL(glUniform1i(sceneShader->uniformLocation("uBaseColor"), 0));
                GL_CALL(glUniform1i(sceneShader->uniformLocation("uRoughness"), 1));
                GL_CALL(glUniform1i(sceneShader->uniformLocation("uAo"), 2));
                GL_CALL(glUniform1i(sceneShader->uniformLocation("uNormal"), 3));
                GL_CALL(glUniform1i(sceneShader->uniformLocation("uBump"), 4));
                GL_CALL(glUniform1i(sceneShader->uniformLocation("uSpecular"), 5));
                // One fixed test light -- no punctual-light system until
                // Phase 4. Direction points toward the light; color is
                // pi so a directly-lit Lambertian surface's brightness
                // matches its albedo (cancels the shader's /pi).
                GL_CALL(glUniform3f(sceneShader->uniformLocation("uLightDir"), 0.4F, 0.8F, 0.6F));
                const glm::vec3 lightColor(glm::pi<float>());
                GL_CALL(glUniform3fv(sceneShader->uniformLocation("uLightColor"), 1,
                                     &lightColor[0]));
                const int uModelLoc = sceneShader->uniformLocation("uModel");
                const int uViewLoc = sceneShader->uniformLocation("uView");
                const int uProjectionLoc = sceneShader->uniformLocation("uProjection");
                const int uNormalMatrixLoc = sceneShader->uniformLocation("uNormalMatrix");
                const int uMetallicFactorLoc = sceneShader->uniformLocation("uMetallicFactor");
                const int uRoughnessFactorLoc = sceneShader->uniformLocation("uRoughnessFactor");
                const int uBoundsMinLoc = sceneShader->uniformLocation("uBoundsMin");
                const int uBoundsMaxLoc = sceneShader->uniformLocation("uBoundsMax");
                const int uObjectIdColorLoc = sceneShader->uniformLocation("uObjectIdColor");
                const int uCameraPosLoc = sceneShader->uniformLocation("uCameraPos");
                const int uAovLoc = sceneShader->uniformLocation("uAov");
                const int uChannelViewLoc = sceneShader->uniformLocation("uChannelView");

                window.setResizeCallback(
                    [&hdrFbo](int width, int height) { hdrFbo.resize(width, height); });

                // aov selects which debug buffer pbr.frag outputs (see its
                // uAov comment for the index order); channelView isolates
                // one R/G/B channel of whatever aov currently shows.
                // userLut is the LUT 'L' cycles through -- kept separate
                // from OcioDisplayTransform's active LUT because non-Beauty
                // AOVs force Raw (see the LUT-select comment in the render
                // loop below) and must not clobber the user's actual choice.
                int aov = 0;
                int channelView = 0;
                auto userLut = engine::gfx::OcioDisplayTransform::Lut::SRGB;

                // Debug-only: 'L' cycles the viewer LUT (sRGB -> Rec709 ->
                // Raw -> sRGB -> ...), Raw being a genuine no-display-encode
                // passthrough for direct encoded-vs-unencoded comparison.
                // R/G/B toggle isolating that channel of the active AOV
                // (pressing the active one again turns it back off). No
                // general input-mapping system for these few keys — Phase
                // 3's WASD/QE/R debug camera is expected to be the next
                // consumer of Window::setKeyCallback, which only holds one
                // callback at a time, so all keys are handled in this one
                // lambda.
                window.setKeyCallback([&userLut, &channelView](int key, int action) {
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
                    } else if (key == GLFW_KEY_R) {
                        channelView = channelView == 1 ? 0 : 1;
                    } else if (key == GLFW_KEY_G) {
                        channelView = channelView == 2 ? 0 : 2;
                    } else if (key == GLFW_KEY_B) {
                        channelView = channelView == 3 ? 0 : 3;
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
                while (!window.shouldClose()) {
                    window.pollEvents();
                    hud.beginFrame();
                    frameStats.tick();

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
                    GL_CALL(glUniformMatrix4fv(uProjectionLoc, 1, GL_FALSE, &projection[0][0]));
                    const glm::vec3 cameraPos = camera.position();
                    GL_CALL(glUniform3fv(uCameraPosLoc, 1, &cameraPos[0]));
                    GL_CALL(glUniform1i(uAovLoc, aov));
                    GL_CALL(glUniform1i(uChannelViewLoc, channelView));
                    int instanceId = 0;
                    for (const engine::scene::MeshInstance& instance : stumpModel->instances) {
                        const auto [worldMin, worldMax] = worldSpaceBounds(
                            instance.mesh.boundsMin(), instance.mesh.boundsMax(),
                            instance.transform);
                        if (!engine::scene::frustumIntersectsAabb(viewProjection, worldMin,
                                                                   worldMax)) {
                            ++instanceId;
                            continue;
                        }
                        GL_CALL(glUniformMatrix4fv(uModelLoc, 1, GL_FALSE,
                                                    &instance.transform[0][0]));
                        const glm::mat3 normalMatrix =
                            glm::inverseTranspose(glm::mat3(instance.transform));
                        GL_CALL(glUniformMatrix3fv(uNormalMatrixLoc, 1, GL_FALSE,
                                                    &normalMatrix[0][0]));
                        GL_CALL(glUniform1f(uMetallicFactorLoc, instance.material.metallicFactor));
                        GL_CALL(
                            glUniform1f(uRoughnessFactorLoc, instance.material.roughnessFactor));
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

                    postTimer.begin();
                    // Debug AOVs (aov != 0) are already-displayable [0,1]
                    // values, not scene-referred radiance -- force the Raw
                    // passthrough so the sRGB/Rec709 display curve doesn't
                    // distort them, restoring the user's chosen LUT for
                    // Beauty.
                    ocioTransform->setActiveLut(
                        aov == 0 ? userLut : engine::gfx::OcioDisplayTransform::Lut::Raw);
                    ocioTransform->bind();
                    postProcess.draw(hdrFbo.colorTexture(), ocioTransform->activeShader(),
                                      {winWidth, winHeight});
                    postTimer.end();

                    const auto now = std::chrono::steady_clock::now();
                    if (now - lastRamSample >= std::chrono::milliseconds(250)) {
                        ramBytes = engine::debug::residentSetBytes();
                        lastRamSample = now;
                    }

                    hud.draw(gpuInfo, frameStats, geomTimer.millisecondsElapsed(),
                             postTimer.millisecondsElapsed(), totalTriangles,
                             static_cast<long long>(winWidth) * winHeight, ramBytes,
                             engine::debug::gpuAllocatedBytes(), aov, channelView,
                             lutName(ocioTransform->activeLut()));
                    hud.render();

                    window.swapBuffers();
                }
            }
        }
    }  // Window destroyed here, while GLFW is still initialized.

    glfwTerminate();
    return exitCode;
}
