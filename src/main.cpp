#include <chrono>
#include <cstdlib>
#include <iostream>
#include <optional>

// GLEW before GLFW — see gl_debug.cpp for why.
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
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
#include "engine/scene/gltf_loader.h"

namespace {

void glfwErrorCallback(int error, const char* description) {
    std::cerr << "GLFW error " << error << ": " << description << '\n';
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

                // One-time texture-unit assignment: uBaseColor samples
                // from GL_TEXTURE0. Explicit even though unit 0 is GL's
                // implicit default for an unset sampler uniform — relying
                // on that default silently breaks the moment the shader
                // gains a second sampler. OcioDisplayTransform sets its
                // own uHdrColor uniform the same way at construction.
                sceneShader->use();
                GL_CALL(glUniform1i(sceneShader->uniformLocation("uBaseColor"), 0));
                const int uModelLoc = sceneShader->uniformLocation("uModel");
                const int uViewLoc = sceneShader->uniformLocation("uView");
                const int uProjectionLoc = sceneShader->uniformLocation("uProjection");
                const int uNormalMatrixLoc = sceneShader->uniformLocation("uNormalMatrix");

                window.setResizeCallback(
                    [&hdrFbo](int width, int height) { hdrFbo.resize(width, height); });

                // Debug-only: 'L' cycles the active viewer LUT
                // (sRGB -> Rec709 -> Raw -> sRGB -> ...), Raw being a
                // genuine no-display-encode passthrough for direct
                // encoded-vs-unencoded comparison. No general input-mapping
                // system introduced for this one key — Phase 3's WASD/QE/R
                // debug camera is expected to be the second consumer of
                // Window::setKeyCallback.
                window.setKeyCallback([&ocioTransform](int key, int action) {
                    if (key == GLFW_KEY_L && action == GLFW_PRESS) {
                        using Lut = engine::gfx::OcioDisplayTransform::Lut;
                        const Lut current = ocioTransform->activeLut();
                        const Lut next = current == Lut::SRGB     ? Lut::Rec709
                                          : current == Lut::Rec709 ? Lut::Raw
                                                                    : Lut::SRGB;
                        ocioTransform->setActiveLut(next);
                        const char* name =
                            next == Lut::SRGB ? "sRGB" : next == Lut::Rec709 ? "Rec709" : "Raw";
                        std::cout << "OcioDisplayTransform: active LUT = " << name << '\n';
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
                    for (const engine::scene::MeshInstance& instance : stumpModel->instances) {
                        GL_CALL(glUniformMatrix4fv(uModelLoc, 1, GL_FALSE,
                                                    &instance.transform[0][0]));
                        const glm::mat3 normalMatrix =
                            glm::inverseTranspose(glm::mat3(instance.transform));
                        GL_CALL(glUniformMatrix3fv(uNormalMatrixLoc, 1, GL_FALSE,
                                                    &normalMatrix[0][0]));
                        instance.material.baseColorTexture.bind(0);
                        instance.mesh.draw();
                    }
                    glDisable(GL_DEPTH_TEST);
                    geomTimer.end();

                    postTimer.begin();
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
                             engine::debug::gpuAllocatedBytes());
                    hud.render();

                    window.swapBuffers();
                }
            }
        }
    }  // Window destroyed here, while GLFW is still initialized.

    glfwTerminate();
    return exitCode;
}
