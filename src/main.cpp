#include <cstdlib>
#include <iostream>
#include <optional>

// GLEW before GLFW — see gl_debug.cpp for why.
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>

#include "engine/gfx/gl_debug.h"
#include "engine/gfx/hdr_framebuffer.h"
#include "engine/gfx/mesh.h"
#include "engine/gfx/ocio_display_transform.h"
#include "engine/gfx/post_process_pass.h"
#include "engine/gfx/shader_program.h"
#include "engine/gfx/texture.h"
#include "engine/platform/window.h"
#include "engine/scene/camera.h"

namespace {

void glfwErrorCallback(int error, const char* description) {
    std::cerr << "GLFW error " << error << ": " << description << '\n';
}

// Stage G verification: reads back test_pattern.exr's known patch centers
// from the backbuffer and logs actual vs. hand-computed expected bytes.
// Coordinates derive from the current framebuffer size and the quad's
// fixed [-0.5,0.5] NDC footprint, so this stays correct after a resize.
void logColorCheck(int fbWidth, int fbHeight, engine::gfx::OcioDisplayTransform::Lut lut) {
    using Lut = engine::gfx::OcioDisplayTransform::Lut;
    const char* lutName = lut == Lut::SRGB ? "sRGB" : lut == Lut::Rec709 ? "Rec709" : "Raw";
    const int expectedGrey = lut == Lut::SRGB ? 118 : lut == Lut::Rec709 ? 125 : 46;

    const int quadLeft = fbWidth / 4;
    const int quadWidth = fbWidth / 2;
    const int y = fbHeight / 2;

    // test_pattern.exr is 700px wide; u is the fraction across it.
    auto sampleAt = [&](float textureX) {
        unsigned char px[3] = {0, 0, 0};
        const int x = quadLeft + static_cast<int>(textureX / 700.0F * static_cast<float>(quadWidth));
        glReadPixels(x, y, 1, 1, GL_RGB, GL_UNSIGNED_BYTE, px);
        return static_cast<int>(px[0]);  // patches are achromatic/primary; R alone suffices
    };

    std::cout << "ColorCheck[" << lutName << "]: black=" << sampleAt(50.0F) << " (expect 0), grey="
              << sampleAt(150.0F) << " (expect ~" << expectedGrey
              << "), white=" << sampleAt(250.0F) << " (expect 255), ramp =";
    for (int i = 0; i < 5; ++i) {
        std::cout << ' ' << sampleAt(600.0F + static_cast<float>(i) * 99.0F / 4.0F);
    }
    std::cout << " (expect non-decreasing)\n";
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
        engine::platform::Window window(1280, 720, "Engine");

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
            glfwSwapInterval(1);

            std::cout << "GL_KHR_debug available: " << std::boolalpha
                      << engine::gfx::khrDebugAvailable() << '\n';

            // Verification-only: viewMatrix()/projectionMatrix() have no
            // render-path consumer yet (Stage D's quad has no MVP) — first
            // real consumer is Phase 1/2. exposure()/ev100() are logged
            // below but not render-path-consumed either yet; see the
            // exposure comment further down for why.
            const engine::scene::Camera camera(
                glm::vec3(0.0F, 0.0F, 3.0F), 0.0F, 0.0F,
                engine::scene::Camera::FilmBack{36.0F, 24.0F}, 50.0F, 0.1F, 100.0F,
                /*aperture=*/2.8F, /*shutterSeconds=*/1.0F / 125.0F, /*iso=*/100.0F);
            const glm::vec3 camPos = camera.position();
            std::cout << "Camera: position=(" << camPos.x << ", " << camPos.y << ", " << camPos.z
                      << ") verticalFov=" << glm::degrees(camera.verticalFovRadians())
                      << " deg ev100=" << camera.ev100() << " exposure=" << camera.exposure()
                      << '\n';

            const auto [fbWidth, fbHeight] = window.framebufferSize();
            engine::gfx::HdrFramebuffer hdrFbo(fbWidth, fbHeight);

            const engine::gfx::Mesh quad = engine::gfx::Mesh::createQuad();

            std::optional<engine::gfx::Texture> testPattern =
                engine::gfx::Texture::createFromExr(ASSET_ROOT_DIR "/textures/test_pattern.exr");

            std::optional<engine::gfx::ShaderProgram> sceneShader =
                engine::gfx::ShaderProgram::loadFromFiles(ASSET_ROOT_DIR "/shaders/quad.vert",
                                                           ASSET_ROOT_DIR "/shaders/quad.frag");
            std::optional<engine::gfx::OcioDisplayTransform> ocioTransform =
                engine::gfx::OcioDisplayTransform::create();

            if (!sceneShader || !ocioTransform || !testPattern) {
                std::cerr << "main: shader compile/link or EXR texture load failed, aborting "
                             "startup\n";
                exitCode = EXIT_FAILURE;
            } else {
                const engine::gfx::PostProcessPass postProcess;

                // One-time texture-unit assignment: uAlbedo samples from
                // GL_TEXTURE0. Explicit even though unit 0 is GL's
                // implicit default for an unset sampler uniform — relying
                // on that default silently breaks the moment either
                // shader gains a second sampler. OcioDisplayTransform sets
                // its own uHdrColor uniform the same way at construction.
                sceneShader->use();
                GL_CALL(glUniform1i(sceneShader->uniformLocation("uAlbedo"), 0));

                // Exposure left at neutral EV=0 (not seeded from
                // camera.exposure()): the test pattern's 0.0/0.18/1.0
                // values are calibration constants, not scene-referred
                // radiance — a real f/2.8 exposure would crush them to
                // near-black and defeat Stage G's known-value checks.
                // exposure()/ev100() are still exercised via the startup
                // log; a real consumer arrives once scene-referred content
                // exists (Phase 2+).

                window.setResizeCallback(
                    [&hdrFbo](int width, int height) { hdrFbo.resize(width, height); });

                // Debug-only: 'L' cycles the active viewer LUT
                // (sRGB -> Rec709 -> Raw -> sRGB -> ...), Raw being a
                // genuine no-display-encode passthrough for direct
                // encoded-vs-unencoded comparison. No general input-mapping
                // system introduced for this one key — Phase 1's WASD/QE/R
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

                // poll -> bind HDR FBO -> clear -> draw quad -> post-process
                // blit (exposure + OCIO display transform) to the default
                // framebuffer -> swap.
                std::optional<engine::gfx::OcioDisplayTransform::Lut> lastLoggedLut;
                while (!window.shouldClose()) {
                    window.pollEvents();

                    hdrFbo.bind();
                    glClearColor(0.0F, 0.0F, 0.0F, 1.0F);
                    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

                    sceneShader->use();
                    testPattern->bind(0);
                    quad.draw();

                    ocioTransform->bind();
                    const auto [winWidth, winHeight] = window.framebufferSize();
                    postProcess.draw(hdrFbo.colorTexture(), ocioTransform->activeShader(),
                                      {winWidth, winHeight});

                    // Stage G verification: logs once at startup and again
                    // on every LUT toggle (see logColorCheck above) —
                    // covers both the numeric-check and
                    // differs-minutely-on-toggle checklist items.
                    if (lastLoggedLut != ocioTransform->activeLut()) {
                        logColorCheck(winWidth, winHeight, ocioTransform->activeLut());
                        lastLoggedLut = ocioTransform->activeLut();
                    }

                    window.swapBuffers();
                }
            }
        }
    }  // Window destroyed here, while GLFW is still initialized.

    glfwTerminate();
    return exitCode;
}
