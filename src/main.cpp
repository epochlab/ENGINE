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
#include "engine/gfx/post_process_pass.h"
#include "engine/gfx/shader_program.h"
#include "engine/gfx/texture.h"
#include "engine/platform/window.h"
#include "engine/scene/camera.h"

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

            // Verification-only: Stage D's quad is drawn directly in clip
            // space (no MVP), so viewMatrix()/projectionMatrix() have no
            // consumer yet. First real consumer is Phase 1/2 once scene
            // geometry needs placement relative to a moving viewpoint.
            const engine::scene::Camera camera(glm::vec3(0.0F, 0.0F, 3.0F), 0.0F, 0.0F,
                                                engine::scene::Camera::FilmBack{36.0F, 24.0F},
                                                50.0F, 0.1F, 100.0F);
            const glm::vec3 camPos = camera.position();
            std::cout << "Camera: position=(" << camPos.x << ", " << camPos.y << ", " << camPos.z
                      << ") verticalFov=" << glm::degrees(camera.verticalFovRadians())
                      << " deg\n";

            const auto [fbWidth, fbHeight] = window.framebufferSize();
            engine::gfx::HdrFramebuffer hdrFbo(fbWidth, fbHeight);

            const engine::gfx::Mesh quad = engine::gfx::Mesh::createQuad();
            const engine::gfx::Texture checkerboard =
                engine::gfx::Texture::createPlaceholderCheckerboard(256);

            std::optional<engine::gfx::ShaderProgram> sceneShader =
                engine::gfx::ShaderProgram::loadFromFiles(ASSET_ROOT_DIR "/shaders/quad.vert",
                                                           ASSET_ROOT_DIR "/shaders/quad.frag");
            std::optional<engine::gfx::ShaderProgram> displayShader =
                engine::gfx::ShaderProgram::loadFromFiles(
                    ASSET_ROOT_DIR "/shaders/fullscreen_triangle.vert",
                    ASSET_ROOT_DIR "/shaders/passthrough.frag");

            if (!sceneShader || !displayShader) {
                std::cerr << "main: shader compile/link failed, aborting startup\n";
                exitCode = EXIT_FAILURE;
            } else {
                const engine::gfx::PostProcessPass postProcess;

                // One-time texture-unit assignment: both shaders sample
                // from GL_TEXTURE0. Explicit even though unit 0 is GL's
                // implicit default for an unset sampler uniform — relying
                // on that default silently breaks the moment either
                // shader gains a second sampler.
                sceneShader->use();
                GL_CALL(glUniform1i(sceneShader->uniformLocation("uAlbedo"), 0));
                displayShader->use();
                GL_CALL(glUniform1i(displayShader->uniformLocation("uHdrColor"), 0));

                window.setResizeCallback(
                    [&hdrFbo](int width, int height) { hdrFbo.resize(width, height); });

                // Stage D loop: poll -> bind HDR FBO -> clear -> draw quad
                // -> post-process blit to the default framebuffer ->
                // swap. Unencoded: raw linear values reach the backbuffer
                // with no sRGB/OCIO transform, so the quad will look
                // visibly washed out until Stage F.
                while (!window.shouldClose()) {
                    window.pollEvents();

                    hdrFbo.bind();
                    glClearColor(0.0F, 0.0F, 0.0F, 1.0F);
                    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

                    sceneShader->use();
                    checkerboard.bind(0);
                    quad.draw();

                    postProcess.draw(hdrFbo.colorTexture(), *displayShader,
                                      window.framebufferSize());

                    window.swapBuffers();
                }
            }
        }
    }  // Window destroyed here, while GLFW is still initialized.

    glfwTerminate();
    return exitCode;
}
