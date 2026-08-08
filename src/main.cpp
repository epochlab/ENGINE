#include <cstdlib>
#include <iostream>

// GLEW before GLFW — see gl_debug.cpp for why.
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>

#include "engine/gfx/gl_debug.h"
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

            // Verification-only: Stage C has no render path yet to consume
            // these matrices as uniforms (that lands in Stage D once a
            // shader/quad exists to bind them). This proves the Euler-angle
            // and lens math actually run and produce sane output, mirroring
            // the GL_KHR_debug log above. Superseded once Stage D wires
            // viewMatrix()/projectionMatrix() into real uniforms.
            const engine::scene::Camera camera(glm::vec3(0.0F, 0.0F, 3.0F), 0.0F, 0.0F,
                                                engine::scene::Camera::FilmBack{36.0F, 24.0F},
                                                50.0F, 0.1F, 100.0F);
            const glm::vec3 camPos = camera.position();
            std::cout << "Camera: position=(" << camPos.x << ", " << camPos.y << ", " << camPos.z
                      << ") verticalFov=" << glm::degrees(camera.verticalFovRadians())
                      << " deg\n";

            // Stage B loop: poll -> clear default framebuffer -> swap.
            // HDR FBO bind / quad draw / post-process blit land in
            // Stage D/F once HdrFramebuffer, Mesh, ShaderProgram and
            // PostProcessPass exist.
            while (!window.shouldClose()) {
                window.pollEvents();

                GL_CALL(glClearColor(0.0F, 0.0F, 0.0F, 1.0F));
                GL_CALL(glClear(GL_COLOR_BUFFER_BIT));

                window.swapBuffers();
            }
        }
    }  // Window destroyed here, while GLFW is still initialized.

    glfwTerminate();
    return exitCode;
}
