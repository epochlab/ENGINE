#include <cstdlib>
#include <iostream>

// GLEW before GLFW — see gl_debug.cpp for why.
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "engine/gfx/gl_debug.h"
#include "engine/platform/window.h"

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
