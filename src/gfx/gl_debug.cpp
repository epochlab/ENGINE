#include "pathtracer/gfx/gl_debug.h"

#include <iomanip>
#include <iostream>

// GLEW must be included before GLFW in any translation unit needing both: GLFW detects GLEW's include guard and
// skips pulling in the platform GL headers itself. Reversing the order redeclares them and fails to compile.
#include <GL/glew.h>
#include <GLFW/glfw3.h>

namespace pathtracer::gfx {

void checkError(const char* file, int line) {
    for (GLenum error = glGetError(); error != GL_NO_ERROR; error = glGetError()) {
        std::cerr << "GL error 0x" << std::hex << std::setw(4) << std::setfill('0') << error
                   << std::dec << " at " << file << ':' << line << '\n';
    }
}

bool khrDebugAvailable() {
    return glfwExtensionSupported("GL_KHR_debug") == GLFW_TRUE;
}

}  // namespace pathtracer::gfx
