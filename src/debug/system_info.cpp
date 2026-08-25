#include "engine/debug/system_info.h"

// GLEW before GLFW — see gl_debug.cpp for why.
#include <GL/glew.h>
#include <GLFW/glfw3.h>

namespace engine::debug {

namespace {
std::string glString(GLenum name) {
    const auto* str = reinterpret_cast<const char*>(glGetString(name));
    return str != nullptr ? std::string(str) : std::string();
}
}  // namespace

GpuInfo queryGpuInfo() {
    GpuInfo info;
    info.renderer = glString(GL_RENDERER);
    info.version = glString(GL_VERSION);
    // Primary monitor, not necessarily the one the window is actually on -- this project doesn't yet track per-window monitor placement, and a single-display dev setup makes the distinction moot today. Guard the monitor itself, not just the video mode -- glfwGetVideoMode asserts/dereferences its argument, so passing it a null monitor crashes before the mode is ever null-checked.
    GLFWmonitor* monitor = glfwGetPrimaryMonitor();
    const GLFWvidmode* mode = monitor != nullptr ? glfwGetVideoMode(monitor) : nullptr;
    info.refreshRateHz = mode != nullptr ? mode->refreshRate : 0;
    return info;
}

}  // namespace engine::debug
