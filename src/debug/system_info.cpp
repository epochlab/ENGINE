#include "engine/debug/system_info.h"

#include <GL/glew.h>

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
    return info;
}

}  // namespace engine::debug
