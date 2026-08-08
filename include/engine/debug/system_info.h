#pragma once

#include <string>

namespace engine::debug {

// GL_RENDERER/GL_VERSION. Neither changes at runtime, so queryGpuInfo()
// is meant to be called once at startup, not per frame.
struct GpuInfo {
    std::string renderer;
    std::string version;
};

[[nodiscard]] GpuInfo queryGpuInfo();

}  // namespace engine::debug
