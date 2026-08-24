#pragma once

#include <string>

namespace engine::debug {

// GL_RENDERER/GL_VERSION + the primary monitor's refresh rate. No
// GL_VENDOR: on this project's sole target (macOS/Apple Silicon),
// GL_RENDERER already reads "Apple M1" etc. -- a separate vendor string
// would just repeat it. None of these change at runtime, so
// queryGpuInfo() is meant to be called once at startup, not per frame.
struct GpuInfo {
    std::string renderer;
    std::string version;
    int refreshRateHz;
};

[[nodiscard]] GpuInfo queryGpuInfo();

}  // namespace engine::debug
