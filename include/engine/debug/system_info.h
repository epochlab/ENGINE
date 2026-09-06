#pragma once

#include <cstddef>
#include <string>

namespace engine::debug {

// GL_RENDERER/GL_VERSION + the primary monitor's refresh rate. No GL_VENDOR: on this project's sole target (macOS/Apple Silicon), GL_RENDERER already reads "Apple M1" etc. -- a separate vendor string would just repeat it. None of these change at runtime, so queryGpuInfo() is meant to be called once at startup, not per frame.
struct GpuInfo {
    std::string renderer;
    std::string version;
    int refreshRateHz;
};

[[nodiscard]] GpuInfo queryGpuInfo();

// Host CPU/memory topology from sysctl, for the startup spec block. No CPU brand string: GL_RENDERER (GpuInfo) already reads "Apple M1" on a unified-memory part, so machdep.cpu.brand_string was a verbatim duplicate of a field the spec block prints anyway. Apple Silicon reports two performance levels; perfLevel1* stay zeroed on a uniform-core machine, where hw.perflevel1.* simply does not exist. Total RAM is deliberately absent -- memory_tracker.h's totalSystemBytes() already reads hw.memsize and is its single owner. Needs no GL context, unlike queryGpuInfo, so it can be called before the window exists.
struct HostInfo {
    std::string perfLevel0Name;         // hw.perflevel0.name -- "Performance" on Apple Silicon
    int perfLevel0LogicalCpu = 0;       // hw.perflevel0.logicalcpu
    std::size_t perfLevel0L2Bytes = 0;  // hw.perflevel0.l2cachesize
    std::string perfLevel1Name;         // hw.perflevel1.name -- "Efficiency"; empty on a uniform-core machine
    int perfLevel1LogicalCpu = 0;
    std::size_t perfLevel1L2Bytes = 0;
    int logicalCpuTotal = 0;   // hw.logicalcpu
    int cacheLineBytes = 0;    // hw.cachelinesize
    std::string osProductVersion;  // kern.osproductversion
    std::string osBuild;           // kern.osversion
};

[[nodiscard]] HostInfo queryHostInfo();

// Compile-time build identity. Nothing here can change after link, so nothing is queried at runtime. gitSha/buildType/march/ipo come from the ENGINE_* defines CMakeLists.txt sets; compiler comes from __VERSION__, which needs no build-system support at all. Pointers to string literals, never freed.
struct BuildInfo {
    const char* compiler;
    const char* buildType;
    const char* gitSha;  // "+dirty" suffixed when the working tree had uncommitted tracked changes at configure time
    const char* march;
    bool ipo;
};

[[nodiscard]] BuildInfo buildInfo();

// Dependency versions. GLFW/GLEW are queried at runtime because they are dynamically loaded, so a header constant could lie about what actually got loaded; glm is header-only, so its macro IS the truth. Embree/OpenEXR/OCIO report their compile-time header versions -- a swapped dylib would go unreported, an accepted limitation, since the runtime alternatives need a live RTCDevice or OCIO config this function does not own.
struct LibraryVersions {
    std::string embree;
    std::string openexr;
    std::string ocio;
    std::string glfw;
    std::string glew;
    std::string glm;
};

[[nodiscard]] LibraryVersions queryLibraryVersions();

}  // namespace engine::debug
