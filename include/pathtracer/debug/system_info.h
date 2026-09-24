#pragma once

#include <cstddef>
#include <string>

namespace pathtracer::debug {

// GL_RENDERER and GL_VERSION; the refresh rate is the window's own display's, measured by DisplayLink. No GL_VENDOR:
// on this project's sole target, GL_RENDERER already names the part.
struct GpuInfo {
    std::string renderer;
    std::string version;
};

[[nodiscard]] GpuInfo queryGpuInfo();

// Host CPU and memory topology from sysctl, for the startup spec block. No CPU brand string: GL_RENDERER already
// reads "Apple M1" on a unified-memory part.
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

// Compile-time build identity. Nothing here can change after link, so nothing is queried at runtime; the fields come
// from the PATHTRACER_* defines CMakeLists sets.
struct BuildInfo {
    const char* compiler;
    const char* buildType;
    const char* gitSha;  // "+dirty" suffixed when the working tree had uncommitted tracked changes at configure time
    const char* march;
    bool ipo;
};

[[nodiscard]] BuildInfo buildInfo();

// The running executable's Mach-O LC_UUID as 32 lowercase hex digits, empty if the image carries none. Identifies the
// exact binary measured, which a git SHA cannot on a dirty tree.
[[nodiscard]] std::string executableUuid();

// Dependency versions. GLFW and GLEW are queried at runtime because they are dynamically loaded and a header constant
// could lie about what was actually loaded; glm is header-only, so its macro is the truth.
struct LibraryVersions {
    std::string embree;
    std::string openexr;
    std::string ocio;
    std::string glfw;
    std::string glew;
    std::string glm;
};

[[nodiscard]] LibraryVersions queryLibraryVersions();

}  // namespace pathtracer::debug
