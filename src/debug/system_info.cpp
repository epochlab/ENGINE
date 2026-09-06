#include "engine/debug/system_info.h"

#include <sys/sysctl.h>

#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>

// GLEW before GLFW: see gl_debug.cpp for why.
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <OpenColorIO/OpenColorABI.h>
#include <OpenEXR/OpenEXRConfig.h>
#include <embree4/rtcore_config.h>
#include <glm/detail/setup.hpp>

namespace engine::debug {

namespace {
std::string glString(GLenum name) {
    const auto* str = reinterpret_cast<const char*>(glGetString(name));
    return str != nullptr ? std::string(str) : std::string();
}

// sysctlbyname is a system boundary: a key can be absent (hw.perflevel1.* on a uniform-core machine) or report an unexpected width, so both are surfaced as an empty/zero result rather than leaving the destination uninitialized.
std::string sysctlString(const char* name) {
    std::size_t size = 0;
    if (sysctlbyname(name, nullptr, &size, nullptr, 0) != 0 || size == 0) {
        return {};
    }
    std::string value(size, '\0');
    if (sysctlbyname(name, value.data(), &size, nullptr, 0) != 0) {
        return {};
    }
    value.resize(size > 0 ? size - 1 : 0);  // sysctl counts the NUL terminator; std::string supplies its own
    return value;
}

// Reads a 32- or 64-bit integer key. The width is not fixed across keys (hw.logicalcpu is 32-bit, hw.perflevel0.l2cachesize 64-bit), so it is taken from the size sysctl itself reports rather than assumed.
std::uint64_t sysctlUint(const char* name) {
    std::uint64_t value = 0;
    std::size_t size = sizeof(value);
    if (sysctlbyname(name, &value, &size, nullptr, 0) != 0) {
        return 0;
    }
    if (size == sizeof(std::uint32_t)) {
        std::uint32_t narrow = 0;
        std::memcpy(&narrow, &value, sizeof(narrow));
        return narrow;
    }
    return size == sizeof(value) ? value : 0;
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

HostInfo queryHostInfo() {
    HostInfo info;
    info.perfLevel0Name = sysctlString("hw.perflevel0.name");
    info.perfLevel0LogicalCpu = static_cast<int>(sysctlUint("hw.perflevel0.logicalcpu"));
    info.perfLevel0L2Bytes = static_cast<std::size_t>(sysctlUint("hw.perflevel0.l2cachesize"));
    info.perfLevel1Name = sysctlString("hw.perflevel1.name");
    info.perfLevel1LogicalCpu = static_cast<int>(sysctlUint("hw.perflevel1.logicalcpu"));
    info.perfLevel1L2Bytes = static_cast<std::size_t>(sysctlUint("hw.perflevel1.l2cachesize"));
    info.logicalCpuTotal = static_cast<int>(sysctlUint("hw.logicalcpu"));
    info.cacheLineBytes = static_cast<int>(sysctlUint("hw.cachelinesize"));
    info.osProductVersion = sysctlString("kern.osproductversion");
    info.osBuild = sysctlString("kern.osversion");
    return info;
}

BuildInfo buildInfo() {
    return BuildInfo{__VERSION__, ENGINE_BUILD_TYPE, ENGINE_GIT_SHA, ENGINE_MARCH, ENGINE_IPO != 0};
}

LibraryVersions queryLibraryVersions() {
    LibraryVersions versions;
    versions.embree = RTC_VERSION_STRING;
    versions.openexr = OPENEXR_VERSION_STRING;
    versions.ocio = OCIO_VERSION_FULL_STR;
    // glfwGetVersionString carries the backend list too ("3.5.0 Cocoa NSGL Null EGL OSMesa monotonic dynamic"), which is worth having: it says which platform and timer backends this GLFW was built with, and so which of them the run could possibly have selected. Compile-time, not the loaded one -- glfwGetPlatform is that -- and long, which is why the spec block gives it a line of its own.
    versions.glfw = glfwGetVersionString();
    versions.glew = reinterpret_cast<const char*>(glewGetString(GLEW_VERSION));
    std::array<char, 32> glmVersion{};
    std::snprintf(glmVersion.data(), glmVersion.size(), "%d.%d.%d", GLM_VERSION_MAJOR,
                   GLM_VERSION_MINOR, GLM_VERSION_PATCH);
    versions.glm = glmVersion.data();
    return versions;
}

}  // namespace engine::debug
