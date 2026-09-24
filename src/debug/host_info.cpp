#include "pathtracer/debug/system_info.h"

#include <mach-o/dyld.h>
#include <mach-o/loader.h>
#include <sys/sysctl.h>

#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "pathtracer_git_sha.h"

namespace pathtracer::debug {

namespace {
// sysctlbyname is a system boundary: a key can be absent or report an unexpected width, so both surface as empty rather than asserted.
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

// Reads a 32- or 64-bit integer key; the width varies by key, so it is taken from the size sysctl itself reports.
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
    return BuildInfo{__VERSION__, PATHTRACER_BUILD_TYPE, PATHTRACER_GIT_SHA, PATHTRACER_MARCH, PATHTRACER_IPO != 0};
}

std::string executableUuid() {
    // Image 0 is the main executable; its load commands follow the 64-bit header directly.
    const auto* header = reinterpret_cast<const mach_header_64*>(_dyld_get_image_header(0));
    const auto* command = reinterpret_cast<const std::uint8_t*>(header + 1);
    for (std::uint32_t i = 0; i < header->ncmds; ++i) {
        const auto* load = reinterpret_cast<const load_command*>(command);
        if (load->cmd == LC_UUID) {
            const auto* uuid = reinterpret_cast<const uuid_command*>(load)->uuid;
            std::array<char, 33> hex{};
            for (int b = 0; b < 16; ++b) {
                std::snprintf(&hex[static_cast<std::size_t>(b) * 2], 3, "%02x", uuid[b]);
            }
            return hex.data();
        }
        command += load->cmdsize;
    }
    return {};
}

}  // namespace pathtracer::debug
