#include "engine/debug/memory_tracker.h"

#include <atomic>
#include <cassert>

#include <mach/mach.h>
#include <mach/mach_host.h>
#include <mach/vm_statistics.h>
#include <sys/sysctl.h>

namespace engine::debug {

namespace {
std::atomic<std::size_t> gGpuBytes{0};
}  // namespace

void trackGpuAlloc(std::size_t bytes) {
    gGpuBytes += bytes;
}

void trackGpuFree(std::size_t bytes) {
    // Freeing more than was ever tracked is a programming-invariant violation (a missing/duplicated trackGpuAlloc call), not user input -- assert rather than let the unsigned counter wrap.
    assert(bytes <= gGpuBytes);
    gGpuBytes -= bytes;
}

std::size_t gpuAllocatedBytes() {
    return gGpuBytes;
}

std::size_t residentSetBytes() {
    mach_task_basic_info info{};
    mach_msg_type_number_t count = MACH_TASK_BASIC_INFO_COUNT;
    const kern_return_t result = task_info(mach_task_self(), MACH_TASK_BASIC_INFO,
                                            reinterpret_cast<task_info_t>(&info), &count);
    return result == KERN_SUCCESS ? static_cast<std::size_t>(info.resident_size) : 0;
}

std::uint64_t totalSystemBytes() {
    std::uint64_t bytes = 0;
    std::size_t size = sizeof(bytes);
    return sysctlbyname("hw.memsize", &bytes, &size, nullptr, 0) == 0 ? bytes : 0;
}

std::size_t availableSystemBytes() {
    vm_size_t pageSize = 0;
    if (host_page_size(mach_host_self(), &pageSize) != KERN_SUCCESS) {
        return 0;
    }
    vm_statistics64_data_t vmStats{};
    mach_msg_type_number_t count = HOST_VM_INFO64_COUNT;
    const kern_return_t result = host_statistics64(
        mach_host_self(), HOST_VM_INFO64, reinterpret_cast<host_info64_t>(&vmStats), &count);
    if (result != KERN_SUCCESS) {
        return 0;
    }
    return static_cast<std::size_t>(static_cast<std::uint64_t>(vmStats.free_count +
                                                                 vmStats.inactive_count) *
                                     static_cast<std::uint64_t>(pageSize));
}

}  // namespace engine::debug
