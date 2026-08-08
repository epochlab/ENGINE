#include "engine/debug/memory_tracker.h"

#include <atomic>

#include <mach/mach.h>

namespace engine::debug {

namespace {
std::atomic<std::size_t> gGpuBytes{0};
}  // namespace

void trackGpuAlloc(std::size_t bytes) {
    gGpuBytes += bytes;
}

void trackGpuFree(std::size_t bytes) {
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

}  // namespace engine::debug
