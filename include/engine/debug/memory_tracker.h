#pragma once

#include <cstddef>

namespace engine::debug {

// One aggregate counter, not a per-object registry: the HUD only needs
// the running total ("meshes + FBO, tracked"), not a breakdown.
void trackGpuAlloc(std::size_t bytes);
void trackGpuFree(std::size_t bytes);
[[nodiscard]] std::size_t gpuAllocatedBytes();

// Resident set size in bytes (mach_task_basic_info via task_info) —
// macOS-only, matching this project's current sole target.
[[nodiscard]] std::size_t residentSetBytes();

}  // namespace engine::debug
