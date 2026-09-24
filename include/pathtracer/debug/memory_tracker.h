#pragma once

#include <cstddef>
#include <cstdint>

namespace pathtracer::debug {

// One aggregate counter, not a per-object registry: the HUD needs the running total (meshes, FBOs, the histogram
// target and its PBOs), never a breakdown.
void trackGpuAlloc(std::size_t bytes);
void trackGpuFree(std::size_t bytes);
[[nodiscard]] std::size_t gpuAllocatedBytes();

// Resident set size in bytes (mach_task_basic_info via task_info). macOS-only, matching this project's current sole target.
[[nodiscard]] std::size_t residentSetBytes();

// Total physical RAM in bytes (sysctl hw.memsize). Fixed for the machine, safe to query once rather than resampling every frame.
[[nodiscard]] std::uint64_t totalSystemBytes();

// Free plus inactive page bytes (host_statistics64/HOST_VM_INFO64): an approximation of available memory, inactive
// pages being reclaimable on demand rather than in use.
[[nodiscard]] std::size_t availableSystemBytes();

}  // namespace pathtracer::debug
