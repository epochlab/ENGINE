#pragma once

#include <cstddef>
#include <cstdint>

namespace engine::debug {

// One aggregate counter, not a per-object registry: the HUD only needs the running total (meshes, FBOs, and the histogram's downsample target/PBOs), not a breakdown.
void trackGpuAlloc(std::size_t bytes);
void trackGpuFree(std::size_t bytes);
[[nodiscard]] std::size_t gpuAllocatedBytes();

// Resident set size in bytes (mach_task_basic_info via task_info). macOS-only, matching this project's current sole target.
[[nodiscard]] std::size_t residentSetBytes();

// Total physical RAM in bytes (sysctl hw.memsize). Fixed for the machine, safe to query once rather than resampling every frame.
[[nodiscard]] std::uint64_t totalSystemBytes();

// Free + inactive page bytes (host_statistics64/HOST_VM_INFO64): an approximation of "available" memory (inactive pages are reclaimable on demand, not in active use), matching Activity Monitor's own heuristic. macOS has no single authoritative "available" value.
[[nodiscard]] std::size_t availableSystemBytes();

}  // namespace engine::debug
