#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include "pathtracer/scene/ray_types.h"

using RTCDeviceTy = struct RTCDeviceTy;
using RTCSceneTy = struct RTCSceneTy;

namespace pathtracer::scene {

// Ray-scene intersection over Intel Embree's SIMD BVH. One RTC_GEOMETRY_TYPE_TRIANGLE geometry over a static
// triangle soup, built once at scene load.
class EmbreeAccel {
public:
    ~EmbreeAccel();

    EmbreeAccel(const EmbreeAccel&) = delete;
    EmbreeAccel& operator=(const EmbreeAccel&) = delete;
    EmbreeAccel(EmbreeAccel&& other) noexcept;
    EmbreeAccel& operator=(EmbreeAccel&& other) noexcept;

    // nullopt if the Embree device or scene fails to initialize, logged to stderr: a real failure mode in native
    // library init, surfaced at the call site rather than aborting.
    static std::optional<EmbreeAccel> build(std::vector<Triangle> triangles);

    [[nodiscard]] std::optional<Hit> intersect(const Ray& ray) const;

    // Any-hit query for shadow rays: true if anything blocks [ray.tMin, ray.tMax], without finding the closest
    // blocker -- cheaper than intersect(), which is all NEE needs.
    [[nodiscard]] bool occluded(const Ray& ray) const;

    [[nodiscard]] int triangleCount() const { return triangleCount_; }

private:
    EmbreeAccel(RTCDeviceTy* device, RTCSceneTy* scene, std::vector<Triangle> triangles);
    void release();

    RTCDeviceTy* device_ = nullptr;
    RTCSceneTy* scene_ = nullptr;
    std::vector<Triangle> triangles_;  // kept alive: the vertex buffer shares this memory with Embree
    int triangleCount_ = 0;
};

// Bytes Embree has allocated for BVH and geometry data, through its own device memory monitor rather than an
// estimate from triangle count.
[[nodiscard]] std::size_t embreeAllocatedBytes();

}  // namespace pathtracer::scene
