#pragma once

#include <optional>
#include <vector>

#include "engine/scene/ray_types.h"

using RTCDeviceTy = struct RTCDeviceTy;
using RTCSceneTy = struct RTCSceneTy;

namespace engine::scene {

struct AabbBounds {
    glm::vec3 min;
    glm::vec3 max;
};

// Ray-scene intersection backed by Intel Embree's SIMD BVH build/traversal. One RTC_GEOMETRY_TYPE_TRIANGLE geometry over a static triangle soup, built once at scene load -- no refit/update API, the scene is static. Backs the path tracer's primary/shadow/bounce rays via single-ray rtcIntersect1/rtcOccluded1, called from tracePath (path_tracer.cpp). Correctness is exercised by tools/embree_validate.cpp against bruteForceIntersect (ray_types.h).
class EmbreeAccel {
public:
    ~EmbreeAccel();

    EmbreeAccel(const EmbreeAccel&) = delete;
    EmbreeAccel& operator=(const EmbreeAccel&) = delete;
    EmbreeAccel(EmbreeAccel&& other) noexcept;
    EmbreeAccel& operator=(EmbreeAccel&& other) noexcept;

    // nullopt if the Embree device or scene fails to initialize (logged to stderr) -- a real failure mode (native library/driver init), surfaced at the call site rather than assumed to always succeed.
    static std::optional<EmbreeAccel> build(std::vector<Triangle> triangles);

    [[nodiscard]] std::optional<Hit> intersect(const Ray& ray) const;

    // Any-hit query for shadow rays (NEE): true if anything blocks [ray.tMin, ray.tMax], without finding the *closest* blocker -- cheaper than intersect() for this use since occlusion doesn't care which occluder is nearest.
    [[nodiscard]] bool occluded(const Ray& ray) const;

    [[nodiscard]] int triangleCount() const { return triangleCount_; }

    // World-space AABB of the whole committed scene -- Embree already computes and caches this during rtcCommitScene, so this is a cheap query, not a re-scan of the triangle data. Feeds the Wireframe AOV's bounding-box edges (rasterizer.cpp).
    [[nodiscard]] AabbBounds sceneBounds() const;

private:
    EmbreeAccel(RTCDeviceTy* device, RTCSceneTy* scene, std::vector<Triangle> triangles);
    void release();

    RTCDeviceTy* device_ = nullptr;
    RTCSceneTy* scene_ = nullptr;
    std::vector<Triangle> triangles_;  // kept alive: the vertex buffer shares this memory with Embree
    int triangleCount_ = 0;
};

}  // namespace engine::scene
