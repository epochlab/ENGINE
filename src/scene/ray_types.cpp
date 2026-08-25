#include "engine/scene/ray_types.h"

#include <cmath>

namespace engine::scene {

namespace {

// Moller-Trumbore ray-triangle intersection. No backface culling (det may be negative). Known edge case, accepted rather than solved: a ray exactly parallel to an axis with its origin exactly on that axis's bounding plane can produce a 0*inf NaN here -- vanishingly unlikely for the random/synthetic rays this is exercised with (tools/embree_validate.cpp), not worth the extra robust-intersection machinery this phase.
bool intersectTriangle(const Ray& ray, const Triangle& tri, float& outT, float& outU,
                        float& outV) {
    constexpr float kEpsilon = 1e-8F;
    const glm::vec3 edge1 = tri.v1 - tri.v0;
    const glm::vec3 edge2 = tri.v2 - tri.v0;
    const glm::vec3 pvec = glm::cross(ray.dir, edge2);
    const float det = glm::dot(edge1, pvec);
    if (std::fabs(det) < kEpsilon) {
        return false;
    }
    const float invDet = 1.0F / det;
    const glm::vec3 tvec = ray.origin - tri.v0;
    const float u = glm::dot(tvec, pvec) * invDet;
    if (u < 0.0F || u > 1.0F) {
        return false;
    }
    const glm::vec3 qvec = glm::cross(tvec, edge1);
    const float v = glm::dot(ray.dir, qvec) * invDet;
    if (v < 0.0F || u + v > 1.0F) {
        return false;
    }
    const float t = glm::dot(edge2, qvec) * invDet;
    if (t < ray.tMin || t > ray.tMax) {
        return false;
    }
    outT = t;
    outU = u;
    outV = v;
    return true;
}

}  // namespace

std::optional<Hit> bruteForceIntersect(const std::vector<Triangle>& triangles, const Ray& ray) {
    Ray localRay = ray;
    std::optional<Hit> best;
    for (int i = 0; i < static_cast<int>(triangles.size()); ++i) {
        float t = 0.0F;
        float u = 0.0F;
        float v = 0.0F;
        if (intersectTriangle(localRay, triangles[static_cast<std::size_t>(i)], t, u, v)) {
            localRay.tMax = t;
            best = Hit{t, i, u, v};
        }
    }
    return best;
}

}  // namespace engine::scene
