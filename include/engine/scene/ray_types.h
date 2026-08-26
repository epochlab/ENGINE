#pragma once

#include <optional>
#include <vector>

#include <glm/glm.hpp>

namespace engine::scene {

struct Ray {
    glm::vec3 origin;
    glm::vec3 dir;  // not required to be unit length
    float tMin;
    float tMax;
};

struct Triangle {
    glm::vec3 v0;
    glm::vec3 v1;
    glm::vec3 v2;
};

struct Hit {
    float t;
    int triangleIndex;  // index into the Triangle list the intersection was built from
    float u;  // barycentric weight on v1 (Moller-Trumbore convention, w=1-u-v on v0)
    float v;  // barycentric weight on v2
};

// Brute-force reference intersection (O(n) triangle scan) -- the correctness oracle for tools/embree_validate.cpp, deliberately dependency-free and simple enough to trust by inspection.
[[nodiscard]] std::optional<Hit> bruteForceIntersect(const std::vector<Triangle>& triangles,
                                                       const Ray& ray);

}  // namespace engine::scene
