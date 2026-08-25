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
    int triangleIndex;  // index into the Triangle list passed to Bvh::build
    float u;  // barycentric weight on v1 (Moller-Trumbore convention, w=1-u-v on v0)
    float v;  // barycentric weight on v2
};

// Binned-SAH-built bounding volume hierarchy over a static triangle soup (Wald 2007, "On fast Construction of SAH-based Bounding Volume Hierarchies"), built once at scene load. No refit/update API -- the scene is static this phase. This is Phase 4 infrastructure for Phase 5's recursive tracing; it is not wired into the rasterized rendering path yet ("no secondary rays" holds this phase). Correctness is exercised by tools/bvh_validate.cpp, not by any rendering behavior.
class Bvh {
public:
    // Public so the free-function builder in bvh.cpp (binned-SAH, not a Bvh member -- kept out-of-class to stay readable) can construct nodes directly; also useful later for a BVH-visualization AOV.
    struct Node {
        glm::vec3 boundsMin{0.0F};
        glm::vec3 boundsMax{0.0F};
        // Leaf: triangleCount > 0, firstTriangle indexes primIndices_.
        // Interior: triangleCount == 0, leftChild/rightChild index nodes_.
        int leftChild = -1;
        int rightChild = -1;
        int firstTriangle = 0;
        int triangleCount = 0;
        [[nodiscard]] bool isLeaf() const { return triangleCount > 0; }
    };

    static Bvh build(std::vector<Triangle> triangles);

    [[nodiscard]] std::optional<Hit> intersect(const Ray& ray) const;

    // Any-hit query for shadow rays (NEE): true if anything blocks [ray.tMin, ray.tMax], without
    // finding the *closest* blocker -- returns on the first triangle hit found, cheaper than
    // intersect() for this use since occlusion doesn't care which occluder is nearest.
    [[nodiscard]] bool occluded(const Ray& ray) const;

    [[nodiscard]] int nodeCount() const { return static_cast<int>(nodes_.size()); }
    [[nodiscard]] int triangleCount() const { return static_cast<int>(triangles_.size()); }

private:
    std::vector<Triangle> triangles_;  // original order, untouched by the build
    std::vector<int> primIndices_;     // reordered so each leaf's range is contiguous
    std::vector<Node> nodes_;          // nodes_[0] is the root
};

// Brute-force reference intersection (O(n) triangle scan), used only to cross-validate Bvh::intersect -- see tools/bvh_validate.cpp.
[[nodiscard]] std::optional<Hit> bruteForceIntersect(const std::vector<Triangle>& triangles,
                                                       const Ray& ray);

}  // namespace engine::scene
