// Standalone correctness check for engine::scene::Bvh (bvh.h): builds a
// BVH over synthetic triangle soup, fires many random rays, and asserts
// Bvh::intersect agrees with a brute-force O(n) reference on every one.
// No test framework exists in this repo -- follows test_pattern.cpp's
// convention of a plain CLI tool with a non-zero exit on failure.

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <random>
#include <vector>

#include <glm/glm.hpp>

#include "engine/scene/bvh.h"

namespace {

using engine::scene::Bvh;
using engine::scene::Hit;
using engine::scene::Ray;
using engine::scene::Triangle;

constexpr int kTriangleCount = 2000;
constexpr int kRayCount = 20000;
constexpr float kTEpsilon = 1e-3F;

// Triangles clustered around random centres -- exercises both ordinary
// SAH splitting (clusters spread across the scene) and the degenerate/
// near-coincident-centroid fallback path (many triangles sharing one
// cluster).
std::vector<Triangle> makeSyntheticTriangles(std::mt19937& rng) {
    std::uniform_real_distribution<float> centerDist(-50.0F, 50.0F);
    std::uniform_real_distribution<float> offsetDist(-1.0F, 1.0F);

    std::vector<Triangle> triangles;
    triangles.reserve(kTriangleCount);
    for (int i = 0; i < kTriangleCount; ++i) {
        const glm::vec3 center(centerDist(rng), centerDist(rng), centerDist(rng));
        const glm::vec3 v0 =
            center + glm::vec3(offsetDist(rng), offsetDist(rng), offsetDist(rng));
        const glm::vec3 v1 =
            center + glm::vec3(offsetDist(rng), offsetDist(rng), offsetDist(rng));
        const glm::vec3 v2 =
            center + glm::vec3(offsetDist(rng), offsetDist(rng), offsetDist(rng));
        triangles.push_back(Triangle{v0, v1, v2});
    }
    return triangles;
}

Ray makeRandomRay(std::mt19937& rng) {
    std::uniform_real_distribution<float> originDist(-80.0F, 80.0F);
    std::uniform_real_distribution<float> dirDist(-1.0F, 1.0F);
    glm::vec3 dir(dirDist(rng), dirDist(rng), dirDist(rng));
    while (glm::length(dir) < 1e-4F) {
        dir = glm::vec3(dirDist(rng), dirDist(rng), dirDist(rng));
    }
    return Ray{glm::vec3(originDist(rng), originDist(rng), originDist(rng)), glm::normalize(dir),
               0.0F, 1000.0F};
}

}  // namespace

int main() {
    std::mt19937 rng(12345);
    const std::vector<Triangle> triangles = makeSyntheticTriangles(rng);
    const Bvh bvh = Bvh::build(triangles);

    std::cout << "bvh_validate: " << triangles.size() << " triangles, " << bvh.nodeCount()
              << " nodes\n";

    int mismatches = 0;
    for (int i = 0; i < kRayCount; ++i) {
        const Ray ray = makeRandomRay(rng);
        const std::optional<Hit> bvhHit = bvh.intersect(ray);
        const std::optional<Hit> bruteHit = engine::scene::bruteForceIntersect(triangles, ray);

        if (bvhHit.has_value() != bruteHit.has_value()) {
            std::cerr << "bvh_validate: hit/miss mismatch at ray " << i << " (bvh "
                      << (bvhHit.has_value() ? "hit" : "miss") << ", brute force "
                      << (bruteHit.has_value() ? "hit" : "miss") << ")\n";
            ++mismatches;
            continue;
        }
        if (bvhHit.has_value() && std::fabs(bvhHit->t - bruteHit->t) > kTEpsilon) {
            std::cerr << "bvh_validate: t mismatch at ray " << i << " (bvh t=" << bvhHit->t
                      << ", brute force t=" << bruteHit->t << ")\n";
            ++mismatches;
        }
    }

    if (mismatches > 0) {
        std::cerr << "bvh_validate: FAILED, " << mismatches << " / " << kRayCount
                  << " rays mismatched\n";
        return EXIT_FAILURE;
    }
    std::cout << "bvh_validate: PASSED, " << kRayCount << " rays cross-checked\n";
    return EXIT_SUCCESS;
}
