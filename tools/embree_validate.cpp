// Standalone correctness check for engine::scene::EmbreeAccel (embree_accel.h): builds an Embree
// scene over synthetic triangle soup, fires many random rays, and asserts EmbreeAccel::intersect/
// occluded agree with bruteForceIntersect (ray_types.h), a deliberately dependency-free O(n)
// reference. Same standalone-CLI convention as bsdf_validate.cpp/nee_validate.cpp/furnace_test.cpp:
// no test framework, non-zero exit on failure.

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <random>
#include <vector>

#include <glm/glm.hpp>

#include "engine/scene/embree_accel.h"
#include "engine/scene/ray_types.h"

namespace {

using engine::scene::EmbreeAccel;
using engine::scene::Hit;
using engine::scene::Ray;
using engine::scene::Triangle;

constexpr int kTriangleCount = 2000;
constexpr int kRayCount = 20000;
constexpr float kTEpsilon = 1e-3F;

// Triangles clustered around random centres -- exercises ordinary BVH splitting (clusters spread
// across the scene) as well as tightly-packed clusters.
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

    std::optional<EmbreeAccel> accel = EmbreeAccel::build(triangles);
    if (!accel) {
        std::cerr << "embree_validate: EmbreeAccel::build failed\n";
        return EXIT_FAILURE;
    }

    std::cout << "embree_validate: " << triangles.size() << " triangles\n";

    int mismatches = 0;
    for (int i = 0; i < kRayCount; ++i) {
        const Ray ray = makeRandomRay(rng);
        const std::optional<Hit> accelHit = accel->intersect(ray);
        const std::optional<Hit> bruteHit = engine::scene::bruteForceIntersect(triangles, ray);

        if (accelHit.has_value() != bruteHit.has_value()) {
            std::cerr << "embree_validate: hit/miss mismatch at ray " << i << " (embree "
                      << (accelHit.has_value() ? "hit" : "miss") << ", brute force "
                      << (bruteHit.has_value() ? "hit" : "miss") << ")\n";
            ++mismatches;
            continue;
        }
        if (accelHit.has_value() && std::fabs(accelHit->t - bruteHit->t) > kTEpsilon) {
            std::cerr << "embree_validate: t mismatch at ray " << i << " (embree t=" << accelHit->t
                      << ", brute force t=" << bruteHit->t << ")\n";
            ++mismatches;
        }

        const bool accelOccluded = accel->occluded(ray);
        if (accelOccluded != bruteHit.has_value()) {
            std::cerr << "embree_validate: occluded() mismatch at ray " << i << " (embree "
                      << (accelOccluded ? "occluded" : "clear") << ", brute force "
                      << (bruteHit.has_value() ? "hit" : "miss") << ")\n";
            ++mismatches;
        }
    }

    if (mismatches > 0) {
        std::cerr << "embree_validate: FAILED, " << mismatches << " / " << kRayCount
                  << " rays mismatched\n";
        return EXIT_FAILURE;
    }
    std::cout << "embree_validate: PASSED, " << kRayCount << " rays cross-checked\n";
    return EXIT_SUCCESS;
}
