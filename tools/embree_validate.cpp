// Correctness check for pathtracer::scene::EmbreeAccel (embree_accel.h): builds an Embree scene over synthetic triangle
// soup, fires many random rays, and asserts EmbreeAccel::intersect/occluded agree with bruteForceIntersect
// (ray_types.h), a deliberately dependency-free O(n) reference.
// Three separately-named checks rather than one, because they fail for different reasons: hit/miss disagreement is a
// traversal or watertightness defect, a t disagreement is an intersector precision defect, and an occluded()
// disagreement is the any-hit path diverging from the closest-hit one -- the last being invisible to the other two,
// since occluded() has its own Embree entry point.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <random>
#include <vector>

#include <glm/glm.hpp>

#include "check.h"
#include "pathtracer/scene/embree_accel.h"
#include "pathtracer/scene/ray_types.h"

namespace {

using pathtracer::scene::EmbreeAccel;
using pathtracer::scene::Hit;
using pathtracer::scene::Ray;
using pathtracer::scene::Triangle;

constexpr int kTriangleCount = 2000;
constexpr int kRayCount = 20000;
constexpr float kTEpsilon = 1e-3F;

// Triangles clustered around random centres -- exercises ordinary BVH splitting (clusters spread across the scene) as well as tightly-packed clusters.
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

// Builds the scene and traces every ray once, handing each comparison to `compare`. Shared by the three checks below so
// a scene's construction is stated once; each check pays for its own trace, which at this size is a fraction of a
// second and is what keeps the checks independently runnable.
template <typename Compare>
void crossCheck(tools::check::Context& ctx, Compare compare) {
    std::mt19937 rng(static_cast<std::mt19937::result_type>(ctx.seed()));
    const std::vector<Triangle> triangles = makeSyntheticTriangles(rng);
    std::optional<EmbreeAccel> accel = EmbreeAccel::build(triangles);
    ctx.plan(1);
    if (!accel) {
        PT_EXPECT(ctx, false, "EmbreeAccel::build failed");
        return;
    }
    int mismatches = 0;
    for (int i = 0; i < kRayCount; ++i) {
        const Ray ray = makeRandomRay(rng);
        mismatches += compare(*accel, triangles, ray) ? 0 : 1;
    }
    char detail[160];
    std::snprintf(detail, sizeof(detail), "%d of %d rays mismatched over %zu triangles", mismatches, kRayCount,
                  triangles.size());
    PT_EXPECT(ctx, mismatches == 0, detail);
}

// Watertightness and traversal: Embree and the O(n) reference must agree on whether a ray hits anything at all.
PT_CHECK(intersect_hit_agreement, Fast, Exact) {
    crossCheck(ctx, [](const EmbreeAccel& accel, const std::vector<Triangle>& triangles, const Ray& ray) {
        return accel.intersect(ray).has_value() ==
               pathtracer::scene::bruteForceIntersect(triangles, ray).has_value();
    });
}

// Intersector precision: where both agree there is a hit, they must agree on WHERE. Rays that miss are not evidence
// either way and are skipped rather than counted as agreement, which would dilute the mismatch fraction.
PT_CHECK(intersect_distance_agreement, Fast, Exact) {
    crossCheck(ctx, [](const EmbreeAccel& accel, const std::vector<Triangle>& triangles, const Ray& ray) {
        const std::optional<Hit> accelHit = accel.intersect(ray);
        const std::optional<Hit> bruteHit = pathtracer::scene::bruteForceIntersect(triangles, ray);
        if (!accelHit.has_value() || !bruteHit.has_value()) {
            return true;
        }
        return std::fabs(accelHit->t - bruteHit->t) <= kTEpsilon;
    });
}

// The any-hit path is a separate Embree entry point from the closest-hit one, so it can diverge without either check
// above noticing: a shadow ray that reports clear through geometry is a light leak, not a rounding difference.
PT_CHECK(occluded_agreement, Fast, Exact) {
    crossCheck(ctx, [](const EmbreeAccel& accel, const std::vector<Triangle>& triangles, const Ray& ray) {
        return accel.occluded(ray) == pathtracer::scene::bruteForceIntersect(triangles, ray).has_value();
    });
}

}  // namespace

PT_CHECK_MAIN("embree")
