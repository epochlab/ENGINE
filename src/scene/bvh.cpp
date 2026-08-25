#include "engine/scene/bvh.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace engine::scene {

namespace {

constexpr int kBinCount = 12;
constexpr int kMaxLeafTriangles = 4;
// Hard cap on leaf size regardless of what the SAH cost model says -- bounds worst-case traversal cost for pathological inputs (e.g. many triangles sharing centroids) where binning can't find a useful split.
constexpr int kHardMaxLeafTriangles = 16;
constexpr float kTraversalCost = 1.0F;
constexpr float kIntersectionCost = 1.0F;
// Hard cap on recursion depth. A locally SAH-optimal split can still be heavily lopsided (e.g. peeling off a handful of spatially distant outlier triangles from a large remaining cluster, repeatedly) -- geometrically valid, but on real mesh data this measurably produces a near-linear-chain tree whose *total* build cost (each level still scanning an O(n)-sized remaining range) is quadratic, not the O(n log n) a balanced split assumes. Capping depth forces a leaf once it's reached, bounding worst-case build cost to O(n * kMaxDepth); it also keeps tree depth safely under Bvh::intersect's fixed 64-slot traversal stack.
constexpr int kMaxDepth = 40;
// Bvh::intersect's fixed traversal-stack size (see its own declaration) -- named here, not just there, so the static_assert below has one thing to check rather than a bare literal repeated in a comment.
constexpr int kTraversalStackSize = 64;
static_assert(kMaxDepth < kTraversalStackSize,
              "kMaxDepth must stay under Bvh::intersect's traversal stack size");

struct Aabb {
    glm::vec3 min{std::numeric_limits<float>::max()};
    glm::vec3 max{std::numeric_limits<float>::lowest()};

    void grow(const glm::vec3& p) {
        min = glm::min(min, p);
        max = glm::max(max, p);
    }
    void grow(const Aabb& b) {
        grow(b.min);
        grow(b.max);
    }
    [[nodiscard]] float surfaceArea() const {
        const glm::vec3 d = max - min;
        if (d.x < 0.0F || d.y < 0.0F || d.z < 0.0F) {
            return 0.0F;
        }
        return 2.0F * ((d.x * d.y) + (d.y * d.z) + (d.z * d.x));
    }
};

Aabb triangleBounds(const Triangle& t) {
    Aabb b;
    b.grow(t.v0);
    b.grow(t.v1);
    b.grow(t.v2);
    return b;
}

// Moller-Trumbore ray-triangle intersection. No backface culling (det may be negative). Known edge case, accepted rather than solved: a ray exactly parallel to an axis with its origin exactly on that axis's bounding plane can produce a 0*inf NaN in the slab test below -- vanishingly unlikely for the random/synthetic rays this BVH is exercised with (tools/bvh_validate.cpp), not worth the extra robust-traversal machinery this phase.
bool intersectTriangle(const Ray& ray, const Triangle& tri, float& outT) {
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
    return true;
}

// Carries the same 0*inf NaN hazard intersectTriangle documents above: a ray exactly parallel to an axis with its origin exactly on that axis's bounding plane. Same accepted-risk reasoning applies here -- vanishingly unlikely for this BVH's exercised inputs, not worth robust-traversal machinery this phase.
bool intersectAabb(const Ray& ray, const glm::vec3& invDir, const glm::vec3& boundsMin,
                    const glm::vec3& boundsMax, float& outTNear) {
    const glm::vec3 t0 = (boundsMin - ray.origin) * invDir;
    const glm::vec3 t1 = (boundsMax - ray.origin) * invDir;
    const glm::vec3 tSmaller = glm::min(t0, t1);
    const glm::vec3 tBigger = glm::max(t0, t1);
    const float tNear =
        glm::max(ray.tMin, glm::max(tSmaller.x, glm::max(tSmaller.y, tSmaller.z)));
    const float tFar = glm::min(ray.tMax, glm::min(tBigger.x, glm::min(tBigger.y, tBigger.z)));
    outTNear = tNear;
    return tNear <= tFar;
}

// Mutable state threaded through the recursive build -- kept out of Bvh itself so Bvh::build's signature/members stay purely about the finished tree.
struct BuildContext {
    const std::vector<Triangle>* triangles;
    std::vector<Aabb> triBounds;
    std::vector<glm::vec3> centroids;
    std::vector<int>* primIndices;
    std::vector<Bvh::Node>* nodes;
};

Aabb rangeBounds(const BuildContext& ctx, int start, int end) {
    Aabb b;
    for (int i = start; i < end; ++i) {
        b.grow(ctx.triBounds[(*ctx.primIndices)[i]]);
    }
    return b;
}

Aabb centroidRangeBounds(const BuildContext& ctx, int start, int end) {
    Aabb b;
    for (int i = start; i < end; ++i) {
        b.grow(ctx.centroids[(*ctx.primIndices)[i]]);
    }
    return b;
}

// Median split on the largest-extent axis: always produces a valid start < mid < end partition regardless of how degenerate the centroid distribution is, guaranteeing the recursion terminates. Used as a fallback when binned SAH can't find a beneficial split.
int medianSplit(BuildContext& ctx, int start, int end) {
    const Aabb centroidBounds = centroidRangeBounds(ctx, start, end);
    const glm::vec3 extent = centroidBounds.max - centroidBounds.min;
    int axis = 0;
    if (extent.y > extent.x && extent.y >= extent.z) {
        axis = 1;
    } else if (extent.z > extent.x && extent.z >= extent.y) {
        axis = 2;
    }
    auto* prims = ctx.primIndices;
    const int mid = start + ((end - start) / 2);
    std::nth_element(prims->begin() + start, prims->begin() + mid, prims->begin() + end,
                      [&](int a, int b) { return ctx.centroids[a][axis] < ctx.centroids[b][axis]; });
    return mid;
}

int buildRecursive(BuildContext& ctx, int start, int end, int depth) {
    const int nodeIndex = static_cast<int>(ctx.nodes->size());
    ctx.nodes->emplace_back();

    const Aabb bounds = rangeBounds(ctx, start, end);
    const int count = end - start;

    auto makeLeaf = [&]() {
        (*ctx.nodes)[nodeIndex] = Bvh::Node{bounds.min, bounds.max, -1, -1, start, count};
    };

    if (count <= kMaxLeafTriangles || depth >= kMaxDepth) {
        makeLeaf();
        return nodeIndex;
    }

    const Aabb centroidBounds = centroidRangeBounds(ctx, start, end);
    const float nodeArea = bounds.surfaceArea();
    const float leafCost = kIntersectionCost * static_cast<float>(count);

    int bestAxis = -1;
    float bestBinSplitPos = 0.0F;  // world-space centroid coordinate of the chosen split
    float bestCost = std::numeric_limits<float>::max();

    for (int axis = 0; axis < 3; ++axis) {
        const float axisMin = centroidBounds.min[axis];
        const float axisMax = centroidBounds.max[axis];
        if (axisMax - axisMin < 1e-6F) {
            continue;  // degenerate axis -- every centroid coincides, binning can't split it
        }
        const float scale = static_cast<float>(kBinCount) / (axisMax - axisMin);

        std::array<Aabb, kBinCount> binBounds{};
        std::array<int, kBinCount> binCount{};
        binCount.fill(0);

        for (int i = start; i < end; ++i) {
            const int prim = (*ctx.primIndices)[i];
            int bin = static_cast<int>((ctx.centroids[prim][axis] - axisMin) * scale);
            bin = std::clamp(bin, 0, kBinCount - 1);
            binBounds[bin].grow(ctx.triBounds[prim]);
            ++binCount[bin];
        }

        std::array<Aabb, kBinCount> leftAccum{};
        std::array<int, kBinCount> leftCountAccum{};
        Aabb running;
        int runningCount = 0;
        for (int b = 0; b < kBinCount; ++b) {
            running.grow(binBounds[b]);
            runningCount += binCount[b];
            leftAccum[b] = running;
            leftCountAccum[b] = runningCount;
        }

        Aabb runningRight;
        int runningRightCount = 0;
        for (int b = kBinCount - 1; b >= 1; --b) {
            runningRight.grow(binBounds[b]);
            runningRightCount += binCount[b];
            const int leftCount = leftCountAccum[b - 1];
            const int rightCount = runningRightCount;
            if (leftCount == 0 || rightCount == 0) {
                continue;
            }
            const float cost = (leftAccum[b - 1].surfaceArea() * static_cast<float>(leftCount)) +
                                (runningRight.surfaceArea() * static_cast<float>(rightCount));
            if (cost < bestCost) {
                bestCost = cost;
                bestAxis = axis;
                bestBinSplitPos = axisMin + (static_cast<float>(b) / scale);
            }
        }
    }

    const bool splitBeneficial =
        bestAxis != -1 && nodeArea > 0.0F && (kTraversalCost + (bestCost / nodeArea)) < leafCost;

    if (!splitBeneficial && count <= kHardMaxLeafTriangles) {
        makeLeaf();
        return nodeIndex;
    }

    int mid;
    if (splitBeneficial) {
        auto* prims = ctx.primIndices;
        const auto middleIt =
            std::partition(prims->begin() + start, prims->begin() + end, [&](int prim) {
                return ctx.centroids[prim][bestAxis] < bestBinSplitPos;
            });
        mid = static_cast<int>(middleIt - prims->begin());
        if (mid == start || mid == end) {
            // All centroids landed on one side of the chosen bin boundary (can happen with heavily clustered data) -- fall back to a guaranteed-valid split rather than recursing on the same range.
            mid = medianSplit(ctx, start, end);
        }
    } else {
        mid = medianSplit(ctx, start, end);
    }

    const int left = buildRecursive(ctx, start, mid, depth + 1);
    const int right = buildRecursive(ctx, mid, end, depth + 1);
    (*ctx.nodes)[nodeIndex] = Bvh::Node{bounds.min, bounds.max, left, right, 0, 0};
    return nodeIndex;
}

}  // namespace

Bvh Bvh::build(std::vector<Triangle> triangles) {
    Bvh bvh;
    bvh.triangles_ = std::move(triangles);
    const int triCount = static_cast<int>(bvh.triangles_.size());
    if (triCount == 0) {
        return bvh;
    }

    BuildContext ctx;
    ctx.triangles = &bvh.triangles_;
    ctx.triBounds.resize(triCount);
    ctx.centroids.resize(triCount);
    bvh.primIndices_.resize(triCount);
    for (int i = 0; i < triCount; ++i) {
        bvh.primIndices_[i] = i;
        const Aabb b = triangleBounds(bvh.triangles_[i]);
        ctx.triBounds[i] = b;
        ctx.centroids[i] = (bvh.triangles_[i].v0 + bvh.triangles_[i].v1 + bvh.triangles_[i].v2) /
                            3.0F;
    }
    ctx.primIndices = &bvh.primIndices_;
    ctx.nodes = &bvh.nodes_;
    bvh.nodes_.reserve(static_cast<std::size_t>(triCount) * 2);

    buildRecursive(ctx, 0, triCount, 0);
    return bvh;
}

std::optional<Hit> Bvh::intersect(const Ray& ray) const {
    if (nodes_.empty()) {
        return std::nullopt;
    }
    const glm::vec3 invDir(1.0F / ray.dir.x, 1.0F / ray.dir.y, 1.0F / ray.dir.z);

    // Fixed-size traversal stack: with a leaf threshold of kMaxLeafTriangles, tree depth stays well within this bound for any triangle count this engine loads scenes at.
    std::array<int, kTraversalStackSize> stack{};
    int stackSize = 0;
    stack[stackSize++] = 0;

    Ray localRay = ray;
    std::optional<Hit> best;

    while (stackSize > 0) {
        const int nodeIndex = stack[--stackSize];
        const Node& node = nodes_[static_cast<std::size_t>(nodeIndex)];

        float tNear = 0.0F;
        if (!intersectAabb(localRay, invDir, node.boundsMin, node.boundsMax, tNear)) {
            continue;
        }
        if (best.has_value() && tNear > localRay.tMax) {
            continue;
        }

        if (node.isLeaf()) {
            for (int i = 0; i < node.triangleCount; ++i) {
                const int triIdx =
                    primIndices_[static_cast<std::size_t>(node.firstTriangle) +
                                 static_cast<std::size_t>(i)];
                float t = 0.0F;
                if (intersectTriangle(localRay, triangles_[static_cast<std::size_t>(triIdx)], t)) {
                    localRay.tMax = t;
                    best = Hit{t, triIdx};
                }
            }
        } else {
            stack[stackSize++] = node.leftChild;
            stack[stackSize++] = node.rightChild;
        }
    }
    return best;
}

std::optional<Hit> bruteForceIntersect(const std::vector<Triangle>& triangles, const Ray& ray) {
    Ray localRay = ray;
    std::optional<Hit> best;
    for (int i = 0; i < static_cast<int>(triangles.size()); ++i) {
        float t = 0.0F;
        if (intersectTriangle(localRay, triangles[static_cast<std::size_t>(i)], t)) {
            localRay.tMax = t;
            best = Hit{t, i};
        }
    }
    return best;
}

}  // namespace engine::scene
