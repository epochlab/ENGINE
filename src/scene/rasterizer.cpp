#include "engine/scene/rasterizer.h"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <tuple>

#include "engine/scene/bsdf.h"
#include "engine/scene/false_color.h"
#include "engine/scene/gbuffer_shading.h"

namespace engine::scene {

namespace {

// View-space (right, up, forward) position plus barycentric weights on the ORIGINAL (unclipped) triangle's v1/v2 (Hit's Moller-Trumbore convention), so a clipped sub-triangle still resolves via interpolateShading(originalTriangle, ...); barycentrics are affine in position, so lerping this tuple along a clip edge is exact.
struct ClipVertex {
    glm::vec3 view;
    float origU;
    float origV;
    bool meshEdge;  // the polygon edge from this vertex to the next lies on the original triangle, not on a clip plane -- only those draw as wireframe
};

// A view-space point projected to screen space -- sx/sy are pixel coordinates (row 0 = top, HdrImage's convention), invZ is 1/viewZ for perspective-correct interpolation.
struct ScreenVertex {
    float sx;
    float sy;
    float invZ;
};

// Fixed-point vertex grid, derived per frame: the most sub-pixel bits for which every edge function stays exact in int64. Hardware fixes 8 (D3D11 16.8, Vulkan subPixelPrecisionBits) for gate cost; here exactness is the only bound, and the finer grid keeps silhouettes on the true geometry the path tracer's primary hit sees.
struct SubPixelGrid {
    int bits;
    float scale;  // 2^bits sub-pixels per pixel
};

// Frustum clipping bounds coordinates to [0, max(W,H)] px, so max(W,H) << bits < 2^30 keeps differences below 2^30, products below 2^60 and edge functions below 2^61.
SubPixelGrid subPixelGrid(int width, int height) {
    const int bits = 30 - std::bit_width(static_cast<unsigned>(std::max(width, height)));
    return SubPixelGrid{bits, std::ldexp(1.0F, bits)};
}

// A clipped vertex snapped to the fixed-point grid -- integer x/y make every edge function exact, which is what makes rasterization watertight: a shared edge evaluates to exactly opposite values in its two triangles, so no pixel centre can fail both.
struct RasterVertex {
    std::int32_t x;
    std::int32_t y;
    float invZ;
    float origU;
    float origV;
};

// One clipped, snapped, winding-normalized sub-triangle ready for scan-conversion.
struct RasterSubTriangle {
    RasterVertex v0;
    RasterVertex v1;
    RasterVertex v2;
    std::array<std::int8_t, 3> bias;  // top-left fill rule per edge (w0, w1, w2): 0 keeps a centre exactly on the edge, -1 leaves it to the neighbour sharing that edge
    std::uint8_t meshEdges;           // bit i: the edge opposite v_i (w_i's edge) is an original mesh edge rather than a clip or fan edge
    float invArea;
    int minX;
    int maxX;
    int minY;
    int maxY;
    int triangleIndex;  // indexes shadingTriangles -- resolves material/instance and the original triangle for interpolateShading
};

// One clipped, screen-projected bounding-box edge ready for line rasterization -- see appendBoxEdges. color is its instance's falseColorForId hue, resolved once per box rather than per covered pixel.
struct RasterLineSegment {
    glm::vec2 p0;
    glm::vec2 p1;
    float invZ0;
    float invZ1;
    int minX;
    int maxX;
    int minY;
    int maxY;
    glm::vec3 color;
};

// The cube's 12 edges as corner-index pairs, matching appendBoxEdges' 8-corner ordering: the 4 of the min-z face, the 4 of the max-z face, then the 4 pillars joining them.
constexpr std::array<std::array<int, 2>, 12> kBoxEdges{
    {{0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6}, {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}}};
constexpr std::size_t kBoxEdgeCount = kBoxEdges.size();

// Fixed on-screen line thickness in pixels, constant regardless of triangle/box size or distance.
constexpr float kLineThicknessPx = 1.0F;

const glm::vec3 kWireframeColor(1.0F, 1.0F, 1.0F);

// Fixed-point coordinate of pixel i's centre, i + 0.5 on the snapped grid.
std::int64_t pixelCenter(int i, const SubPixelGrid& grid) {
    return (static_cast<std::int64_t>(i) << grid.bits) + (std::int64_t{1} << (grid.bits - 1));
}

// 2D cross product (b-a) x (p-a), the Pineda 1988 edge function, exact in integers -- positive when p is right of directed edge a->b on the y-down screen.
std::int64_t edgeFunction(const RasterVertex& a, const RasterVertex& b, std::int64_t px, std::int64_t py) {
    return (static_cast<std::int64_t>(b.x - a.x) * (py - a.y)) - (static_cast<std::int64_t>(b.y - a.y) * (px - a.x));
}

// Top-left fill rule (D3D11.3 functional spec §3.4; Giesen 2013) for positive-area winding on the y-down screen: a centre exactly on a top (horizontal, interior below) or left (interior to its right) edge is covered. Reversing an edge flips the predicate, so of two triangles sharing an edge exactly one claims its centres, and coverage partitions the plane.
std::int8_t topLeftBias(const RasterVertex& a, const RasterVertex& b) {
    const std::int32_t dx = b.x - a.x;
    const std::int32_t dy = b.y - a.y;
    return (dy < 0 || (dy == 0 && dx > 0)) ? 0 : -1;
}

// A sub-triangle's three edge functions (w_i opposite v_i) at one pixel centre, or their per-pixel x step.
struct EdgeWeights {
    std::int64_t w0;
    std::int64_t w1;
    std::int64_t w2;
};

EdgeWeights edgeWeights(const RasterSubTriangle& st, std::int64_t px, std::int64_t py) {
    return EdgeWeights{edgeFunction(st.v1, st.v2, px, py), edgeFunction(st.v2, st.v0, px, py),
                       edgeFunction(st.v0, st.v1, px, py)};
}

// d(w)/d(px) for edge a->b is a.y - b.y; one pixel is 2^bits grid units. Exact, so stepping reproduces edgeWeights bit for bit (Pineda 1988's incremental evaluation).
EdgeWeights edgeStepX(const RasterSubTriangle& st, const SubPixelGrid& grid) {
    return EdgeWeights{static_cast<std::int64_t>(st.v1.y - st.v2.y) << grid.bits,
                       static_cast<std::int64_t>(st.v2.y - st.v0.y) << grid.bits,
                       static_cast<std::int64_t>(st.v0.y - st.v1.y) << grid.bits};
}

// Barycentric coverage from exact edge weights, shared by the depth and shading passes so a winner's barycentrics are the ones it was chosen on.
struct Coverage {
    bool covered;
    float b0;
    float b1;
    float b2;
};

Coverage coverage(const RasterSubTriangle& st, const EdgeWeights& w) {
    if (w.w0 + st.bias[0] < 0 || w.w1 + st.bias[1] < 0 || w.w2 + st.bias[2] < 0) {
        return Coverage{false, 0.0F, 0.0F, 0.0F};
    }
    return Coverage{true, static_cast<float>(w.w0) * st.invArea, static_cast<float>(w.w1) * st.invArea,
                    static_cast<float>(w.w2) * st.invArea};
}

// The view frustum as 5 view-space half-spaces dot((x, y, z, 1), plane) >= 0: near, left, right, bottom, top.
using FrustumPlanes = std::array<glm::vec4, 5>;

// Each clip plane adds at most one vertex to a convex polygon: a triangle against 5 planes has at most 8.
constexpr std::size_t kMaxClipVertices = 3 + std::tuple_size_v<FrustumPlanes>;
using ClipPolygon = std::array<ClipVertex, kMaxClipVertices>;

FrustumPlanes frustumPlanes(float nearClip, const Camera::ViewBasis& basis) {
    return {glm::vec4(0.0F, 0.0F, 1.0F, -nearClip), glm::vec4(1.0F, 0.0F, basis.halfWidth, 0.0F),
            glm::vec4(-1.0F, 0.0F, basis.halfWidth, 0.0F), glm::vec4(0.0F, 1.0F, basis.halfHeight, 0.0F),
            glm::vec4(0.0F, -1.0F, basis.halfHeight, 0.0F)};
}

float planeDistance(const glm::vec3& v, const glm::vec4& plane) {
    return glm::dot(glm::vec4(v, 1.0F), plane);
}

// Outcode (Blinn & Newell 1978): bit p set when v is outside plane p.
unsigned outcode(const glm::vec3& v, const FrustumPlanes& planes) {
    unsigned code = 0;
    for (std::size_t p = 0; p < planes.size(); ++p) {
        code |= (planeDistance(v, planes[p]) < 0.0F ? 1U : 0U) << p;
    }
    return code;
}

// Edge-plane intersection computed from the lexicographically smaller endpoint, so the two triangles sharing an edge (which traverse it in opposite directions) produce bitwise-identical clip vertices -- otherwise their clipped shared edges differ by rounding and crack.
ClipVertex intersectPlane(const ClipVertex& a, const ClipVertex& b, const glm::vec4& plane) {
    const bool bFirst = std::tie(b.view.x, b.view.y, b.view.z) < std::tie(a.view.x, a.view.y, a.view.z);
    const ClipVertex& p = bFirst ? b : a;
    const ClipVertex& q = bFirst ? a : b;
    const float dp = planeDistance(p.view, plane);
    const float t = dp / (dp - planeDistance(q.view, plane));
    return ClipVertex{glm::mix(p.view, q.view, t), glm::mix(p.origU, q.origU, t), glm::mix(p.origV, q.origV, t),
                      false};
}

// One Sutherland-Hodgman pass against a single plane; returns the output vertex count.
int clipAgainstPlane(const ClipPolygon& in, int count, const glm::vec4& plane, ClipPolygon& out) {
    int outCount = 0;
    for (int i = 0; i < count; ++i) {
        const ClipVertex& cur = in[static_cast<std::size_t>(i)];
        const ClipVertex& nxt = in[static_cast<std::size_t>((i + 1) % count)];
        const bool curIn = planeDistance(cur.view, plane) >= 0.0F;
        if (curIn) {
            out[static_cast<std::size_t>(outCount++)] = cur;
        }
        if (curIn != (planeDistance(nxt.view, plane) >= 0.0F)) {
            // Exiting, the new vertex starts an edge along the plane; entering, it starts the surviving part of cur->nxt.
            ClipVertex hit = intersectPlane(cur, nxt, plane);
            hit.meshEdge = !curIn && cur.meshEdge;
            out[static_cast<std::size_t>(outCount++)] = hit;
        }
    }
    return outCount;
}

// Clips the triangle in poly[0..2] to the view frustum in place; returns the vertex count (0 if culled). Frustum rather than near-only clipping is what bounds snapped coordinates to the viewport. A triangle that needs any clip is clipped against all 5 planes in fixed order, so two neighbours apply the identical plane sequence to their shared edge whatever their other vertices do.
int clipToFrustum(ClipPolygon& poly, const FrustumPlanes& planes) {
    const unsigned c0 = outcode(poly[0].view, planes);
    const unsigned c1 = outcode(poly[1].view, planes);
    const unsigned c2 = outcode(poly[2].view, planes);
    if ((c0 & c1 & c2) != 0) {
        return 0;
    }
    int count = 3;
    if ((c0 | c1 | c2) == 0) {
        return count;
    }
    ClipPolygon scratch{};
    for (const glm::vec4& plane : planes) {
        count = clipAgainstPlane(poly, count, plane, scratch);
        if (count < 3) {
            return 0;
        }
        poly = scratch;
    }
    return count;
}

// Two-vertex near-plane clip for box edges -- simpler than clipToFrustum (no polygon): 0 output vertices if both endpoints are behind the near plane, otherwise exactly 2.
int clipSegmentNearPlane(const glm::vec3& a, const glm::vec3& b, const glm::vec3& camPos,
                          const glm::vec3& forward, float nearClip, glm::vec3& outA, glm::vec3& outB) {
    const float za = glm::dot(a - camPos, forward);
    const float zb = glm::dot(b - camPos, forward);
    const bool aIn = za >= nearClip;
    const bool bIn = zb >= nearClip;
    if (!aIn && !bIn) {
        return 0;
    }
    if (aIn && bIn) {
        outA = a;
        outB = b;
        return 2;
    }
    const float t = (nearClip - za) / (zb - za);
    const glm::vec3 clipPoint = glm::mix(a, b, t);
    outA = aIn ? a : clipPoint;
    outB = aIn ? clipPoint : b;
    return 2;
}

// World to view space (right, up, forward components relative to the camera) -- the basis is orthonormal, so these are Camera::primaryRay's ndc weights scaled by viewZ.
glm::vec3 toView(const glm::vec3& world, const glm::vec3& camPos, const Camera::ViewBasis& basis) {
    const glm::vec3 d = world - camPos;
    return {glm::dot(d, basis.right), glm::dot(d, basis.up), glm::dot(d, basis.forward)};
}

// Inverts Camera::primaryRay's ndcX/ndcY->direction math, then maps ndc to pixel coordinates by renderRasterGBuffer's own pixel-center convention.
ScreenVertex projectToScreen(const glm::vec3& view, const Camera::ViewBasis& basis, int width, int height) {
    const float ndcX = view.x / (view.z * basis.halfWidth);
    const float ndcY = view.y / (view.z * basis.halfHeight);
    const float sx = ((ndcX + 1.0F) * 0.5F) * static_cast<float>(width);
    const float sy = ((1.0F - ndcY) * 0.5F) * static_cast<float>(height);
    return ScreenVertex{sx, sy, 1.0F / view.z};
}

RasterVertex snapToGrid(const ClipVertex& v, const Camera::ViewBasis& basis, int width, int height,
                        const SubPixelGrid& grid) {
    const ScreenVertex s = projectToScreen(v.view, basis, width, height);
    return RasterVertex{static_cast<std::int32_t>(std::lrint(s.sx * grid.scale)),
                        static_cast<std::int32_t>(std::lrint(s.sy * grid.scale)), s.invZ, v.origU, v.origV};
}

// Normalizes winding to positive area (glTF mirrored-scale nodes and Embree's double-sided intersection mean input winding isn't fixed), bounds the pixel centres it can cover, appends to `out` -- no-op if exactly degenerate on the snapped grid or covering no pixel centre.
void pushSubTriangle(RasterVertex v0, RasterVertex v1, RasterVertex v2, std::array<bool, 3> meshEdge,
                      int triangleIndex, int width, int height, const SubPixelGrid& grid,
                      std::vector<RasterSubTriangle>& out) {
    std::int64_t area = edgeFunction(v0, v1, v2.x, v2.y);
    if (area == 0) {
        return;
    }
    if (area < 0) {
        std::swap(v1, v2);
        std::swap(meshEdge[1], meshEdge[2]);
        area = -area;
    }
    // First/last pixel whose centre lies inside the snapped extent: ceil/floor of (extent - half) / pixel, as arithmetic shifts (floor division for negatives since C++20).
    const std::int32_t half = std::int32_t{1} << (grid.bits - 1);
    const std::int32_t roundUp = (std::int32_t{1} << grid.bits) - 1;
    const int minX = std::max(0, (std::min({v0.x, v1.x, v2.x}) - half + roundUp) >> grid.bits);
    const int maxX = std::min(width - 1, (std::max({v0.x, v1.x, v2.x}) - half) >> grid.bits);
    const int minY = std::max(0, (std::min({v0.y, v1.y, v2.y}) - half + roundUp) >> grid.bits);
    const int maxY = std::min(height - 1, (std::max({v0.y, v1.y, v2.y}) - half) >> grid.bits);
    if (minX > maxX || minY > maxY) {
        return;
    }
    const auto meshEdges =
        static_cast<std::uint8_t>((meshEdge[0] ? 1U : 0U) | (meshEdge[1] ? 2U : 0U) | (meshEdge[2] ? 4U : 0U));
    out.push_back(RasterSubTriangle{v0, v1, v2, {topLeftBias(v1, v2), topLeftBias(v2, v0), topLeftBias(v0, v1)},
                                    meshEdges, 1.0F / static_cast<float>(area), minX, maxX, minY, maxY,
                                    triangleIndex});
}

// Clips/snaps/winding-normalizes every triangle once per call, in parallel over chunks of the triangle list -- cheap per-triangle math, but at a few million triangles doing it on one thread is a frame-rate ceiling by itself. Each chunk appends to its own vector, so the appends need no synchronization, and the chunks are concatenated in order afterwards: the result is the identical sequence a sequential build produces, which matters because the z-test below is first-writer-wins at exactly equal depth.
std::vector<RasterSubTriangle> buildSubTriangles(const Camera& camera,
                                                  const std::vector<ShadingTriangle>& shadingTriangles,
                                                  int width, int height, const SubPixelGrid& grid,
                                                  ThreadPool& threadPool) {
    const glm::vec3 camPos = camera.position();
    const Camera::ViewBasis basis = camera.viewBasis(static_cast<float>(width) / static_cast<float>(height));
    const FrustumPlanes planes = frustumPlanes(camera.nearClip(), basis);

    const int triangleCount = static_cast<int>(shadingTriangles.size());
    // Four chunks per worker, not one: parallelFor hands them out on demand, so the extra granularity absorbs the imbalance between a chunk that is entirely behind the camera and one that is entirely on screen.
    const int chunkCount =
        std::max(1, std::min(triangleCount, static_cast<int>(threadPool.threadCount()) * 4));
    const int chunkSize = (triangleCount + chunkCount - 1) / chunkCount;
    std::vector<std::vector<RasterSubTriangle>> chunks(static_cast<std::size_t>(chunkCount));

    threadPool.parallelFor(chunkCount, [&](int chunk) {
        const int begin = chunk * chunkSize;
        const int end = std::min(begin + chunkSize, triangleCount);
        std::vector<RasterSubTriangle>& out = chunks[static_cast<std::size_t>(chunk)];
        out.reserve(static_cast<std::size_t>(std::max(0, end - begin)));
        for (int i = begin; i < end; ++i) {
            const ShadingTriangle& tri = shadingTriangles[static_cast<std::size_t>(i)];
            ClipPolygon poly{};
            poly[0] = ClipVertex{toView(tri.v0.position, camPos, basis), 0.0F, 0.0F, true};
            poly[1] = ClipVertex{toView(tri.v1.position, camPos, basis), 1.0F, 0.0F, true};
            poly[2] = ClipVertex{toView(tri.v2.position, camPos, basis), 0.0F, 1.0F, true};
            const int count = clipToFrustum(poly, planes);
            if (count < 3) {
                continue;
            }
            std::array<RasterVertex, kMaxClipVertices> snapped{};
            for (int k = 0; k < count; ++k) {
                snapped[static_cast<std::size_t>(k)] =
                    snapToGrid(poly[static_cast<std::size_t>(k)], basis, width, height, grid);
            }
            // Fan over the convex clipped polygon; its internal diagonals join already-snapped vertices, so they are watertight too, and are never mesh edges.
            for (int k = 1; k + 1 < count; ++k) {
                const std::array<bool, 3> meshEdge{poly[static_cast<std::size_t>(k)].meshEdge,
                                                   k + 2 == count && poly[static_cast<std::size_t>(count - 1)].meshEdge,
                                                   k == 1 && poly[0].meshEdge};
                pushSubTriangle(snapped[0], snapped[static_cast<std::size_t>(k)],
                                snapped[static_cast<std::size_t>(k + 1)], meshEdge, i, width, height, grid, out);
            }
        }
    });

    std::size_t total = 0;
    for (const std::vector<RasterSubTriangle>& chunk : chunks) {
        total += chunk.size();
    }
    std::vector<RasterSubTriangle> subTriangles;
    subTriangles.reserve(total);
    for (const std::vector<RasterSubTriangle>& chunk : chunks) {
        subTriangles.insert(subTriangles.end(), chunk.begin(), chunk.end());
    }
    return subTriangles;
}

// Per-row lists of the sub-triangles whose bounding box covers that row, so renderRow visits only those instead of rejecting the whole array one triangle at a time. Flat CSR (count, prefix sum, fill) rather than a vector per row, which would be `height` heap allocations per frame.
// The cost this removes is memory bandwidth, not comparisons: RasterSubTriangle is 88 bytes and the scan is linear, so every row streamed the entire array. That is invisible while the array fits in cache -- at the 20561-triangle scene it is 1.8 MB and the scan costs nothing measurable -- and dominant once it does not: at 5M triangles it is 440 MB, read 1152 times per frame.
// Memory is O(sum of row spans), so a scene of few very large triangles can need more of it than the sub-triangle array itself. That is the opposite regime from the one this exists for, where triangles are small and each spans a handful of rows.
struct RowBuckets {
    std::vector<std::size_t> offsets;  // height+1 entries, offsets[y]..offsets[y+1] is row y's range in `indices`
    std::vector<int> indices;          // indexes the bucketed span array; int since a clip splits at most one triangle into two, bounding this by 2x the scene's triangle count
};

// Templated over the element rather than duplicated: sub-triangles and box edges both carry minY/maxY and both need the same per-row lists, so one body serves both and the two cannot drift apart.
template <typename Span>
RowBuckets buildRowBuckets(const std::vector<Span>& spans, int height) {
    RowBuckets buckets;
    buckets.offsets.assign(static_cast<std::size_t>(height) + 1, 0);
    for (const Span& st : spans) {
        for (int y = st.minY; y <= st.maxY; ++y) {
            ++buckets.offsets[static_cast<std::size_t>(y) + 1];
        }
    }
    for (int y = 0; y < height; ++y) {
        buckets.offsets[static_cast<std::size_t>(y) + 1] +=
            buckets.offsets[static_cast<std::size_t>(y)];
    }
    buckets.indices.resize(buckets.offsets[static_cast<std::size_t>(height)]);

    // Per-row write cursor. Filing in increasing sub-triangle index leaves each row's list in the same relative order the old full-array scan visited them in, so the z-test's tie-break at exactly equal depth is unchanged.
    std::vector<std::size_t> cursor(buckets.offsets.begin(), buckets.offsets.end() - 1);
    for (std::size_t i = 0; i < spans.size(); ++i) {
        const Span& st = spans[i];
        for (int y = st.minY; y <= st.maxY; ++y) {
            buckets.indices[cursor[static_cast<std::size_t>(y)]++] = static_cast<int>(i);
        }
    }
    return buckets;
}

// Appends one AABB's 12 edges (8-corner topology), near-clipped and projected -- mirrors buildSubTriangles' role for 12 segments instead of the scene's triangle list. Appends rather than returns so the per-instance loop concatenates into one array without a vector per box.
void appendBoxEdges(const Camera& camera, const AabbBounds& box, const glm::vec3& color, int width,
                     int height, std::vector<RasterLineSegment>& out) {
    const glm::vec3 camPos = camera.position();
    const Camera::ViewBasis basis = camera.viewBasis(static_cast<float>(width) / static_cast<float>(height));
    const std::array<glm::vec3, 8> corners{
        glm::vec3(box.min.x, box.min.y, box.min.z), glm::vec3(box.max.x, box.min.y, box.min.z),
        glm::vec3(box.max.x, box.max.y, box.min.z), glm::vec3(box.min.x, box.max.y, box.min.z),
        glm::vec3(box.min.x, box.min.y, box.max.z), glm::vec3(box.max.x, box.min.y, box.max.z),
        glm::vec3(box.max.x, box.max.y, box.max.z), glm::vec3(box.min.x, box.max.y, box.max.z)};
    for (const std::array<int, 2>& edge : kBoxEdges) {
        glm::vec3 a{};
        glm::vec3 b{};
        if (clipSegmentNearPlane(corners[static_cast<std::size_t>(edge[0])],
                                  corners[static_cast<std::size_t>(edge[1])], camPos, basis.forward,
                                  camera.nearClip(), a, b) == 0) {
            continue;
        }
        const ScreenVertex sa = projectToScreen(toView(a, camPos, basis), basis, width, height);
        const ScreenVertex sb = projectToScreen(toView(b, camPos, basis), basis, width, height);
        const int minX = std::max(0, static_cast<int>(std::floor(std::min(sa.sx, sb.sx))));
        const int maxX = std::min(width - 1, static_cast<int>(std::ceil(std::max(sa.sx, sb.sx))));
        const int minY = std::max(0, static_cast<int>(std::floor(std::min(sa.sy, sb.sy))));
        const int maxY = std::min(height - 1, static_cast<int>(std::ceil(std::max(sa.sy, sb.sy))));
        if (minX > maxX || minY > maxY) {
            continue;
        }
        out.push_back(RasterLineSegment{
            {sa.sx, sa.sy}, {sb.sx, sb.sy}, sa.invZ, sb.invZ, minX, maxX, minY, maxY, color});
    }
}

// Resolves and writes every G-buffer field for one covered, z-winning pixel -- same sampling calls tracePath's bounce-0 block makes (gbuffer_shading.h), never a lighting/BSDF evaluation. origU/origV are the perspective-correct barycentric coordinates on the ORIGINAL (unclipped) triangle. wireframe is a screen-space distance-to-edge test against the sub-triangle's original mesh edges (clip-plane and fan edges excluded), sharing nearLineSegmentPx with the box edges -- evaluated only at this already-z-tested pixel, so hidden-line removal is free.
void shadePixel(RasterGBuffer& result, int x, int y, float viewZ, float origU, float origV,
                 const ShadingTriangle& triangle, const Material& material,
                 const PathTraceSettings& settings, const RasterSubTriangle& st,
                 const SubPixelGrid& grid) {
    const ShadingVertex shading = interpolateShading(triangle, origU, origV);
    const ShadingFrame frame = buildShadingFrame(shading, material, settings);
    const BsdfParams params =
        resolveBsdfParams(material, shading.uv, shading.colour, settings, std::nullopt);

    const glm::vec2 p(static_cast<float>(x) + 0.5F, static_cast<float>(y) + 0.5F);
    const glm::vec2 p0 = glm::vec2(st.v0.x, st.v0.y) / grid.scale;
    const glm::vec2 p1 = glm::vec2(st.v1.x, st.v1.y) / grid.scale;
    const glm::vec2 p2 = glm::vec2(st.v2.x, st.v2.y) / grid.scale;
    const bool wire = ((st.meshEdges & 1U) != 0 && nearLineSegmentPx(p, p1, p2, kLineThicknessPx).near) ||
                      ((st.meshEdges & 2U) != 0 && nearLineSegmentPx(p, p2, p0, kLineThicknessPx).near) ||
                      ((st.meshEdges & 4U) != 0 && nearLineSegmentPx(p, p0, p1, kLineThicknessPx).near);

    writeTexel(result.depth, x, y, glm::vec3(viewZ));
    writeTexel(result.lookahead, x, y,
                glm::vec3(std::clamp(1.0F - (viewZ / settings.lookaheadDistance), 0.0F, 1.0F)));
    writeTexel(result.worldPos, x, y, shading.position);
    writeTexel(result.uv, x, y, glm::vec3(glm::fract(shading.uv), 0.0F));
    writeTexel(result.normal, x, y, frame.normal);
    writeTexel(result.geomNormal, x, y, glm::normalize(shading.normal));
    writeTexel(result.albedo, x, y, params.baseColor);
    writeTexel(result.metallic, x, y, glm::vec3(params.metallic));
    writeTexel(result.roughness, x, y, glm::vec3(params.roughness));
    writeTexel(result.tangent, x, y, frame.tangent);
    writeTexel(result.objectId, x, y, falseColorForId(triangle.instanceIndex));
    writeTexel(result.alpha, x, y, glm::vec3(1.0F));
    writeTexel(result.wireframe, x, y, wire ? kWireframeColor : glm::vec3(0.0F));
    writeTexel(result.iorAov, x, y, glm::vec3(settings.ior));
}

// Pass one of two: resolves visibility for the row without shading anything, recording each pixel's depth and the sub-triangle index that owns it. Splitting this out is what bounds shading to one evaluation per visible pixel -- the single-pass form shaded on every depth improvement, so a pixel behind N nearer-in-list surfaces paid N full shades (8 bilinear fetches, a shading frame and 14 texel writes each) to keep one.
// Keeps the single-pass tie-break exactly: `>=` rejects equal depth, so the first sub-triangle in row order still wins a tie, and row order is the sub-triangle list order buildRowBuckets preserves.
void depthPassRow(int y, const std::vector<RasterSubTriangle>& subTriangles,
                   const RowBuckets& rowBuckets, const SubPixelGrid& grid, float* zRow, int* winnerRow) {
    const std::int64_t py = pixelCenter(y, grid);
    const std::size_t rowEnd = rowBuckets.offsets[static_cast<std::size_t>(y) + 1];
    for (std::size_t k = rowBuckets.offsets[static_cast<std::size_t>(y)]; k < rowEnd; ++k) {
        const int index = rowBuckets.indices[k];
        const RasterSubTriangle& st = subTriangles[static_cast<std::size_t>(index)];
        const EdgeWeights step = edgeStepX(st, grid);
        EdgeWeights w = edgeWeights(st, pixelCenter(st.minX, grid), py);
        for (int x = st.minX; x <= st.maxX; ++x) {
            const Coverage cov = coverage(st, w);
            w.w0 += step.w0;
            w.w1 += step.w1;
            w.w2 += step.w2;
            if (!cov.covered) {
                continue;
            }
            const float viewZ =
                1.0F / ((cov.b0 * st.v0.invZ) + (cov.b1 * st.v1.invZ) + (cov.b2 * st.v2.invZ));
            if (viewZ >= zRow[x]) {
                continue;
            }
            zRow[x] = viewZ;
            winnerRow[x] = index;
        }
    }
}

// Pass two of two: shades each covered pixel exactly once from the winner the depth pass recorded. Walks the row in x order rather than in triangle order, so the 14 AOV writes advance linearly through each image instead of scattering across it.
// viewZ is read back from the depth buffer rather than recomputed: it is the value this same winner stored, so reading it is both cheaper and exact where a recomputation would only be exact by argument.
void shadeRow(RasterGBuffer& result, int y, int width, const std::vector<RasterSubTriangle>& subTriangles,
               const std::vector<ShadingTriangle>& shadingTriangles,
               const std::vector<MeshInstance>& instances,
               const std::vector<PathTraceSettings>& perInstanceSettings,
               const SubPixelGrid& grid, const float* zRow, const int* winnerRow) {
    const std::int64_t py = pixelCenter(y, grid);
    for (int x = 0; x < width; ++x) {
        if (winnerRow[x] < 0) {
            continue;
        }
        const RasterSubTriangle& st = subTriangles[static_cast<std::size_t>(winnerRow[x])];
        const Coverage cov = coverage(st, edgeWeights(st, pixelCenter(x, grid), py));
        const float viewZ = zRow[x];
        const float origU = ((cov.b0 * st.v0.origU * st.v0.invZ) + (cov.b1 * st.v1.origU * st.v1.invZ) +
                             (cov.b2 * st.v2.origU * st.v2.invZ)) *
                            viewZ;
        const float origV = ((cov.b0 * st.v0.origV * st.v0.invZ) + (cov.b1 * st.v1.origV * st.v1.invZ) +
                             (cov.b2 * st.v2.origV * st.v2.invZ)) *
                            viewZ;
        const ShadingTriangle& triangle = shadingTriangles[static_cast<std::size_t>(st.triangleIndex)];
        const Material& material = instances[static_cast<std::size_t>(triangle.instanceIndex)].material;
        const PathTraceSettings& instanceSettings =
            perInstanceSettings[static_cast<std::size_t>(triangle.instanceIndex)];
        shadePixel(result, x, y, viewZ, origU, origV, triangle, material, instanceSettings, st, grid);
    }
}

// Bounding-box edges: real line segments z-tested against the row's now-finalized depth (real geometry occludes them) but never written back to it, so box edges never occlude each other -- every edge shows unless real mesh blocks it, including where two instances' boxes overlap. Drawn into the same wireframe AOV as the mesh edges, in the instance's own false colour, taking precedence over white where both apply.
// Bucketed by row for the same reason the sub-triangles are: the segment count is 12 per instance, so a full-array scan per row would grow with the scene's object count.
void drawBoxEdgesRow(RasterGBuffer& result, int y, const std::vector<RasterLineSegment>& boxEdges,
                      const RowBuckets& rowBuckets, const float* zRow) {
    const std::size_t rowEnd = rowBuckets.offsets[static_cast<std::size_t>(y) + 1];
    for (std::size_t k = rowBuckets.offsets[static_cast<std::size_t>(y)]; k < rowEnd; ++k) {
        const RasterLineSegment& seg = boxEdges[static_cast<std::size_t>(rowBuckets.indices[k])];
        for (int x = seg.minX; x <= seg.maxX; ++x) {
            const glm::vec2 p(static_cast<float>(x) + 0.5F, static_cast<float>(y) + 0.5F);
            const LineProximity prox = nearLineSegmentPx(p, seg.p0, seg.p1, kLineThicknessPx);
            if (!prox.near) {
                continue;
            }
            const float viewZ = 1.0F / glm::mix(seg.invZ0, seg.invZ1, prox.t);
            if (viewZ <= zRow[x]) {
                writeTexel(result.wireframe, x, y, seg.color);
            }
        }
    }
}

// Every AOV image in one place, so the reallocation and the per-row clear below cannot disagree about which fields exist -- adding an AOV to RasterGBuffer without adding it here leaves it uncleared, which this array's fixed size catches at compile time.
std::array<engine::gfx::HdrImage*, 14> aovImages(RasterGBuffer& g) {
    return {&g.iorAov, &g.depth,    &g.lookahead, &g.worldPos, &g.uv,      &g.normal,
            &g.geomNormal, &g.albedo, &g.metallic, &g.roughness, &g.tangent,
            &g.objectId, &g.alpha,  &g.wireframe};
}

}  // namespace

void renderRasterGBuffer(const Camera& camera, const std::vector<ShadingTriangle>& shadingTriangles,
                          const std::vector<MeshInstance>& instances,
                          const std::vector<PathTraceSettings>& perInstanceSettings,
                          const std::vector<AabbBounds>& instanceBounds, int width, int height,
                          ThreadPool& threadPool, RasterGBuffer& result) {
    const std::array<engine::gfx::HdrImage*, 14> images = aovImages(result);
    // Reallocated only on a resolution change; every other call reuses the storage and relies on renderRow's clear. makeImage's own zeroing is redundant against that clear but runs once per resize, not once per frame.
    if (result.depth.width != width || result.depth.height != height) {
        for (engine::gfx::HdrImage* image : images) {
            *image = makeImage(width, height);
        }
    }
    ++result.generation;

    const SubPixelGrid grid = subPixelGrid(width, height);
    const std::vector<RasterSubTriangle> subTriangles =
        buildSubTriangles(camera, shadingTriangles, width, height, grid, threadPool);
    const RowBuckets rowBuckets = buildRowBuckets(subTriangles, height);
    // One box per instance, in the instance's ObjectID false colour. An instance that contributed no triangles has an empty box and no edges to draw.
    std::vector<RasterLineSegment> boxEdges;
    boxEdges.reserve(instanceBounds.size() * kBoxEdgeCount);
    for (std::size_t i = 0; i < instanceBounds.size(); ++i) {
        if (isEmpty(instanceBounds[i])) {
            continue;
        }
        appendBoxEdges(camera, instanceBounds[i], falseColorForId(static_cast<int>(i)), width, height,
                        boxEdges);
    }
    const RowBuckets boxRowBuckets = buildRowBuckets(boxEdges, height);
    const std::size_t pixelCount = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    std::vector<float> zbuffer(pixelCount);
    // The depth pass's other output: which sub-triangle owns each pixel, -1 for uncovered. 4 bytes per pixel, sized like the z-buffer because both are written by whichever worker owns the row.
    std::vector<int> winners(pixelCount);

    const auto renderRow = [&](int y) {
        // Clearing this row of every AOV is what makes the buffers reusable across calls: the worker that is about to overwrite the row zeroes it first, in parallel and while it is already cache-warm, instead of 14 sequential full-image memsets before the dispatch. An uncovered pixel therefore still reads back zero (alpha 0, the miss test every consumer uses) exactly as a freshly allocated image did.
        const std::size_t rowStart = static_cast<std::size_t>(y) * static_cast<std::size_t>(width);
        for (engine::gfx::HdrImage* image : images) {
            float* row = image->rgba.data() + (rowStart * 4);
            std::fill(row, row + (static_cast<std::size_t>(width) * 4), 0.0F);
        }
        float* zRow = zbuffer.data() + rowStart;
        int* winnerRow = winners.data() + rowStart;
        std::fill(zRow, zRow + width, std::numeric_limits<float>::max());
        std::fill(winnerRow, winnerRow + width, -1);
        for (int x = 0; x < width; ++x) {
            writeTexel(result.iorAov, x, y, glm::vec3(-1.0F));
        }

        depthPassRow(y, subTriangles, rowBuckets, grid, zRow, winnerRow);
        shadeRow(result, y, width, subTriangles, shadingTriangles, instances, perInstanceSettings, grid,
                 zRow, winnerRow);
        drawBoxEdgesRow(result, y, boxEdges, boxRowBuckets, zRow);
    };

    threadPool.parallelFor(height, renderRow);
}

}  // namespace engine::scene
