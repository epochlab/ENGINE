#include "engine/scene/rasterizer.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

#include "engine/scene/bsdf.h"
#include "engine/scene/false_color.h"
#include "engine/scene/gbuffer_shading.h"

namespace engine::scene {

namespace {

// Barycentric-linear (position, origU, origV) vertex -- origU/origV are weights on the ORIGINAL (unclipped) triangle's v1/v2 (Hit's Moller-Trumbore convention), so a clipped sub-triangle still resolves via interpolateShading(originalTriangle, ...); barycentric coords are affine in position, so lerping this tuple along a clip edge is exact. Box edges (which have no "original triangle") just pass 0,0 -- unused there.
struct ClipVertex {
    glm::vec3 position;
    float origU;
    float origV;
};

// A ClipVertex projected to screen space -- sx/sy are pixel coordinates (row 0 = top, HdrImage's convention), invZ is 1/viewZ for perspective-correct interpolation.
struct ScreenVertex {
    float sx;
    float sy;
    float invZ;
    float origU;
    float origV;
};

// One clipped, screen-projected, winding-normalized sub-triangle ready for scan-conversion.
struct RasterSubTriangle {
    ScreenVertex v0;
    ScreenVertex v1;
    ScreenVertex v2;
    float invArea;
    int minX;
    int maxX;
    int minY;
    int maxY;
    int triangleIndex;  // indexes shadingTriangles -- resolves material/instance and the original triangle for interpolateShading
};

// One clipped, screen-projected bounding-box edge ready for line rasterization -- see buildBoxEdges.
struct RasterLineSegment {
    glm::vec2 p0;
    glm::vec2 p1;
    float invZ0;
    float invZ1;
    int minX;
    int maxX;
    int minY;
    int maxY;
};

// Fixed on-screen line thickness in pixels, constant regardless of triangle/box size or distance.
constexpr float kLineThicknessPx = 1.0F;

const glm::vec3 kWireframeColor(1.0F, 1.0F, 1.0F);
const glm::vec3 kBoundingBoxColor(1.0F, 1.0F, 0.0F);

// 2D cross product (b-a) x (p-a), the Pineda 1988 edge function -- positive when p is left of directed edge a->b.
float edgeFunction(const ScreenVertex& a, const ScreenVertex& b, float px, float py) {
    return ((b.sx - a.sx) * (py - a.sy)) - ((b.sy - a.sy) * (px - a.sx));
}

// Barycentric coverage of one pixel centre against a sub-triangle. Shared by the depth pass and the shading pass rather than duplicated: identical source expressions guarantee identical floating-point contraction at both call sites, so the barycentrics the shading pass recomputes for a winner are bit-for-bit the ones the depth pass chose it on.
struct Coverage {
    bool covered;
    float b0;
    float b1;
    float b2;
};

Coverage coverPixel(const RasterSubTriangle& st, float px, float py) {
    const float w0 = edgeFunction(st.v1, st.v2, px, py);
    const float w1 = edgeFunction(st.v2, st.v0, px, py);
    const float w2 = edgeFunction(st.v0, st.v1, px, py);
    if (w0 < 0.0F || w1 < 0.0F || w2 < 0.0F) {
        return Coverage{false, 0.0F, 0.0F, 0.0F};
    }
    return Coverage{true, w0 * st.invArea, w1 * st.invArea, w2 * st.invArea};
}

// Single-plane Sutherland-Hodgman clip against viewZ >= nearClip -- one plane adds at most one vertex, so `out` never needs more than 4 slots; fixed 3-iteration loop (one per input edge) gives a provable bound. Returns the clipped vertex count (0 if fully behind the near plane).
int clipNearPlane(const std::array<ClipVertex, 3>& in, const glm::vec3& camPos,
                   const glm::vec3& forward, float nearClip, std::array<ClipVertex, 4>& out) {
    int count = 0;
    for (int i = 0; i < 3; ++i) {
        const ClipVertex& cur = in[static_cast<std::size_t>(i)];
        const ClipVertex& nxt = in[static_cast<std::size_t>((i + 1) % 3)];
        const float curZ = glm::dot(cur.position - camPos, forward);
        const float nxtZ = glm::dot(nxt.position - camPos, forward);
        const bool curIn = curZ >= nearClip;
        const bool nxtIn = nxtZ >= nearClip;
        if (curIn) {
            out[static_cast<std::size_t>(count++)] = cur;
        }
        if (curIn != nxtIn) {
            const float t = (nearClip - curZ) / (nxtZ - curZ);
            out[static_cast<std::size_t>(count++)] =
                ClipVertex{glm::mix(cur.position, nxt.position, t), glm::mix(cur.origU, nxt.origU, t),
                           glm::mix(cur.origV, nxt.origV, t)};
        }
    }
    return count;
}

// Two-vertex near-plane clip for box edges -- simpler than clipNearPlane (no fan-triangulation): 0 output vertices if both endpoints are behind the near plane, otherwise exactly 2.
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

// Inverts Camera::primaryRay's ndcX/ndcY->direction math -- the basis is orthonormal, so a world point's fwd/right/up components relative to the camera give ndcX/ndcY directly, then renderRasterGBuffer's own pixel-center convention maps those to pixel coordinates.
ScreenVertex projectToScreen(const ClipVertex& v, const glm::vec3& camPos,
                              const Camera::ViewBasis& basis, int width, int height) {
    const glm::vec3 d = v.position - camPos;
    const float viewZ = glm::dot(d, basis.forward);
    const float ndcX = glm::dot(d, basis.right) / (viewZ * basis.halfWidth);
    const float ndcY = glm::dot(d, basis.up) / (viewZ * basis.halfHeight);
    const float sx = ((ndcX + 1.0F) * 0.5F) * static_cast<float>(width);
    const float sy = ((1.0F - ndcY) * 0.5F) * static_cast<float>(height);
    return ScreenVertex{sx, sy, 1.0F / viewZ, v.origU, v.origV};
}

constexpr float kMinTriangleArea = 1e-6F;  // screen-space pixel^2 -- filters near-edge-on/degenerate triangles

// Normalizes winding to positive screen-space area (glTF mirrored-scale nodes and Embree's double-sided intersection mean input winding isn't fixed), clamps the bbox to the viewport, appends to `out` -- no-op if degenerate or off-screen.
void pushSubTriangle(ScreenVertex v0, ScreenVertex v1, ScreenVertex v2, int triangleIndex,
                      int width, int height, std::vector<RasterSubTriangle>& out) {
    float area = edgeFunction(v0, v1, v2.sx, v2.sy);
    if (std::fabs(area) < kMinTriangleArea) {
        return;
    }
    if (area < 0.0F) {
        std::swap(v1, v2);
        area = -area;
    }
    const int minX = std::max(0, static_cast<int>(std::floor(std::min({v0.sx, v1.sx, v2.sx}))));
    const int maxX =
        std::min(width - 1, static_cast<int>(std::ceil(std::max({v0.sx, v1.sx, v2.sx}))));
    const int minY = std::max(0, static_cast<int>(std::floor(std::min({v0.sy, v1.sy, v2.sy}))));
    const int maxY =
        std::min(height - 1, static_cast<int>(std::ceil(std::max({v0.sy, v1.sy, v2.sy}))));
    if (minX > maxX || minY > maxY) {
        return;
    }
    out.push_back(RasterSubTriangle{v0, v1, v2, 1.0F / area, minX, maxX, minY, maxY, triangleIndex});
}

// Clips/projects/winding-normalizes every triangle once per call, in parallel over chunks of the triangle list -- cheap per-triangle math, but at a few million triangles doing it on one thread is a frame-rate ceiling by itself. Each chunk appends to its own vector, so the appends need no synchronization, and the chunks are concatenated in order afterwards: the result is the identical sequence a sequential build produces, which matters because the z-test below is first-writer-wins at exactly equal depth.
std::vector<RasterSubTriangle> buildSubTriangles(const Camera& camera,
                                                  const std::vector<ShadingTriangle>& shadingTriangles,
                                                  int width, int height, ThreadPool& threadPool) {
    const glm::vec3 camPos = camera.position();
    const Camera::ViewBasis basis = camera.viewBasis(static_cast<float>(width) / static_cast<float>(height));

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
            const std::array<ClipVertex, 3> verts{
                ClipVertex{tri.v0.position, 0.0F, 0.0F}, ClipVertex{tri.v1.position, 1.0F, 0.0F},
                ClipVertex{tri.v2.position, 0.0F, 1.0F}};
            std::array<ClipVertex, 4> clipped{};
            const int clippedCount =
                clipNearPlane(verts, camPos, basis.forward, camera.nearClip(), clipped);
            if (clippedCount < 3) {
                continue;
            }
            std::array<ScreenVertex, 4> screen{};
            for (int k = 0; k < clippedCount; ++k) {
                screen[static_cast<std::size_t>(k)] =
                    projectToScreen(clipped[static_cast<std::size_t>(k)], camPos, basis, width, height);
            }
            pushSubTriangle(screen[0], screen[1], screen[2], i, width, height, out);
            if (clippedCount == 4) {
                pushSubTriangle(screen[0], screen[2], screen[3], i, width, height, out);
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
// The cost this removes is memory bandwidth, not comparisons: RasterSubTriangle is 84 bytes and the scan is linear, so every row streamed the entire array. That is invisible while the array fits in cache -- at the 20561-triangle scene it is 1.7 MB and the scan costs nothing measurable -- and dominant once it does not: at 5M triangles it is 420 MB, read 1152 times per frame.
// Memory is O(sum of row spans), so a scene of few very large triangles can need more of it than the sub-triangle array itself. That is the opposite regime from the one this exists for, where triangles are small and each spans a handful of rows.
struct RowBuckets {
    std::vector<std::size_t> offsets;  // height+1 entries, offsets[y]..offsets[y+1] is row y's range in `indices`
    std::vector<int> indices;          // indexes subTriangles; int since a clip splits at most one triangle into two, bounding this by 2x the scene's triangle count
};

RowBuckets buildRowBuckets(const std::vector<RasterSubTriangle>& subTriangles, int height) {
    RowBuckets buckets;
    buckets.offsets.assign(static_cast<std::size_t>(height) + 1, 0);
    for (const RasterSubTriangle& st : subTriangles) {
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
    for (std::size_t i = 0; i < subTriangles.size(); ++i) {
        const RasterSubTriangle& st = subTriangles[i];
        for (int y = st.minY; y <= st.maxY; ++y) {
            buckets.indices[cursor[static_cast<std::size_t>(y)]++] = static_cast<int>(i);
        }
    }
    return buckets;
}

// Builds the AABB's 12 edges (8-corner topology), near-clipped and projected once per call -- mirrors buildSubTriangles' role for a fixed 12 segments instead of the scene's triangle list.
std::vector<RasterLineSegment> buildBoxEdges(const Camera& camera, const AabbBounds& box, int width,
                                              int height) {
    const glm::vec3 camPos = camera.position();
    const Camera::ViewBasis basis = camera.viewBasis(static_cast<float>(width) / static_cast<float>(height));
    const std::array<glm::vec3, 8> corners{
        glm::vec3(box.min.x, box.min.y, box.min.z), glm::vec3(box.max.x, box.min.y, box.min.z),
        glm::vec3(box.max.x, box.max.y, box.min.z), glm::vec3(box.min.x, box.max.y, box.min.z),
        glm::vec3(box.min.x, box.min.y, box.max.z), glm::vec3(box.max.x, box.min.y, box.max.z),
        glm::vec3(box.max.x, box.max.y, box.max.z), glm::vec3(box.min.x, box.max.y, box.max.z)};
    constexpr std::array<std::array<int, 2>, 12> kEdges{
        {{0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6}, {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}}};

    std::vector<RasterLineSegment> segments;
    segments.reserve(kEdges.size());
    for (const std::array<int, 2>& edge : kEdges) {
        glm::vec3 a{};
        glm::vec3 b{};
        if (clipSegmentNearPlane(corners[static_cast<std::size_t>(edge[0])],
                                  corners[static_cast<std::size_t>(edge[1])], camPos, basis.forward,
                                  camera.nearClip(), a, b) == 0) {
            continue;
        }
        const ScreenVertex sa = projectToScreen(ClipVertex{a, 0.0F, 0.0F}, camPos, basis, width, height);
        const ScreenVertex sb = projectToScreen(ClipVertex{b, 0.0F, 0.0F}, camPos, basis, width, height);
        const int minX = std::max(0, static_cast<int>(std::floor(std::min(sa.sx, sb.sx))));
        const int maxX = std::min(width - 1, static_cast<int>(std::ceil(std::max(sa.sx, sb.sx))));
        const int minY = std::max(0, static_cast<int>(std::floor(std::min(sa.sy, sb.sy))));
        const int maxY = std::min(height - 1, static_cast<int>(std::ceil(std::max(sa.sy, sb.sy))));
        if (minX > maxX || minY > maxY) {
            continue;
        }
        segments.push_back(RasterLineSegment{{sa.sx, sa.sy}, {sb.sx, sb.sy}, sa.invZ, sb.invZ, minX, maxX,
                                              minY, maxY});
    }
    return segments;
}

// Resolves and writes every G-buffer field for one covered, z-winning pixel -- same sampling calls tracePath's bounce-0 block makes (gbuffer_shading.h), never a lighting/BSDF evaluation. origU/origV are the perspective-correct barycentric coordinates on the ORIGINAL (unclipped) triangle. wireframe is a screen-space distance-to-edge test against the triangle's own 3 projected edges (v0/v1/v2), sharing nearLineSegmentPx with BoundingBox -- evaluated only at this already-z-tested pixel, so hidden-line removal is free.
void shadePixel(RasterGBuffer& result, int x, int y, float viewZ, float origU, float origV,
                 const ShadingTriangle& triangle, const Material& material,
                 const PathTraceSettings& settings, const glm::vec3& camPos, const ScreenVertex& v0,
                 const ScreenVertex& v1, const ScreenVertex& v2) {
    const ShadingVertex shading = interpolateShading(triangle, origU, origV);
    const ShadingFrame frame = buildShadingFrame(shading, material, settings);
    const BsdfParams params = resolveBsdfParams(material, shading.uv, settings);
    const glm::vec3 woWorld = glm::normalize(camPos - shading.position);
    const float ndotV = std::max(glm::dot(frame.normal, woWorld), 1e-4F);
    const float fresnelVal = fresnelSchlick(ndotV, params.f0).x;
    const float aoVal = engine::gfx::sampleBilinear(material.aoTexture, shading.uv).r;

    const glm::vec2 p(static_cast<float>(x) + 0.5F, static_cast<float>(y) + 0.5F);
    const glm::vec2 p0(v0.sx, v0.sy);
    const glm::vec2 p1(v1.sx, v1.sy);
    const glm::vec2 p2(v2.sx, v2.sy);
    const bool wire = nearLineSegmentPx(p, p0, p1, kLineThicknessPx).near ||
                      nearLineSegmentPx(p, p1, p2, kLineThicknessPx).near ||
                      nearLineSegmentPx(p, p2, p0, kLineThicknessPx).near;

    writeTexel(result.depth, x, y, glm::vec3(viewZ));
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
    writeTexel(result.fresnel, x, y, glm::vec3(fresnelVal, 1.0F - fresnelVal, 0.0F));
    writeTexel(result.ao, x, y, glm::vec3(aoVal));
    writeTexel(result.wireframe, x, y, wire ? kWireframeColor : glm::vec3(0.0F));
    writeTexel(result.iorAov, x, y, glm::vec3(settings.ior));
}

// Pass one of two: resolves visibility for the row without shading anything, recording each pixel's depth and the sub-triangle index that owns it. Splitting this out is what bounds shading to one evaluation per visible pixel -- the single-pass form shaded on every depth improvement, so a pixel behind N nearer-in-list surfaces paid N full shades (8 bilinear fetches, a shading frame and 15 texel writes each) to keep one.
// Keeps the single-pass tie-break exactly: `>=` rejects equal depth, so the first sub-triangle in row order still wins a tie, and row order is the sub-triangle list order buildRowBuckets preserves.
void depthPassRow(int y, const std::vector<RasterSubTriangle>& subTriangles,
                   const RowBuckets& rowBuckets, float* zRow, int* winnerRow) {
    const float py = static_cast<float>(y) + 0.5F;
    const std::size_t rowEnd = rowBuckets.offsets[static_cast<std::size_t>(y) + 1];
    for (std::size_t k = rowBuckets.offsets[static_cast<std::size_t>(y)]; k < rowEnd; ++k) {
        const int index = rowBuckets.indices[k];
        const RasterSubTriangle& st = subTriangles[static_cast<std::size_t>(index)];
        for (int x = st.minX; x <= st.maxX; ++x) {
            const Coverage cov = coverPixel(st, static_cast<float>(x) + 0.5F, py);
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

// Pass two of two: shades each covered pixel exactly once from the winner the depth pass recorded. Walks the row in x order rather than in triangle order, so the 15 AOV writes advance linearly through each image instead of scattering across it.
// viewZ is read back from the depth buffer rather than recomputed: it is the value this same winner stored, so reading it is both cheaper and exact where a recomputation would only be exact by argument.
void shadeRow(RasterGBuffer& result, int y, int width, const std::vector<RasterSubTriangle>& subTriangles,
               const std::vector<ShadingTriangle>& shadingTriangles,
               const std::vector<MeshInstance>& instances, const PathTraceSettings& settings,
               const glm::vec3& camPos, const float* zRow, const int* winnerRow) {
    const float py = static_cast<float>(y) + 0.5F;
    for (int x = 0; x < width; ++x) {
        if (winnerRow[x] < 0) {
            continue;
        }
        const RasterSubTriangle& st = subTriangles[static_cast<std::size_t>(winnerRow[x])];
        const Coverage cov = coverPixel(st, static_cast<float>(x) + 0.5F, py);
        const float viewZ = zRow[x];
        const float origU = ((cov.b0 * st.v0.origU * st.v0.invZ) + (cov.b1 * st.v1.origU * st.v1.invZ) +
                             (cov.b2 * st.v2.origU * st.v2.invZ)) *
                            viewZ;
        const float origV = ((cov.b0 * st.v0.origV * st.v0.invZ) + (cov.b1 * st.v1.origV * st.v1.invZ) +
                             (cov.b2 * st.v2.origV * st.v2.invZ)) *
                            viewZ;
        const ShadingTriangle& triangle = shadingTriangles[static_cast<std::size_t>(st.triangleIndex)];
        const Material& material = instances[static_cast<std::size_t>(triangle.instanceIndex)].material;
        shadePixel(result, x, y, viewZ, origU, origV, triangle, material, settings, camPos, st.v0, st.v1,
                   st.v2);
    }
}

// Bounding-box edges: real line segments z-tested against the row's now-finalized depth (real geometry occludes them) but never written back to it, so box edges never occlude each other -- all 12 show unless real mesh blocks them. Drawn into the same wireframe AOV as the mesh edges, in yellow, taking precedence over white where both apply.
void drawBoxEdgesRow(RasterGBuffer& result, int y, const std::vector<RasterLineSegment>& boxEdges,
                      const float* zRow) {
    for (const RasterLineSegment& seg : boxEdges) {
        if (y < seg.minY || y > seg.maxY) {
            continue;
        }
        for (int x = seg.minX; x <= seg.maxX; ++x) {
            const glm::vec2 p(static_cast<float>(x) + 0.5F, static_cast<float>(y) + 0.5F);
            const LineProximity prox = nearLineSegmentPx(p, seg.p0, seg.p1, kLineThicknessPx);
            if (!prox.near) {
                continue;
            }
            const float viewZ = 1.0F / glm::mix(seg.invZ0, seg.invZ1, prox.t);
            if (viewZ <= zRow[x]) {
                writeTexel(result.wireframe, x, y, kBoundingBoxColor);
            }
        }
    }
}

// Every AOV image in one place, so the reallocation and the per-row clear below cannot disagree about which fields exist -- adding an AOV to RasterGBuffer without adding it here leaves it uncleared, which this array's fixed size catches at compile time.
std::array<engine::gfx::HdrImage*, 15> aovImages(RasterGBuffer& g) {
    return {&g.iorAov, &g.depth,    &g.worldPos, &g.uv,      &g.normal,
            &g.geomNormal, &g.albedo, &g.metallic, &g.roughness, &g.tangent,
            &g.objectId, &g.alpha,  &g.fresnel,  &g.ao,      &g.wireframe};
}

}  // namespace

void renderRasterGBuffer(const Camera& camera, const EmbreeAccel& accel,
                          const std::vector<ShadingTriangle>& shadingTriangles,
                          const std::vector<MeshInstance>& instances,
                          const PathTraceSettings& settings, int width, int height,
                          ThreadPool& threadPool, RasterGBuffer& result) {
    const std::array<engine::gfx::HdrImage*, 15> images = aovImages(result);
    // Reallocated only on a resolution change; every other call reuses the storage and relies on renderRow's clear. makeImage's own zeroing is redundant against that clear but runs once per resize, not once per frame.
    if (result.depth.width != width || result.depth.height != height) {
        for (engine::gfx::HdrImage* image : images) {
            *image = makeImage(width, height);
        }
    }
    ++result.generation;

    const std::vector<RasterSubTriangle> subTriangles =
        buildSubTriangles(camera, shadingTriangles, width, height, threadPool);
    const RowBuckets rowBuckets = buildRowBuckets(subTriangles, height);
    const AabbBounds sceneBounds = accel.sceneBounds();
    const std::vector<RasterLineSegment> boxEdges = buildBoxEdges(camera, sceneBounds, width, height);
    const std::size_t pixelCount = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    std::vector<float> zbuffer(pixelCount);
    // The depth pass's other output: which sub-triangle owns each pixel, -1 for uncovered. 4 bytes per pixel, sized like the z-buffer because both are written by whichever worker owns the row.
    std::vector<int> winners(pixelCount);

    const glm::vec3 camPos = camera.position();

    const auto renderRow = [&](int y) {
        // Clearing this row of every AOV is what makes the buffers reusable across calls: the worker that is about to overwrite the row zeroes it first, in parallel and while it is already cache-warm, instead of 15 sequential full-image memsets before the dispatch. An uncovered pixel therefore still reads back zero (alpha 0, the miss test every consumer uses) exactly as a freshly allocated image did.
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

        depthPassRow(y, subTriangles, rowBuckets, zRow, winnerRow);
        shadeRow(result, y, width, subTriangles, shadingTriangles, instances, settings, camPos, zRow,
                 winnerRow);
        drawBoxEdgesRow(result, y, boxEdges, zRow);
    };

    threadPool.parallelFor(height, renderRow);
}

}  // namespace engine::scene
