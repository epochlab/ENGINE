#pragma once

#include <cstdint>
#include <vector>

#include "engine/gfx/hdr_image.h"
#include "engine/scene/camera.h"
#include "engine/scene/embree_accel.h"
#include "engine/scene/gltf_loader.h"
#include "engine/scene/path_tracer.h"
#include "engine/scene/shading_scene.h"
#include "engine/scene/thread_pool.h"

namespace engine::scene {

// Primary-hit-only G-buffer AOVs computed by a standalone CPU rasterizer instead of tracing an Embree primary ray per pixel -- no lighting model, no BSDF sampling, no recursion, just geometry projection plus gbuffer_shading.h's material/texture sampling. Sole producer of all 15: main.cpp's selectPathTracedImage routes every one of these AOVs here, and the path tracer computes none of them (path_tracer.h).
struct RasterGBuffer {
    engine::gfx::HdrImage iorAov;
    engine::gfx::HdrImage depth;
    engine::gfx::HdrImage worldPos;
    engine::gfx::HdrImage uv;
    engine::gfx::HdrImage normal;
    engine::gfx::HdrImage geomNormal;
    engine::gfx::HdrImage albedo;
    engine::gfx::HdrImage metallic;
    engine::gfx::HdrImage roughness;
    engine::gfx::HdrImage tangent;
    engine::gfx::HdrImage objectId;
    engine::gfx::HdrImage alpha;
    engine::gfx::HdrImage fresnel;
    engine::gfx::HdrImage ao;
    // Color-coded, not a plain 0/1 mask: white (1,1,1) near a mesh triangle edge, yellow (1,1,0) near a scene-bounding-box edge (drawn on top, so yellow wins where both apply), black elsewhere.
    engine::gfx::HdrImage wireframe;
    // Bumped by every renderRasterGBuffer call. The buffer is reused in place rather than republished, so its address no longer changes between renders and a consumer caching by pointer identity (main.cpp's display texture) would never see an update -- this is what it keys on instead. 0 means no render has run yet and the images are still empty.
    std::uint64_t generation = 0;
};

// Row-parallel (ThreadPool, dispatched over rows -- each worker owns disjoint rows, so no synchronization is needed on the shared z-buffer/output images) edge-function rasterization (Pineda 1988) with a single near-plane Sutherland-Hodgman clip per triangle. Runs synchronously on the render thread, so these AOVs are correct on the frame a rasterizer-backed AOV is selected; Beauty and the light-transport AOVs are untouched, still converging asynchronously through PathTraceDriver. threadPool: owned by the caller and reused across calls, same convention as renderPathTraced's own parameter.
// out: owned by the caller and reused across calls, like threadPool. Its 15 images are reallocated only when width/height change and are otherwise cleared per row inside the parallel loop -- the same bytes the old per-call makeImage zeroing touched, but written in parallel, in the row about to be overwritten, instead of as 15 sequential full-image memsets beforehand. At 2048x1152 that allocate-and-zero was 566 MB per call.
void renderRasterGBuffer(const Camera& camera, const EmbreeAccel& accel,
                          const std::vector<ShadingTriangle>& shadingTriangles,
                          const std::vector<MeshInstance>& instances,
                          const PathTraceSettings& settings, int width, int height,
                          ThreadPool& threadPool, RasterGBuffer& out);

}  // namespace engine::scene
