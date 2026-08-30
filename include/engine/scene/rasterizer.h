#pragma once

#include <vector>

#include "engine/gfx/hdr_image.h"
#include "engine/scene/camera.h"
#include "engine/scene/embree_accel.h"
#include "engine/scene/gltf_loader.h"
#include "engine/scene/path_tracer.h"
#include "engine/scene/row_thread_pool.h"
#include "engine/scene/shading_scene.h"

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
};

// Row-parallel (RowThreadPool, dispatched over rows -- each worker owns disjoint rows, so no synchronization is needed on the shared z-buffer/output images) edge-function rasterization (Pineda 1988) with a single near-plane Sutherland-Hodgman clip per triangle. Runs synchronously on the render thread every frame, so these AOVs are correct from frame 1; Beauty and the light-transport AOVs are untouched, still converging asynchronously through PathTraceDriver. threadPool: owned by the caller and reused across calls, same convention as renderPathTraced's own parameter.
[[nodiscard]] RasterGBuffer renderRasterGBuffer(const Camera& camera, const EmbreeAccel& accel,
                                                 const std::vector<ShadingTriangle>& shadingTriangles,
                                                 const std::vector<MeshInstance>& instances,
                                                 const PathTraceSettings& settings, int width,
                                                 int height, RowThreadPool& threadPool);

}  // namespace engine::scene
