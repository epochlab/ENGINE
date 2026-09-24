#pragma once

#include <cstdint>
#include <vector>

#include "pathtracer/gfx/hdr_image.h"
#include "pathtracer/scene/camera.h"
#include "pathtracer/scene/gltf_loader.h"
#include "pathtracer/scene/path_tracer.h"
#include "pathtracer/scene/shading_scene.h"
#include "pathtracer/scene/thread_pool.h"

namespace pathtracer::scene {

// Primary-hit-only G-buffer AOVs from a standalone CPU rasterizer rather than an Embree primary ray per pixel: no
// lighting, no BSDF sampling, no recursion, just geometry and material lookups at the first surface.
struct RasterGBuffer {
    pathtracer::gfx::HdrImage iorAov;
    pathtracer::gfx::HdrImage depth;
    // `depth` on a fixed declared scale: 1 at the camera plane falling linearly to 0 at lookaheadDistance, clamped,
    // so a consumer reads proximity without sourcing a near/far pair.
    pathtracer::gfx::HdrImage lookahead;
    pathtracer::gfx::HdrImage worldPos;
    pathtracer::gfx::HdrImage uv;
    pathtracer::gfx::HdrImage normal;
    pathtracer::gfx::HdrImage geomNormal;
    pathtracer::gfx::HdrImage albedo;
    pathtracer::gfx::HdrImage metallic;
    pathtracer::gfx::HdrImage roughness;
    pathtracer::gfx::HdrImage tangent;
    pathtracer::gfx::HdrImage objectId;
    pathtracer::gfx::HdrImage alpha;
    // Colour-coded, not a plain 0/1 mask: white near a mesh triangle edge, each instance's falseColorForId hue
    // (false_color.h, the hue its ObjectID pixels carry) near the instance boundary.
    pathtracer::gfx::HdrImage wireframe;
    // Bumped by every renderRasterGBuffer call. The buffer is reused in place rather than republished, so a consumer
    // caching by pointer needs this to know the contents changed.
    std::uint64_t generation = 0;
};

// Watertight edge-function rasterization, row-parallel over ThreadPool: workers own disjoint rows, so the shared
// z-buffer and output images need no synchronization. out is caller-owned and reused, reallocated only on a size
// change. instanceBounds is one world-space AABB per instance (computeInstanceBounds), computed once at load.
void renderRasterGBuffer(const Camera& camera, const std::vector<ShadingTriangle>& shadingTriangles,
                          const std::vector<MeshInstance>& instances,
                          const std::vector<PathTraceSettings>& perInstanceSettings,
                          const std::vector<AabbBounds>& instanceBounds, int width, int height,
                          ThreadPool& threadPool, RasterGBuffer& out);

}  // namespace pathtracer::scene
