#pragma once

#include <cstdint>
#include <vector>

#include "engine/gfx/hdr_image.h"
#include "engine/scene/bvh.h"
#include "engine/scene/camera.h"
#include "engine/scene/environment_map.h"
#include "engine/scene/gltf_loader.h"

namespace engine::scene {

struct PathTraceSettings {
    int samplesPerPixel;
    int maxBounces;
    int russianRouletteStartBounce;
    float rrMinProb = 0.05F;
    float rrMaxProb = 0.95F;
};

// bounceHeatmap/iorAov: single-channel data broadcast to RGB (alpha=1), matching HdrImage's fixed
// 4-floats/texel layout so both can go straight through Texture::createFromFloatPixels unchanged.
struct PathTraceResult {
    engine::gfx::HdrImage beauty;
    engine::gfx::HdrImage iorAov;          // per-material IOR at the primary hit, -1 = miss
    engine::gfx::HdrImage bounceHeatmap;   // mean bounce depth at termination, across samples
};

// Blocking, multithreaded (one thread per hardware core, dynamic row scheduling) unidirectional
// path trace: BSDF-sampled recursive bounces (no NEE -- Phase 7), Russian roulette from
// russianRouletteStartBounce, radiance from the environment map on a miss. No punctual-light
// contribution -- point/directional lights have no hittable geometry, matching this phase's
// documented no-NEE scope.
[[nodiscard]] PathTraceResult renderPathTraced(const Camera& camera, const Bvh& bvh,
                                                const std::vector<ShadingTriangle>& shadingTriangles,
                                                const std::vector<MeshInstance>& instances,
                                                const EnvironmentMap& environmentMap, int width,
                                                int height, float envRotationRadians,
                                                const PathTraceSettings& settings,
                                                std::uint32_t runSeed);

}  // namespace engine::scene
