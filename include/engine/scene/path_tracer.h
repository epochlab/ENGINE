#pragma once

#include <atomic>
#include <cstdint>
#include <vector>

#include <glm/glm.hpp>

#include "engine/gfx/hdr_image.h"
#include "engine/scene/camera.h"
#include "engine/scene/embree_accel.h"
#include "engine/scene/environment_map.h"
#include "engine/scene/gltf_loader.h"
#include "engine/scene/row_thread_pool.h"

namespace engine::scene {

// samplesPerPixel is "samples per renderPathTraced() call" -- when driven progressively by PathTraceDriver, that's samples per accumulated pass (typically 1), not the total sample count of the converged image; convergence comes from the driver accumulating many passes, not from a large value here.
struct PathTraceSettings {
    int samplesPerPixel;
    int maxBounces;  // secondary/indirect bounces beyond the always-traced primary hit; 0 = direct lighting only
    int russianRouletteStartBounce;
    float rrMinProb = 0.05F;  // floor: stops a near-zero-throughput path being killed with near-certainty
    // Ceiling of exactly 1.0: a path carrying full throughput must never be terminated. Any lower caps survival for no gain -- it saves a fraction of deep-path tracing and pays for it with variance costing more than that fraction in extra samples.
    float rrMaxProb = 1.0F;
    // Sourced from MaterialConfig/material.json -- see resolveRoughness/buildShadingFrame (path_tracer.cpp).
    float bumpStrength;
    float roughnessMin;
    float roughnessMax;
    // Global material override, sourced from MaterialConfig/material.json -- see resolveBsdfParams (path_tracer.cpp).
    glm::vec3 diffuseColour;
    float ior;
    float transmissionFactor;
    float metallicFactor;
    float roughnessFactor;
};

// One renderPathTraced() call's raw output -- what a single pass computes, and what PathTraceDriver republishes in full on every accumulated pass. Single-channel fields are broadcast to RGB (alpha=1), matching HdrImage's fixed 4-floats/texel layout so every field can go straight through Texture::createFromFloatPixels unchanged. Every field here is a per-sample quantity averaged across the call's samples, and re-averaged across accumulated passes by the driver. The primary-hit G-buffer AOVs (depth/worldPos/normal/albedo/metallic/roughness/tangent/objectId/alpha/fresnel/ao/uv/geomNormal/IOR) are NOT here: rasterizer.h's RasterGBuffer is their only producer, refreshed synchronously on the render thread every trigger change, and main.cpp's selectPathTracedImage routed every one of them there -- the path-traced copies were computed, stored and published to no reader at all. Wireframe/BoundingBox are likewise rasterizer.h-only.
struct PathTraceResult {
    engine::gfx::HdrImage beauty;
    engine::gfx::HdrImage bounceHeatmap;   // mean bounce depth at termination, across samples
    engine::gfx::HdrImage shadow;          // fraction of the primary hit's NEE samples toward the env light that were occluded -- 1.0 = fully shadowed, 0.0 = fully lit or no primary hit (background); averaged across samples and re-averaged across passes, so it converges from a binary per-sample test into continuous soft-shadow/penumbra density

    // Light-transport component breakdown, replacing a single combined "IBL" term. Averaged the same way beauty is (across samples/passes), and PHYSICAL: every value written here is the same radiance that went into beauty, attributed rather than rescaled. THE FIVE BUCKETS PLUS BACKGROUND SUM TO BEAUTY EXACTLY -- background being the bounce-0 miss, the camera seeing the environment with no surface interaction, which is deliberately unbucketed (production renderers keep it out of the surface-transport AOVs too) and is the only radiance beauty carries that these five do not. tools/integrator_validate.cpp asserts the identity per pixel with showSky off, which zeroes the background term.
    // Bucketing rule: a path is bucketed once, by the lobe sampled at its first (bounce 0) surface interaction, independent of however many further bounces it takes -- except that any transmission-lobe sample, at bounce 0 or later, stickily overrides the bucket to Refraction from that point on. Direct vs Indirect is not tracked separately: it falls out of which bounce the radiance-contributing event lands on, bounce 0's NEE and bounce 1's BSDF-sampled miss being the two halves of the same one-vertex path. Bounce 0's NEE contribution is the one place a single sample writes several buckets, split by the lobe that actually carried the light (bsdf.h's BsdfEval) rather than by the lobe the continuation ray drew.
    // Not an LPE. Bucketing by first event describes a debug viewer's decomposition, not an arbitrary path expression -- a diffuse bounce off a red wall onto a specular surface still reads as diffuse transport, and later-bounce surfaces' colours legitimately tint the indirect buckets.
    engine::gfx::HdrImage directDiffuse;
    engine::gfx::HdrImage indirectDiffuse;
    engine::gfx::HdrImage directSpecular;
    engine::gfx::HdrImage indirectSpecular;
    engine::gfx::HdrImage refraction;
};

// Blocking, multithreaded (one thread per hardware core, dynamic row scheduling) unidirectional path trace: BSDF-sampled recursive bounces with next-event estimation (environment-only, MIS power heuristic against BSDF sampling), Russian roulette from russianRouletteStartBounce. No punctual-light contribution -- point/directional lights have no hittable geometry. showSky: gates only the primary ray's own miss (the camera seeing the background directly) -- indirect bounces and NEE always sample real environment radiance regardless, so hiding the background doesn't unlight the scene. generation/requestedGeneration: cooperative cancellation for PathTraceDriver's async use -- each row worker checks generation.load() != requestedGeneration once per row (cheap, same polling idiom as the row-stealing atomic inside RowThreadPool) and returns early if a newer request has superseded this one, leaving the (discarded) result's unwritten rows however writeTexel last left them. A direct/synchronous caller not using cancellation can pass a generation atomic holding requestedGeneration's own value, which never goes stale. threadPool: row-parallel dispatch, owned by the caller and reused across calls (PathTraceDriver keeps one alive for its whole lifetime) -- avoids paying OS thread-creation/join cost on every pass.
[[nodiscard]] PathTraceResult renderPathTraced(const Camera& camera, const EmbreeAccel& accel,
                                                const std::vector<ShadingTriangle>& shadingTriangles,
                                                const std::vector<MeshInstance>& instances,
                                                const EnvironmentMap& environmentMap, int width,
                                                int height, float envRotationRadians, bool showSky,
                                                float envExposure, const PathTraceSettings& settings,
                                                std::uint32_t runSeed,
                                                const std::atomic<std::uint64_t>& generation,
                                                std::uint64_t requestedGeneration,
                                                RowThreadPool& threadPool);

}  // namespace engine::scene
