#pragma once

#include <atomic>
#include <cstdint>
#include <vector>

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
    float rrMinProb = 0.05F;
    float rrMaxProb = 0.95F;
    // Sourced from MaterialConfig/material.json -- see resolveRoughness/buildShadingFrame (path_tracer.cpp).
    float bumpStrength;
    float roughnessMin;
    float roughnessMax;
};

// Single-channel fields are broadcast to RGB (alpha=1), matching HdrImage's fixed 4-floats/texel layout so every field can go straight through Texture::createFromFloatPixels unchanged. beauty/bounceHeatmap are averaged across every sample of every call (and, under PathTraceDriver, across every accumulated pass); shadow is a single binary NEE sample within one call but is likewise re-averaged across accumulated passes under PathTraceDriver, converging into continuous soft-shadow density (see its own comment below). Every other field is read once, off the primary ray's hit record at sample 0 only (same precedent as iorAov) -- under PathTraceDriver they're populated on the first pass of a generation and left untouched on every later pass, since the primary hit is deterministic given an unchanged camera/scene and doesn't benefit from re-averaging. worldPos/normal/geomNormal are stored raw (world-space metres / unit vectors in [-1,1]), scene-referred values, not remapped to a [0,1] display range -- display-side remapping, if any, happens downstream. One renderPathTraced() call's raw output -- what a single pass computes. PathTraceDriver splits this into PathTraceGBuffer (published once, on pass 1) and PathTraceDynamic (republished every pass) at its publish boundary, since 16 of these 24 fields never change after the first pass; see those two structs' own doc comments. renderPathTraced itself stays unaware of that distinction -- it always computes and returns the full 24 fields, same as a synchronous/non-driver caller would want.
struct PathTraceResult {
    engine::gfx::HdrImage beauty;
    engine::gfx::HdrImage iorAov;          // per-material IOR at the primary hit, -1 = miss
    engine::gfx::HdrImage bounceHeatmap;   // mean bounce depth at termination, across samples

    // Primary-hit G-buffer AOVs.
    engine::gfx::HdrImage depth;       // planar camera-space Z, metres (Arnold/RenderMan/EXR "Z" convention); 0 on a primary miss
    engine::gfx::HdrImage worldPos;    // raw world-space hit position; 0 on a primary miss
    engine::gfx::HdrImage uv;          // fract(uv), 0 on a primary miss
    engine::gfx::HdrImage normal;      // shading (normal-mapped) normal, raw [-1,1]
    engine::gfx::HdrImage geomNormal;  // smooth interpolated vertex normal, before normal-mapping, raw [-1,1]
    engine::gfx::HdrImage albedo;      // base color, no lighting applied
    engine::gfx::HdrImage metallic;
    engine::gfx::HdrImage roughness;
    engine::gfx::HdrImage tangent;     // raw [-1,1]
    engine::gfx::HdrImage objectId;    // false-colored mesh instance id, see engine::scene::falseColorForId
    engine::gfx::HdrImage alpha;       // 1.0 on a primary hit, 0.0 on a primary miss -- a real coverage mask, since this renderer isn't opaque-only-by-construction
    engine::gfx::HdrImage fresnel;     // Schlick term at the primary hit's view angle
    engine::gfx::HdrImage ao;          // baked AO texture sample at the primary hit (not ray-traced AO)
    engine::gfx::HdrImage shadow;      // fraction of accumulated passes where the primary hit's NEE sample toward the env light was occluded -- 1.0 = fully shadowed, 0.0 = fully lit or no primary hit (background); re-averaged across passes like beauty, so it converges from a single pass's binary sample into continuous soft-shadow/penumbra density over time
    engine::gfx::HdrImage wireframe;   // 1.0 near a hit triangle's edge (barycentric distance), 0.0 elsewhere/no hit
    engine::gfx::HdrImage boundingBox; // 1.0 near an edge of the scene's wireframe bounding cube -- independent of mesh hit, drawn over background too

    // Light-transport component breakdown, replacing a single combined "IBL" term. Averaged the same way beauty is (across samples/passes). A path is bucketed once, by the lobe type sampled at its first (bounce 0) surface interaction (Diffuse/SpecularReflection), independent of however many further bounces it takes -- Direct vs Indirect falls out of whether the path's radiance-contributing event happens after exactly one bounce or more than one, not a separately tracked decision. Refraction is orthogonal to this: any transmission-lobe sample, at bounce 0 or any later bounce, stickily overrides the path's bucket to Refraction from that point on, regardless of what the bucket was before. refraction is PHYSICAL/unmodified. directDiffuse/indirectDiffuse and directSpecular/indirectSpecular are all DELIGHTED, not physical: at bounce 0, each isolates its own lobe's contribution -- from both NEE's shadow ray (evaluateDiffuseRaw / evaluateSpecularOnly, bsdf.h) and a BSDF-sampled continuation ray that misses geometry and hits the environment directly (BsdfSample::rawThroughputWeight, bsdf.h) -- from the vertex's combined diffuse+specular BSDF value, factoring out the primary surface's own base color texture where the lobe's own value carries it (kd in place of baseColor*kd for diffuse; the specular lobe's F*G2/G1 weight has no baseColor at metallic=0 to begin with). Because bounce 0's non-bucketed lobe is dropped rather than attributed elsewhere, these four buckets plus refraction do NOT sum to beauty (nor to each other) the way a naive partition would -- each reads as "how much light of this transport type is arriving," not a literal decomposition of the beauty image. Later-bounce surfaces' colors still legitimately tint the indirect buckets (that's real bounce transport, not this object's own texture).
    engine::gfx::HdrImage directDiffuse;
    engine::gfx::HdrImage indirectDiffuse;
    engine::gfx::HdrImage directSpecular;
    engine::gfx::HdrImage indirectSpecular;
    engine::gfx::HdrImage refraction;
};

// PathTraceResult's 16 primary-hit fields -- deterministic given an unchanged camera/scene, so PathTraceDriver captures these once (pass 1 of a generation) and never rebuilds/republishes them again, instead of paying their copy cost on every pass alongside the 8 fields that actually accumulate (see PathTraceDynamic). Field meanings are identical to PathTraceResult's own doc comments above.
struct PathTraceGBuffer {
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
    engine::gfx::HdrImage wireframe;
    engine::gfx::HdrImage boundingBox;
};

// PathTraceResult's 8 fields that genuinely re-average across passes -- see PathTraceGBuffer's doc comment for why these are split out. Field meanings are identical to PathTraceResult's own doc comments above.
struct PathTraceDynamic {
    engine::gfx::HdrImage beauty;
    engine::gfx::HdrImage bounceHeatmap;
    engine::gfx::HdrImage shadow;
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
