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

// samplesPerPixel is "samples per renderPathTraced() call" -- when driven progressively by
// PathTraceDriver, that's samples per accumulated pass (typically 1), not the total sample count of
// the converged image; convergence comes from the driver accumulating many passes, not from a large
// value here.
struct PathTraceSettings {
    int samplesPerPixel;
    int maxBounces;  // secondary/indirect bounces beyond the always-traced primary hit; 0 = direct lighting only
    int russianRouletteStartBounce;
    float rrMinProb = 0.05F;
    float rrMaxProb = 0.95F;
};

// Single-channel fields are broadcast to RGB (alpha=1), matching HdrImage's fixed 4-floats/texel
// layout so every field can go straight through Texture::createFromFloatPixels unchanged.
//
// beauty/bounceHeatmap are averaged across every sample of every call (and, under PathTraceDriver,
// across every accumulated pass); every other field below is read once, off the primary ray's hit
// record at sample 0 only (same precedent as iorAov) -- under PathTraceDriver they're populated on
// the first pass of a generation and left untouched on every later pass, since the primary hit is
// deterministic given an unchanged camera/scene and doesn't benefit from re-averaging.
//
// worldPos/normal/geomNormal are stored raw (world-space metres / unit vectors in [-1,1]), scene-referred
// values, not remapped to a [0,1] display range -- display-side remapping, if any, happens downstream.
//
// One renderPathTraced() call's raw output -- what a single pass computes. PathTraceDriver splits
// this into PathTraceGBuffer (published once, on pass 1) and PathTraceDynamic (republished every
// pass) at its publish boundary, since 14 of these 21 fields never change after the first pass; see
// those two structs' own doc comments. renderPathTraced itself stays unaware of that distinction --
// it always computes and returns the full 21 fields, same as a synchronous/non-driver caller would
// want.
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

    // Light-transport component breakdown, replacing a single combined "IBL" term. Averaged the same
    // way beauty is (across samples/passes). A path is bucketed once, by the lobe type sampled at its
    // first (bounce 0) surface interaction (Diffuse/SpecularReflection), independent of however many
    // further bounces it takes -- Direct vs Indirect falls out of whether the path's radiance-
    // contributing event happens after exactly one bounce or more than one, not a separately tracked
    // decision. Refraction is orthogonal to this: any transmission-lobe sample, at bounce 0 or any
    // later bounce, stickily overrides the path's bucket to Refraction from that point on, regardless
    // of what the bucket was before -- so directSpecular+indirectSpecular+refraction (PHYSICAL,
    // unmodified) plus the true (albedo-multiplied) diffuse contribution sums to beauty MINUS whatever
    // radiance the camera ray picked up by missing all geometry on bounce 0 (seeing the environment
    // directly -- not attributed to any of these five, the same way a "background" AOV is
    // conventionally kept separate from surface-interaction AOVs in production renderers).
    //
    // directDiffuse/indirectDiffuse are DELIGHTED, not physical: the primary (bounce-0) surface's own
    // base color texture is factored out (replaced by the diffuse lobe's raw kd weight) so these read
    // as "how much light is arriving," not "light times this object's own texture" -- so they do NOT
    // sum into beauty the way the other three buckets do. Later-bounce surfaces' colors still
    // legitimately tint indirectDiffuse (that's real bounce transport, not this object's own texture).
    engine::gfx::HdrImage directDiffuse;
    engine::gfx::HdrImage indirectDiffuse;
    engine::gfx::HdrImage directSpecular;
    engine::gfx::HdrImage indirectSpecular;
    engine::gfx::HdrImage refraction;
};

// PathTraceResult's 14 primary-hit fields -- deterministic given an unchanged camera/scene, so
// PathTraceDriver captures these once (pass 1 of a generation) and never rebuilds/republishes them
// again, instead of paying their copy cost on every pass alongside the 7 fields that actually
// accumulate (see PathTraceDynamic). Field meanings are identical to PathTraceResult's own doc
// comments above.
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
};

// PathTraceResult's 7 fields that genuinely re-average across passes -- see PathTraceGBuffer's doc
// comment for why these are split out. Field meanings are identical to PathTraceResult's own doc
// comments above.
struct PathTraceDynamic {
    engine::gfx::HdrImage beauty;
    engine::gfx::HdrImage bounceHeatmap;
    engine::gfx::HdrImage directDiffuse;
    engine::gfx::HdrImage indirectDiffuse;
    engine::gfx::HdrImage directSpecular;
    engine::gfx::HdrImage indirectSpecular;
    engine::gfx::HdrImage refraction;
};

// Blocking, multithreaded (one thread per hardware core, dynamic row scheduling) unidirectional
// path trace: BSDF-sampled recursive bounces with next-event estimation (environment-only, MIS power
// heuristic against BSDF sampling), Russian roulette from russianRouletteStartBounce. No punctual-
// light contribution -- point/directional lights have no hittable geometry.
//
// showSky: gates only the primary ray's own miss (the camera seeing the background directly) --
// indirect bounces and NEE always sample real environment radiance regardless, so hiding the
// background doesn't unlight the scene.
//
// generation/requestedGeneration: cooperative cancellation for PathTraceDriver's async use -- each
// row worker checks generation.load() != requestedGeneration once per row (cheap, same polling idiom
// as the row-stealing atomic inside RowThreadPool) and returns early if a newer request has superseded
// this one, leaving the (discarded) result's unwritten rows however writeTexel last left them. A
// direct/synchronous caller not using cancellation can pass a generation atomic holding
// requestedGeneration's own value, which never goes stale.
//
// threadPool: row-parallel dispatch, owned by the caller and reused across calls (PathTraceDriver
// keeps one alive for its whole lifetime) -- avoids paying OS thread-creation/join cost on every pass.
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
