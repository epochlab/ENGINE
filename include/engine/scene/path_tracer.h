#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <bit>
#include <cmath>
#include <cstdint>
#include <vector>

#include <glm/glm.hpp>

#include "engine/debug/render_stats.h"
#include "engine/gfx/hdr_image.h"
#include "engine/scene/camera.h"
#include "engine/scene/embree_accel.h"
#include "engine/scene/gltf_loader.h"
#include "engine/scene/light.h"
#include "engine/scene/thread_pool.h"

namespace engine::scene {

// Square destination tiles, each owned outright by one worker: splatting crosses pixel boundaries, so the row-disjoint invariant the rasterizer still relies on cannot hold here. Size trades halo waste against load-balancing granularity -- the halo re-traces (size+2*kFilterExtent)^2/size^2 of a tile, 4.2% here against 6.3% at 64, while doubling to 128 quarters the number of work items a small render has to spread across its workers. 96 and 128 measured indistinguishable at 1080p; 64 measurably worse.
// In the header rather than path_tracer.cpp only so the startup spec block can report the real value instead of duplicating the literal.
inline constexpr int kPathTraceTileSize = 96;

// samplesPerPixel is "samples per renderPathTraced() call" -- when driven progressively by PathTraceDriver, that's samples per accumulated pass (typically 1), not the total sample count of the converged image; convergence comes from the driver accumulating many passes, not from a large value here.
struct PathTraceSettings {
    int samplesPerPixel;
    int maxBounces;  // secondary/indirect bounces beyond the always-traced primary hit; 0 = direct lighting only
    int russianRouletteStartBounce;
    // Ambient-occlusion ray length bound, and the scale of the obscurance falloff over it (Zhukov et al. 1998; Iones et al. 2003): a hit at distance t contributes 1 - (1 - t/aoMaxDistance)^2 visibility, so the AO AOV measures local contact rather than whole-room enclosure and reaches full visibility smoothly at the bound instead of stepping there.
    // Weaker than the hard cutoff it replaces at the same value: this is now the distance occlusion has faded to nothing by, not the one it fully occludes out to, so a scene tuned against the old behaviour wants a larger number here. Scene-scale dependent, sourced from profile.json. Defaulted because the validation tools build settings by value-init plus assignment, where an undefaulted field would silently be 0 and disable every AO ray.
    float aoMaxDistance = 0.25F;
    float rrMinProb = 0.05F;  // floor: stops a near-zero-throughput path being killed with near-certainty
    // Ceiling of exactly 1.0: a path carrying full throughput must never be terminated. Any lower caps survival for no gain -- it saves a fraction of deep-path tracing and pays for it with variance costing more than that fraction in extra samples.
    float rrMaxProb = 1.0F;
    // Sourced from the scene's material file (SceneConfig::materialPath, loaded via loadMaterialConfig) -- see resolveRoughness/buildShadingFrame (path_tracer.cpp).
    float bumpStrength;
    float roughnessMin;
    float roughnessMax;
    // Global material override, sourced from the scene's material file (SceneConfig::materialPath, loaded via loadMaterialConfig) -- see resolveBsdfParams (path_tracer.cpp).
    glm::vec3 diffuseColour;
    float ior;
    float abbe = 0.0F;  // Abbe number pairing with ior for dispersion; 0 = none, see bsdf.h's cauchyIor
    float transmissionFactor;
    float metallicFactor;
    float roughnessFactor;
    float diffuseRoughness;  // EON rough-diffuse parameter r in [0,1]; 0 = Lambertian, see bsdf.h's BsdfParams
    // The transmission lobe's only tint (transmissionFactor>0 only), realized either in the volume or on the surface by transmissionDepth -- see scene_config.h's MaterialConfig and bsdf.h's BsdfParams.
    glm::vec3 transmissionColor = glm::vec3(1.0F);
    float transmissionDepth = 0.0F;
    // Gulbrandsen 2014 edgetint for the conductor lobe; 1 = white edge, see bsdf.h's BsdfParams.
    glm::vec3 edgeTint = glm::vec3(1.0F);
};

// Stops either side of unity that the over-range readout is exact over. The display exposure control reaches +/-13.95 EV (the HUD's aperture f/1-f/22, shutter 1/4000-1 s and ISO 50-6400 sliders against profile.json's defaults), so 16 -- the next binade bound above that -- puts every threshold those controls can produce strictly inside the bracket. Thresholds outside it are clamped, which is precisely what makes the count exact everywhere inside: a value beyond either end folds into that end's bin, where it is unambiguously above or below every interior threshold.
inline constexpr int kOverRangeEvRadius = 16;
inline constexpr float kOverRangeMin = 1.0F / static_cast<float>(1U << kOverRangeEvRadius);

// Mantissa bits kept above the shift, so each binade is split into 2^this bins: 1024 bins per stop. Chosen by measurement rather than taste -- the readout's only error is the occupancy of the bin holding the threshold, which on cornell at 2048x1152 falls 2.6 -> 0.65 -> 0.18 -> 0.047 percentage points as this goes 4 -> 6 -> 8 -> 10. 0.047 pp is below half the 0.1 pp the HUD's "%.1f%%" can display, so the digit on screen is the digit an exact per-texel count would print.
inline constexpr int kOverRangeSubBinBits = 10;
inline constexpr int kOverRangeBinShift = 23 - kOverRangeSubBinBits;  // float32 carries 23 mantissa bits
inline constexpr int kOverRangeBinCount = 2 * kOverRangeEvRadius * (1 << kOverRangeSubBinBits);
inline constexpr int kOverRangeBinOrigin =
    static_cast<int>(std::bit_cast<std::uint32_t>(kOverRangeMin) >> kOverRangeBinShift);

// Bin of `value`. Positive IEEE-754 floats are monotone under integer comparison of their bit patterns, so a right shift of those bits IS a monotone binning of the whole positive range: the edges are exactly representable floats, the widths are one binade over 2^kOverRangeSubBinBits (uniform in stops to within the mantissa's 1.4x spread across a binade), and it costs a shift and a subtract rather than a log2. The clamp is what brackets it, and also what keeps the index in range for every bit pattern -- negatives and -0 fold into bin 0, where they can never exceed a positive threshold, and the infinities and NaNs fold into the top bin, where they read as over-range, which is the truthful answer for a texel that cannot be displayed at all.
[[nodiscard]] inline int overRangeBin(float value) {
    if (std::signbit(value)) {
        return 0;
    }
    const auto shifted = static_cast<int>(std::bit_cast<std::uint32_t>(value) >> kOverRangeBinShift);
    return std::clamp(shifted - kOverRangeBinOrigin, 0, kOverRangeBinCount - 1);
}

using OverRangeHistogram = std::array<std::uint32_t, kOverRangeBinCount>;

// Exposure-free reduction of `beauty` for the HUD's over-range readout, computed on the driver thread. Exposure is display-stage only (not in main.cpp's PathTraceInputState, so moving it never retraces), which is exactly why neither readout can be evaluated here: a fixed threshold would freeze on a converged image while the slider moves. Exposure is factored out instead -- the displayed peak is `exposure * rawPeak`, and the displayed fraction is `aboveBin` read at `1/exposure` -- so both stay live under a slider drag for O(1) render-thread work in place of one pass over every texel.
struct OverRangeStats {
    float rawPeak = 0.0F;  // max over texels of max(R,G,B) before exposure; the peak stays exact, only the fraction is quantised
    // Complementary CDF rather than the histogram it is folded from, so the reader indexes instead of summing: aboveBin[b] is the number of texels whose bin is >= b, hence aboveBin[0] is the texel count and aboveBin[kOverRangeBinCount] is 0.
    std::array<std::uint32_t, kOverRangeBinCount + 1> aboveBin{};
};

// One renderPathTraced() call's raw output -- what a single pass computes, and what PathTraceDriver republishes in full on every accumulated pass. Single-channel fields are broadcast to RGB (alpha=1), matching HdrImage's fixed 4-floats/texel layout so every field can go straight through Texture::createFromFloatPixels unchanged. Every field here is a per-sample quantity averaged across the call's samples, and re-averaged across accumulated passes by the driver. The primary-hit G-buffer AOVs (depth/worldPos/normal/albedo/metallic/roughness/tangent/objectId/alpha/fresnel/uv/geomNormal/IOR) are NOT here: rasterizer.h's RasterGBuffer is their only producer, refreshed synchronously on the render thread every trigger change, and main.cpp's selectPathTracedImage routed every one of them there -- the path-traced copies were computed, stored and published to no reader at all. Wireframe/BoundingBox are likewise rasterizer.h-only.
struct PathTraceResult {
    engine::gfx::HdrImage beauty;
    engine::gfx::HdrImage bounceHeatmap;   // mean bounce depth at termination, across samples
    // Cosine-weighted obscurance (Zhukov et al. 1998; Iones et al. 2003), the distance-weighted generalisation of ambient occlusion (Miller 1994; Landis 2002), one bounded ray per sample: the hemisphere above the primary hit weighted by how far each direction reaches before obstruction, saturating at aoMaxDistance. 1.0 = unoccluded, the OPPOSITE polarity to `shadow` below -- AO keeps the baked-texture convention it replaces, where white is open sky. Background (no primary hit) is 1.0; averaged and re-averaged exactly as shadow is. Each sample already carries a graded value rather than a 0/1 draw, so the contact gradient is present at one sample per pixel and does not have to converge into existence.
    engine::gfx::HdrImage ao;
    engine::gfx::HdrImage shadow;          // fraction of the primary hit's NEE samples toward the env light that were occluded -- 1.0 = fully shadowed, 0.0 = fully lit or no primary hit (background); averaged across samples and re-averaged across passes, so it converges from a binary per-sample test into continuous soft-shadow/penumbra density

    // Light-transport component breakdown, replacing a single combined "IBL" term. Averaged the same way beauty is (across samples/passes), and PHYSICAL: every value written here is the same radiance that went into beauty, attributed rather than rescaled. THE FIVE BUCKETS PLUS BACKGROUND SUM TO BEAUTY EXACTLY -- background being the bounce-0 miss, the camera seeing the environment with no surface interaction, which is deliberately unbucketed (production renderers keep it out of the surface-transport AOVs too) and is the only radiance beauty carries that these five do not. tools/integrator_validate.cpp asserts the identity per pixel with showSky off, which zeroes the background term.
    // Bucketing rule: a path is bucketed once, by the lobe sampled at its first (bounce 0) surface interaction, independent of however many further bounces it takes -- except that any transmission-lobe sample, at bounce 0 or later, stickily overrides the bucket to Refraction from that point on. Direct vs Indirect is not tracked separately: it falls out of which bounce the radiance-contributing event lands on, bounce 0's NEE and bounce 1's BSDF-sampled miss being the two halves of the same one-vertex path. Bounce 0's NEE contribution is the one place a single sample writes several buckets, split by the lobe that actually carried the light (bsdf.h's BsdfEval) rather than by the lobe the continuation ray drew.
    // Not an LPE. Bucketing by first event describes a debug viewer's decomposition, not an arbitrary path expression -- a diffuse bounce off a red wall onto a specular surface still reads as diffuse transport, and later-bounce surfaces' colours legitimately tint the indirect buckets.
    engine::gfx::HdrImage directDiffuse;
    engine::gfx::HdrImage indirectDiffuse;
    engine::gfx::HdrImage directSpecular;
    engine::gfx::HdrImage indirectSpecular;
    engine::gfx::HdrImage refraction;

    // Reduced from `beauty` by PathTraceDriver immediately before publish, so it describes exactly the pixels published with it. Here rather than on PassRecord for that reason: PassRecord is republished for cancelled passes too, whose pixels never reach the screen. Left zeroed by renderPathTraced, which does not write it.
    OverRangeStats overRange;
};

// All 9 images zeroed at width x height -- what renderPathTraced's `out` parameter must be, allocated once by the caller and reused across passes.
[[nodiscard]] PathTraceResult makePathTraceResult(int width, int height);

// Blocking, multithreaded (one thread per hardware core, dynamically scheduled square tiles) unidirectional path trace: BSDF-sampled recursive bounces with next-event estimation against `lights` (a LightSet -- the environment map and/or zero or more rectangular emitters, MIS power heuristic against BSDF sampling), Russian roulette from russianRouletteStartBounce. Samples are reconstructed through a Blackman-Harris filter of 1.5px radius rather than accumulated per pixel, so each one contributes to several pixels and every output image is a weighted mean over the filter's support; all 9 images share one weight, which is what keeps the transport buckets an exact partition of beauty through filtering. showSky: gates only the primary ray's own miss (the camera seeing the background directly) -- indirect bounces and NEE always sample real light radiance regardless, so hiding the background doesn't unlight the scene. instanceLightIndex: parallel to `instances`, -1 for ordinary geometry, the index into `lights`' quads for an emitter instance -- a hit on one is Le, not a BSDF vertex (see tracePath). out: caller-owned destination, which MUST already be sized width x height (see makePathTraceResult) -- taken by reference rather than returned so a progressive driver allocates its buffers once instead of 9 fresh images per pass. Every pixel of every image is written, so no pre-clear is needed and a reused buffer carries nothing over from the previous pass. generation/requestedGeneration: cooperative cancellation for PathTraceDriver's async use -- each worker checks generation.load() != requestedGeneration once per tile (cheap, same polling idiom as the index-stealing atomic inside ThreadPool) and returns early if a newer request has superseded this one, leaving `out`'s unwritten tiles however the previous pass left them, which is safe precisely because a cancelled pass is discarded whole and the next pass rewrites every tile. A direct/synchronous caller not using cancellation can pass a generation atomic holding requestedGeneration's own value, which never goes stale. stats: ray/tile counters for this pass, owned by the caller and reused across calls exactly as threadPool is -- reset() before the call, read after it returns (see render_stats.h for why the reads are safe without an explicit fence). threadPool: parallel dispatch, owned by the caller and reused across calls (PathTraceDriver keeps one alive for its whole lifetime) -- avoids paying OS thread-creation/join cost on every pass. scrambleSeed/sampleBase: the sampler's two independent inputs, and they are NOT interchangeable (see sampler.h). sampleBase is the count of samples already accumulated for this image, so pass N supplies N and the sampler advances along its Sobol sequence; scrambleSeed randomizes that sequence and must stay FIXED for every pass of one accumulation, varying only when the image is restarted. A scrambleSeed that changes per pass, or a sampleBase stuck at 0, degrades the sampler to plain Monte Carlo -- which is exactly the defect this pair of parameters replaced, and what sampler_validate's pass-direction check guards.
void renderPathTraced(const Camera& camera, const EmbreeAccel& accel,
                       const std::vector<ShadingTriangle>& shadingTriangles,
                       const std::vector<MeshInstance>& instances,
                       const std::vector<int>& instanceLightIndex, const LightSet& lights,
                       int width, int height, bool showSky, const PathTraceSettings& settings,
                       const std::vector<PathTraceSettings>& perInstanceSettings,
                       std::uint32_t scrambleSeed, int sampleBase, int sampleCount,
                       const std::atomic<std::uint64_t>& generation,
                       std::uint64_t requestedGeneration, ThreadPool& threadPool,
                       engine::debug::PassStats& stats, PathTraceResult& out);

}  // namespace engine::scene
