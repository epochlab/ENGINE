#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <bit>
#include <cmath>
#include <cstdint>
#include <vector>

#include <glm/glm.hpp>

#include "pathtracer/debug/render_stats.h"
#include "pathtracer/gfx/hdr_image.h"
#include "pathtracer/scene/camera.h"
#include "pathtracer/scene/embree_accel.h"
#include "pathtracer/scene/gltf_loader.h"
#include "pathtracer/scene/light.h"
#include "pathtracer/scene/thread_pool.h"

namespace pathtracer::scene {

// Square destination tiles, one owner each: splatting crosses pixel boundaries, so the rasterizer's row-disjoint
// invariant cannot hold here. Size trades halo waste against scheduling granularity -- the halo re-traces 4.2% of a tile
// here against 6.3% at 64. 96 and 128 measured indistinguishable at 1080p; 64 measurably worse. Here so spec can read it.
inline constexpr int kPathTraceTileSize = 96;

// samplesPerPixel is per renderPathTraced() call: under PathTraceDriver that is samples per accumulated pass, typically
// 1, not the converged image's total. Convergence comes from the driver accumulating passes, not from a large value.
struct PathTraceSettings {
    int samplesPerPixel;
    int maxBounces;  // secondary/indirect bounces beyond the always-traced primary hit; 0 = direct lighting only
    int russianRouletteStartBounce;
    // AO ray length bound and the scale of the obscurance falloff over it (Zhukov 1998; Iones 2003): a hit at t gives
    // 1 - (1 - t/aoMaxDistance)^2 visibility, so AO measures local contact and reaches full visibility smoothly.
    // This is the distance occlusion fades to nothing by, not the one it occludes out to. Defaulted: see rrMaxProb.
    float aoMaxDistance = 1.0F;
    // Horizon of the Lookahead AOV ramp (rasterizer.h RasterGBuffer::lookahead), scene units: camera-space Z maps
    // linearly to 1 at the camera plane and 0 here, clamped. Read by the rasterizer, carried here as shadePixel's bundle.
    // Defaulted because the validators value-init then assign, and an undefaulted 0 would divide by zero every pixel.
    float lookaheadDistance = 10.0F;
    float rrMinProb = 0.05F;  // floor: stops a near-zero-throughput path being killed with near-certainty
    // Exactly 1.0: a path at full throughput must never be terminated. Lower caps survival for no gain -- the deep-path
    // tracing it saves costs more in the extra samples its variance needs. Defaulted because the validators value-init.
    float rrMaxProb = 1.0F;
    // From the scene's material file (SceneConfig::materialPath) -- see resolveRoughness/buildShadingFrame.
    float bumpStrength;
    float roughnessMin;
    float roughnessMax;
    // Global material override from the scene's material file (SceneConfig::materialPath) -- see resolveBsdfParams.
    glm::vec3 diffuseColour;
    float ior;
    float abbe = 0.0F;  // Abbe number pairing with ior for dispersion; 0 = none, see bsdf.h's cauchyIor
    float transmissionFactor;
    float metallicFactor;
    float roughnessFactor;
    float diffuseRoughness;  // EON rough-diffuse parameter r in [0,1]; 0 = Lambertian, see bsdf.h's BsdfParams
    // The transmission lobe's only tint (transmissionFactor>0), realized in the volume or on the surface by
    // transmissionDepth -- see scene_config.h MaterialConfig and bsdf.h BsdfParams.
    glm::vec3 transmissionColor = glm::vec3(1.0F);
    float transmissionDepth = 0.0F;
    // Gulbrandsen 2014 edgetint for the conductor lobe; 1 = white edge, see bsdf.h's BsdfParams.
    glm::vec3 edgeTint = glm::vec3(1.0F);
};

// Stops either side of unity the over-range readout is exact over: the exposure controls reach +/-13.95 EV, so 16 is the
// next binade bound above that. See DERIVATIONS.md "Over-range readout binning".
inline constexpr int kOverRangeEvRadius = 16;
inline constexpr float kOverRangeMin = 1.0F / static_cast<float>(1U << kOverRangeEvRadius);

// Mantissa bits kept above the shift: 1024 bins per stop. Measured error 2.6 -> 0.65 -> 0.18 -> 0.047 pp as this goes
// 4 -> 6 -> 8 -> 10 on cornell at 2048x1152; 0.047 pp is below half the 0.1 pp the HUD's "%.1f%%" can display.
inline constexpr int kOverRangeSubBinBits = 10;
inline constexpr int kOverRangeBinShift = 23 - kOverRangeSubBinBits;  // float32 carries 23 mantissa bits
inline constexpr int kOverRangeBinCount = 2 * kOverRangeEvRadius * (1 << kOverRangeSubBinBits);
inline constexpr int kOverRangeBinOrigin =
    static_cast<int>(std::bit_cast<std::uint32_t>(kOverRangeMin) >> kOverRangeBinShift);

// Bin of `value`. Positive IEEE-754 floats are monotone under integer comparison of their bits, so a right shift is a
// monotone binning of the positive range for a shift and a subtract rather than a log2. The clamp brackets the range
// and keeps every bit pattern in bounds: negatives fold to bin 0, non-finites to the top. See DERIVATIONS.md.
[[nodiscard]] inline int overRangeBin(float value) {
    if (std::signbit(value)) {
        return 0;
    }
    const auto shifted = static_cast<int>(std::bit_cast<std::uint32_t>(value) >> kOverRangeBinShift);
    return std::clamp(shifted - kOverRangeBinOrigin, 0, kOverRangeBinCount - 1);
}

using OverRangeHistogram = std::array<std::uint32_t, kOverRangeBinCount>;

// Exposure-free reduction of `beauty` for the HUD's over-range readout, computed on the driver thread. Exposure is
// factored out, not applied, so the readout stays live under a slider drag for O(1) render-thread work.
struct OverRangeStats {
    float rawPeak = 0.0F;  // max over texels of max(R,G,B) before exposure; exact, only the fraction is quantised
    // Complementary CDF, not the histogram it folds from, so the reader indexes instead of summing: aboveBin[b] counts
    // texels whose bin is >= b, so aboveBin[0] is the texel count and aboveBin[kOverRangeBinCount] is 0.
    std::array<std::uint32_t, kOverRangeBinCount + 1> aboveBin{};
};

// One renderPathTraced() call's raw output, republished in full by PathTraceDriver on every accumulated pass. Each field
// is a per-sample quantity averaged over the call's samples and re-averaged over passes; single-channel fields are
// broadcast to RGB so each goes straight to createFromFloatPixels. G-buffer AOVs live in rasterizer.h alone.
struct PathTraceResult {
    pathtracer::gfx::HdrImage beauty;
    pathtracer::gfx::HdrImage bounceHeatmap;   // mean bounce depth at termination, across samples
    // Cosine-weighted obscurance (Zhukov 1998; Iones 2003), the distance-weighted generalisation of AO (Miller 1994;
    // Landis 2002), one bounded ray per sample, saturating at aoMaxDistance. 1.0 = unoccluded, the OPPOSITE polarity to
    // `shadow`; background is 1.0. Each sample is graded, not a 0/1 draw, so the contact gradient exists at 1 spp.
    pathtracer::gfx::HdrImage ao;
    // Fraction of the primary hit's NEE samples toward the env light that were occluded: 1.0 = fully shadowed, 0.0 =
    // fully lit or background. Averaged across samples and passes, so a binary per-sample test converges to penumbra.
    pathtracer::gfx::HdrImage shadow;

    // Light-transport breakdown, averaged as beauty is and physical: radiance attributed, never rescaled. The five
    // buckets plus the unbucketed background sum to beauty exactly (integrator_validate asserts it per pixel, showSky
    // off). Bucketed by the bounce-0 lobe, transmission sticky. See DERIVATIONS.md "Transport AOV bucketing".
    pathtracer::gfx::HdrImage directDiffuse;
    pathtracer::gfx::HdrImage indirectDiffuse;
    pathtracer::gfx::HdrImage directSpecular;
    pathtracer::gfx::HdrImage indirectSpecular;
    pathtracer::gfx::HdrImage refraction;

    // Expected Fresnel at the primary hit over the visible normal distribution: E[F(wo.wh)], wh ~ D_vis(wo), one VNDF
    // draw per sample (bsdf.h fresnelAtMicrofacet), averaged as the lanes above are.
    // 0 where bounce 0 has no BSDF vertex -- the ray missed, or hit an emitter quad, which returns before shading.
    pathtracer::gfx::HdrImage fresnel;

    // Reduced from `beauty` by PathTraceDriver immediately before publish, so it describes exactly the pixels published
    // with it. Not on PassRecord, which is republished for cancelled passes whose pixels never reach the screen.
    OverRangeStats overRange;
    std::uint64_t generation = 0;  // the request whose accumulation this is, stamped by PathTraceDriver at publish; 0 from renderPathTraced
    int samples = 0;  // passes averaged in, stamped with generation so image and count publish as one snapshot
};

// All 10 images zeroed at width x height -- what renderPathTraced's `out` must be, allocated once and reused.
[[nodiscard]] PathTraceResult makePathTraceResult(int width, int height);

// Blocking, multithreaded unidirectional path trace: BSDF-sampled bounces with NEE against `lights` (MIS power
// heuristic), Russian roulette from russianRouletteStartBounce. Samples reconstruct through a 1.5px Blackman-Harris
// filter, all 10 images sharing one weight. Parameter contracts: DERIVATIONS.md "Path-traced render contract".
void renderPathTraced(const Camera& camera, const EmbreeAccel& accel,
                       const std::vector<ShadingTriangle>& shadingTriangles,
                       const std::vector<MeshInstance>& instances,
                       const std::vector<int>& instanceLightIndex, const LightSet& lights,
                       int width, int height, bool showSky, const PathTraceSettings& settings,
                       const std::vector<PathTraceSettings>& perInstanceSettings,
                       std::uint32_t scrambleSeed, int sampleBase, int sampleCount,
                       const std::atomic<std::uint64_t>& generation,
                       std::uint64_t requestedGeneration, ThreadPool& threadPool,
                       pathtracer::debug::PassStats& stats, PathTraceResult& out);

}  // namespace pathtracer::scene
