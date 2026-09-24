// Standalone correctness check for pathtracer::scene::bsdf (bsdf.h): verifies the combined specular+diffuse pdf never integrates to more than the total lobe-selection mass.
// Upper bound only, deliberately: VNDF reflection sampling is not normalized over the hemisphere (samples reflecting below the horizon are discarded), so the true integral is the horizon-clipped mass, which has no closed form and is measured instead by the white furnace test below.
// Also runs a furnace test (uniform incident radiance from every direction, including through transmission) via BSDF importance sampling: must never reflect/transmit more energy than received.
// Same standalone-CLI convention as embree_validate.cpp/furnace_test.cpp: no test framework, non-zero exit on failure.

#include <array>
#include <atomic>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <optional>
#include <random>
#include <thread>
#include <tuple>
#include <vector>

#include <glm/glm.hpp>

#include "check.h"
#include "conductor_reference.h"
#include "fixtures.h"
#include "pathtracer/scene/bsdf.h"
#include "pathtracer/scene/fresnel_dielectric.h"
#include "pathtracer/scene/sampler.h"

#include "stats.h"

namespace {

using pathtracer::scene::BsdfParams;
using tools::stats::simpson;

using tools::fixtures::kPi;
using tools::fixtures::sampleUniformHemisphere;
using tools::reference::cosineAverageFresnel;
using tools::reference::referenceConductorFresnel;
using tools::reference::referenceConductorFresnelAt;
using tools::reference::referenceConductorIor;

// Each check keeps its own `ok` accumulator and its per-row stderr diagnostics -- those carry the parameters, the
// measured value and the reference, which is what makes a failure diagnosable -- and reports one verdict. The detail
// is in the rows, not in the assertion count.
void finish(tools::check::Context& ctx, bool ok, const char* what) {
    ctx.plan(1);
    PT_EXPECT(ctx, ok, what);
}

// edgeTint defaults to white, the no-dip edge Schlick always produced, so every pre-existing case here
// is a strict subset of the swept coverage rather than a shifted version of it.
BsdfParams makeParams(float roughness, float metallic, float transmissionFactor,
                       float diffuseRoughness = 0.0F, glm::vec3 edgeTint = glm::vec3(1.0F)) {
    const glm::vec3 baseColor(1.0F);  // worst case: full white albedo
    const glm::vec3 f0 = glm::mix(glm::vec3(0.04F), baseColor, metallic);
    return BsdfParams{baseColor,          metallic, roughness,          f0, edgeTint,
                       /*ior=*/1.5F, transmissionFactor, diffuseRoughness,
                       pathtracer::scene::eonAlbedoInversion(baseColor, diffuseRoughness),
                       /*transmissionTint=*/glm::vec3(1.0F)};
}

// A colored/dark conductor (f0=0.5, not the white f0=baseColor=1 makeParams gives at metallic=1): the case that caught evaluateDiffuseLobe's pdf-gating bug.
// A white f0 clamps specularProb to 0.95, leaving only 5% diffuse selection mass to hide a diffuse-pdf bug under this test's tolerance; f0=0.5 leaves ~50%, large enough for the same bug to fail loudly.
BsdfParams makeColoredMetalParams(float roughness, glm::vec3 edgeTint = glm::vec3(1.0F)) {
    const glm::vec3 baseColor(1.0F);
    return BsdfParams{baseColor,    1.0F, roughness, glm::vec3(0.5F), edgeTint,
                       /*ior=*/1.5F, /*transmissionFactor=*/0.0F, /*diffuseRoughness=*/0.0F,
                       pathtracer::scene::eonAlbedoInversion(baseColor, 0.0F),
                       /*transmissionTint=*/glm::vec3(1.0F)};
}

// Uniform-solid-angle hemisphere sample (PBRT-style inversion, same as furnace_test.cpp): z=u1, r=sqrt(1-u1^2), phi=2*pi*u2, pdf=1/(2*pi).
// Integrates pdfBsdf(wo, .) over the hemisphere; for an opaque material (transmissionFactor=0) this must not exceed 1.0, since all sampling probability mass is in the two continuous lobes the pdf covers.
// Multiple importance sampling with the balance heuristic (Veach 1997 sec. 9.2) over two proposals that between them cover both regimes: uniform hemisphere for the tails, sampleBsdf's own density for the lobe. Uniform alone stops working once the lobe is narrow -- at roughness 0.05 it spans ~2e-5 sr, which 200k uniform draws hit a handful of times at O(100) weight each, and the estimate is then too noisy to bound at all (measured 1.67 against a truth of <= 1, and still 1.17 at a hundred times the samples).
// sampleBsdf draws from exactly pdfBsdf, so the second proposal's density IS the integrand and the combined balance-heuristic density collapses to N1/(2*pi) + N2*p(x): one pdf evaluation per sample, bounded below by the uniform term and above by N2*p, so it is well conditioned at every roughness. Unbiased despite that density being sub-normalised (sampleBsdf returns nullopt below the horizon), because the heuristic needs only the expected sample count per solid angle, which is N2*q2 either way.
// Upper-bound only, not an equality: VNDF reflection sampling discards samples reflected below the horizon, so the true integral is the horizon-clipped mass, which has no closed form. The estimator is now tight enough that a real double-counted pdf shows up as an excess rather than drowning in variance.
// diffuseRoughness is swept because at 0 -- the only value this used to test, and the one principled.json ships -- eonUniformMixWeight is pow(0, 0.1) = 0 exactly, so pdfEon degenerates to cltcPdf alone and CLTC itself degenerates to plain cosine. Neither the LTC fit's own normalisation nor the uniform/CLTC one-sample MIS mixture was reached at all. The metallic=1 rows matter most here: metallic zeroes diffuseKd but NOT diffuseProb, so a conductor still carries the full CLTC density with none of its value, and a mis-normalised fit shows up in the mixture denominator rather than in any picture.
PT_CHECK(pdf_normalization, Slow, Statistical) {
    std::mt19937 rng(7);
    constexpr int kUniformSamples = 200000;
    constexpr int kBsdfSamples = 200000;
    constexpr std::uint32_t kBsdfSeed = 7;
    constexpr float kTolerance = 0.05F;
    constexpr double kUniformPdf = 1.0 / (2.0 * kPi);
    // The balance-heuristic denominator, shared by both proposals: N1*q1 + N2*q2 with q2 == pdfBsdf.
    const auto combinedDensity = [](double p) {
        return (kUniformSamples * kUniformPdf) + (kBsdfSamples * p);
    };
    const std::array<float, 4> roughnesses = {0.05F, 0.25F, 0.5F, 1.0F};
    const std::array<float, 2> metallics = {0.0F, 1.0F};
    const std::array<float, 4> ndotVs = {0.2F, 0.6F, 1.0F, -0.6F};
    const std::array<float, 3> diffuseRoughnesses = {0.0F, 0.5F, 1.0F};

    bool ok = true;
    double worstIntegral = 0.0;
    for (float roughness : roughnesses) {
        for (float metallic : metallics) {
            for (float diffuseRoughness : diffuseRoughnesses) {
                for (float ndotV : ndotVs) {
                    const BsdfParams params = makeParams(roughness, metallic, 0.0F, diffuseRoughness);
                    const glm::vec3 wo(std::sqrt(std::max(0.0F, 1.0F - (ndotV * ndotV))), 0.0F, ndotV);
                    double integral = 0.0;
                    for (int i = 0; i < kUniformSamples; ++i) {
                        glm::vec3 wi = sampleUniformHemisphere(rng);
                        // pdfBsdf mirrors wi into wo's hemisphere, so for a below-surface wo the density over the +z hemisphere is identically zero.
                        // Integrating the +z hemisphere there measured 0 and passed the <=1.0 assertion vacuously, so the exiting-side rows tested nothing; flip the sampled hemisphere to match wo's side.
                        if (ndotV < 0.0F) {
                            wi.z = -wi.z;
                        }
                        const double p = pathtracer::scene::pdfBsdf(params, wo, wi);
                        integral += p / combinedDensity(p);
                    }
                    // sampleBsdf returns wi in woLocal's own convention and reports the density it drew from, so no second pdfBsdf evaluation is needed and none of the mirroring above applies.
                    for (int i = 0; i < kBsdfSamples; ++i) {
                        pathtracer::scene::Sampler sampler(0, 0, i, kBsdfSamples, kBsdfSeed);
                        const std::optional<pathtracer::scene::BsdfSample> sample =
                            pathtracer::scene::sampleBsdf(params, wo, sampler);
                        if (sample.has_value()) {
                            integral += sample->pdf / combinedDensity(sample->pdf);
                        }
                    }
                    worstIntegral = std::max(worstIntegral, integral);
                    if (!(integral <= 1.0 + kTolerance)) {
                        std::cerr << "bsdf_validate: FAILED pdf normalization UPPER bound at roughness="
                                  << roughness << " metallic=" << metallic
                                  << " diffuseRoughness=" << diffuseRoughness << " ndotV=" << ndotV
                                  << " integral=" << integral << " (expected <= 1.0)\n";
                        ok = false;
                    }
                }
            }
        }
    }
    std::cout << "bsdf_validate: pdf normalization, worst integral " << worstIntegral << " (must be <= "
              << 1.0 + kTolerance << ")\n";
    finish(ctx, ok, "pdf_normalization failed; see the rows above");
    return;
}

// sampleBsdf's reported density must equal pdfBsdf re-evaluated at the direction it returned -- the contract BsdfSample::pdf states in bsdf.h ("exactly what pdfBsdf would return for it") and that nothing asserted. checkPdfNormalization above reads sample->pdf, the density the sampler reports about itself, and never re-evaluates it, so that contract had no instrument at all.
// What it covers was measured, not assumed. NOT a mixture term missing from the density: sampleBsdf reports evaluateContinuousLobes' own pdf, so both sides are then wrong together and this stays green -- deleting the msReflect density term leaves 0 failures here and fails checkFurnace at Lo=1.28 instead -- checkSamplingChiSquare below is the instrument that does catch it directly. What it does cover is the sign-mirroring round trip, since sampleBsdf returns wi in woLocal's convention and pdfBsdf re-mirrors it and nothing else in the suite closes that loop, and any future strategy reporting a hand-computed density beside the mixture rather than through it -- the usual optimisation once a lobe's own pdf is already in hand.
// Exact equality, not a tolerance: both sides are the same arithmetic over the same deterministic LobeProbabilities, so any difference is a broken round trip rather than drift. Delta transmission reports 0 on both sides and is asserted like every other row.
// Swept over checkPdfNormalization's grid plus transmissionFactor, including the ndotV<0 exiting rows the mirroring claim rests on, so every strategy in the ladder is drawn: VNDF reflection, EON, both multiple-scattering cosine lobes, rough and smooth refraction.
PT_CHECK(sample_density_consistency, Slow, Exact) {
    constexpr int kSampleCount = 8000;
    constexpr std::uint32_t kSeed = 11;
    const std::array<float, 4> roughnesses = {0.05F, 0.25F, 0.5F, 1.0F};
    const std::array<float, 3> metallics = {0.0F, 0.5F, 1.0F};
    const std::array<float, 3> transmissions = {0.0F, 0.5F, 1.0F};
    const std::array<float, 3> diffuseRoughnesses = {0.0F, 0.5F, 1.0F};
    const std::array<float, 4> ndotVs = {0.2F, 0.6F, 1.0F, -0.6F};

    bool ok = true;
    long long compared = 0;
    for (float roughness : roughnesses) {
        for (float metallic : metallics) {
            for (float transmission : transmissions) {
                for (float diffuseRoughness : diffuseRoughnesses) {
                    for (float ndotV : ndotVs) {
                        const BsdfParams params =
                            makeParams(roughness, metallic, transmission, diffuseRoughness);
                        const glm::vec3 wo(std::sqrt(std::max(0.0F, 1.0F - (ndotV * ndotV))), 0.0F,
                                            ndotV);
                        for (int i = 0; i < kSampleCount && ok; ++i) {
                            pathtracer::scene::Sampler sampler(0, 0, i, kSampleCount, kSeed);
                            const std::optional<pathtracer::scene::BsdfSample> sample =
                                pathtracer::scene::sampleBsdf(params, wo, sampler);
                            if (!sample.has_value()) {
                                continue;
                            }
                            ++compared;
                            const float reevaluated =
                                pathtracer::scene::pdfBsdf(params, wo, sample->wiLocal);
                            if (reevaluated == sample->pdf) {
                                continue;
                            }
                            std::cerr << "bsdf_validate: FAILED sample/pdf consistency at roughness="
                                      << roughness << " metallic=" << metallic
                                      << " transmission=" << transmission
                                      << " diffuseRoughness=" << diffuseRoughness
                                      << " ndotV=" << ndotV << " sample->pdf=" << sample->pdf
                                      << " pdfBsdf=" << reevaluated << '\n';
                            ok = false;
                        }
                    }
                }
            }
        }
    }
    std::cout << "bsdf_validate: sample/pdf consistency, " << compared
              << " sampled directions re-evaluated (exact equality)\n";
    finish(ctx, ok, "sample_density_consistency failed; see the rows above");
    return;
}

glm::vec3 furnaceLo(const BsdfParams& params, const glm::vec3& wo, int sampleCount, std::uint32_t seed) {
    glm::vec3 accum(0.0F);
    for (int i = 0; i < sampleCount; ++i) {
        pathtracer::scene::Sampler sampler(0, 0, i, sampleCount, seed);
        const std::optional<pathtracer::scene::BsdfSample> sample =
            pathtracer::scene::sampleBsdf(params, wo, sampler);
        if (sample.has_value()) {
            accum += sample->throughputWeight;  // L0=1
        }
    }
    return accum / static_cast<float>(sampleCount);
}

float maxChannel(const glm::vec3& v) { return std::max({v.x, v.y, v.z}); }
float minChannel(const glm::vec3& v) { return std::min({v.x, v.y, v.z}); }

// Asserts the PASS condition rather than testing for the failure condition: NaN compares false against every ordered operator, so `min < lo || max > hi` is satisfied by no NaN and lets one through silently. A NaN reading used to pass this whole suite (measured: two nan rows in the white furnace, exit 0).
bool withinBand(const glm::vec3& value, float centre, float tolerance) {
    return minChannel(value) >= centre - tolerance && maxChannel(value) <= centre + tolerance;
}

// Support coverage: every direction the BSDF has value at must carry mixture density, the condition under which the one-sample MIS estimator is unbiased (Veach 1997 sec. 9.2); where it fails, that energy is never drawn and the render is darker by it, not noisier.
// The gap it closes: sampling_chi_square tests the shape a strategy draws, and the furnaces test totals within a band, so a strategy whose selection is gated off while its lobe still has value is invisible to both -- msTransmit's forward-eta selection gate against its reciprocal-eta value gate did exactly that at roughness 0.128-0.168 on the entering side, under 1e-3 of the energy.
// Exact, not a tolerance: f > 0 with pdf == 0 is a support defect at any magnitude. The converse (density on a zero-valued lobe) is variance, not bias, and is not asserted.
// Roughness is swept at 8 steps per escape-table node through the band where the msTransmit lobe switches on, plus coarse points either side; both sides of the interface, every authored-range ior, the whole sphere of wi.
PT_CHECK(strategy_coverage, Fast, Exact) {
    constexpr int kMuNodes = 16;
    constexpr int kPhiNodes = 8;
    constexpr float kBandStep = 1.0F / 248.0F;
    std::vector<float> roughnesses = {0.05F, 0.25F, 0.5F, 1.0F};
    for (float roughness = 0.12F; roughness <= 0.18F; roughness += kBandStep) {
        roughnesses.push_back(roughness);
    }
    const std::array<float, 7> iors = {1.063F, 1.2F, 1.33F, 1.5F, 1.5168F, 2.0F, 2.4F};
    const std::array<float, 6> ndotVs = {0.95F, 0.6F, 0.2F, -0.2F, -0.6F, -0.95F};
    const std::array<float, 2> transmissions = {0.5F, 1.0F};

    bool ok = true;
    long long valued = 0;
    long long farValued = 0;
    long long uncovered = 0;
    for (float roughness : roughnesses) {
        for (float ior : iors) {
            for (float transmission : transmissions) {
                const BsdfParams params{glm::vec3(1.0F), 0.0F, roughness,
                                         glm::vec3(0.04F), glm::vec3(1.0F), ior,
                                         transmission, /*diffuseRoughness=*/0.0F,
                                         pathtracer::scene::eonAlbedoInversion(glm::vec3(1.0F), 0.0F),
                                         /*transmissionTint=*/glm::vec3(1.0F)};
                for (float ndotV : ndotVs) {
                    const glm::vec3 wo(std::sqrt(std::max(0.0F, 1.0F - (ndotV * ndotV))), 0.0F, ndotV);
                    for (int mi = 0; mi < 2 * kMuNodes; ++mi) {
                        const float z = -1.0F + ((static_cast<float>(mi) + 0.5F) / kMuNodes);
                        const float r = std::sqrt(std::max(0.0F, 1.0F - (z * z)));
                        for (int pi = 0; pi < kPhiNodes; ++pi) {
                            const float phi = 2.0F * kPi * (static_cast<float>(pi) + 0.5F) / kPhiNodes;
                            const glm::vec3 wi(r * std::cos(phi), r * std::sin(phi), z);
                            if (!(maxChannel(pathtracer::scene::evaluateBsdf(params, wo, wi)) > 0.0F)) {
                                continue;
                            }
                            ++valued;
                            farValued += (wi.z * wo.z < 0.0F) ? 1 : 0;
                            if (pathtracer::scene::pdfBsdf(params, wo, wi) > 0.0F) {
                                continue;
                            }
                            if (uncovered++ < 8) {
                                std::cerr << "bsdf_validate: FAILED strategy coverage at roughness=" << roughness
                                          << " ior=" << ior << " transmission=" << transmission
                                          << " ndotV=" << ndotV << " wi.z=" << wi.z << " phi=" << phi
                                          << " -- the BSDF has value here and no strategy samples it\n";
                            }
                            ok = false;
                        }
                    }
                }
            }
        }
    }
    // Anti-vacuity: the far hemisphere is where the gated lobe lives, so a sweep that never found value there tested nothing.
    if (farValued == 0) {
        std::cerr << "bsdf_validate: FAILED strategy coverage found no far-side direction with value\n";
        ok = false;
    }
    std::cout << "  strategy coverage: " << valued << " valued directions (" << farValued << " far-side), "
              << uncovered << " with zero mixture density\n";
    finish(ctx, ok, "strategy_coverage failed; see the rows above");
    return;
}

// Furnace test through sampleBsdf: uniform radiance L0=1 from every direction (both hemispheres, since transmission can receive from the far side); estimator Lo = mean(throughputWeight) since throughputWeight already folds in f*cosTheta/pdf.
// ndotV sweep includes negative values (woLocal.z<0, the exiting side of a transmissive dielectric) and a value past the ior=1.5 critical angle (~41.8deg, cosTheta~0.745) to force total internal reflection.
// Energy bound is 1.0 (L0) everywhere except the exiting side (ndotV<0) of a transmissive material below the critical angle, where sampleBsdf's eta^2 non-symmetric radiance-compression factor (Veach 1997 sec. 5.2, see bsdf.cpp's transmission branch) legitimately raises Lo above L0: L/n^2 is the invariant along a ray, so radiance increases going from a denser medium (ior=1.5, inside) into a rarer one (1.0, outside) by up to ior^2.
// The naive Lo<=1 bound only holds for eta==1 interfaces (pure reflection) or the entering side, where this same factor is <1, exactly compensating so a round trip through the surface loses no net energy.
PT_CHECK(furnace_energy_bound, Slow, Statistical) {
    constexpr int kSampleCount = 200000;
    constexpr float kTolerance = 0.1F;
    constexpr float kIor = 1.5F;  // matches makeParams/makeColoredMetalParams
    const std::array<float, 4> roughnesses = {0.05F, 0.25F, 0.5F, 1.0F};
    const std::array<float, 2> metallics = {0.0F, 1.0F};
    const std::array<float, 3> transmissions = {0.0F, 0.5F, 1.0F};
    const std::array<float, 5> ndotVs = {0.2F, 0.6F, 1.0F, -0.9F, -0.3F};  // last two: exiting/TIR

    bool ok = true;
    std::uint32_t seed = 0;
    for (float roughness : roughnesses) {
        for (float metallic : metallics) {
            for (float transmission : transmissions) {
                for (float ndotV : ndotVs) {
                    ++seed;
                    const BsdfParams params = makeParams(roughness, metallic, transmission);
                    const glm::vec3 wo(std::sqrt(std::max(0.0F, 1.0F - (ndotV * ndotV))), 0.0F,
                                        ndotV);
                    const float maxLo = maxChannel(furnaceLo(params, wo, kSampleCount, seed));
                    const bool exitingTransmissive = ndotV < 0.0F && transmission > 0.0F;
                    const float energyBound = exitingTransmissive ? kIor * kIor : 1.0F;
                    if (!(maxLo <= energyBound + kTolerance)) {
                        std::cerr << "bsdf_validate: FAILED furnace test at roughness=" << roughness
                                  << " metallic=" << metallic << " transmission=" << transmission
                                  << " ndotV=" << ndotV << " Lo=" << maxLo
                                  << " (expected <= " << energyBound << ")\n";
                        ok = false;
                    }
                }
            }
        }
    }

    // Colored conductor (f0=0.5): the case that caught evaluateDiffuseLobe's pdf-gating bug (a white f0's clamped 95% specular probability hid it under this test's tolerance).
    for (float roughness : roughnesses) {
        for (float ndotV : ndotVs) {
            ++seed;
            const BsdfParams params = makeColoredMetalParams(roughness);
            const glm::vec3 wo(std::sqrt(std::max(0.0F, 1.0F - (ndotV * ndotV))), 0.0F, ndotV);
            const float maxLo = maxChannel(furnaceLo(params, wo, kSampleCount, seed));
            if (!(maxLo <= 1.0F + kTolerance)) {
                std::cerr << "bsdf_validate: FAILED colored-metal furnace test at roughness="
                          << roughness << " ndotV=" << ndotV << " Lo=" << maxLo
                          << " (expected <= 1.0)\n";
                ok = false;
            }
        }
    }
    finish(ctx, ok, "furnace_energy_bound failed; see the rows above");
    return;
}

// TWO-SIDED white furnace: a white, non-absorbing surface under uniform L0=1 radiance must return exactly 1.0 (every photon it receives leaves again); checkFurnace above only ever asserts Lo<=bound, so it cannot see energy loss, this BSDF's actual failure mode.
// Restricted to cases where 1.0 is analytically correct: white base color, no transmission, entering side. A colored conductor (f0=0.5) legitimately absorbs with no closed-form expectation, so it stays upper-bound-only in checkFurnace.
// Single-scatter GGX loses the energy the Smith G2 masks away (Heitz, Hanika, d'Eon, Dachsbacher 2016): a white conductor at roughness 1.0 measured 0.307, under a third of the light received. Kulla-Conty multiple-scattering compensation plus the directional-albedo diffuse coupling (bsdf.cpp) return it, making 1.0 a correctness target, not a regression baseline: both bounds share the same tolerance and a shortfall is a bug.
// Half the rows sit deliberately off the albedo table's grid; there the measured value is E_true + (1 - E_interpolated), so these rows exercise the table's bilinear error. They are no longer the only bound on it -- checkAlbedoTableInterpolation measures it directly, per axis, three orders finer than the 2% a Monte Carlo band can reach -- but they are the only one that exercises it through the shipped transport rather than through a lookup called in isolation, which is a different thing to be sure of.
struct WhiteFurnaceCase {
    float roughness;
    float ndotV;
    bool offGrid;
};

PT_CHECK(white_furnace_two_sided, Slow, Statistical) {
    constexpr int kSampleCount = 400000;
    constexpr float kTolerance = 0.02F;
    const std::array<WhiteFurnaceCase, 14> cases = {{
        {0.05F, 1.0F, false},
        {0.05F, 0.4F, false},
        {0.25F, 1.0F, false},
        {0.25F, 0.4F, false},
        {0.50F, 1.0F, false},
        {0.50F, 0.4F, false},
        {1.00F, 1.0F, false},
        {1.00F, 0.4F, false},
        // Off-grid, and re-placed with the table's grid for the third time -- at k/31 they were 0.37/0.63/0.82 and 0.565/0.31, at k/127 they were 0.3661/0.6339/0.8228 and 0.563/0.311. Roughness rows now land on k/255 and the mu axis is uniform in sqrt(mu), so a mu that was mid-cell on the old linear axis is not mid-cell on this one: these are mid-cell on BOTH axes as the lookups actually index them, which is what the comment above needs to stay true.
        // Each value is the exact float landing on index k+0.5 of the axis that indexes it: roughness (k+0.5)/255, mu ((k+0.5)/255)^2, so the lookup's own blend weight is 0.5 on both.
        // Nothing derives them from albedoGridRes() on purpose. A row that recomputed its own worst case from the grid would follow the grid wherever it went and could never be seen to go stale, which is the failure this list has now survived twice by being written down.
        {0.3666667F, 0.5464398F, true},
        {0.3666667F, 0.2949981F, true},
        {0.6333333F, 0.5464398F, true},
        {0.6333333F, 0.2949981F, true},
        {0.8215686F, 0.5464398F, true},
        {0.8215686F, 0.2949981F, true},
    }};

    bool ok = true;
    std::uint32_t seed = 9000;
    std::cout << "bsdf_validate: white furnace energy (1.0 = perfectly energy-conserving)\n";
    std::cout << "  roughness  ndotV  conductor  dielectric\n";
    for (const WhiteFurnaceCase& entry : cases) {
        ++seed;
        const glm::vec3 wo(std::sqrt(std::max(0.0F, 1.0F - (entry.ndotV * entry.ndotV))), 0.0F,
                            entry.ndotV);
        const glm::vec3 conductor =
            furnaceLo(makeParams(entry.roughness, 1.0F, 0.0F), wo, kSampleCount, seed);
        const glm::vec3 dielectric =
            furnaceLo(makeParams(entry.roughness, 0.0F, 0.0F), wo, kSampleCount, seed + 500U);
        std::cout << "  " << entry.roughness << "       " << entry.ndotV << "    "
                  << minChannel(conductor) << "   " << minChannel(dielectric)
                  << (entry.offGrid ? "   (off-grid)" : "") << '\n';

        const std::array<std::pair<const char*, glm::vec3>, 2> measured = {
            {{"conductor", conductor}, {"dielectric", dielectric}}};
        for (const auto& [label, value] : measured) {
            if (!withinBand(value, 1.0F, kTolerance)) {
                std::cerr << "bsdf_validate: FAILED white-" << label
                          << " furnace energy conservation at roughness=" << entry.roughness
                          << " ndotV=" << entry.ndotV << " Lo=[" << minChannel(value) << ", "
                          << maxChannel(value) << "] (expected 1.0 +/- " << kTolerance << ")\n";
                ok = false;
            }
        }
    }
    finish(ctx, ok, "white_furnace_two_sided failed; see the rows above");
    return;
}

// EON rough-diffuse energy preservation: the same two-sided white furnace test as
// checkWhiteFurnaceTwoSided above, swept over diffuseRoughness instead of (specular) roughness.
// Classical Oren-Nayar variants (and this codebase's own diffuse lobe before EON) lose energy as
// diffuseRoughness increases -- exactly the problem EON's analytic multiple-scattering compensation
// term exists to fix (Portsmouth, Kutz, Hill 2025) -- so this must read 1.0 at every value, same
// correctness target as the specular sweep. Roughness is fixed at a mid-range value since it is the
// specular lobe's own parameter, orthogonal to diffuseRoughness.
// Conductors are swept too, and asserted EXACTLY invariant in diffuseRoughness rather than merely
// energy-conserving. diffuseKd is zeroed at metallic=1, so a conductor has no diffuse lobe and
// diffuseRoughness must reach nothing: computeLobeProbabilities hands that slot's whole selection mass
// to msReflect, leaving lobes.diffuse identically 0 and sampleEon uncalled. This is the row that went
// unguarded while the reflected multiple-scattering lobe borrowed the diffuse strategy -- a conductor
// then drew a CLTC shape set by diffuseRoughness for a lobe of zero value, measurable only as variance,
// which an energy band cannot see.
PT_CHECK(eon_diffuse_furnace, Slow, Statistical) {
    constexpr int kSampleCount = 400000;
    constexpr float kTolerance = 0.02F;
    constexpr float kRoughness = 0.5F;
    const std::array<float, 5> diffuseRoughnesses = {0.0F, 0.25F, 0.5F, 0.75F, 1.0F};
    const std::array<float, 3> ndotVs = {1.0F, 0.6F, 0.2F};
    const std::array<float, 2> metallics = {0.0F, 1.0F};

    bool ok = true;
    std::cout << "bsdf_validate: EON diffuse-roughness furnace energy (1.0 = perfectly energy-conserving)\n";
    std::cout << "  metallic  diffuseRoughness  ndotV  Lo\n";
    for (std::size_t m = 0; m < metallics.size(); ++m) {
        const float metallic = metallics[m];
        // Per-ndotV reading at diffuseRoughness 0, the reference the conductor rows must reproduce bit for bit.
        std::array<glm::vec3, 3> baseline{};
        for (float diffuseRoughness : diffuseRoughnesses) {
            for (std::size_t v = 0; v < ndotVs.size(); ++v) {
                const float ndotV = ndotVs[v];
                // Seeded by (metallic, ndotV) only, deliberately NOT by diffuseRoughness: the invariance assertion below is exact equality, so the two readings it compares must be built from the same sample sequence or the comparison measures the realization rather than the shape.
                const std::uint32_t seed = 20000 + static_cast<std::uint32_t>((m * ndotVs.size()) + v);
                const BsdfParams params = makeParams(kRoughness, metallic, 0.0F, diffuseRoughness);
                const glm::vec3 wo(std::sqrt(std::max(0.0F, 1.0F - (ndotV * ndotV))), 0.0F, ndotV);
                const glm::vec3 lo = furnaceLo(params, wo, kSampleCount, seed);
                std::cout << "  " << metallic << "         " << diffuseRoughness << "              "
                          << ndotV << "    " << minChannel(lo) << '\n';
                if (!withinBand(lo, 1.0F, kTolerance)) {
                    std::cerr << "bsdf_validate: FAILED EON diffuse furnace energy at metallic="
                              << metallic << " diffuseRoughness=" << diffuseRoughness
                              << " ndotV=" << ndotV << " Lo=[" << minChannel(lo) << ", "
                              << maxChannel(lo) << "] (expected 1.0 +/- " << kTolerance << ")\n";
                    ok = false;
                }
                if (diffuseRoughness == 0.0F) {
                    baseline[v] = lo;
                } else if (metallic == 1.0F && lo != baseline[v]) {
                    std::cerr << "bsdf_validate: FAILED conductor diffuseRoughness invariance at ndotV="
                              << ndotV << " diffuseRoughness=" << diffuseRoughness << " Lo="
                              << minChannel(lo) << " vs " << minChannel(baseline[v])
                              << " (a conductor has no diffuse lobe; diffuseRoughness must reach nothing)\n";
                    ok = false;
                }
            }
        }
    }
    finish(ctx, ok, "eon_diffuse_furnace failed; see the rows above");
    return;
}

// Paper Listing 1's E_EON at normal incidence, the EON directional albedo rho*E_F + rho_ms*(1-E_F), transcribed here independently of bsdf.cpp's evaluateEon and re-deriving c1/c2 from literals, so the check is not the same arithmetic tested against itself -- the discipline checkAverageFresnel applies to the Fresnel averages.
// Normal incidence needs no FON G-term: the exact albedo's G (eq. 13) and the renderer's quartic fit (evalFonAlbedoApprox) BOTH vanish at mu=1, leaving AF = 1/(1+c1*r) either way. That is why eq. 29, and so the inversion built on it, is exact here rather than inheriting the fit's <0.1% error -- and why that error reaches this check only through measureDiffuseAlbedo, which integrates over every mu, where kFitTolerance covers it.
glm::vec3 referenceEonAlbedo(const glm::vec3& rho, float r) {
    const float c1 = 0.5F - (2.0F / (3.0F * kPi));
    const float c2 = (2.0F / 3.0F) - (28.0F / (15.0F * kPi));
    const float eFon = 1.0F / (1.0F + (c1 * r));
    const float avgEFon = eFon * (1.0F + (c2 * r));
    const glm::vec3 rhoMs = (rho * rho) * avgEFon / (glm::vec3(1.0F) - (rho * (1.0F - avgEFon)));
    return (rho * eFon) + (rhoMs * (1.0F - eFon));
}

// A bare EON diffuse surface. ior=1 is what makes the measurement below exact rather than approximate: exact dielectric Fresnel is identically zero there while Schlick's (1-c)^5 tail is not, so coatFresnelRatio and dielectricFresnelAvg both collapse, diffuseKdAt becomes exactly 1, and evaluateBsdfSplit's diffuse channel is the raw EON lobe with no coupling factor multiplying it. Same device checkBeerLambert uses to remove the interface from a transmission measurement.
// checkIndexMatchedCoat asserts that collapse rather than assuming it, which is what the specular roughness parameter is for: it is inert here by the same argument, so sweeping it is the instrument.
// f0 is 0 to match ior=1 rather than for effect: it feeds the specular lobe only, which this check never reads.
BsdfParams makeDiffuseParams(const glm::vec3& baseColor, float diffuseRoughness,
                              float roughness = 0.5F, float ior = 1.0F) {
    return BsdfParams{baseColor,          /*metallic=*/0.0F,          roughness,
                       glm::vec3(0.0F),    /*edgeTint=*/glm::vec3(1.0F), ior,
                       /*transmissionFactor=*/0.0F, diffuseRoughness,
                       pathtracer::scene::eonAlbedoInversion(baseColor, diffuseRoughness),
                       /*transmissionTint=*/glm::vec3(1.0F)};
}

struct AlbedoEstimate {
    glm::vec3 mean;
    glm::vec3 stdError;
};

// Cosine-weighted hemispherical integral of the SHIPPED diffuse lobe at normal incidence -- the directional albedo the renderer actually produces. Uniform-hemisphere estimator (pdf 1/2pi), so the integrand is f*cos*2pi.
// The second moment is accumulated alongside the first so the assertion band can be the estimator's own standard error, computed from the run, rather than a tolerance picked until the suite went green.
AlbedoEstimate measureDiffuseAlbedo(const BsdfParams& params, int sampleCount, std::mt19937& rng) {
    const glm::vec3 wo(0.0F, 0.0F, 1.0F);
    glm::vec3 sum(0.0F);
    glm::vec3 sumSq(0.0F);
    for (int i = 0; i < sampleCount; ++i) {
        const glm::vec3 wi = sampleUniformHemisphere(rng);
        const glm::vec3 sample =
            pathtracer::scene::evaluateBsdfSplit(params, wo, wi).diffuse * wi.z * 2.0F * kPi;
        sum += sample;
        sumSq += sample * sample;
    }
    const auto n = static_cast<float>(sampleCount);
    const glm::vec3 mean = sum / n;
    const glm::vec3 variance = glm::max((sumSq / n) - (mean * mean), glm::vec3(0.0F));
    return {mean, glm::sqrt(variance / n)};
}

// The observed albedo must equal the AUTHORED albedo: the property EON's Appendix A inversion exists to provide, and the one thing nothing else in this suite can see. Every other case in every validator runs at either baseColor 1 or diffuseRoughness 0, where the inversion is exactly the identity -- measured, by reverting it and finding all five validators byte-identical -- so without this check the inversion could be deleted silently.
// Two assertions per row, complementary rather than redundant. The analytic one is closed-form against closed-form and catches an error in the inversion algebra; the Monte Carlo one integrates the shipped lobe and additionally catches a diffuseRho that is computed and never consumed, which no closed-form identity can.
// The chromatic row carries the weight: the multiple-scattering saturation the inversion undoes is per-channel, so a grey row cannot distinguish a correct inversion from one that merely preserves overall brightness. r=0 and baseColor=1 rows must read the identity exactly, which is what proves this change is a strict superset of the old behaviour rather than a shift of it.
PT_CHECK(eon_albedo_inversion, Slow, Statistical) {
    constexpr int kSampleCount = 400000;
    // Two named, bounded residuals and nothing else. kSigmaBand is a confidence level on the estimator's own measured standard error; kFitTolerance is the paper's stated <0.1% bound on evalFonAlbedoApprox, the quartic the renderer evaluates in place of the exact FON albedo this check's reference uses. Neither is a tuned number: the first is derived per row from the run, the second is the documented error of an approximation the renderer deliberately ships.
    constexpr float kSigmaBand = 5.0F;
    constexpr float kFitTolerance = 0.001F;
    constexpr float kAnalyticTolerance = 1e-5F;
    const std::array<float, 5> diffuseRoughnesses = {0.0F, 0.25F, 0.5F, 0.75F, 1.0F};
    const std::array<glm::vec3, 3> albedos = {glm::vec3(1.0F), glm::vec3(0.5F),
                                               glm::vec3(0.8F, 0.3F, 0.1F)};

    bool ok = true;
    std::mt19937 rng(31337);
    std::cout << "bsdf_validate: EON albedo inversion, observed vs authored albedo at normal incidence\n";
    std::cout << "  diffuseRoughness  authored              rho                   observed              worst err\n";
    for (const glm::vec3& authored : albedos) {
        for (float diffuseRoughness : diffuseRoughnesses) {
            const BsdfParams params = makeDiffuseParams(authored, diffuseRoughness);
            const glm::vec3 analytic = referenceEonAlbedo(params.diffuseRho, diffuseRoughness);
            const AlbedoEstimate measured = measureDiffuseAlbedo(params, kSampleCount, rng);

            const glm::vec3 analyticErr = glm::abs(analytic - authored);
            const glm::vec3 band =
                (kSigmaBand * measured.stdError) + (kFitTolerance * glm::max(authored, 0.01F));
            const glm::vec3 measuredErr = glm::abs(measured.mean - authored);

            std::cout << "  " << diffuseRoughness << "               [" << authored.x << ", "
                       << authored.y << ", " << authored.z << "]   [" << params.diffuseRho.x << ", "
                       << params.diffuseRho.y << ", " << params.diffuseRho.z << "]   ["
                       << measured.mean.x << ", " << measured.mean.y << ", " << measured.mean.z
                       << "]   " << maxChannel(measuredErr) << '\n';

            if (maxChannel(analyticErr) > kAnalyticTolerance) {
                std::cerr << "bsdf_validate: FAILED EON albedo inversion (analytic) at diffuseRoughness="
                           << diffuseRoughness << " authored=[" << authored.x << ", " << authored.y
                           << ", " << authored.z << "] E_EON=[" << analytic.x << ", " << analytic.y
                           << ", " << analytic.z << "] err=" << maxChannel(analyticErr)
                           << " (expected <= " << kAnalyticTolerance << ")\n";
                ok = false;
            }
            if (!glm::all(glm::lessThanEqual(measuredErr, band))) {
                std::cerr << "bsdf_validate: FAILED EON albedo inversion (measured) at diffuseRoughness="
                           << diffuseRoughness << " authored=[" << authored.x << ", " << authored.y
                           << ", " << authored.z << "] observed=[" << measured.mean.x << ", "
                           << measured.mean.y << ", " << measured.mean.z
                           << "] err=" << maxChannel(measuredErr) << " (expected <= "
                           << maxChannel(band) << ")\n";
                ok = false;
            }
        }
    }
    finish(ctx, ok, "eon_albedo_inversion failed; see the rows above");
    return;
}

// EON BRDF value (paper eq. 16-19) in double, transcribed independently of bsdf.cpp's evaluateEon: c1/c2 re-derived from their literals and every term written out rather than shared, the discipline referenceEonAlbedo applies one level up to the albedo.
// The quartic albedo fit's coefficients (paper eq. 14) are the paper's data, not arithmetic, so quoting them is transcription and not the same expression tested against itself. What this reference cannot catch is a typo inside that quartic; checkEonAlbedoInversion's Monte Carlo integral of the shipped lobe covers it, and the two checks are complementary for that reason.
// The 1e-7 floors are reproduced rather than dropped. They are the model's guarded evaluation at r=0, where 1-E_F and 1-<E_F> are all identically zero, not a tolerance -- omitting them would leave the reference disagreeing at exactly the row that matters least and force a looser bound on every row that matters more.
glm::vec3 referenceEon(const glm::vec3& rho, float r, const glm::vec3& wi, const glm::vec3& wo) {
    const double c1 = 0.5 - (2.0 / (3.0 * kPi));
    const double c2 = (2.0 / 3.0) - (28.0 / (15.0 * kPi));
    const double muI = wi.z;
    const double muO = wo.z;
    const double s = static_cast<double>(glm::dot(wi, wo)) - (muI * muO);
    const double sOverT = s > 0.0 ? s / std::max(muI, muO) : s;
    const double af = 1.0 / (1.0 + (c1 * r));
    const double avgEFon = af * (1.0 + (c2 * r));
    // Paper eq. 14's quartic in (1 - mu), evaluated as an explicit polynomial rather than bsdf.cpp's Horner nesting.
    const auto eFon = [&](double mu) {
        const double m = 1.0 - mu;
        const double gOverPi = (0.0571085289 * m) + (0.491881867 * m * m) +
                                (-0.332181442 * m * m * m) + (0.0714429953 * m * m * m * m);
        return (1.0 + (r * gOverPi)) * af;
    };
    constexpr double kEps = 1e-7;
    const double shadow = (std::max(kEps, 1.0 - eFon(muO)) * std::max(kEps, 1.0 - eFon(muI))) /
                           std::max(kEps, 1.0 - avgEFon);
    glm::vec3 result(0.0F);
    for (int c = 0; c < 3; ++c) {
        const double rhoC = rho[c];
        const double rhoMs = (rhoC * rhoC * avgEFon) / (1.0 - (rhoC * (1.0 - avgEFon)));
        result[c] = static_cast<float>(((rhoC * af * (1.0 + (r * sOverT))) + (rhoMs * shadow)) / kPi);
    }
    return result;
}

// The instrument for coatAlbedo's fresnelAvg ARGUMENT, which nothing else in this suite can see. checkAverageFresnel pins dielectricFresnelAvg the function; reverting the call sites (bsdf.cpp:545, :821, :825, all fed by the one dielectricFresnelAvg at :818) to Karis' schlickFresnelAvg(coatF0) leaves all six validators green while moving 1882 of the 1886 channels cornell changed, because clay.json's ior 1.55 is every wall, the floor and the ceiling.
// ior=1 is the only point where the ARGUMENT's collapse is resolvable without reading the albedo table at all: there its coefficients are multiplied by exact zeros and drop out of the expression entirely, so this check needs no reference for E and asserts at tolerance zero. checkCoatFresnelAvg covers the argument's value at ior>1, which needs that reference and could not exist until the table moved offline and got accurate enough to carry it.
// The collapse is exact in float32, term by term: dielectricF0(1)=+0; dielectricFresnelAvg(1)=+0 because every node of its quadrature rule evaluates fresnelDielectric at etaI==etaT, where etaI/etaT is exactly 1, cos2Transmitted returns mu*mu, sqrt(mu*mu) is exactly mu and both polarisations are an exact (c-c)/(c+c), so multiScatterTint(0,E)=+0; fresnelDielectric(mu,1,1)=+0 for the same reason, one level down, so coatFresnelRatio=+0; hence coatAlbedo=+0, diffuseCoupling=(1-0)/(1-0)=1, diffuseKdAt=1, and .diffuse is evaluateEon multiplied by exactly 1.0F. evaluateEon reads only diffuseRho/diffuseRoughness/wi/wo, and alpha reaches .specular alone, so params.roughness has no other route into the diffuse channel: an index-matched interface is optically absent, and its roughness cannot be observable.
// The mean and the ratio now collapse for one reason rather than two: both are fresnelDielectric at an index-matched interface, so the zero this asserts is structural in the function the single scatter itself evaluates, and cannot be lost to a refit of the mean.
// Hence tolerance exactly zero, from x*1.0F == x -- an algebraic guarantee, not a measured run. The Karis revert breaks it by a measured 7.8e-4 relative, worst at roughness 0.92 / mu_o = mu_i = 1, about 8000 ULP at the diffuse value's magnitude.
// Two facts that set the sweep, both against instinct. The deviation peaks at NORMAL incidence, not grazing: coat = msTint*(1-E(mu)) and E rises toward grazing at high roughness (0.31 at mu=1, ~0.99 at the first grid column), so a grazing-first sweep is much weaker. And it changes sign below mu~0.3 at roughness 1, so only |delta|==0 is a safe predicate; any signed bound breaks.
// The sweep reaches mu 1e-5 deliberately, and the four rows below 2.44e-4 are the regression test for the cos^2 Snell form: the old 1 - r^2*(1 - mu*mu) transcription rounded 1.0F-mu*mu to exactly 1.0F there, reported total internal reflection at an interface that has no critical angle, and returned fresnelDielectric(mu,1,1) = 1.0F instead of +0, which breaks every term of the collapse above. cos2Transmitted collapses to mu*mu at r == 1, so the sliver is now exact rather than excluded. Do NOT extend the sweep to ior>1 -- the identity is exact only at index match.
// Residual, deliberate: any g(ior) with g(1)=0 passes here, notably 2*dielectricFresnelAvg(ior). This check pins the argument's collapse; checkAverageFresnel pins the function's value. Neither alone is sufficient and both are cheap.
PT_CHECK(index_matched_coat, Fast, Exact) {
    // Exact, from the collapse above. The second bound is a float32-vs-double residual on the same closed form, ~15 operations deep, measured worst 2.03e-7 at the grazing tail -- 4.9x under, thin on purpose. It is not the instrument; it is the backstop that stops a roughness-INDEPENDENT corruption (a pinned diffuseKd, a lost 1/(1-coatAvg) normalisation, a channel swap) from passing as bit-identical, which the invariance assertion alone cannot see.
    constexpr float kInvarianceTolerance = 0.0F;
    constexpr float kValueTolerance = 1e-6F;
    // 0.0 is the reference row every other is compared against. 0.3661 and 0.92 sit deliberately off the table's k/127 grid, the device checkWhiteFurnaceTwoSided's off-grid cases use; 0.92 is where the revert's deviation is largest.
    const std::array<float, 8> roughnesses = {0.0F, 0.05F, 0.25F, 0.3661F, 0.5F, 0.75F, 0.92F, 1.0F};
    // The tail below 2.44e-4 (2^-12, where 1.0F-mu*mu rounds to 1.0F) is where the old Snell transcription falsely reported TIR; 1.7263349e-4 is the exact cosine the CHANGELOG records the failure at.
    const std::array<float, 11> cosines = {1.0F,   0.8F,         0.6F,    0.4F,    0.2F, 0.05F,
                                           1e-2F, 1e-3F, 2.44e-4F, 1.7263349e-4F, 1e-5F};
    const std::array<float, 3> diffuseRoughnesses = {0.0F, 0.5F, 1.0F};
    // The chromatic row carries the weight for the value assertion, the reason checkEonAlbedoInversion gives: at baseColor 1 the inversion is the identity and a grey row cannot tell a correct result from one that merely preserves brightness.
    const std::array<glm::vec3, 2> albedos = {glm::vec3(1.0F), glm::vec3(0.8F, 0.3F, 0.1F)};

    bool ok = true;
    int rowsChecked = 0;
    float worstOverall = 0.0F;
    float worstValueErr = 0.0F;
    std::cout << "bsdf_validate: index-matched coat, diffuse channel vs specular roughness (ior 1)\n";
    std::cout << "  baseColor              diffuseRoughness  worst |delta|  at roughness/mu_o/mu_i\n";
    for (const glm::vec3& albedo : albedos) {
        for (float diffuseRoughness : diffuseRoughnesses) {
            float worst = 0.0F;
            float worstRoughness = 0.0F;
            float worstMuO = 0.0F;
            float worstMuI = 0.0F;
            for (float muO : cosines) {
                for (float muI : cosines) {
                    // Non-coplanar pair, checkReciprocity's construction: a shared azimuth would leave a swapped-phi bug invisible.
                    const float sinO = std::sqrt(std::max(0.0F, 1.0F - (muO * muO)));
                    const float sinI = std::sqrt(std::max(0.0F, 1.0F - (muI * muI)));
                    const glm::vec3 wo(sinO, 0.0F, muO);
                    const glm::vec3 wi(sinI * std::cos(1.1F), sinI * std::sin(1.1F), muI);
                    const glm::vec3 reference =
                        pathtracer::scene::evaluateBsdfSplit(
                            makeDiffuseParams(albedo, diffuseRoughness, roughnesses[0]), wo, wi)
                            .diffuse;
                    const glm::vec3 analytic = referenceEon(
                        pathtracer::scene::eonAlbedoInversion(albedo, diffuseRoughness),
                        diffuseRoughness, wi, wo);
                    const float scale = std::max(maxChannel(analytic), 1e-6F);
                    const float valueErr = maxChannel(glm::abs(reference - analytic)) / scale;
                    worstValueErr = std::max(worstValueErr, valueErr);
                    if (!(valueErr <= kValueTolerance)) {
                        std::cerr << "bsdf_validate: FAILED index-matched coat (value) at baseColor=["
                                   << albedo.x << ", " << albedo.y << ", " << albedo.z
                                   << "] diffuseRoughness=" << diffuseRoughness << " mu_o=" << muO
                                   << " mu_i=" << muI << " diffuse=[" << reference.x << ", "
                                   << reference.y << ", " << reference.z << "] reference EON=["
                                   << analytic.x << ", " << analytic.y << ", " << analytic.z
                                   << "] relative err=" << valueErr << " (expected <= "
                                   << kValueTolerance << ")\n";
                        ok = false;
                    }
                    for (std::size_t i = 1; i < roughnesses.size(); ++i) {
                        const glm::vec3 value =
                            pathtracer::scene::evaluateBsdfSplit(
                                makeDiffuseParams(albedo, diffuseRoughness, roughnesses[i]), wo, wi)
                                .diffuse;
                        const float delta = maxChannel(glm::abs(value - reference)) / scale;
                        ++rowsChecked;
                        if (delta > worst) {
                            worst = delta;
                            worstRoughness = roughnesses[i];
                            worstMuO = muO;
                            worstMuI = muI;
                        }
                        if (!(delta <= kInvarianceTolerance)) {
                            std::cerr << "bsdf_validate: FAILED index-matched coat at baseColor=["
                                       << albedo.x << ", " << albedo.y << ", " << albedo.z
                                       << "] diffuseRoughness=" << diffuseRoughness
                                       << " roughness=" << roughnesses[i] << " mu_o=" << muO
                                       << " mu_i=" << muI << " diffuse=[" << value.x << ", "
                                       << value.y << ", " << value.z << "] vs roughness 0 ["
                                       << reference.x << ", " << reference.y << ", " << reference.z
                                       << "] relative delta=" << delta
                                       << " (expected exactly 0: at ior 1 the coat is optically absent, so its roughness cannot reach the diffuse channel)\n";
                            ok = false;
                        }
                    }
                }
            }
            worstOverall = std::max(worstOverall, worst);
            std::cout << "  [" << albedo.x << ", " << albedo.y << ", " << albedo.z << "]         "
                       << diffuseRoughness << "               " << worst << "            "
                       << worstRoughness << " / " << worstMuO << " / " << worstMuI << '\n';
        }
    }
    std::cout << "  " << rowsChecked << " roughness comparisons, worst |delta| " << worstOverall
               << ", worst value error " << worstValueErr << '\n';
    // Anti-vacuity: a sweep that asserted nothing would print zeros too.
    if (rowsChecked == 0) {
        std::cerr << "bsdf_validate: FAILED index-matched coat -- no rows asserted\n";
        ok = false;
    }
    finish(ctx, ok, "index_matched_coat failed; see the rows above");
    return;
}

// Mean throughput through sampleBsdf with every transmitted draw converted back from radiance to energy. sampleBsdf applies the non-symmetric eta^2 radiance compression on refraction (Veach 1997 sec. 5.2), so a transmitted sample carries radiance and a raw mean is bounded by ior^2, not 1.0, which is why checkFurnace can only assert an upper bound on its transmissive rows and never sees energy loss there.
// Dividing those draws by eta^2 puts every sample in one domain with an analytic answer; LobeType::Transmission is exactly the far-hemisphere draws, delta and rough alike.
// Accumulated in double: at 200k samples of about 1 a float sum carries an ulp of 0.008, and throughputs that cluster just above an exact 1 round down every time -- a systematic 1e-3 that reads as model error.
glm::vec3 transmissiveEnergyLo(const BsdfParams& params, const glm::vec3& wo, int sampleCount,
                                std::uint32_t seed) {
    const float eta = wo.z < 0.0F ? params.ior : 1.0F / params.ior;  // etaI/etaT, exiting vs entering
    const float etaSq = eta * eta;
    glm::dvec3 accum(0.0);
    for (int i = 0; i < sampleCount; ++i) {
        pathtracer::scene::Sampler sampler(0, 0, i, sampleCount, seed);
        const std::optional<pathtracer::scene::BsdfSample> sample =
            pathtracer::scene::sampleBsdf(params, wo, sampler);
        if (!sample.has_value()) {
            continue;
        }
        accum += glm::dvec3(sample->type == pathtracer::scene::LobeType::Transmission
                                 ? sample->throughputWeight / etaSq
                                 : sample->throughputWeight);
    }
    return glm::vec3(accum / static_cast<double>(sampleCount));
}

// TWO-SIDED energy balance for a transmissive interface: the counterpart to checkWhiteFurnaceTwoSided, which is restricted to "no transmission, entering side" since those are the only rows where 1.0 is correct in the radiance domain.
// In the energy domain 1.0 is correct everywhere: a white, non-absorbing interface reflects, refracts, or hands the rest to the diffuse substrate, and the multiple-scattering lobes return what the Smith G2 masked; nothing is absorbed at any roughness, side, or transmissionFactor.
// Gates two failure modes the radiance-domain checks structurally cannot see: multiple-scattering compensation delivered over the refraction-reachable cone only rather than the whole far hemisphere, and a transmission lobe whose value drops transmissionFactor or (1-metallic) while its selection probability keeps them (the factors cancel out of throughput, so only an absolute bound catches it).
// metallic=1 rows cover a conductor, which must transmit nothing however its transmissionFactor is set.
PT_CHECK(transmissive_energy_balance, Slow, Statistical) {
    constexpr int kSampleCount = 200000;
    // Same tolerance as the opaque white furnace: 1.0 is a correctness target, not a baseline, and the residual under it is deterministic model error rather than noise, which is why this stays a fixed target and not a replicate-derived band.
    // Measured worst 0.0036, at transmissionFactor 0.5 entering, where the diffuse coat coupling renormalises by an averaged rescale (the white furnace bounds that residual under 1%). The fully transmissive rows sit at 0.0012, the escape table's interpolation between its nodes.
    constexpr float kTolerance = 0.02F;
    const std::array<float, 4> roughnesses = {0.05F, 0.4F, 0.7F, 1.0F};
    const std::array<float, 4> ndotVs = {1.0F, 0.6F, -0.9F, -0.4F};  // entering, entering, exiting, TIR
    const std::array<float, 2> transmissions = {0.5F, 1.0F};
    const std::array<float, 2> metallics = {0.0F, 1.0F};

    bool ok = true;
    std::uint32_t seed = 12000;
    std::cout << "bsdf_validate: transmissive energy balance (1.0 = perfectly energy-conserving)\n";
    std::cout << "  roughness  ndotV  transmission  metallic  Lo\n";
    for (float roughness : roughnesses) {
        for (float transmission : transmissions) {
            for (float metallic : metallics) {
                for (float ndotV : ndotVs) {
                    ++seed;
                    const BsdfParams params = makeParams(roughness, metallic, transmission);
                    const glm::vec3 wo(std::sqrt(std::max(0.0F, 1.0F - (ndotV * ndotV))), 0.0F,
                                        ndotV);
                    const glm::vec3 lo = transmissiveEnergyLo(params, wo, kSampleCount, seed);
                    std::cout << "  " << roughness << "        " << ndotV << "     " << transmission
                              << "           " << metallic << "       " << minChannel(lo) << '\n';
                    if (!withinBand(lo, 1.0F, kTolerance)) {
                        std::cerr << "bsdf_validate: FAILED transmissive energy balance at roughness="
                                  << roughness << " ndotV=" << ndotV
                                  << " transmission=" << transmission << " metallic=" << metallic
                                  << " Lo=[" << minChannel(lo) << ", " << maxChannel(lo)
                                  << "] (expected 1.0 +/- " << kTolerance << ")\n";
                        ok = false;
                    }
                }
            }
        }
    }
    finish(ctx, ok, "transmissive_energy_balance failed; see the rows above");
    return;
}

// Round-trip energy closure of a rough dielectric: a white, non-absorbing slab reads exactly 1.0 under a uniform environment, since every photon that enters leaves and the eta^2 compression cancels over the crossing pair.
// Unlike transmissive_energy_balance, which weighs one vertex at a handful of directions, this integrates the escape table over every direction a transmitted path actually visits, entering and exiting, which is where its interpolation error in mu and eta accumulated: 1.2% / 0.6% / 0.9% dark at roughness 0.4 / 0.7 / 1.0 on the 16^2-sample, 32 x 32 x 16 table this replaced.
// BSDF sampling alone (fixtures::slabWalkLo), so no integrator term can move it; integrator_validate's transmissive_slab_energy holds the Embree render to this same walk.
// Replicated over independent scramble seeds, so the statistical half-width is the run's own Student-t interval rather than a hand-set tolerance. Closure also has a deterministic floor: the escape a vertex reads is interpolated between table nodes, and a path crosses several, so the band adds that floor times the walk's own measured vertices per path.
// The floor is the table's accuracy against the generator's exact quadrature, not a fitted tolerance: 1e-3 over a measured worst 6.2e-4 (tools/albedo_table.cpp's grid read through bsdf.cpp's lookup, ior 1.5, roughness 0.4 to 1, both sides, all mu). Without it the check would reject the table's own resolution as an error the moment variance dropped below it.
PT_CHECK(transmissive_slab_walk, Slow, Statistical) {
    // Sized so the band resolves the smallest shortfall the old table left, 0.6% at roughness 0.7, with room to spare.
    constexpr int kPathsPerReplicate = 1 << 20;
    constexpr double kEscapeInterpolationFloor = 1e-3;
    const std::array<float, 3> roughnesses = {0.4F, 0.7F, 1.0F};

    ctx.plan(static_cast<int>(2 * roughnesses.size()));
    std::cout << "bsdf_validate: white rough-glass slab by BSDF sampling alone (1.0 = energy closes)\n";
    for (float roughness : roughnesses) {
        char label[64];
        std::snprintf(label, sizeof(label), "slab walk r=%g", static_cast<double>(roughness));
        const std::uint64_t rowSeed = ctx.subSeed(label);
        const BsdfParams params = makeParams(roughness, /*metallic=*/0.0F, /*transmissionFactor=*/1.0F);
        std::array<tools::fixtures::SlabWalk, tools::stats::kReplicates> replicates{};
        std::atomic<int> next{0};
        std::vector<std::thread> workers;
        for (int w = 0; w < std::min(ctx.threads(), tools::stats::kReplicates); ++w) {
            workers.emplace_back([&] {
                for (int r = next++; r < tools::stats::kReplicates; r = next++) {
                    replicates[static_cast<std::size_t>(r)] =
                        tools::fixtures::slabWalkLo(params, kPathsPerReplicate, static_cast<std::uint32_t>(rowSeed + r));
                }
            });
        }
        for (std::thread& worker : workers) {
            worker.join();
        }
        tools::stats::Welford lo;
        tools::stats::Welford vertices;
        long long truncated = 0;
        for (const tools::fixtures::SlabWalk& replicate : replicates) {
            lo.add(replicate.mean);
            vertices.add(replicate.verticesPerPath);
            truncated += replicate.truncated;
        }
        const double half = (tools::stats::studentTTwoSided(ctx.alpha(), tools::stats::kReplicates - 1) * lo.standardError()) +
                            (kEscapeInterpolationFloor * vertices.mean());
        char detail[256];
        std::snprintf(detail, sizeof(detail), "%s: Lo %.5f vs 1 +/- %.5f over %.2f vertices per path", label, lo.mean(), half, vertices.mean());
        std::cout << "  " << detail << '\n';
        PT_EXPECT(ctx, std::abs(lo.mean() - 1.0) <= half, detail);
        std::snprintf(detail, sizeof(detail), "%s: %lld paths reached the depth cap, which would truncate the estimate", label, truncated);
        PT_EXPECT(ctx, truncated == 0, detail);
    }
}

// A white, non-absorbing transmissive dielectric with an explicit baseColor and transmissionTint -- the one configuration this suite never had, makeParams above hardcoding baseColor 1 and every transmissive row leaving the tint white.
BsdfParams makeTransmissiveTintParams(float roughness, const glm::vec3& baseColor,
                                       const glm::vec3& transmissionTint) {
    return BsdfParams{baseColor,          /*metallic=*/0.0F,            roughness,
                       glm::vec3(0.04F),   /*edgeTint=*/glm::vec3(1.0F), /*ior=*/1.5F,
                       /*transmissionFactor=*/1.0F, /*diffuseRoughness=*/0.0F,
                       pathtracer::scene::eonAlbedoInversion(baseColor, 0.0F), transmissionTint};
}

// Snell refraction of wo about +z, transcribed here independently of bsdf.cpp: the direction the interface actually refracts into, where the single-scattering transmission lobe is strongest.
glm::vec3 refractAboutZ(const glm::vec3& wo, float eta) {
    const float sin2ThetaT = eta * eta * std::max(0.0F, 1.0F - (wo.z * wo.z));
    const float cosThetaT = std::sqrt(std::max(0.0F, 1.0F - sin2ThetaT));
    return {-eta * wo.x, -eta * wo.y, -cosThetaT};
}

// Throughput of sampleBsdf's smooth delta transmission branch, which carries its tint on a different code path from the continuous lobes and so needs measuring separately.
// pdf == 0 identifies the delta branch: a rough transmission sample returns a real density. The seed is fixed and the lobe-selection probabilities are colourless, so every params variation below draws the identical lobe and the identical direction.
std::optional<glm::vec3> deltaTransmitThroughput(const BsdfParams& params, const glm::vec3& wo,
                                                  std::uint32_t seed) {
    constexpr int kAttempts = 64;
    for (int i = 0; i < kAttempts; ++i) {
        pathtracer::scene::Sampler sampler(0, 0, i, kAttempts, seed);
        const std::optional<pathtracer::scene::BsdfSample> sample =
            pathtracer::scene::sampleBsdf(params, wo, sampler);
        if (sample.has_value() && sample->type == pathtracer::scene::LobeType::Transmission &&
            sample->pdf == 0.0F) {
            return sample->throughputWeight;
        }
    }
    return std::nullopt;
}

// The transmission-tint convention, and the only instrument in this suite that can see it: every other transmissive case here runs at baseColor 1 and transmissionTint 1, where both are exactly the identity.
// OpenPBR and Arnold both make transmissionColor the sole tint of transmitted light -- realized in the volume at transmissionDepth > 0, on the surface at 0 -- while base_color is "the observed reflection color (viewed at normal incidence under uniform illumination)" and leaves transmitted light "unaffected". glTF KHR_materials_transmission tints with baseColor only for want of a transmission colour of its own, its own text defining that tint as the infinitely-thin absorption "1.0 - baseColor", which is the depth == 0 case under another name.
// Two complementary assertions per row. Independence of baseColor is asserted EXACTLY: at metallic 0 baseColor reaches f0 not at all, and the diffuse lobe is identically zero on the far side, so no term of the transmission value can legitimately move by one bit.
// Linearity in the tint carries a few-ULP relative band instead, because the rough lobe sums a single-scattering and a multiple-scattering term and FP multiplication does not distribute over a sum. That band is numerical, not physical slack: a tint applied to only one of the two terms misses by the other term's whole share.
// Rows span both code paths a tint must travel: the rough continuous lobe (evaluateTransmissionLobe plus transmitMultiScatter, which share one returned value here) and sampleBsdf's smooth delta branch.
PT_CHECK(transmission_tint, Fast, Exact) {
    constexpr float kUlpBand = 1e-6F;
    constexpr float kSmoothRoughness = 0.005F;   // below bsdf.cpp's smooth threshold, so transmission is the delta branch
    const glm::vec3 baseColour(0.2F, 0.5F, 0.9F);
    const glm::vec3 tint(0.3F, 0.6F, 0.9F);
    const glm::vec3 white(1.0F);
    const std::array<float, 3> roughnesses = {0.3F, 0.6F, 1.0F};
    const std::array<float, 3> ndotVs = {0.95F, 0.7F, 0.35F};

    bool ok = true;
    int measured = 0;
    std::uint32_t seed = 31000;

    // One row's pair of assertions, shared by the rough and smooth paths: T must not move with baseColor at all, and must scale exactly with the tint.
    const auto assertRow = [&](const char* lobe, float roughness, float ndotV,
                                const glm::vec3& tWhite, const glm::vec3& tBaseColoured,
                                const glm::vec3& tTinted, const glm::vec3& tBoth) {
        if (maxChannel(tWhite) <= 0.0F) {
            return;   // no transmission at this configuration; the backstop below catches an all-skipped run
        }
        ++measured;
        const glm::vec3 expected = tint * tWhite;
        std::cout << "  " << lobe << "  roughness " << roughness << "  ndotV " << ndotV
                  << "   T(white) [" << tWhite.x << ", " << tWhite.y << ", " << tWhite.z
                  << "]   tinted/expected [" << tTinted.x / expected.x << ", "
                  << tTinted.y / expected.y << ", " << tTinted.z / expected.z << "]\n";
        for (int c = 0; c < 3; ++c) {
            if (tBaseColoured[c] != tWhite[c] || tBoth[c] != tTinted[c]) {
                std::cerr << "bsdf_validate: FAILED transmission tint at " << lobe << " roughness="
                          << roughness << " ndotV=" << ndotV << " channel " << c
                          << " -- baseColor moved the transmitted value from " << tWhite[c]
                          << " to " << tBaseColoured[c]
                          << ". baseColor is a reflection quantity (OpenPBR base_color) and must not "
                             "reach the transmission lobe; transmissionTint is what tints it.\n";
                ok = false;
            }
            if (std::fabs(tTinted[c] - expected[c]) > kUlpBand * std::fabs(expected[c])) {
                std::cerr << "bsdf_validate: FAILED transmission tint linearity at " << lobe
                          << " roughness=" << roughness << " ndotV=" << ndotV << " channel " << c
                          << " -- measured " << tTinted[c] << ", expected " << expected[c]
                          << ". transmissionTint multiplies the transmitted value once, so the lobe "
                             "must be exactly linear in it.\n";
                ok = false;
            }
        }
    };

    std::cout << "bsdf_validate: transmission tint convention (transmissionColor tints transmission, baseColor does not)\n";
    for (float ndotV : ndotVs) {
        ++seed;
        const glm::vec3 wo(std::sqrt(std::max(0.0F, 1.0F - (ndotV * ndotV))), 0.0F, ndotV);
        const glm::vec3 wi = refractAboutZ(wo, 1.0F / 1.5F);
        for (float roughness : roughnesses) {
            const auto rough = [&](const glm::vec3& bc, const glm::vec3& tn) {
                return pathtracer::scene::evaluateBsdfSplit(makeTransmissiveTintParams(roughness, bc, tn),
                                                         wo, wi)
                    .transmission;
            };
            assertRow("rough ", roughness, ndotV, rough(white, white), rough(baseColour, white),
                       rough(white, tint), rough(baseColour, tint));
        }
        // The delta branch does not depend on the roughness sweep above -- it is selected by being below the smooth threshold -- so it is measured once per view angle rather than once per row.
        const auto smooth = [&](const glm::vec3& bc, const glm::vec3& tn) {
            return deltaTransmitThroughput(makeTransmissiveTintParams(kSmoothRoughness, bc, tn), wo,
                                            seed)
                .value_or(glm::vec3(0.0F));
        };
        assertRow("smooth", kSmoothRoughness, ndotV, smooth(white, white), smooth(baseColour, white),
                   smooth(white, tint), smooth(baseColour, tint));
    }

    // Backstop: every assertion above is skipped where the interface transmits nothing, so a change that silently zeroed the transmission lobe would otherwise pass this check by vacuum.
    if (measured == 0) {
        std::cerr << "bsdf_validate: FAILED transmission tint -- no row transmitted anything, so "
                     "nothing was asserted\n";
        ok = false;
    }
    finish(ctx, ok, "transmission_tint failed; see the rows above");
    return;
}

// Exact unpolarized dielectric Fresnel, entering orientation, in double -- the reference dielectricFresnelAvg's quadrature rule is measured against below.
double referenceDielectricFresnel(double cosTheta, double ior) {
    const double c = std::clamp(cosTheta, 0.0, 1.0);
    const double sinT2 = (1.0 - (c * c)) / (ior * ior);
    if (sinT2 >= 1.0) {
        return 1.0;
    }
    const double cosT = std::sqrt(1.0 - sinT2);
    const double rs = (c - (ior * cosT)) / (c + (ior * cosT));
    const double rp = ((ior * c) - cosT) / ((ior * c) + cosT);
    return 0.5 * ((rs * rs) + (rp * rp));
}

// The instrument the suite never had. F_avg attenuates every repeated bounce of the Kulla-Conty multiple-scattering lobe, and NO energy test in this file can see an error in it: checkWhiteFurnaceTwoSided runs at f0=1, where Schlick's mean, the quadrature rule and the truth all agree to 1e-4, and checkFurnace's coloured-metal rows are upper-bound-only and so blind to a loss. Measuring F_avg directly is the honest fix; testing around it is not available, because a two-sided grey-conductor furnace has no closed form.
// Truth is this file's own independent reference -- literal k^2, complex arithmetic, double -- not the shipped Fresnel, so a transcription error in bsdf.cpp shows up here instead of cancelling. The two inversions agree to 4e-12 (checkConductorFresnel pins that), twelve orders under the tolerance, so what this measures is the quadrature rule's own fit error and nothing else.
// Conductor tolerance is the fit's measured bound (max 4.0e-4 over the full clamped domain, worst near r=0.48) plus headroom. Karis' Schlick mean fails it by 216x at r=0.255 g=1, which is the point: reverting bsdf.cpp to schlickFresnelAvg must fail this test, and must NOT fail at g=1 r=1 where the old suite did all its conductor energy checking.
PT_CHECK(average_fresnel, Fast, Exact) {
    constexpr double kConductorTolerance = 5e-4;
    // The working band: everything any material actually authors. Measured worst 5.5e-5 at ior 1.0575 over a 0.0025-step scan of [1.05, 3.0], so this is ~1.8x headroom, matching the conductor's 5e-4 over a measured 4.0e-4. Deliberately thin, so a refit or a tolerance change trips this rather than passing silently -- the two-constant rational fit this replaced misses it by 60x.
    constexpr double kDielectricTolerance = 1e-4;
    // The near-index-match band, stated separately rather than absorbed into the one above, which it would loosen 8x over a region no material occupies. As ior -> 1 the reflectance becomes a boundary layer -- F(0) = 1 for every ior > 1, collapsing over a width ~sqrt(ior-1) -- and three fixed nodes cannot resolve it. Measured worst 5.9e-4 at ior 1.0057, where truth is itself 1.8e-3; it reaches coatAlbedo scaled by ~0.054, so under 3e-5 on the coupling.
    constexpr double kNearIndexMatchTolerance = 8e-4;
    constexpr double kNearIndexMatchIor = 1.05;
    const std::array<double, 12> reflectivities = {1e-4, 0.01, 0.1,  0.25, 0.4,  0.48,
                                                    0.555, 0.7, 0.85, 0.95, 0.99, 1.0};
    const std::array<double, 8> edgeTints = {0.0, 0.1, 0.25, 0.5, 0.6, 0.75, 0.9, 1.0};
    // Both regimes, and the two iors that ship (glass.json 1.5168, clay.json 1.55). ior=1 is where an index-matched interface reflects nothing and the rule must return exactly 0 -- structurally here, since every node evaluates fresnelDielectric at etaI==etaT; it is the collapse Karis' mean of coatF0 does not have (it returns 1/21 there).
    const std::array<double, 14> iors = {1.0,  1.005, 1.02, 1.05,   1.1,  1.2, 1.33, 1.5,
                                          1.5168, 1.55,  1.8,  2.0, 2.5, 3.0};

    bool ok = true;
    std::cout << "bsdf_validate: average Fresnel vs quadrature (F_avg = 2*int F(mu)*mu dmu)\n";
    std::cout << "  conductor: worst |rule - truth| over edgeTint, per reflectivity\n";
    for (double reflectivity : reflectivities) {
        double worst = 0.0;
        double worstTint = 0.0;
        for (double edgeTint : edgeTints) {
            const std::complex<double> eta = referenceConductorIor(reflectivity, edgeTint);
            const double truth = cosineAverageFresnel(
                [&](double mu) { return referenceConductorFresnelAt(eta, mu); });
            const glm::vec3 rule = pathtracer::scene::conductorFresnelAvg(
                glm::vec3(static_cast<float>(eta.real())), glm::vec3(static_cast<float>(eta.imag())));
            const double error = std::abs(static_cast<double>(rule.x) - truth);
            if (error > worst) {
                worst = error;
                worstTint = edgeTint;
            }
            if (!(error <= kConductorTolerance)) {
                std::cerr << "bsdf_validate: FAILED conductor F_avg at r=" << reflectivity
                          << " edgeTint=" << edgeTint << " rule=" << rule.x << " vs quadrature " << truth
                          << " (error " << error << ", tolerance " << kConductorTolerance << ")\n";
                ok = false;
            }
        }
        std::cout << "    r " << reflectivity << "   worst " << worst << " at edgeTint " << worstTint
                  << '\n';
    }

    std::cout << "  dielectric: |dielectricFresnelAvg - truth| per ior\n";
    for (double ior : iors) {
        const double truth =
            cosineAverageFresnel([&](double mu) { return referenceDielectricFresnel(mu, ior); });
        const double rule = pathtracer::scene::dielectricFresnelAvg(static_cast<float>(ior));
        const double error = std::abs(rule - truth);
        const double tolerance =
            ior < kNearIndexMatchIor ? kNearIndexMatchTolerance : kDielectricTolerance;
        std::cout << "    ior " << ior << "   rule " << rule << "   truth " << truth << "   error "
                  << error << "   tolerance " << tolerance << '\n';
        if (!(error <= tolerance)) {
            std::cerr << "bsdf_validate: FAILED dielectric F_avg at ior=" << ior << " rule=" << rule
                      << " vs quadrature " << truth << " (error " << error << ", tolerance "
                      << tolerance << ")\n";
            ok = false;
        }
    }
    finish(ctx, ok, "average_fresnel failed; see the rows above");
    return;
}

// --- Independent reference for the reflect-side albedo table, and with it the instrument for coatAlbedo's fresnelAvg VALUE.
// Nothing in this suite could read that table before. checkWhiteFurnaceTwoSided bounds it only to 2%, 500x looser than what follows needs, and the table was a private startup build until it moved offline into src/scene/albedo_table.inc (tools/albedo_table.cpp). Rebuilt here by composite Simpson rather than the generator's Gauss-Legendre, in double, so this is a check ON the committed .inc and not the .inc restated -- the same deliberate-duplication rule referenceEon and referenceConductorIor already follow.
// Domain and measure are the generator's, and they are why a fixed rule can do this at all. With tan(theta_h) = alpha*tan(psi) (Walter et al. 2007 eq. 35) GGX NDF sampling flattens the peak into D(h)cos(h) dw = sin(psi)cos(psi) dpsi dphi / pi, and in that frame both cosines are single harmonics: wo.h = R cos(theta_h - d) and wi.z = R cos(2 theta_h - d), with R = hypot(sin tv cos phi, cos tv) and d = atan2(sin tv cos phi, cos tv). The horizon clip wi.z > 0 is then exactly theta_h < (d + pi/2)/2, so no sample is discarded and nothing invalid is ever evaluated, where a VNDF estimator would be stuck at first order across the discontinuity it keeps inside its domain.
// The phi split at pi/2 is not cosmetic: d sweeps its entire -pi/2..pi/2 range within |cos phi| < mu, a boundary layer that narrows with mu, and as a panel endpoint it is resolved rather than straddled.
// The INNER rule is Gauss-Legendre, not Simpson, and that is a measured requirement rather than a preference. The psi substitution that flattens the NDF peak compresses the other end: wi.z falls from O(1) to 0 over an O(1) span of theta_h, which the substitution maps to a span of psi narrower by ~alpha*mu, so at mu = 1/127 the whole horizon layer is ~1e-3 wide against a 96-panel width of 1.6e-2. Any composite rule of fixed panel width straddles it -- measured convergence order ~1.2, with the ENTIRE error of a 96-panel Simpson sitting in its last panel pair. Gauss-Legendre puts its outermost node O(1/n^2) from the endpoint, which is inside the layer, so it resolves what no affordable refinement of a uniform mesh does: at that cell Simpson 96 is 5.2e-3 and 3072 panels still leaves 3.6e-5, against 1.5e-7 here. There is no closed-form breakpoint to split at instead -- the natural candidate, wi.z = mu, lands at an eighth of the layer's width.
// What that costs is a sharper statement of the deliberate-duplication rule, so state it rather than let it erode: this now shares a quadrature FAMILY with the generator and no longer catches an error in the family itself. It remains an independent transcription of the integrand, the domain and the measure, written here from the derivation and never included or linked from src/ or tools/albedo_table.cpp, which is where a transcription error would be; and it runs at twice the generator's node count, so the two are not the same arithmetic. checkAlbedoTableInterpolation's control row prints their disagreement on every run rather than assuming it away.
// Height-correlated Smith G2 divided by cosO, 2 cosI / (cosI s(cosO) + cosO s(cosI)) with s(c) = sqrt(alpha^2 + (1-alpha^2) c^2). Algebraically the Lambda form it replaced, since 1 + Lambda(cosO) + Lambda(cosI) = (cosI s(cosO) + cosO s(cosI)) / (2 cosO cosI), but written so no cosine is ever divided by.
// That matters to this reference twice over. The Lambda form carries a max(cos^2, 1e-8) clamp that bites on the incident cosine near the horizon; and the albedo it builds is an integral proportional to mu, so recovering E by dividing the result by mu amplifies the rule's relative error by 1/mu -- unusable once the table's warped mu axis reaches 6.2e-5, and catastrophic at the 2.5e-6 the first cell is sampled at. Folding the 1/mu into this closed form removes both, and makes E(0) = 1 directly evaluable rather than a limit.
double referenceSmithG2OverCosO(double cosO, double cosI, double alpha) {
    const double alpha2 = alpha * alpha;
    const auto radical = [&](double c) { return std::sqrt(alpha2 + ((1.0 - alpha2) * c * c)); };
    return 2.0 * cosI / ((cosI * radical(cosO)) + (cosO * radical(cosI)));
}

constexpr double kPiDouble = 3.14159265358979324;

// Gauss-Legendre nodes and weights mapped to [0,1], by Newton iteration on P_n through Bonnet's recurrence (Press et al., Numerical Recipes 3rd ed., sec. 4.6.1). Weights sum to 1, so the node array doubles as the [0,1] average.
// Built once per node count: the rule is a constant, and 192 Newton solves per call would cost more than the integral it serves.
struct GaussLegendreRule {
    std::vector<double> node;
    std::vector<double> weight;
};

template <int N>
const GaussLegendreRule& gaussLegendreRule() {
    static const GaussLegendreRule rule = [] {
        GaussLegendreRule built{std::vector<double>(N), std::vector<double>(N)};
        for (int i = 0; i < N; ++i) {
            double x = std::cos(kPiDouble * (i + 0.75) / (N + 0.5));
            double derivative = 0.0;
            for (int iteration = 0; iteration < 100; ++iteration) {
                double p0 = 1.0;
                double p1 = 0.0;
                for (int k = 0; k < N; ++k) {
                    const double p2 = p1;
                    p1 = p0;
                    p0 = ((((2.0 * k) + 1.0) * x * p1) - (k * p2)) / (k + 1.0);
                }
                derivative = N * ((x * p0) - p1) / ((x * x) - 1.0);
                const double step = p0 / derivative;
                x -= step;
                if (std::abs(step) <= 1e-16) {
                    break;
                }
            }
            built.node[static_cast<std::size_t>(i)] = 0.5 * (1.0 - x);
            built.weight[static_cast<std::size_t>(i)] = 1.0 / ((1.0 - (x * x)) * derivative * derivative);
        }
        return built;
    }();
    return rule;
}

// Gauss-Legendre over [lower, upper], on any value type with + and scalar *, matching simpson's shape.
template <int N, typename F>
auto gaussLegendre(double lower, double upper, F f) -> decltype(f(lower)) {
    const GaussLegendreRule& rule = gaussLegendreRule<N>();
    decltype(f(lower)) sum = f(lower) * 0.0;
    for (std::size_t i = 0; i < rule.node.size(); ++i) {
        sum = sum + (rule.weight[i] * f(lower + ((upper - lower) * rule.node[i])));
    }
    return (upper - lower) * sum;
}

// Composite Simpson over [lower, upper] with an even panel count, on any value type with + and scalar *.

// bsdf.cpp's roughness floor, mirrored so every reference below evaluates the alpha the lobe actually ships at rather than an unclamped one the shading never sees. One copy, because two oracles that disagree about the floor disagree about which function they are measuring.
double alphaAt(double roughness) {
    constexpr double kMinAlpha = 0.02 * 0.02;
    return std::max(roughness * roughness, kMinAlpha);
}

// Schlick-split directional albedo: .x is the a channel, .y the b, so Ess(f0) = f0*a + b and a + b = E.
glm::dvec2 referenceDirectionalAlbedo(double mu, double alpha) {
    constexpr int kPanels = 96;     // phi, even for Simpson; doubling it moves no digit this file resolves
    constexpr int kPsiNodes = 192;  // psi, twice the generator's, so the control row compares two rules and not one
    const double sinTv = std::sqrt(std::max(0.0, 1.0 - (mu * mu)));
    const auto azimuth = [&](double phi) {
        const double horizontal = sinTv * std::cos(phi);
        const double radius = std::sqrt((horizontal * horizontal) + (mu * mu));
        const double delta = std::atan2(horizontal, mu);
        const double psiMax = std::atan(std::tan(0.5 * (delta + (0.5 * kPiDouble))) / alpha);
        return gaussLegendre<kPsiNodes>(0.0, psiMax, [&](double psi) {
            const double thetaH = std::atan(alpha * std::tan(psi));
            const double woDotH = radius * std::cos(thetaH - delta);
            const double wiZ = radius * std::cos((2.0 * thetaH) - delta);
            const double weight = (woDotH / std::cos(thetaH)) *
                                   referenceSmithG2OverCosO(mu, wiZ, alpha) * std::sin(psi) * std::cos(psi);
            const double fc = std::pow(std::clamp(1.0 - woDotH, 0.0, 1.0), 5.0);
            return glm::dvec2(weight * (1.0 - fc), weight * fc);
        });
    };
    const glm::dvec2 half = simpson(0.0, 0.5 * kPiDouble, kPanels, azimuth) +
                             simpson(0.5 * kPiDouble, kPiDouble, kPanels, azimuth);
    return (2.0 / kPiDouble) * half;
}

// Cosine-weighted mean, 2*int_0^1 E(mu)*mu dmu. The mu=0 endpoint contributes exactly 0 because the mu weight kills a bounded E; the directional form is evaluable there now that it no longer divides by mu, so no guard stands in for that.
glm::dvec2 referenceAverageAlbedo(double alpha) {
    constexpr int kPanels = 64;
    return 2.0 * simpson(0.0, 1.0, kPanels,
                          [&](double mu) { return referenceDirectionalAlbedo(mu, alpha) * mu; });
}


// --- The instrument for src/scene/albedo_table.inc's INTERPOLATION error, the table's second error source.
// The generator prints the first on every bake: verifyReflect rebuilds at doubled node count and reports the quadrature's own residual, 3.0e-5. But the shipped grid is then read bilinearly, and that error is a separate quantity no code produced -- it reached the tree as comments on tools/albedo_table.cpp and bsdf.cpp carrying numbers from an ad-hoc measurement nothing reproduces. Re-measuring them here showed both misattributed: the recorded "7.1e-3 first bin" is really 4.4e-2, and the "3.2e-5 away from it" is the ROUGHNESS axis, not the mu axis, which is ~1e-5 there.
// Nothing else in the suite resolves it. checkWhiteFurnaceTwoSided bounds it to 2%, 600x too loose, and pays for that bound with Monte Carlo noise that no node count removes; checkCoatFresnelAvg deliberately reads the table's values through a reference that does not interpolate, to keep the two errors separate, and in any case the whole table is only ~2e-5 of its 1.2e-4 residual.
// Four measurements, each reported separately, because they are four different levers and a single worst would hide which one moved: the axes are measured one at a time by holding the other on exact nodes, and the first mu cell is split out from the rest of its axis because a grazing boundary layer and smooth curvature are not the same failure.
// Every bound below is the measured worst plus ~1.7x, the convention checkAverageFresnel already uses, so each axis trips on its own row rather than under a combined figure. Where the worsts now sit is itself the result: all three directional ones are on the lowest-alpha rows (roughness <= 0.022, at the kMinAlpha floor or just off it) at mu <= 4.4e-3, where the lobe is a near-mirror and the layer is narrower than a cell however the axis is warped. That region is sub-degree grazing on a mirror and every integral consuming E weights it by cos; what a shading path at ordinary angles sees is the control row's 3.0e-5, which is the stored values' own quadrature residual and not an interpolation error at all.
// Each is dense along the axis it measures and spread along the other, which is what the error's own shape asks for: the roughness error is a smooth single-signed hump, so it needs every cell on that axis and only a spread of mu; the mu error varies sharply toward grazing and needs the converse. The first-cell sweep is dense in roughness instead, because its peak sits at one particular low roughness where the layer is narrower than the cell.
// Reference is this file's own referenceDirectionalAlbedo/referenceAverageAlbedo -- an independent double-precision transcription, measured to 1.5e-7 at the worst grazing cell -- so what is measured is the committed .inc read through bsdf.cpp's own axis arithmetic, and nothing is compared against itself. The control row below is what keeps that claim honest.
// Asserted on the worst of the two Schlick channels rather than on E = a+b, which is strictly stronger and is what every caller needs: coatAlbedo reads split.at(f0) = f0*a + b at whatever f0 the material authors, and two channel errors that cancel in the sum need not cancel there.
struct InterpolationError {
    double worst;
    double roughness;
    double mu;
};

void recordWorst(InterpolationError& error, double delta, double roughness, double mu) {
    if (delta > error.worst) {
        error = {delta, roughness, mu};
    }
}

// Worst over both Schlick channels of the shipped lookup against the reference, at one (mu, roughness).
double directionalAlbedoError(double mu, double roughness) {
    const glm::vec2 shipped = pathtracer::scene::directionalAlbedoSplit(static_cast<float>(mu),
                                                                     static_cast<float>(roughness));
    const glm::dvec2 exact = referenceDirectionalAlbedo(mu, alphaAt(roughness));
    return std::max(std::abs(static_cast<double>(shipped.x) - exact.x),
                     std::abs(static_cast<double>(shipped.y) - exact.y));
}

// Evenly spread node indices over [first, last], endpoints included: the "held on exact nodes" coordinate of each
// measurement, where that axis contributes no interpolation error of its own and the other one is isolated.
// The mu axis is swept from node 0, which the warp made a real node at mu = 0 exactly: E(0, alpha) = 1 is an analytic
// identity, the generator asserts it on every row, and the reference evaluates it directly now that neither side
// divides by mu. It is the sharpest column in the table rather than the one that has to be skipped.
std::vector<int> spreadNodes(int first, int last, int count) {
    std::vector<int> nodes(static_cast<std::size_t>(count));
    for (int k = 0; k < count; ++k) {
        nodes[static_cast<std::size_t>(k)] = first + ((k * (last - first)) / (count - 1));
    }
    return nodes;
}

// Row-parallel over the swept axis: rows share no accumulator and are combined in index order, so the reported
// worst is identical to the serial one and this check stays Exact rather than becoming schedule-dependent.
template <typename Row>
void parallelRows(int rows, int threads, Row row) {
    std::atomic<int> next{0};
    std::vector<std::thread> workers;
    for (int w = 0; w < std::min(threads, rows); ++w) {
        workers.emplace_back([&] {
            for (int i = next++; i < rows; i = next++) {
                row(i);
            }
        });
    }
    for (std::thread& worker : workers) {
        worker.join();
    }
}

PT_CHECK(albedo_table_interpolation, Slow, Exact) {
    // Each bound is the measured worst plus headroom, in the convention checkAverageFresnel already uses: thin
    // enough that a regeneration losing accuracy on any one axis trips that axis' own row rather than passing
    // under a combined figure. They are bounds on the COMMITTED table, so they move when it is rebaked.
    constexpr double kControlTolerance = 5e-5;
    constexpr double kRoughnessAxisTolerance = 1e-3;
    constexpr double kMuAxisTolerance = 3.7e-3;
    constexpr double kFirstMuCellTolerance = 3.1e-3;
    constexpr double kAverageAlbedoTolerance = 8e-6;
    // Fractions across the first mu cell. Its error is not a midpoint maximum like a smooth cell's: the layer sits
    // against the mu = 0 edge, so where inside the cell the worst falls depends on how the layer's width compares
    // to the cell's, and the sweep says so rather than assuming.
    const std::array<double, 4> firstCellFractions = {0.2, 0.4, 0.6, 0.8};
    constexpr int kSpread = 16;

    const glm::ivec2 res = pathtracer::scene::albedoGridRes();
    const std::vector<int> muNodes = spreadNodes(0, res.y - 1, kSpread);
    const std::vector<int> roughnessNodes = spreadNodes(0, res.x - 1, kSpread);

    // Control, and it is listed first because every row below is only as trustworthy as this one: both axes on
    // exact nodes, so the lookup returns a stored value verbatim and no interpolation happens at all. What is left
    // is the disagreement between this file's Simpson reference and the generator's Gauss-Legendre, whose own
    // residual verifyReflect prints at 3.0e-5 against a doubled rule. If this row is not small, the instrument is
    // the limit and the four measurements are measuring it rather than the table.
    std::vector<InterpolationError> controlRows(static_cast<std::size_t>(res.x));
    parallelRows(res.x, ctx.threads(), [&](int ri) {
        const double roughness = pathtracer::scene::albedoGridRoughness(static_cast<float>(ri));
        InterpolationError row{0.0, roughness, 0.0};
        for (int mi : muNodes) {
            const double mu = pathtracer::scene::albedoGridMu(static_cast<float>(mi));
            recordWorst(row, directionalAlbedoError(mu, roughness), roughness, mu);
        }
        controlRows[static_cast<std::size_t>(ri)] = row;
    });

    // Roughness axis: every cell midpoint on that axis, held on exact mu nodes.
    std::vector<InterpolationError> roughnessRows(static_cast<std::size_t>(res.x - 1));
    parallelRows(res.x - 1, ctx.threads(), [&](int ri) {
        const double roughness = pathtracer::scene::albedoGridRoughness(static_cast<float>(ri) + 0.5F);
        InterpolationError row{0.0, roughness, 0.0};
        for (int mi : muNodes) {
            const double mu = pathtracer::scene::albedoGridMu(static_cast<float>(mi));
            recordWorst(row, directionalAlbedoError(mu, roughness), roughness, mu);
        }
        roughnessRows[static_cast<std::size_t>(ri)] = row;
    });

    // mu axis: every cell midpoint on that axis EXCEPT the first, held on exact roughness nodes. The first cell is
    // measured on its own below -- folding it in here would let one grazing boundary layer set the whole axis' bound.
    std::vector<InterpolationError> muRows(roughnessNodes.size());
    parallelRows(static_cast<int>(roughnessNodes.size()), ctx.threads(), [&](int k) {
        const int ri = roughnessNodes[static_cast<std::size_t>(k)];
        const double roughness = pathtracer::scene::albedoGridRoughness(static_cast<float>(ri));
        InterpolationError row{0.0, roughness, 0.0};
        for (int mi = 1; mi + 1 < res.y; ++mi) {
            const double mu = pathtracer::scene::albedoGridMu(static_cast<float>(mi) + 0.5F);
            recordWorst(row, directionalAlbedoError(mu, roughness), roughness, mu);
        }
        muRows[static_cast<std::size_t>(k)] = row;
    });

    // First mu cell: dense in roughness, on exact nodes there, so what is left is the cell alone.
    std::vector<InterpolationError> firstCellRows(static_cast<std::size_t>(res.x));
    parallelRows(res.x, ctx.threads(), [&](int ri) {
        const double roughness = pathtracer::scene::albedoGridRoughness(static_cast<float>(ri));
        InterpolationError row{0.0, roughness, 0.0};
        for (double fraction : firstCellFractions) {
            const double mu = pathtracer::scene::albedoGridMu(static_cast<float>(fraction));
            recordWorst(row, directionalAlbedoError(mu, roughness), roughness, mu);
        }
        firstCellRows[static_cast<std::size_t>(ri)] = row;
    });

    // Eavg's own 1-D lerp, which reaches coatAlbedoAvg and through it the 1/(1-coatAlbedoAvg) denominator of the
    // whole diffuse coupling -- a different route into the shade than the directional lookups, so its own number.
    std::vector<InterpolationError> averageRows(static_cast<std::size_t>(res.x - 1));
    parallelRows(res.x - 1, ctx.threads(), [&](int ri) {
        const double roughness = pathtracer::scene::albedoGridRoughness(static_cast<float>(ri) + 0.5F);
        const glm::vec2 shipped = pathtracer::scene::averageAlbedoSplit(static_cast<float>(roughness));
        const glm::dvec2 exact = referenceAverageAlbedo(alphaAt(roughness));
        const double delta = std::max(std::abs(static_cast<double>(shipped.x) - exact.x),
                                       std::abs(static_cast<double>(shipped.y) - exact.y));
        averageRows[static_cast<std::size_t>(ri)] = {delta, roughness, -1.0};
    });

    const auto reduce = [](const std::vector<InterpolationError>& rows) {
        InterpolationError worst{0.0, 0.0, 0.0};
        for (const InterpolationError& row : rows) {
            recordWorst(worst, row.worst, row.roughness, row.mu);
        }
        return worst;
    };

    const std::array<std::tuple<const char*, InterpolationError, double>, 5> measurements = {{
        {"control: both axes on nodes    ", reduce(controlRows), kControlTolerance},
        {"roughness axis, mu on nodes    ", reduce(roughnessRows), kRoughnessAxisTolerance},
        {"mu axis cells 1..n, r on nodes ", reduce(muRows), kMuAxisTolerance},
        {"first mu cell, r on nodes      ", reduce(firstCellRows), kFirstMuCellTolerance},
        {"Eavg lerp, 1-D in roughness    ", reduce(averageRows), kAverageAlbedoTolerance},
    }};

    bool ok = true;
    std::cout << "bsdf_validate: albedo table interpolation error vs independent quadrature, worst Schlick channel\n";
    std::cout << "  measurement                                 worst        at roughness   mu\n";
    for (const auto& [name, worst, tolerance] : measurements) {
        std::cout << "  " << name << "   " << worst.worst << "   " << worst.roughness << "   "
                  << worst.mu << '\n';
        if (!(worst.worst <= tolerance)) {
            std::cerr << "bsdf_validate: FAILED albedo table interpolation on the " << name
                       << " -- worst " << worst.worst << " at roughness " << worst.roughness << " mu "
                       << worst.mu << " exceeds " << tolerance
                       << ". The committed src/scene/albedo_table.inc lost accuracy on this axis.\n";
            ok = false;
        }
    }
    finish(ctx, ok, "albedo_table_interpolation failed; see the rows above");
    return;
}

// bsdf.cpp's coatAlbedo in double, with fresnelAvg left free: the whole point is to recover the value the call sites pass.
double referenceCoatAlbedo(const glm::dvec2& split, double albedoAvg, double f0, double fresnelRatio,
                            double fresnelAvg) {
    const double tint = (fresnelAvg * fresnelAvg * albedoAvg) /
                         std::max(1.0 - (fresnelAvg * (1.0 - albedoAvg)), 1e-4);
    return (((f0 * split.x) + split.y) * fresnelRatio) + (tint * (1.0 - (split.x + split.y)));
}

// The albedo-table reference is this check's whole cost -- referenceDirectionalAlbedo is a 96x96 Simpson and referenceAverageAlbedo a 64-panel Simpson over it -- and it depends only on roughness and mu, never on ior. Evaluated once per (roughness, mu) rather than once per row, which is the same arithmetic on the same inputs and so leaves every number below unchanged.
struct CoatAlbedos {
    glm::dvec2 splitAvg;
    std::array<glm::dvec2, 3> split;   // parallel to the check's cosines
};

CoatAlbedos coatAlbedos(double roughness, const std::array<double, 3>& cosines) {
    const double alpha = alphaAt(roughness);
    return {referenceAverageAlbedo(alpha),
             {referenceDirectionalAlbedo(cosines[0], alpha),
              referenceDirectionalAlbedo(cosines[1], alpha),
              referenceDirectionalAlbedo(cosines[2], alpha)}};
}

// Everything the coupling model needs that does not depend on the free fresnelAvg, so the bisection below re-runs no quadrature.
struct CoatGeometry {
    glm::dvec2 splitWo;
    glm::dvec2 splitWi;
    glm::dvec2 splitAvg;
    double f0;
    double karisAvg;   // schlickFresnelAvg(coatF0), the Schlick basis the table is built on and the rescale divides by
    double ratioWo;
    double ratioWi;
};

CoatGeometry coatGeometry(double ior, const CoatAlbedos& albedos, int indexO, int indexI, double muO,
                           double muI) {
    const double r = (ior - 1.0) / (ior + 1.0);
    const double f0 = r * r;
    const auto schlick = [&](double mu) {
        return f0 + ((1.0 - f0) * std::pow(std::clamp(1.0 - mu, 0.0, 1.0), 5.0));
    };
    return {albedos.split[indexO],
             albedos.split[indexI],
             albedos.splitAvg,
             f0,
             f0 + ((1.0 - f0) / 21.0),
             referenceDielectricFresnel(muO, ior) / std::max(schlick(muO), 1e-6),
             referenceDielectricFresnel(muI, ior) / std::max(schlick(muI), 1e-6)};
}

// The diffuse channel's full coupling, (1 - coat(wo))/(1 - coatAvg) * (1 - coat(wi)), as a function of the fresnelAvg the three call sites pass. fresnelAvg reaches it twice: as the multiple-scattering attenuation, and as the numerator of coatAlbedoAvg's own Schlick rescale, which is the same dielectricAvg local in bsdf.cpp -- so substituting one candidate for another moves both, exactly as a revert would.
double referenceCoupling(const CoatGeometry& geometry, double fresnelAvg) {
    const double albedoAvg = geometry.splitAvg.x + geometry.splitAvg.y;
    const double coatAvg = referenceCoatAlbedo(geometry.splitAvg, albedoAvg, geometry.f0,
                                                fresnelAvg / std::max(geometry.karisAvg, 1e-6),
                                                fresnelAvg);
    const double wo = 1.0 - referenceCoatAlbedo(geometry.splitWo, albedoAvg, geometry.f0,
                                                 geometry.ratioWo, fresnelAvg);
    const double wi = 1.0 - referenceCoatAlbedo(geometry.splitWi, albedoAvg, geometry.f0,
                                                 geometry.ratioWi, fresnelAvg);
    return (wo / std::max(1.0 - coatAvg, 1e-4)) * wi;
}

// The instrument for coatAlbedo's fresnelAvg VALUE at working ior, which checkIndexMatchedCoat says outright it cannot supply: it pins the argument's COLLAPSE at index match, where every table coefficient is multiplied by an exact zero, and notes that any g(ior) with g(1)=0 passes it -- notably 2*dielectricFresnelAvg(ior). This is the other half.
// Method: the diffuse channel is exactly evaluateEon times the coat coupling, so dividing it by this file's own referenceEon -- the one checkIndexMatchedCoat already pins to 1e-6 relative -- leaves the coupling alone, with no accessor into the albedo table needed. referenceCoupling models that coupling with fresnelAvg free, and the sweep inverts it, so what is reported is the F_avg the shipped code actually used rather than a pass/fail on a difference.
// Deliberately NOT measured as diffuse(ior)/diffuse(ior=1), which is the tempting form since checkIndexMatchedCoat proves the denominator is exactly evaluateEon: that identity holds only for a fresnelAvg that collapses at index match, so the very revert this must catch (Karis' mean returns 1/21 at ior=1) would corrupt the denominator too and the recovered number would stop meaning what it says. Measured that way the Karis revert reads 0.0435 rather than its actual 0.0857 -- still a failure, but a failure reported as the wrong cause.
// Truth is 2*int F(mu)*mu dmu by the same cosineAverageFresnel checkAverageFresnel uses, ~1e-13 accurate, so the tolerance is not set by the reference. Nor is it set by dielectricFresnelAvg, whose own error at these iors is 3.3e-6 to 2.2e-5 since it became a quadrature rule over the same fresnelDielectric the coat reflects by. What it spends is the albedo table and this inversion: measured worst 3.5e-5 at ior 1.33, so 6e-5 is ~1.7x headroom on the worst row.
// That worst was 1.2e-4 before two separate corrections, and the split between them is worth keeping because it is not what the earlier accounting assumed. Roughly half was this file's OWN reference: referenceDirectionalAlbedo's psi rule could not resolve the horizon layer and was wrong by 5.2e-3 at mu = 1/127, which leaked into the coupling model at the mu = 0.4 rows every worst row sits on -- 1.2e-4 to 5.1e-5 on that fix alone, with the table untouched. The rest was the table, which then went 5.1e-5 to 3.5e-5 on the sqrt(mu) mu axis and the 256-row roughness axis. So the residual the old comment could not account for was an instrument error, not a model one.
// That inverts what this check is FOR. It was the instrument for F_avg; it is now an instrument for src/scene/albedo_table.inc, and a regeneration of that table that loses accuracy trips here first -- though checkAlbedoTableInterpolation now reads the table directly and per axis, so this one is no longer the ONLY thing that would notice, and it sees the table only where its own sweep looks. F_avg enters coatAlbedo only through multiScatterTint(F, Eavg)*(1-E(mu)), whose derivative in F is ~0.054, so an error e in E recovers as e/0.054 in F_avg -- at the old 32x32 startup table's ~1.5e-3 that is 0.028, 140x this tolerance, which is why this check could not exist before the bake moved offline.
// Worth recording that the same derivation predicted ~1.2e-3 for the offline bake's 3e-5 quadrature plus its bilinear error, and the realised residual was 1.2e-4, 10x better. The e/0.054 route is the multiple-scattering path alone; the dC/dF column shows the recovery's actual conditioning is ~0.81-0.91, because coatAlbedoAvg's fresnelAvg/karisAvg rescale sits in the 1/(1-coatAlbedoAvg) denominator and carries most of the signal. The prediction was pessimistic by the ratio of those two routes, not wrong in kind.
// The ior range is now the full one, 1.1 to 3.0. It was 1.5 to 1.8 because the rational fit was not uniformly better than the Karis mean a revert would install -- it crossed over near ior 1.42 -- so below that the separation was a coin toss, and above 2.0 the fit failed its own bound. Neither constraint survives a rule that is within 2.3e-5 at every swept ior, and 4.5e-5 across the whole continuous range.
// All three candidates re-measured by mutation, which is the only way this claim means anything, and each fails at every ior in the sweep. Reverting to schlickFresnelAvg(coatF0) recovers 0.0856466 against that function's own 0.0857143 -- the instrument still names it, to 7e-5 -- and fails by 31x at ior 1.5, 123x at 1.1, and 3.5x at its weakest (ior 2.5, where Karis happens to cross truth). Substituting 2*dielectricFresnelAvg fails by 126x to 1381x. Reverting to the rational fit this replaced, which this check used to PASS at 0.0035, fails by 12x at ior 1.5 and 30x at 1.1. Unmutated, the recovery lands 7.0e-5 from dielectricFresnelAvg's own value at ior 1.5.
PT_CHECK(coat_fresnel_average, Slow, Exact) {
    constexpr double kTolerance = 6e-5;
    // Residual of the recovered root, not an accuracy claim: it catches a coupling the model cannot reproduce at ANY fresnelAvg (a lost 1/(1-coatAvg), a dropped wi-side factor), which an in-range root would otherwise launder into a plausible number.
    constexpr double kResidualTolerance = 1e-6;
    const std::array<double, 9> iors = {1.1, 1.33, 1.5, 1.5168, 1.55, 1.8, 2.0, 2.5, 3.0};
    // Roughness 0 is excluded: 1-E is ~0 there, so F_avg reaches nothing and is not observable at all. Nothing here is aligned to the table's grid, so the shipped side interpolates and this reference does not -- that difference is inside the tolerance and is measured by the worst-error column rather than designed away.
    const std::array<double, 4> roughnesses = {0.25, 0.5, 0.75, 1.0};
    const std::array<double, 3> cosines = {0.4, 0.7, 1.0};
    const glm::vec3 albedo(0.8F, 0.3F, 0.1F);
    constexpr float kDiffuseRoughness = 0.5F;

    // Hoisted out of the ior loop as well as the row loop: the albedo reference does not depend on ior, so the whole sweep costs four referenceAverageAlbedo and twelve referenceDirectionalAlbedo evaluations however many iors are swept.
    std::array<CoatAlbedos, roughnesses.size()> albedosByRoughness{};
    for (size_t r = 0; r < roughnesses.size(); ++r) {
        albedosByRoughness[r] = coatAlbedos(roughnesses[r], cosines);
    }

    bool ok = true;
    int rowsChecked = 0;
    std::cout << "bsdf_validate: coat F_avg recovered from the diffuse coupling vs exact quadrature\n";
    std::cout << "  ior      truth      worst recovered   |err|      dC/dF    at roughness/mu_o/mu_i\n";
    for (double ior : iors) {
        const double truth =
            cosineAverageFresnel([&](double mu) { return referenceDielectricFresnel(mu, ior); });
        double worstError = 0.0;
        double worstRecovered = 0.0;
        double worstRoughness = 0.0;
        double worstMuO = 0.0;
        double worstMuI = 0.0;
        double worstSlope = 0.0;
        for (size_t r = 0; r < roughnesses.size(); ++r) {
            const double roughness = roughnesses[r];
            for (size_t o = 0; o < cosines.size(); ++o) {
                const double muO = cosines[o];
                for (size_t i = 0; i < cosines.size(); ++i) {
                    const double muI = cosines[i];
                    const float sinO = std::sqrt(std::max(0.0F, 1.0F - static_cast<float>(muO * muO)));
                    const float sinI = std::sqrt(std::max(0.0F, 1.0F - static_cast<float>(muI * muI)));
                    const glm::vec3 wo(sinO, 0.0F, static_cast<float>(muO));
                    const glm::vec3 wi(sinI * std::cos(1.1F), sinI * std::sin(1.1F),
                                        static_cast<float>(muI));
                    const BsdfParams coated = makeDiffuseParams(albedo, kDiffuseRoughness,
                                                                 static_cast<float>(roughness),
                                                                 static_cast<float>(ior));
                    const glm::vec3 diffuse = pathtracer::scene::evaluateBsdfSplit(coated, wo, wi).diffuse;
                    const glm::vec3 bare = referenceEon(
                        pathtracer::scene::eonAlbedoInversion(albedo, kDiffuseRoughness),
                        kDiffuseRoughness, wi, wo);
                    const double measured =
                        static_cast<double>(maxChannel(diffuse)) / maxChannel(bare);

                    // The bracket is the model's own monotone branch, not a fixed interval. referenceCoupling is unimodal in fresnelAvg rather than monotone: raising fresnelAvg raises coatAlbedoAvg fastest through the fresnelAvg/schlickFresnelAvg(coatF0) rescale, and once that drives coatAlbedoAvg past its own turning point the 1/(1-coatAlbedoAvg) denominator sends the coupling back down. The turning point lies INSIDE the fixed [0, 0.6] bracket this used to bisect, at every ior swept -- measured 0.104 at ior 1.1 rising to 0.419 at 3.0, always at roughness 1 and normal incidence -- so a measurement whose root sat on the rising branch could still be answered with the bracket's ceiling, which is what widening the sweep to ior 1.1 exposed.
                    // Located by ternary search, which is what unimodality buys and is exact to (2/3)^60 of the interval. Bisection then runs on [0, argmax], where the inversion is well posed, and a measurement with no root there is reported by the residual below. All three mutations named above stay inside the branch at every row and so fail on value, not on form: the residual fires on none of them.
                    // Monotone increasing, which is not the obvious direction: raising F_avg raises coatAlbedo everywhere, but it raises coatAlbedoAvg fastest through the F_avg/schlickFresnelAvg(coatF0) rescale, and that sits in the 1/(1-coatAlbedoAvg) denominator. The dC/dF column below reports that slope per ior rather than remembering it: it is what converts a coupling error into the recovered F_avg error, so a row is only as trustworthy as its own conditioning, and the low-ior rows are the ones where the multiple-scattering route's own 2*F*Eavg contribution has faded.
                    const CoatGeometry geometry = coatGeometry(ior, albedosByRoughness[r],
                                                                static_cast<int>(o),
                                                                static_cast<int>(i), muO, muI);
                    constexpr double kBracketCeiling = 0.6;   // wide enough to contain every candidate a revert could install, before the monotone branch is cut out of it
                    double peakLow = 0.0;
                    double peakHigh = kBracketCeiling;
                    for (int step = 0; step < 60; ++step) {
                        const double third = (peakHigh - peakLow) / 3.0;
                        const double lowerThird = peakLow + third;
                        const double upperThird = peakHigh - third;
                        if (referenceCoupling(geometry, lowerThird) <
                            referenceCoupling(geometry, upperThird)) {
                            peakLow = lowerThird;   // the peak is right of lowerThird
                        } else {
                            peakHigh = upperThird;
                        }
                    }
                    const double peak = 0.5 * (peakLow + peakHigh);
                    double low = 0.0;
                    double high = peak;
                    for (int step = 0; step < 60; ++step) {
                        const double middle = 0.5 * (low + high);
                        (referenceCoupling(geometry, middle) < measured ? low : high) = middle;
                    }
                    const double recovered = 0.5 * (low + high);
                    const double residual = std::abs(referenceCoupling(geometry, recovered) - measured);
                    const double error = std::abs(recovered - truth);
                    // Central difference, wide enough to stay clear of the bisection's own 0.6*2^-60 resolution and narrow enough that the coupling is linear across it.
                    constexpr double kSlopeStep = 1e-4;
                    const double slope = (referenceCoupling(geometry, recovered + kSlopeStep) -
                                           referenceCoupling(geometry, recovered - kSlopeStep)) /
                                          (2.0 * kSlopeStep);
                    ++rowsChecked;
                    if (error > worstError) {
                        worstError = error;
                        worstRecovered = recovered;
                        worstRoughness = roughness;
                        worstMuO = muO;
                        worstMuI = muI;
                        worstSlope = slope;
                    }
                    if (!(residual <= kResidualTolerance)) {
                        std::cerr << "bsdf_validate: FAILED coat F_avg inversion at ior=" << ior
                                   << " roughness=" << roughness << " mu_o=" << muO << " mu_i=" << muI
                                   << " -- measured coupling " << measured
                                   << " is not reproduced at any fresnelAvg on the model's monotone branch [0, "
                                   << peak << "] (closest " << recovered << ", residual " << residual
                                   << "). The coat's form, not its F_avg, has changed.\n";
                        ok = false;
                    }
                    if (!(error <= kTolerance)) {
                        std::cerr << "bsdf_validate: FAILED coat F_avg at ior=" << ior
                                   << " roughness=" << roughness << " mu_o=" << muO << " mu_i=" << muI
                                   << " -- coatAlbedo used fresnelAvg=" << recovered
                                   << ", exact cosine-weighted Fresnel is " << truth << " (error "
                                   << error << ", tolerance " << kTolerance << ")\n";
                        ok = false;
                    }
                }
            }
        }
        std::cout << "    " << ior << "   " << truth << "   " << worstRecovered << "   " << worstError
                   << "   " << worstSlope << "   " << worstRoughness << " / " << worstMuO << " / "
                   << worstMuI << '\n';
    }
    // Anti-vacuity, checkIndexMatchedCoat's device: a sweep that asserted nothing would print clean too.
    if (rowsChecked == 0) {
        std::cerr << "bsdf_validate: FAILED coat F_avg -- no rows asserted\n";
        ok = false;
    }
    std::cout << "  " << rowsChecked << " rows inverted\n";
    finish(ctx, ok, "coat_fresnel_average failed; see the rows above");
    return;
}

// Cauchy dispersion (bsdf.cpp's cauchyIor), asserted against the contract it exists to satisfy rather than
// against a restatement of its own formula: (ior, abbe) means "index n_d at the d line, and an Abbe number
// V_d = (n_d-1)/(n_F-n_C)", so those two identities ARE the specification, and a test that recomputed
// A + B/lambda^2 here would only prove the expression was typed twice.
// Nothing else in the suite can see an error here. Every render-based transmissive check runs at ior 1.0,
// where B is exactly 0 and dispersion is identically the no-op -- deliberately, since that is what lets
// checkBeerLambert isolate the channel estimator from the refraction geometry. This check carries the
// whole proof of the optics.
PT_CHECK(cauchy_dispersion, Fast, Exact) {
    // Both bands are float32 rounding headroom, not fit error: the algebra is exact. The Abbe band is
    // relative because n_F - n_C is a ~0.008 difference of two ~1.5 quantities, so it carries the ~187x
    // cancellation amplification of their own ulp; the d-line band is absolute since nothing cancels there.
    constexpr float kDLineTolerance = 1e-6F;
    constexpr float kAbbeRelativeTolerance = 1e-4F;

    struct Glass {
        const char* name;
        float iorD;
        float abbe;
    };
    // Real catalogue materials spanning the physical range: crown, dense flint, and the two the OpenPBR
    // spec names as the common mid-dispersion cases. Nothing here is fitted; they are published constants.
    const std::array<Glass, 4> glasses{{
        {"Schott N-BK7 (crown)", 1.5168F, 64.17F},
        {"Schott SF10 (dense flint)", 1.72825F, 28.53F},
        {"water, 20C", 1.333F, 55.4F},
        {"diamond", 2.417F, 55.3F},
    }};
    constexpr float kLambdaDNm = 587.56F;
    constexpr float kLambdaFNm = 486.13F;
    constexpr float kLambdaCNm = 656.27F;

    bool ok = true;
    std::cout << "bsdf_validate: Cauchy dispersion inverted from (ior, abbe)\n";
    for (const Glass& glass : glasses) {
        const float nD = pathtracer::scene::cauchyIor(glass.iorD, glass.abbe, kLambdaDNm);
        const float nF = pathtracer::scene::cauchyIor(glass.iorD, glass.abbe, kLambdaFNm);
        const float nC = pathtracer::scene::cauchyIor(glass.iorD, glass.abbe, kLambdaCNm);
        const float measuredAbbeDifference = nF - nC;
        // The Abbe number's definition, rearranged. Computed here from the authored inputs alone.
        const float expectedAbbeDifference = (glass.iorD - 1.0F) / glass.abbe;

        const float nRed = pathtracer::scene::cauchyIor(glass.iorD, glass.abbe,
                                                     pathtracer::scene::kRgbWavelengthsNm.x);
        const float nGreen = pathtracer::scene::cauchyIor(glass.iorD, glass.abbe,
                                                       pathtracer::scene::kRgbWavelengthsNm.y);
        const float nBlue = pathtracer::scene::cauchyIor(glass.iorD, glass.abbe,
                                                      pathtracer::scene::kRgbWavelengthsNm.z);

        std::cout << "  " << glass.name;
        for (std::size_t pad = std::string(glass.name).size(); pad < 28; ++pad) {
            std::cout << ' ';
        }
        std::cout << "n_d " << nD << "   n_F-n_C " << measuredAbbeDifference << " (expected "
                  << expectedAbbeDifference << ")   RGB [" << nRed << ", " << nGreen << ", " << nBlue
                  << "]   dn(B-R) " << (nBlue - nRed) << '\n';

        if (!(std::fabs(nD - glass.iorD) <= kDLineTolerance)) {
            std::cerr << "bsdf_validate: FAILED d-line index for " << glass.name << " -- n(lambda_d) "
                      << nD << ", authored ior " << glass.iorD
                      << ". The authored ior IS the index at the d line; a dispersion curve that does not"
                         " pass through it has changed the material, not just spread it.\n";
            ok = false;
        }
        if (!(std::fabs(measuredAbbeDifference - expectedAbbeDifference) <=
              kAbbeRelativeTolerance * expectedAbbeDifference)) {
            std::cerr << "bsdf_validate: FAILED Abbe difference for " << glass.name << " -- n_F - n_C "
                      << measuredAbbeDifference << ", expected " << expectedAbbeDifference
                      << " = (ior-1)/abbe. That equation is the definition of the Abbe number, so the"
                         " authored abbe does not mean what it says.\n";
            ok = false;
        }
        // Normal dispersion: index falls with wavelength, so blue bends most. Pins the sign of B and the
        // ordering of kRgbWavelengthsNm together, which is exactly the pair the refraction direction needs.
        if (!(nBlue > nGreen && nGreen > nRed)) {
            std::cerr << "bsdf_validate: FAILED normal dispersion ordering for " << glass.name
                      << " -- RGB indices [" << nRed << ", " << nGreen << ", " << nBlue
                      << "] are not increasing toward blue. A transparent dielectric has no anomalous"
                         " dispersion in the visible band.\n";
            ok = false;
        }
    }

    // abbe = 0 is the off switch every non-dispersive material in the repo relies on, and it must be
    // exact rather than merely close: any drift here changes every existing render.
    std::cout << "  abbe 0 returns the authored ior unchanged at every wavelength\n";
    for (const Glass& glass : glasses) {
        for (float lambda : {kLambdaFNm, pathtracer::scene::kRgbWavelengthsNm.z, kLambdaDNm,
                             pathtracer::scene::kRgbWavelengthsNm.x, kLambdaCNm}) {
            const float n = pathtracer::scene::cauchyIor(glass.iorD, 0.0F, lambda);
            if (n != glass.iorD) {
                std::cerr << "bsdf_validate: FAILED abbe=0 no-op at ior " << glass.iorD << " lambda "
                          << lambda << " -- returned " << n
                          << ", expected the authored ior bit-for-bit.\n";
                ok = false;
            }
        }
    }
    finish(ctx, ok, "cauchy_dispersion failed; see the rows above");
    return;
}

// Conductor Fresnel (bsdf.cpp's conductorIorFromReflectivity/fresnelConductor), verified through the
// public API.
// At metallic=1 the diffuse lobe carries diffuseKd=0, so evaluateBsdf is the specular lobe alone:
// f(g) = K*F_g(woDotNh) + M, where K = D*G2/(4*muO*muI) is pure geometry and M is the Kulla-Conty
// multiple-scattering term. M is attenuated by F_avg, which conductorFresnelAvg makes a function of the
// complex IOR and therefore of edgeTint, so M does NOT cancel out of a difference across edgeTint. It
// used to: schlickFresnelAvg took F_avg from f0 alone, and this test was built on that cancellation.
// Both halves are therefore split by roughness rather than swept over both:
//   0.05 -- E(mu) ~ 1, so M is under this test's own tolerances and f(g) is the Fresnel term alone. The
//     exact ratio identity (f(g)-f(1))/(f(0)-f(1)) == (F_g-F_1)/(F_0-F_1) holds, pinning the whole
//     reflectance curve, inversion included, against the independent reference above. F does not depend
//     on roughness, so testing the curve at one roughness is testing it everywhere.
//   0.6 -- M is a third of the value, and its edgeTint dependence is the point: the normal-incidence
//     value must RISE monotonically with edgeTint, because F_avg does and multiScatterTint is monotone in
//     it. That is the assertion Karis structurally cannot pass -- schlickFresnelAvg(f0) is constant in g,
//     so on the old code every reading here was byte-identical and the span exactly zero.
// wo/wi are a coplanar pair mirrored about the normal, unlike checkReciprocity's deliberately
// non-coplanar ones: that puts nh exactly on +z so woDotNh IS the swept cosine, which is what makes the
// comparison against a reference evaluated at that cosine exact rather than approximate.
PT_CHECK(conductor_fresnel, Fast, Exact) {
    // The ratio is a quotient of differences of float BSDF values, so it carries the cancellation of both.
    constexpr float kRatioTolerance = 2e-3F;
    // Normal incidence is an exact identity, not a fit: R(theta=0) == r for every g (paper sec. 2.3.1).
    constexpr float kNormalIncidenceTolerance = 1e-5F;
    // Roughness at or below which the multiple-scattering deficit (1-E) leaves M under the identity's own
    // tolerance, so the two assertions that need M to vanish can use it. Measured: the identity holds to
    // 1e-5 relative at 0.05 and is violated by 1.5e-2 at 0.6, so the split is decisive, not marginal.
    constexpr float kMsNegligibleRoughness = 0.1F;
    // The property Schlick structurally cannot have: at grazing a black edge tint must sit well below a
    // white one. Schlick's (1-c)^5 tail forces every metal to exactly 1 there, so on the old code this
    // separation is identically zero whatever edgeTint says.
    constexpr float kMinGrazingSeparation = 0.05F;
    // Below this relative span between edgeTint 0 and 1 the normalised ratio is a quotient of two
    // near-equal tiny numbers. 1e-3 sits an order of magnitude under the smallest span that carries
    // signal (0.0183, measured at r=0.95 cos=0.6) and three above float32's own precision.
    constexpr float kMinConditionedSpan = 1e-3F;
    // 1.0 is the white case makeParams gives at metallic=1 and the one the white furnace runs on. It also
    // drives n to ~5e-5, where the reflectance is the most numerically delicate: it is what caught the
    // cancellation in fresnelConductor's `a`, which the 0.95 row passes straight over.
    const std::array<float, 4> reflectivities = {0.1F, 0.5F, 0.95F, 1.0F};
    const std::array<float, 5> edgeTints = {0.0F, 0.25F, 0.5F, 0.75F, 1.0F};
    const std::array<float, 2> roughnesses = {0.05F, 0.6F};
    const std::array<float, 3> cosines = {0.6F, 0.35F, 0.15F};

    bool ok = true;
    int ratiosChecked = 0;
    int monotonicRowsChecked = 0;
    std::cout << "bsdf_validate: conductor Fresnel, edgeTint 0.00 -> 1.00 (Gulbrandsen 2014)\n";
    std::cout << "  r     cos    F(g=0)   F(g=1)   grazing separation\n";
    for (float reflectivity : reflectivities) {
        const auto params = [&](float roughness, glm::vec3 edgeTint) {
            return BsdfParams{glm::vec3(1.0F),          1.0F, roughness, glm::vec3(reflectivity),
                               edgeTint,                 /*ior=*/1.5F,
                               /*transmissionFactor=*/0.0F, /*diffuseRoughness=*/0.0F,
                               pathtracer::scene::eonAlbedoInversion(glm::vec3(1.0F), 0.0F),
                               /*transmissionTint=*/glm::vec3(1.0F)};
        };
        // R(theta=0) == r, independent of edgeTint: wo == wi == +z puts woDotNh at exactly 1.
        const glm::vec3 normalIncidence(0.0F, 0.0F, 1.0F);
        for (float roughness : roughnesses) {
            std::array<float, 5> atNormal{};
            for (std::size_t i = 0; i < edgeTints.size(); ++i) {
                atNormal[i] = maxChannel(pathtracer::scene::evaluateBsdf(
                    params(roughness, glm::vec3(edgeTints[i])), normalIncidence, normalIncidence));
            }
            const float atWhite = atNormal.back();
            if (roughness <= kMsNegligibleRoughness) {
                for (std::size_t i = 0; i < edgeTints.size(); ++i) {
                    if (!(std::abs(atNormal[i] - atWhite) <= kNormalIncidenceTolerance * atWhite)) {
                        std::cerr
                            << "bsdf_validate: FAILED conductor Fresnel normal-incidence identity at r="
                            << reflectivity << " roughness=" << roughness
                            << " edgeTint=" << edgeTints[i] << " f=" << atNormal[i] << " vs " << atWhite
                            << "; R(theta=0) must equal r for every edgeTint\n";
                        ok = false;
                    }
                }
                continue;
            }
            // Same conditioning guard the ratio below uses: at r=1 every edge tint gives the same mirror,
            // so the span across edgeTint is float noise rather than signal (measured 6e-6 relative there
            // against 1.5e-2 at r=0.1). monotonicRowsChecked keeps the skip honest.
            if ((atWhite - atNormal.front()) / atWhite < kMinConditionedSpan) {
                continue;
            }
            ++monotonicRowsChecked;
            // Strict, with no epsilon: the guard above conditions the whole g=0->1 span, but the compare is
            // per adjacent step, and the tightest real step is r=0.95 g=0->0.25 at 6.6e-5 relative in F_avg
            // -- hundreds of times float32's own precision even after M attenuates it. Strictness is what
            // makes a constant F_avg fail rather than round into a pass.
            for (std::size_t i = 1; i < edgeTints.size(); ++i) {
                if (!(atNormal[i] > atNormal[i - 1])) {
                    std::cerr << "bsdf_validate: FAILED conductor F_avg edgeTint dependence at r="
                              << reflectivity << " roughness=" << roughness << " edgeTint "
                              << edgeTints[i - 1] << " -> " << edgeTints[i] << " gave " << atNormal[i - 1]
                              << " -> " << atNormal[i]
                              << "; the multiple-scattering lobe must brighten monotonically with edge "
                                 "tint, which a Schlick F_avg cannot do at any f0\n";
                    ok = false;
                }
            }
        }

        for (float cosine : cosines) {
            const float sine = std::sqrt(std::max(0.0F, 1.0F - (cosine * cosine)));
            const glm::vec3 wo(sine, 0.0F, cosine);
            const glm::vec3 wi(-sine, 0.0F, cosine);
            const double referenceWhite = referenceConductorFresnel(reflectivity, 1.0, cosine);
            const double referenceBlack = referenceConductorFresnel(reflectivity, 0.0, cosine);
            for (float roughness : roughnesses) {
                const float atWhite =
                    maxChannel(pathtracer::scene::evaluateBsdf(params(roughness, glm::vec3(1.0F)), wo, wi));
                const float atBlack =
                    maxChannel(pathtracer::scene::evaluateBsdf(params(roughness, glm::vec3(0.0F)), wo, wi));
                const float span = atBlack - atWhite;
                // The assertion that edgeTint reaches the lobe at all: a dropped edgeTint leaves every
                // reading identical and span exactly zero. Holds at every r, including 1.
                if (!(std::abs(span) > 0.0F)) {
                    std::cerr << "bsdf_validate: FAILED conductor Fresnel -- edgeTint 0 and 1 gave the "
                                 "identical value "
                              << atWhite << " at r=" << reflectivity << " roughness=" << roughness
                              << " cos=" << cosine << ", so edgeTint reaches nothing\n";
                    ok = false;
                    continue;
                }
                // The exact ratio identity needs M to cancel across edgeTint, which it does only where the
                // multiple-scattering deficit is negligible; the rough row's edgeTint dependence is real
                // and is asserted at normal incidence above instead.
                if (roughness > kMsNegligibleRoughness) {
                    continue;
                }
                // The quotient below divides one span by another, so it only carries signal where the
                // span is above float noise. At r=1 every edge tint agrees to ~3e-5 (a perfect mirror is
                // white at every angle however it is tinted) and the ratio is pure rounding; those rows
                // are still covered by the normal-incidence identity above, which is what caught the
                // cancellation in fresnelConductor's `a`. ratiosChecked keeps the skip honest.
                if (std::abs(span) / atWhite < kMinConditionedSpan) {
                    continue;
                }
                for (float edgeTint : edgeTints) {
                    ++ratiosChecked;
                    const float measured = maxChannel(
                        pathtracer::scene::evaluateBsdf(params(roughness, glm::vec3(edgeTint)), wo, wi));
                    const double expected =
                        (referenceConductorFresnel(reflectivity, edgeTint, cosine) - referenceWhite) /
                        (referenceBlack - referenceWhite);
                    const double ratio = static_cast<double>(measured - atWhite) / span;
                    if (!(std::abs(ratio - expected) <= kRatioTolerance)) {
                        std::cerr << "bsdf_validate: FAILED conductor Fresnel curve at r=" << reflectivity
                                  << " roughness=" << roughness << " cos=" << cosine
                                  << " edgeTint=" << edgeTint << " normalised " << ratio << " vs reference "
                                  << expected << '\n';
                        ok = false;
                    }
                }
            }
            // Reported as reflectance, which the reference gives directly; the assertion is on the
            // measured BSDF values, so it is the shipped code being tested and not the reference.
            const float atWhite =
                maxChannel(pathtracer::scene::evaluateBsdf(params(0.05F, glm::vec3(1.0F)), wo, wi));
            const float atBlack =
                maxChannel(pathtracer::scene::evaluateBsdf(params(0.05F, glm::vec3(0.0F)), wo, wi));
            const float separation = (atWhite - atBlack) / atWhite;
            std::cout << "  " << reflectivity << "   " << cosine << "   " << referenceBlack << "   "
                      << referenceWhite << "   " << separation << '\n';
            // Not asserted at r=1: a perfect mirror reflects everything at every angle whatever its edge
            // tint (measured 3.4e-4 separation, and the reference agrees), so there is no dip to require
            // there. The dip is a mid-reflectivity property, which is where the rows above assert it.
            if (cosine == cosines.back() && reflectivity < 1.0F &&
                !(separation >= kMinGrazingSeparation)) {
                std::cerr << "bsdf_validate: FAILED conductor Fresnel grazing dip at r=" << reflectivity
                          << " cos=" << cosine << " separation=" << separation << " (expected >= "
                          << kMinGrazingSeparation
                          << "); a black edge tint must fall well below a white one at grazing, which is "
                             "the behaviour Schlick cannot express at any f0\n";
                ok = false;
            }
        }
    }
    if (ratiosChecked == 0) {
        std::cerr << "bsdf_validate: FAILED conductor Fresnel -- every row was skipped as too "
                     "ill-conditioned for the normalised ratio, so the reflectance curve was never "
                     "compared against the reference\n";
        ok = false;
    }
    if (monotonicRowsChecked == 0) {
        std::cerr << "bsdf_validate: FAILED conductor Fresnel -- every rough row was skipped as too "
                     "ill-conditioned, so F_avg's edgeTint dependence was never asserted\n";
        ok = false;
    }
    std::cout << "  conductor Fresnel curve: " << ratiosChecked << " points vs reference, "
              << monotonicRowsChecked << " rough rows monotone in edgeTint\n";
    finish(ctx, ok, "conductor_fresnel failed; see the rows above");
    return;
}

// The specular lobe's closed form at the mirrored pair, in double: nh is exactly +z there, so sin(theta_h) is 0 and the GGX denominator collapses to alpha^2, giving D = 1/(pi*alpha^2) without evaluating the shipped D at all.
// G2 is Smith height-correlated with both lambdas at the same cosine, the quantity bsdf.cpp's smithVisibility evaluates in its division-free form.
double specularGeometry(double alpha, double cosine) {
    const double alpha2 = alpha * alpha;
    const double d = 1.0 / (kPiDouble * alpha2);
    const double tan2 = (1.0 - (cosine * cosine)) / (cosine * cosine);
    const double lambda = 0.5 * (-1.0 + std::sqrt(1.0 + (alpha2 * tan2)));
    const double g2 = 1.0 / (1.0 + (2.0 * lambda));
    return (d * g2) / (4.0 * cosine * cosine);
}

// The only instrument in this file that reads the specular lobe's ABSOLUTE magnitude, and the only one that can: sampleBsdf returns f/pdf and both carry D, so every furnace, reciprocity and round-trip check cancels a constant factor on it; checkConductorFresnel's normalised ratio cancels the geometry term by construction; checkPdfNormalization asserts an upper bound, which a suppressed pdf passes vacuously.
// That gap let distributionGGX ship two independent errors, both invisible in throughput and both wrong wherever the value is read absolutely -- which is NEE's f and the MIS pdf weights, i.e. every direct highlight on a smooth surface. Measured here: a denominator floor suppressing D by 124340x at roughness 0.02 and 81.5x at 0.05, and float32 cancellation in ndotH^2*(alpha^2-1)+1 costing a further 20% at 0.02 and 3.3e-4 at 0.1.
// f(cos) = K*F(cos) + M, with K = D*G2/(4*muO*muI) pure geometry and M the Kulla-Conty multiple-scattering term; dividing the measured lobe by the K above recovers the Fresnel reflectance, comparable directly against referenceDielectricFresnel. This couples the check to D and G2 as well as to Fresnel, and the coupling IS the coverage: absolute magnitude is the axis nothing else in the suite tests.
// metallic=0 gates the conductor path (and params.f0) off entirely, transmissionFactor=0 leaves the reflection lobe as the only thing present, and reading .specular off BsdfEval isolates it from the diffuse substrate, so no black-baseColor trick is needed.
// Roughness stays at or below checkConductorFresnel's kMsNegligibleRoughness so M sits under the tolerance -- it measures below float32 noise here, two orders under it; the two lowest rows are glass.json's and chrome.json's own values, which is what makes this the regression test for both D errors.
// The sweep only reaches nh = +z, so it pins D at the lobe peak and says nothing about the tails; that is the right trade, since the peak is what a direct highlight is made of and the tails carry no absolute reference to compare against.
// The grazing rows run to cos 1e-7, through the band where a 1e-6 floor on 4*muO*muI (engaging at cos 5e-4) used to darken the lobe as 4c^2/1e-6: it is the regression test for smithVisibility's clamp-free form, which must match the unfloored reference K all the way to the silhouette.
PT_CHECK(dielectric_fresnel, Fast, Exact) {
    // Float32 round-off in the shipped lobe against a double reference, relative to F since rounding is: M is not resolvable at these roughnesses. Measured worst 3.98e-7 (6.7 float32 unit roundoffs) at ior 2.5, roughness 0.05, cos 4e-4, plus ~17% headroom. A fit-shaped error cannot hide under a bound this tight -- Schlick misses by 0.02 at ior 1.5168 cos 0.5, five orders above it.
    constexpr double kFresnelTolerance = 4.7e-7;
    // Normal incidence is an exact identity, not a fit: referenceDielectricFresnel(1, n) is ((n-1)/(n+1))^2 with both polarisations equal, and nh, woDotNh and G2 are all exactly 1 there, so the only residual is float32 evaluation of F itself. Measured worst 2.54e-8 at ior 2.5, plus ~18% headroom.
    constexpr double kNormalIncidenceTolerance = 3e-8;
    constexpr float kMinAlpha = 0.02F * 0.02F;   // bsdf.cpp's roughness floor, mirrored so K uses the alpha the lobe actually used
    const std::array<double, 4> iors = {1.1, 1.5, 1.5168, 2.5};   // 1.5168 is glass.json's own N-BK7 value
    const std::array<float, 3> roughnesses = {0.02F, 0.05F, 0.1F};
    const std::array<float, 14> cosines = {1.0F,  0.9F,  0.7F,  0.5F,  0.35F, 0.25F, 0.15F,
                                           0.08F, 0.02F, 4e-4F, 2e-4F, 1e-4F, 1e-5F, 1e-7F};

    bool ok = true;
    int rowsChecked = 0;
    std::cout << "bsdf_validate: dielectric Fresnel, absolute lobe magnitude vs exact unpolarized\n";
    std::cout << "  ior      rough   worst |err|/tol   F(cos=1e-7) measured / exact\n";
    for (double ior : iors) {
        for (float roughness : roughnesses) {
            const double alpha =
                std::max(static_cast<double>(roughness) * roughness, static_cast<double>(kMinAlpha));
            const glm::vec3 baseColor(1.0F);
            const BsdfParams params{baseColor,
                                     /*metallic=*/0.0F,
                                     roughness,
                                     /*f0=*/glm::vec3(0.04F),
                                     /*edgeTint=*/glm::vec3(1.0F),
                                     static_cast<float>(ior),
                                     /*transmissionFactor=*/0.0F,
                                     /*diffuseRoughness=*/0.0F,
                                     pathtracer::scene::eonAlbedoInversion(baseColor, 0.0F),
                                     /*transmissionTint=*/glm::vec3(1.0F)};
            double worstError = 0.0;
            double previous = -1.0;
            double atGrazing = 0.0;
            for (float cosine : cosines) {
                const float sine = std::sqrt(std::max(0.0F, 1.0F - (cosine * cosine)));
                const glm::vec3 wo(sine, 0.0F, cosine);
                const glm::vec3 wi(-sine, 0.0F, cosine);
                const double measured =
                    static_cast<double>(pathtracer::scene::evaluateBsdfSplit(params, wo, wi).specular.x) /
                    specularGeometry(alpha, cosine);
                const double expected = referenceDielectricFresnel(cosine, ior);
                const double tolerance = cosine == 1.0F ? kNormalIncidenceTolerance : kFresnelTolerance * expected;
                ++rowsChecked;
                worstError = std::max(worstError, std::abs(measured - expected) / tolerance);
                atGrazing = measured;
                if (!(std::abs(measured - expected) <= tolerance)) {
                    std::cerr << "bsdf_validate: FAILED dielectric Fresnel at ior=" << ior
                              << " roughness=" << roughness << " cos=" << cosine << " measured " << measured
                              << " vs reference " << expected << " (tolerance " << tolerance
                              << "); the lobe's absolute magnitude is D*G2*F/(4*muO*muI), so this fires on an "
                                 "error in any of the three\n";
                    ok = false;
                }
                // Strict, no epsilon: unpolarized external reflection is monotone in theta for every n > 1, and the smallest real step here is cos 1.0 -> 0.9 at 1.9e-2 relative, five orders above float32's own precision. Strictness is what makes a Fresnel pinned to its normal-incidence value fail rather than round into a pass.
                if (previous >= 0.0 && !(measured > previous)) {
                    std::cerr << "bsdf_validate: FAILED dielectric Fresnel monotonicity at ior=" << ior
                              << " roughness=" << roughness << " cos=" << cosine << " gave " << measured
                              << " after " << previous
                              << "; reflectance must rise strictly as the view approaches grazing\n";
                    ok = false;
                }
                previous = measured;
            }
            std::cout << "  " << ior << "      " << roughness << "    " << worstError << "    " << atGrazing
                      << " / " << referenceDielectricFresnel(cosines.back(), ior) << '\n';
        }
    }
    // Index match, kept out of the sweep above rather than folded into it: at ior 1 the true curve is identically zero, so the strict monotonicity assertion -- stated for n > 1, where reflectance rises toward grazing -- is the wrong predicate and would fail on correct code.
    // Asserted at exactly zero, no tolerance and no division by specularGeometry: F is the only ior-dependent factor in the lobe, so F == +0 zeroes the product whatever D and G2 are, and the reference is algebraic rather than measured.
    // The cosines run three orders below the main sweep's 0.02 floor because that is where the old Snell transcription failed, in two separate ways, and this is the direct regression test for both -- checkIndexMatchedCoat sees the same faults only after they have propagated through the coat coupling.
    // Below 2.44e-4 (2^-12) it rounded 1.0F-mu*mu to exactly 1.0F, reported total internal reflection at an interface with no critical angle and returned 1.0F: measured 4.8e+11 in the lobe at roughness 0.02, cos 1e-5, since D/(4*mu*mu) multiplies it by ~5e11 there.
    // Above the sliver it still leaked, from a SECOND cancellation the cos^2 form also removes: cosThetaT came from 1 - sinThetaT*sinThetaT, and at mu 0.02 that subtracts 0.99960004 from 1 in float32, so cosThetaT missed mu by ~1e-6 and the polarisation terms, which must cancel to exactly zero at r == 1, left F ~ 6e-10 -- measured 0.337797 in the lobe at roughness 0.02, cos 0.02. cos2Transmitted returns mu*mu exactly at r == 1, so cosThetaT == mu bit-for-bit and both terms are exactly zero.
    // Measured on the pre-fix code: 6 of these 8 rows fail at every roughness, cos 1.0 and 0.5 being the only ones where the cancellation is harmless.
    const std::array<float, 8> indexMatchedCosines = {1.0F,     0.5F,          0.02F,  1e-3F,
                                                      2.44e-4F, 1.7263349e-4F, 1e-4F, 1e-5F};
    int indexMatchedRows = 0;
    for (float roughness : roughnesses) {
        const glm::vec3 baseColor(1.0F);
        const BsdfParams params{baseColor,
                                 /*metallic=*/0.0F,
                                 roughness,
                                 /*f0=*/glm::vec3(0.04F),
                                 /*edgeTint=*/glm::vec3(1.0F),
                                 /*ior=*/1.0F,
                                 /*transmissionFactor=*/0.0F,
                                 /*diffuseRoughness=*/0.0F,
                                 pathtracer::scene::eonAlbedoInversion(baseColor, 0.0F),
                                 /*transmissionTint=*/glm::vec3(1.0F)};
        for (float cosine : indexMatchedCosines) {
            const float sine = std::sqrt(std::max(0.0F, 1.0F - (cosine * cosine)));
            const glm::vec3 wo(sine, 0.0F, cosine);
            const glm::vec3 wi(-sine, 0.0F, cosine);
            const float measured = maxChannel(pathtracer::scene::evaluateBsdfSplit(params, wo, wi).specular);
            ++indexMatchedRows;
            if (!(measured == 0.0F)) {
                std::cerr << "bsdf_validate: FAILED dielectric Fresnel at index match, roughness="
                          << roughness << " cos=" << cosine << " gave " << measured
                          << "; an ior-1 interface has no critical angle and reflects nothing, so the "
                             "specular lobe must be exactly zero at every angle\n";
                ok = false;
            }
        }
    }
    // No anti-vacuity guard here, unlike checkIndexMatchedCoat and checkCoatFresnelAvg: both arrays are non-empty at compile time and the counter increments unconditionally, so a zero count is unreachable rather than merely unlikely. The count is still printed, which is what makes a future conditioning skip visible.
    // No conditioning skip anywhere above, unlike checkConductorFresnel's normalised ratio: every row of the sweep is compared, and the count is printed so that stays visible.
    std::cout << "  dielectric Fresnel: " << rowsChecked << " points vs reference, "
              << indexMatchedRows << " index-matched rows at exactly zero\n";
    finish(ctx, ok, "dielectric_fresnel failed; see the rows above");
    return;
}

// Helmholtz reciprocity: f(wo->wi) == f(wi->wo). The continuous lobes are symmetric by construction after the directional-albedo diffuse coupling landed: D and G2 are symmetric, Fresnel is evaluated at the shared half-vector, and both the coupling and the multiple-scattering lobe are products of matching wo-side and wi-side factors, so this is an equality to float precision, not a statistical bound.
// It fails hard on the pre-coupling code, where the diffuse lobe carried (1 - F(mu_o)) alone; not an energy error (the furnace passed throughout) but a misdistribution across view/light geometry, and the blocker for every bidirectional transport algorithm (BDPT, VCM, light tracing, photon mapping), all of which require symmetric f.
// Transmission is excluded (transmissionFactor=0, both cosines positive): radiance transport across a refracting interface is genuinely non-symmetric, so f(wo->wi)==f(wi->wo) is the wrong invariant there; the eta^2-corrected one it does satisfy lives in checkTransmissionReciprocity below.
PT_CHECK(reciprocity, Fast, Exact) {
    constexpr float kRelativeTolerance = 1e-4F;
    const std::array<float, 4> roughnesses = {0.05F, 0.25F, 0.5F, 1.0F};
    const std::array<float, 3> metallics = {0.0F, 0.5F, 1.0F};
    const std::array<float, 4> cosines = {1.0F, 0.7F, 0.4F, 0.15F};
    // At 0 the diffuse lobe is Lambertian and reciprocal for free, so the sweep is what actually puts EON under this check. Every term of evaluateEon is symmetric under the wo/wi swap by construction -- dot(wi,wo), max(muI,muO), and the (1-eFonO)*(1-eFonI) product -- which is a property the model guarantees and nothing here asserted.
    const std::array<float, 3> diffuseRoughnesses = {0.0F, 0.5F, 1.0F};

    bool ok = true;
    for (float roughness : roughnesses) {
        for (float metallic : metallics) {
            for (float diffuseRoughness : diffuseRoughnesses) {
                const BsdfParams params = makeParams(roughness, metallic, 0.0F, diffuseRoughness);
                for (float muA : cosines) {
                    for (float muB : cosines) {
                        // Non-coplanar pair: a shared azimuth would leave a swapped-phi bug invisible.
                        const float sinA = std::sqrt(std::max(0.0F, 1.0F - (muA * muA)));
                        const float sinB = std::sqrt(std::max(0.0F, 1.0F - (muB * muB)));
                        const glm::vec3 wo(sinA, 0.0F, muA);
                        const glm::vec3 wi(sinB * std::cos(1.1F), sinB * std::sin(1.1F), muB);
                        const glm::vec3 forward = pathtracer::scene::evaluateBsdf(params, wo, wi);
                        const glm::vec3 reverse = pathtracer::scene::evaluateBsdf(params, wi, wo);
                        const float scale = std::max(maxChannel(forward), maxChannel(reverse));
                        if (!(maxChannel(glm::abs(forward - reverse)) <=
                              kRelativeTolerance * std::max(scale, 1e-4F))) {
                            std::cerr << "bsdf_validate: FAILED reciprocity at roughness=" << roughness
                                      << " metallic=" << metallic
                                      << " diffuseRoughness=" << diffuseRoughness << " muO=" << muA
                                      << " muI=" << muB << " f(wo->wi)=" << forward.x
                                      << " f(wi->wo)=" << reverse.x << '\n';
                            ok = false;
                        }
                    }
                }
            }
        }
    }
    finish(ctx, ok, "reciprocity failed; see the rows above");
    return;
}

// eta^2-corrected reciprocity for the transmission lobe: f_t(wo->wi)*eta_wi^2 == f_t(wi->wo)*eta_wo^2. Every term in evaluateTransmissionLobe is symmetric under the swap except denom = (wo.h) + etaR*(wi.h), which the reversed frame rescales by etaI/etaT; squared, that is exactly the eta ratio above. With wo outside and wi inside it reads f(wo->wi)*ior^2 == f(wi->wo).
// Catches a misplaced etaR^2, a flipped denom orientation or an un-flipped ht -- O(1) errors (a stray eta^2 is 2.25x or 0.44x at ior 1.5) invisible to the furnace and round-trip tests, which assert only totals and in which the two sides' errors cancel.
// SINGLE SCATTER ONLY, permanently -- not a symptom of a fixable bug. transmitMultiScatter (bsdf.cpp) is (1-escapeWo) at wo's eta times the escape-deficit density at the reciprocal eta, each normalised within its own orientation, which makes its total exact (checkTransmissiveEnergyBalance sweeps this to roughness 1.0) but cannot make it reciprocal: the swap exchanges which orientation each factor is read at, which is inherent to transmissive multiple scattering, not an implementation gap. Known limitation (docs/roadmap.md transport #1): it blocks bidirectional transport through rough glass, not this unidirectional integrator. 0.40 fails hard (up to 5x forward/reverse mismatch) for exactly this reason -- do not chase that by widening the sweep.
// Isolated by magnitude, with no new accessor: the multiple-scattering term is live at every rough roughness, but at 0.05 and 0.10 its deficit is so far below the single-scatter peak these constructed pairs sit on that the worst mismatch is unchanged by it (7.7e-5 and 3.0e-5 with the term on and off), two orders inside the tolerance. Both roughnesses stay above kSmoothAlpha or there is no continuous lobe to test at all.
// wi is CONSTRUCTED, not sampled: at these roughnesses the lobe is a fraction of a degree wide, so an arbitrary far-side direction returns zero on both sides and the check passes having tested nothing. Refract wo through the macro normal, then perturb by a multiple of alpha for off-peak pairs; the non-zero-pair count is asserted for the same reason.
// transmissionFactor is pinned at 1.0. Between 0 and 1 the entering side scales the lobe by it and the exiting side by 1.0 -- a modelling asymmetry (inside the medium there is no substrate to withhold anything), not a Jacobian error, so sweeping it would test the convention rather than the invariant.
PT_CHECK(transmission_reciprocity, Fast, Exact) {
    // Not checkReciprocity's 1e-4: D is sharply peaked at these alphas (2.5e-3 to 1e-2) and the two queries build ht from differently scaled sums, so a few-ULP direction difference is amplified by dD/D ~ 4/alpha^2 off the peak. Worst measured 6.4e-3 at roughness 0.05, 2.1e-3 at 0.10; full discriminating power against the O(1) structural errors above survives at 1e-2.
    constexpr float kRelativeTolerance = 1e-2F;
    const std::array<float, 2> roughnesses = {0.05F, 0.10F};
    const std::array<float, 3> iors = {1.2F, 1.5F, 2.0F};
    const std::array<float, 4> cosines = {1.0F, 0.9F, 0.7F, 0.5F};
    const std::array<float, 5> offsets = {0.0F, 0.5F, 1.0F, 2.0F, 4.0F};  // multiples of alpha
    // Perturbation axes. Out-of-plane and diagonal put wo and wi at different azimuths, so a swapped-phi bug cannot hide the way it would on a coplanar pair.
    const std::array<glm::vec3, 3> axes = {
        {{0.0F, 1.0F, 0.0F}, {1.0F, 0.0F, 0.0F}, {0.70710678F, 0.70710678F, 0.0F}}};

    bool ok = true;
    int pairsSeen = 0;
    for (float roughness : roughnesses) {
        const float alpha = roughness * roughness;
        for (float ior : iors) {
            const BsdfParams params{glm::vec3(1.0F), 0.0F, roughness,
                                     glm::vec3(0.04F), glm::vec3(1.0F), ior,
                                     /*transmissionFactor=*/1.0F, /*diffuseRoughness=*/0.0F,
                                     pathtracer::scene::eonAlbedoInversion(glm::vec3(1.0F), 0.0F),
                                     /*transmissionTint=*/glm::vec3(1.0F)};
            for (float mu : cosines) {
                const glm::vec3 wo(std::sqrt(std::max(0.0F, 1.0F - (mu * mu))), 0.0F, mu);
                const glm::vec3 refracted =
                    glm::refract(-wo, glm::vec3(0.0F, 0.0F, 1.0F), 1.0F / ior);
                for (float offset : offsets) {
                    const float angle = offset * alpha;
                    for (std::size_t axisIndex = 0; axisIndex < axes.size(); ++axisIndex) {
                        // A zero offset lands on the refracted direction whatever the axis is, so only the first pass over it is a distinct pair.
                        if (offset == 0.0F && axisIndex > 0) {
                            continue;
                        }
                        const glm::vec3& axis = axes[axisIndex];
                        const glm::vec3 tangent =
                            glm::normalize(axis - (refracted * glm::dot(axis, refracted)));
                        const glm::vec3 wi = glm::normalize((std::cos(angle) * refracted) +
                                                             (std::sin(angle) * tangent));
                        const glm::vec3 forward =
                            pathtracer::scene::evaluateBsdf(params, wo, wi) * ior * ior;
                        const glm::vec3 reverse = pathtracer::scene::evaluateBsdf(params, wi, wo);
                        const float scale = std::max(maxChannel(forward), maxChannel(reverse));
                        if (scale <= 0.0F) {
                            continue;
                        }
                        ++pairsSeen;
                        if (!(maxChannel(glm::abs(forward - reverse)) <= kRelativeTolerance * scale)) {
                            std::cerr << "bsdf_validate: FAILED transmission reciprocity at roughness="
                                      << roughness << " ior=" << ior << " muO=" << mu
                                      << " offset=" << offset << "*alpha"
                                      << " f(wo->wi)*ior^2=" << maxChannel(forward)
                                      << " f(wi->wo)=" << maxChannel(reverse) << '\n';
                            ok = false;
                        }
                    }
                }
            }
        }
    }
    if (pairsSeen == 0) {
        std::cerr << "bsdf_validate: FAILED transmission reciprocity observed zero non-zero pairs -- "
                     "every constructed wi missed the lobe, so nothing was tested\n";
        ok = false;
    }
    std::cout << "  transmission reciprocity: " << pairsSeen << " non-zero pairs\n";
    finish(ctx, ok, "transmission_reciprocity failed; see the rows above");
    return;
}

// Round trip through a transmissive interface: sampleBsdf applies a non-symmetric eta^2 compression on refraction (Veach 1997 sec. 5.2), entering scales by (1/ior)^2, exiting by ior^2, so a ray that enters and leaves the same surface must lose no net energy.
// checkFurnace tests each side separately against a per-side bound (1.0 entering, ior^2 exiting), which passes even if the two factors do not actually cancel; this asserts the invariant bsdf.cpp's own comment claims.
PT_CHECK(transmission_round_trip, Slow, Statistical) {
    constexpr int kSampleCount = 200000;
    // Not tight to 1.0: each side's furnace value also contains that interface's reflected lobe, so the product carries a Fresnel cross-term that grows toward grazing (measured 1.027 at normal incidence, 1.058 at 60 degrees).
    // The band still has large discriminating power: factors that compounded rather than cancelled would land near ior^2=2.25, and ones that under-cancelled near 1/2.25=0.44.
    constexpr float kTolerance = 0.08F;
    constexpr float kIorRoundTrip = 1.5F;  // matches makeParams
    const std::array<float, 3> ndotVs = {1.0F, 0.8F, 0.5F};

    bool ok = true;
    std::uint32_t seed = 4000;
    for (float ndotV : ndotVs) {
        ++seed;
        // Snell-correct pairing: a ray entering at cos(thetaI) travels inside the medium at cos(thetaT), sin(thetaT)=sin(thetaI)/ior, so the exiting leg must be probed at thetaT, not thetaI.
        // Reusing thetaI puts the exit past the critical angle (cos~0.745 at ior 1.5), where the interface totally internally reflects and no round trip exists at all.
        const BsdfParams params = makeParams(0.05F, 0.0F, 1.0F);
        const float sinThetaI = std::sqrt(std::max(0.0F, 1.0F - (ndotV * ndotV)));
        const float sinThetaT = sinThetaI / kIorRoundTrip;
        const float cosThetaT = std::sqrt(std::max(0.0F, 1.0F - (sinThetaT * sinThetaT)));
        const glm::vec3 woIn(sinThetaI, 0.0F, ndotV);
        const glm::vec3 woOut(sinThetaT, 0.0F, -cosThetaT);
        const float entering = maxChannel(furnaceLo(params, woIn, kSampleCount, seed));
        const float exiting = maxChannel(furnaceLo(params, woOut, kSampleCount, seed + 700U));
        const float roundTrip = entering * exiting;
        std::cout << "  transmission round trip ndotV=" << ndotV << ": " << entering << " x "
                  << exiting << " = " << roundTrip << '\n';
        // Two-sided deliberately: an upper bound alone catches the eta^2 factors compounding but is blind to them under-cancelling, which loses energy on every round trip through glass, the same blind spot the white furnace test above exists to close.
        if (!(roundTrip >= 1.0F - kTolerance && roundTrip <= 1.0F + kTolerance)) {
            std::cerr << "bsdf_validate: FAILED transmission round trip at ndotV=" << ndotV
                      << " -- entering " << entering << " x exiting " << exiting << " = " << roundTrip
                      << "; the eta^2 radiance-compression factors must cancel over a round trip "
                         "(expected 1.0 +/- " << kTolerance << ").\n";
            ok = false;
        }
    }
    finish(ctx, ok, "transmission_round_trip failed; see the rows above");
    return;
}

// Transmission through an index-matched interface, the one configuration where the answer needs no reference at all: at ior 1 there is no interface, so a transmissive surface must pass every ray straight through, at every angle, and lose nothing.
// This is the only check in the suite that reaches the Snell block's cos^2 TIR predicate at ior 1. It decided total internal reflection from 1 - cos^2(thetaI), which rounds to exactly 1.0F below cos 2^-12, so it reported TIR at an interface with no critical angle and returned no sample at all. A lost transmission sample is silent: it is energy deleted from the estimator, not a wrong value, so no furnace, reciprocity or chi-square row above can see it -- only the absence asserted here.
// It is also the only check that pins ior 1 to that same block at EVERY roughness. An index-matched interface is not a rough interface, it is no interface, and transmissionIsRough says so (PBRT-v4's DielectricBxDF branches its value, pdf and sampler on the same `eta == 1 || EffectivelySmooth()`). Before it did, the rough branch reached refractAbout, which returns -wo about every microfacet normal at ior 1, and evaluateTransmissionLobe then normalised the resulting zero half-vector: measured NaN throughput on 7783 of 7783 transmission draws at roughness 0.1 and 6666 of 7783 at roughness 1.0, the remainder being msTransmit draws, which do not form that half-vector. NaN is silent for the same reason a lost sample is -- it is not a wrong value any row above can average, it poisons the pixel for the rest of the render.
// The existence assertion is binary and carries the check; it needs no tolerance and cannot be tuned. The direction and throughput assertions are the backstop that stops a sample that merely EXISTS from passing while pointing somewhere an index-matched interface cannot send it, and the reflection rows are the same statement on the near hemisphere: exact Fresnel is identically zero at ior 1, so nothing may come back.
PT_CHECK(index_matched_transmission, Fast, Exact) {
    // Straight-through is algebraic at r == 1 -- cos(thetaT) == cos(thetaI) and the tangential components scale by exactly 1 -- but it is reached through sqrt(fl(wo.z*wo.z)), which is not required to return |wo.z| to the last bit. Measured worst 0 over the whole sweep; the bound is one float32 epsilon of headroom, not a fitted number.
    constexpr float kDirectionTolerance = 1.2e-7F;
    // NOT throughput == tint: sampleBsdf returns f/pdf, which divides by the lobe-selection probability, so a single draw carries tint/P and reads 0.842105 against a 0.8 tint at P = 0.95. That factor is what makes the estimator unbiased, and asserting it away would assert a bias in.
    // The noise-free invariant is chromaticity instead: at ior 1 the only surviving factor is the tint itself, so throughput must be a positive SCALAR multiple of it, whatever P happens to be. A tint corruption, a channel swap or a per-channel Fresnel leak breaks the ratio; the selection probability cannot. Measured worst 0.
    constexpr float kChromaticityTolerance = 1.2e-7F;
    constexpr int kRoughDraws = 4096;
    constexpr float kSmoothRoughness = 0.02F;   // alpha 4e-4, below bsdf.cpp's kSmoothAlpha: the delta branch
    constexpr float kRoughRoughness = 0.3F;     // alpha 0.09, comfortably above it: the refractAbout branch
    // Reaches 1e-5 for the same reason checkIndexMatchedCoat does: 2.44e-4 is 2^-12, and every row at or below it returns nullopt on the pre-fix code.
    const std::array<float, 8> cosines = {1.0F,     0.7F,          0.1F,   1e-3F,
                                          2.44e-4F, 1.7263349e-4F, 1e-4F, 1e-5F};
    // Chromatic on purpose: a white tint cannot tell a preserved throughput from one that merely kept its brightness, the reason checkIndexMatchedCoat gives for its own chromatic row.
    const glm::vec3 tint(0.8F, 0.5F, 0.2F);

    bool ok = true;
    int rowsChecked = 0;
    float worstDirection = 0.0F;
    float worstChromaticity = 0.0F;
    int roughRejections = 0;
    int transmittedAtNormal = -1;   // set by the first row, cos 1, where no formulation can report TIR
    std::cout << "bsdf_validate: index-matched transmission, straight through at every angle (ior 1)\n";
    for (float cosine : cosines) {
        const float sine = std::sqrt(std::max(0.0F, 1.0F - (cosine * cosine)));
        const glm::vec3 wo(sine, 0.0F, cosine);
        const BsdfParams params = makeTransmissiveTintParams(kSmoothRoughness, glm::vec3(1.0F), tint);
        const BsdfParams smoothParams{params.baseColor,  params.metallic,
                                       params.roughness,  params.f0,
                                       params.edgeTint,   /*ior=*/1.0F,
                                       params.transmissionFactor, params.diffuseRoughness,
                                       params.diffuseRho, params.transmissionTint};
        pathtracer::scene::Sampler sampler(0, 0, 0, 1, 9100U);
        const std::optional<pathtracer::scene::BsdfSample> smooth =
            pathtracer::scene::sampleBsdf(smoothParams, wo, sampler);
        ++rowsChecked;
        if (!smooth.has_value()) {
            std::cerr << "bsdf_validate: FAILED index-matched transmission at cos=" << cosine
                      << " -- the smooth branch returned no sample; an ior-1 interface has no critical "
                         "angle, so refraction cannot fail at any angle and the energy is simply lost\n";
            ok = false;
            continue;
        }
        const float directionErr = maxChannel(glm::abs(smooth->wiLocal + wo));
        const glm::vec3 ratio = smooth->throughputWeight / tint;
        const float chromaticityErr = maxChannel(ratio) - minChannel(ratio);
        worstDirection = std::max(worstDirection, directionErr);
        worstChromaticity = std::max(worstChromaticity, chromaticityErr);
        if (!(directionErr <= kDirectionTolerance)) {
            std::cerr << "bsdf_validate: FAILED index-matched transmission direction at cos=" << cosine
                      << " -- got (" << smooth->wiLocal.x << ", " << smooth->wiLocal.y << ", "
                      << smooth->wiLocal.z << "), expected -wo; an index-matched interface cannot bend a ray\n";
            ok = false;
        }
        if (!(chromaticityErr <= kChromaticityTolerance)) {
            std::cerr << "bsdf_validate: FAILED index-matched transmission chromaticity at cos=" << cosine
                      << " -- throughput/tint is (" << ratio.x << ", " << ratio.y << ", " << ratio.z
                      << "), not one scalar; at ior 1 nothing but the tint can colour a transmitted ray\n";
            ok = false;
        }

        // The rough draws below must select that same delta branch and carry that same throughput. Every draw is asserted, not merely counted: a NaN throughput is invisible to a count, and a count is all this used to make.
        const BsdfParams roughParams{smoothParams.baseColor,  smoothParams.metallic,
                                      kRoughRoughness,         smoothParams.f0,
                                      smoothParams.edgeTint,   /*ior=*/1.0F,
                                      smoothParams.transmissionFactor, smoothParams.diffuseRoughness,
                                      smoothParams.diffuseRho, smoothParams.transmissionTint};
        int rejected = 0;
        int transmitted = 0;
        int notDelta = 0;
        int notFinite = 0;
        float roughDirection = 0.0F;
        float roughChromaticity = 0.0F;
        for (int i = 0; i < kRoughDraws; ++i) {
            pathtracer::scene::Sampler roughSampler(0, 0, i, kRoughDraws, 9200U);
            const std::optional<pathtracer::scene::BsdfSample> rough =
                pathtracer::scene::sampleBsdf(roughParams, wo, roughSampler);
            if (!rough.has_value()) {
                ++rejected;
                continue;
            }
            if (rough->type != pathtracer::scene::LobeType::Transmission) {
                continue;
            }
            ++transmitted;
            const glm::vec3 weight = rough->throughputWeight;
            if (!std::isfinite(weight.x) || !std::isfinite(weight.y) || !std::isfinite(weight.z)) {
                ++notFinite;
                continue;
            }
            if (rough->pdf != 0.0F) {
                ++notDelta;
            }
            const glm::vec3 roughRatio = weight / tint;
            roughDirection = std::max(roughDirection, maxChannel(glm::abs(rough->wiLocal + wo)));
            roughChromaticity =
                std::max(roughChromaticity, maxChannel(roughRatio) - minChannel(roughRatio));
        }
        worstDirection = std::max(worstDirection, roughDirection);
        worstChromaticity = std::max(worstChromaticity, roughChromaticity);
        roughRejections += rejected;
        if (transmittedAtNormal < 0) {
            transmittedAtNormal = transmitted;
        }
        std::cout << "  cos " << cosine << ": rough draws " << kRoughDraws << ", transmission "
                  << transmitted << ", rejected " << rejected << ", non-delta " << notDelta
                  << ", non-finite " << notFinite << '\n';
        if (notFinite != 0) {
            std::cerr << "bsdf_validate: FAILED index-matched rough transmission at cos=" << cosine
                      << " -- " << notFinite << " of " << transmitted
                      << " transmission draws carried a non-finite throughput; at ior 1 the lobe is a "
                         "delta and every draw must carry tint/P, so a NaN here is the zero half-vector "
                         "an index-matched rough refraction forms\n";
            ok = false;
        }
        if (notDelta != 0) {
            std::cerr << "bsdf_validate: FAILED index-matched rough transmission at cos=" << cosine
                      << " -- " << notDelta << " of " << transmitted
                      << " transmission draws reported a continuous pdf; an index-matched interface has "
                         "no rough lobe to sample at any roughness\n";
            ok = false;
        }
        // Nothing may reflect. Exact dielectric Fresnel is identically zero at ior 1, so the reflection lobe has no value to carry -- but the Kulla-Conty compensation reads its escaping fraction from a table whose eta axis is log-spaced and never lands on 1, and the interpolated reflected share it returns is energy the interface did not reflect. Measured up to 0.0191 over these rows before computeLobeProbabilities took the exact index-matched boundary, the grazing rows worst; asserted at zero, not at a bound, because zero is what the Fresnel says.
        for (float wiZ : {0.9F, 0.5F, 0.15F}) {
            const glm::vec3 wi(std::sqrt(std::max(0.0F, 1.0F - (wiZ * wiZ))), 0.0F, wiZ);
            const glm::vec3 reflected = pathtracer::scene::evaluateBsdfSplit(roughParams, wo, wi).specular;
            if (maxChannel(reflected) != 0.0F) {
                std::cerr << "bsdf_validate: FAILED index-matched reflection at cos=" << cosine
                          << " wi.z=" << wiZ << " -- reflected (" << reflected.x << ", " << reflected.y
                          << ", " << reflected.z
                          << "), expected exactly 0; an index-matched interface reflects nothing at any "
                             "angle, so this is multiple-scattering compensation returning a deficit that "
                             "does not exist\n";
                ok = false;
            }
        }

        // The same two backstops the smooth row above carries, at the same tolerances: existing and finite is not enough, the sample must also point where an index-matched interface can send it and carry nothing but the tint.
        if (!(roughDirection <= kDirectionTolerance)) {
            std::cerr << "bsdf_validate: FAILED index-matched rough transmission direction at cos="
                      << cosine << " -- worst |wi + wo| " << roughDirection
                      << " over the rough draws; an index-matched interface cannot bend a ray whatever "
                         "microfacet normal it draws\n";
            ok = false;
        }
        if (!(roughChromaticity <= kChromaticityTolerance)) {
            std::cerr << "bsdf_validate: FAILED index-matched rough transmission chromaticity at cos="
                      << cosine << " -- worst throughput/tint spread " << roughChromaticity
                      << " over the rough draws; at ior 1 nothing but the tint can colour a transmitted "
                         "ray\n";
            ok = false;
        }
        // The count of TRANSMISSION samples, not the count of rejections: at ior 1 the reflection lobe is identically zero, and sampleBsdf discards some of its draws on guards that have nothing to do with refraction, so a rejection bound would be asserting against those instead (measured: 6 of the 205 reflection draws at cos 0.1, carrying zero energy either way).
        // Every row draws the identical sampler sequence and, with F == 0 at every angle, the identical lobe selection, so the transmission count is a constant of the sweep rather than a statistic -- it must not vary with wo at all. A false TIR shows up here as a deficit against the normal-incidence row, which is the exact quantity the fix restores, with no tolerance and no noise.
        if (transmitted != transmittedAtNormal) {
            std::cerr << "bsdf_validate: FAILED index-matched rough transmission at cos=" << cosine
                      << " -- " << transmitted << " transmission samples against "
                      << transmittedAtNormal
                      << " at normal incidence; the lobe selection is angle-independent at ior 1, so a "
                         "deficit is refraction failing at an interface whose critical angle does not exist\n";
            ok = false;
        }
    }
    // The rough assertions are all per-draw, so a change that stopped the interface transmitting at all would satisfy every one of them by vacuum -- and the count invariant with it, since transmittedAtNormal is seeded from the same collapsed first row.
    if (transmittedAtNormal <= 0) {
        std::cerr << "bsdf_validate: FAILED index-matched transmission -- no rough draw transmitted, so "
                     "every rough assertion above was made about nothing\n";
        ok = false;
    }
    // No anti-vacuity guard on the angle count: cosines is non-empty at compile time and the counter increments unconditionally, so a zero count is unreachable. The count is printed, which is what keeps a future skip visible.
    std::cout << "  " << rowsChecked << " angles asserted, worst direction error " << worstDirection
              << ", worst chromaticity spread " << worstChromaticity << ", rough rejections "
              << roughRejections << " of " << kRoughDraws * static_cast<int>(cosines.size()) << '\n';
    finish(ctx, ok, "index_matched_transmission failed; see the rows above");
    return;
}


// --- Total internal reflection at a real critical angle.
// checkIndexMatchedTransmission and checkIndexMatchedCoat pin the ior == 1 degeneracy, where there IS no critical angle
// -- that is the case the cos^2 Snell form exists to fix. Neither says anything about TIR where it genuinely occurs,
// which until now was covered by a single checkFurnace row past the ior 1.5 critical angle. These two close that.
namespace {

// cos of the critical angle for a ray leaving the denser medium: sin(thetaC) = etaT/etaI, so cos(thetaC) =
// sqrt(1 - (etaT/etaI)^2). Stated in this file independently of bsdf.cpp's cos2Transmitted, so a fault in that
// predicate cannot define away the angle it is being measured against.
double criticalCosine(double iorDense) {
    const double ratio = 1.0 / iorDense;
    return std::sqrt(std::max(0.0, 1.0 - (ratio * ratio)));
}

}  // namespace

// Inside the TIR cone the interface must reflect EXACTLY all of it: at and past the critical angle cosThetaT is zero,
// both polarisation terms are exactly +/-1, and their mean is exactly 1.0F. Asserting exact equality rather than a
// tolerance is what makes this a regression test for the predicate rather than for the Fresnel arithmetic -- a
// formulation that merely approaches 1 near grazing would pass a banded check and fail this one.
// Outside the cone the reflectance must be strictly below 1, or the lobe has reported a critical angle that the
// interface does not have -- the exact defect the cos^2 form fixed at ior 1, here at ior > 1 where the cone is real.
PT_CHECK(critical_angle_onset, Fast, Exact) {
    // Spans the shipped range: glass.json is 1.5, clay.json's coat is 1.55, and 1.33/2.4 bracket it with water and
    // diamond so the cone's width varies by more than a factor of two across the sweep.
    const std::array<double, 5> iors = {1.33, 1.5, 1.5168, 1.55, 2.4};
    // Offsets in cos, straddling the critical cosine. The two smallest are below 2^-12, the scale at which the old
    // 1 - cos^2 transcription lost the distinction entirely.
    const std::array<double, 6> offsets = {1e-5, 2.44e-4, 1e-3, 1e-2, 0.1, 0.3};

    bool ok = true;
    int rowsChecked = 0;
    float worstOutside = 0.0F;
    std::cout << "bsdf_validate: total internal reflection onset at the critical angle (ior > 1)\n";
    for (double ior : iors) {
        const double muC = criticalCosine(ior);
        for (double offset : offsets) {
            // Exiting orientation: etaI is the dense medium. Inside the cone means a SMALLER cosine than critical.
            const double inside = muC - offset;
            if (inside > 0.0) {
                ++rowsChecked;
                const float f = pathtracer::scene::fresnelDielectric(static_cast<float>(inside),
                                                                  static_cast<float>(ior), 1.0F);
                if (!(f == 1.0F)) {
                    std::cerr << "bsdf_validate: FAILED TIR onset at ior=" << ior << " cos=" << inside
                              << " (critical " << muC << ", inside the cone by " << offset << ") -- reflectance "
                              << f << ", must be exactly 1.0: past the critical angle cosThetaT is zero and both "
                                 "polarisation terms are exactly one\n";
                    ok = false;
                }
            }
            const double outside = muC + offset;
            if (outside <= 1.0) {
                ++rowsChecked;
                const float f = pathtracer::scene::fresnelDielectric(static_cast<float>(outside),
                                                                  static_cast<float>(ior), 1.0F);
                worstOutside = std::max(worstOutside, f);
                if (!(f < 1.0F)) {
                    std::cerr << "bsdf_validate: FAILED TIR onset at ior=" << ior << " cos=" << outside
                              << " (critical " << muC << ", outside the cone by " << offset << ") -- reflectance "
                              << f << ", must be below 1.0: this direction refracts, so reporting total internal "
                                 "reflection deletes the transmitted energy entirely\n";
                    ok = false;
                }
            }
        }
    }
    // A sweep that checked nothing would pass vacuously, and the bracket above is conditional on both sides.
    if (rowsChecked == 0) {
        std::cerr << "bsdf_validate: FAILED TIR onset -- no rows checked\n";
        ok = false;
    }
    std::cout << "  rows " << rowsChecked << ", worst reflectance just outside the cone " << worstOutside << '\n';
    finish(ctx, ok, "critical_angle_onset failed; see the rows above");
    return;
}

// The two sites that decide TIR from the MACRO normal -- fresnelDielectric and sampleBsdf's smooth transmission branch
// -- now share one cos2Transmitted predicate rather than two separately-rounded transcriptions. That they agree is the
// property the shared helper buys, and it is exactly what a future refactor would break: a site that re-derives the
// condition locally stays self-consistent and silently disagrees with the other.
// Deliberately NOT asserted against the rough branch. refractAbout decides TIR per sampled MICROFACET, so the macro
// critical angle does not bound it, and the transmit-side multiple-scattering lobe is cosine-distributed over the far
// hemisphere and is not a refraction at all -- measured 115/4096 transmitted draws at roughness 0.3 a full 0.3 in
// cosine INSIDE the cone, which is correct behaviour and not a disagreement. A macro-level assertion there would be
// asserting something untrue.
// Both directions carry weight and fail differently: a transmitted sample inside the cone is energy arriving from a
// direction Snell cannot reach, and no transmitted sample outside it is energy deleted from the estimator rather than
// redirected -- the silent failure mode checkIndexMatchedTransmission exists to catch at ior 1, here at ior > 1.
PT_CHECK(tir_predicate_agreement, Fast, Exact) {
    // Enough draws that "transmission is reachable" is not a statement about one lucky lobe selection: just outside the
    // cone the transmitted share is already ~26% (measured 1068/4096 at ior 1.33), so a zero count over this many draws
    // is a structural absence rather than a sampling accident.
    constexpr int kDraws = 4096;
    // Angular resolution is set by the finest offset below: a divergence that moves the critical cosine by less than
    // 1e-3 falls between rows and is not detectable here. Measured by mutation -- shifting the smooth branch's
    // threshold to cos^2ThetaT < 0.05 (critical cosine +0.021 at ior 1.33) raises 5 rows, while < 0.002 (+0.0009)
    // raises none. That is the honest bound on this check, not a claim of exactness.
    constexpr float kSmoothRoughness = 0.02F;  // below bsdf.cpp's kSmoothAlpha: the delta branch
    const std::array<double, 3> iors = {1.33, 1.5, 2.4};
    const std::array<double, 4> offsets = {1e-3, 1e-2, 0.1, 0.25};

    bool ok = true;
    int rowsChecked = 0;
    std::cout << "bsdf_validate: TIR predicate agreement, fresnelDielectric vs the smooth refraction branch\n";
    for (double ior : iors) {
        const double muC = criticalCosine(ior);
        for (double offset : offsets) {
            for (const bool insideCone : {true, false}) {
                const double mu = insideCone ? muC - offset : muC + offset;
                if (mu <= 0.0 || mu > 1.0) {
                    continue;
                }
                ++rowsChecked;
                const float muF = static_cast<float>(mu);
                const bool fresnelSaysTir =
                    pathtracer::scene::fresnelDielectric(muF, static_cast<float>(ior), 1.0F) == 1.0F;

                // Exiting side: woLocal.z < 0 is the orientation in which the dense medium is the incident one, and the
                // only one in which a critical angle exists at all.
                const float sine = std::sqrt(std::max(0.0F, 1.0F - (muF * muF)));
                const glm::vec3 wo(sine, 0.0F, -muF);
                // ior taken from the row, not makeParams' fixed 1.5: the critical angle above is derived from this
                // same value, and a material at a different ior would be measured against the wrong cone.
                const BsdfParams base = makeParams(kSmoothRoughness, 0.0F, 1.0F);
                const BsdfParams params{base.baseColor,          base.metallic,
                                         base.roughness,          base.f0,
                                         base.edgeTint,           static_cast<float>(ior),
                                         base.transmissionFactor, base.diffuseRoughness,
                                         base.diffuseRho,         base.transmissionTint};

                int transmitted = 0;
                for (int i = 0; i < kDraws; ++i) {
                    pathtracer::scene::Sampler sampler(0, 0, i, kDraws, 5100U);
                    const std::optional<pathtracer::scene::BsdfSample> sample =
                        pathtracer::scene::sampleBsdf(params, wo, sampler);
                    transmitted +=
                        sample.has_value() && sample->type == pathtracer::scene::LobeType::Transmission ? 1 : 0;
                }

                // The two sites must reach the same verdict from the same macro geometry. Stated as the agreement
                // itself rather than as two independent thresholds, so the check cannot pass by both being wrong.
                if (!(fresnelSaysTir == (transmitted == 0))) {
                    std::cerr << "bsdf_validate: FAILED TIR agreement at ior=" << ior << " cos=" << mu
                              << " (critical " << muC << ") -- fresnelDielectric "
                              << (fresnelSaysTir ? "reports" : "does not report")
                              << " total internal reflection, but the smooth branch transmitted " << transmitted
                              << " of " << kDraws << " draws; the two sites decide from the same cos2Transmitted and "
                                 "must agree\n";
                    ok = false;
                }
                // Inside the cone the count must be exactly zero, not merely small: a delta lobe has one Snell
                // direction and inside the cone it does not exist, so any transmitted draw is unreachable energy.
                if (fresnelSaysTir && !(transmitted == 0)) {
                    std::cerr << "bsdf_validate: FAILED TIR agreement at ior=" << ior << " cos=" << mu
                              << " -- " << transmitted << " transmitted draws inside the critical cone\n";
                    ok = false;
                }
            }
        }
    }
    if (rowsChecked == 0) {
        std::cerr << "bsdf_validate: FAILED TIR agreement -- no rows checked\n";
        ok = false;
    }
    std::cout << "  rows " << rowsChecked << '\n';
    finish(ctx, ok, "tir_predicate_agreement failed; see the rows above");
    return;
}

}  // namespace

// --- Goodness of fit between where sampleBsdf's draws actually land and the density pdfBsdf reports for them.
// The gap this closes: checkSampleDensityConsistency proves only that the two code paths agree with each other, not that either describes the realised distribution -- its own comment records that deleting the msReflect density term leaves it green. A sampler and a pdf that are wrong together are invisible to every other check here except as an energy shift the furnace bands may be too loose to resolve.
// Pearson's chi-square over equal-solid-angle bins of the whole sphere, with expected counts from integrating pdfBsdf over each bin (Mitsuba's chi2test; PBRT-v4's BSDF sampling tests). Bins are uniform in (cos theta, phi) because that is the measure of dw, so every bin subtends the same solid angle and the quadrature below integrates the density directly.
// Rejected draws are a cell in their own right, which is what makes this a test of the total sampling mass as well as its shape: sampleBsdf returning nullopt is exactly the event "wi left the sampled support", of probability 1 - integral(pdf), and a pdf that integrates to the wrong total shows up here as a rejection-cell mismatch rather than passing unnoticed.
// scrambleSeed is drawn fresh per sample -- the one configuration sampler.h warns forfeits stratification -- deliberately: a chi-square needs iid draws, and a low-discrepancy point set would make the null distribution wrong in the dangerous direction, understating the statistic and hiding a real mismatch.
struct ChiSquareCase {
    float roughness;
    float metallic;
    float transmission;
    float ndotV;
};

PT_CHECK(sampling_chi_square, Slow, Statistical) {
    constexpr int kCosBins = 16;
    constexpr int kPhiBins = 8;
    constexpr int kPanels = 256;            // even, for Simpson, per axis per bin; measured, not guessed: the peaked refraction lobe at roughness 0.2 is mis-integrated badly enough to report p=1e-78 on correct code at 48 panels and to still fail at 64, passes from 96, and the p-value stops moving past this
    // Sized for power at the suite's family-wise rate: 200000 draws at the former 1% family rate, scaled by the noncentrality ratio for equal 99% power at 128 dof (119.2 -> 165.6, x1.39), so moving to kFamilyAlpha loses no detectable effect.
    constexpr int kSampleCount = 280000;
    constexpr double kMinExpected = 5.0;    // Cochran's rule, the count below which a cell's chi-square term is not trustworthy and must be pooled
    constexpr std::uint32_t kSeed = 0x9E3779B9U;

    // Transmissive rows sit either side of the interface so the transmitted multiple-scattering lobe is drawn at both eta orientations; the metallic rows are where the reflected one carries the whole diffuse selection mass.
    const std::array<ChiSquareCase, 12> cases = {{
        {0.2F, 0.0F, 1.0F, 0.8F},  {0.2F, 0.0F, 1.0F, -0.6F},
        {0.4F, 0.0F, 1.0F, 0.8F},  {0.4F, 0.0F, 1.0F, -0.6F},
        {0.7F, 0.0F, 1.0F, 0.8F},  {0.7F, 0.0F, 1.0F, -0.6F},
        {1.0F, 0.0F, 1.0F, 0.8F},  {1.0F, 0.0F, 1.0F, -0.6F},
        {0.3F, 1.0F, 0.0F, 0.7F},  {0.8F, 1.0F, 0.0F, 0.7F},
        {0.5F, 0.0F, 0.0F, 0.5F},  {1.0F, 0.0F, 0.0F, 0.5F},
    }};
    // The suite's corrected significance, split across the grid by Sidak so twelve independent cases share it.
    ctx.plan(1);
    const double perCase = tools::stats::sidak(ctx.alpha(), static_cast<int>(cases.size()));

    bool ok = true;
    double worstP = 1.0;
    for (const ChiSquareCase& testCase : cases) {
        const BsdfParams params = makeParams(testCase.roughness, testCase.metallic, testCase.transmission);
        const glm::vec3 wo(std::sqrt(std::max(0.0F, 1.0F - (testCase.ndotV * testCase.ndotV))), 0.0F, testCase.ndotV);

        std::vector<double> expected(static_cast<std::size_t>(kCosBins) * kPhiBins + 1, 0.0);
        double mass = 0.0;
        for (int ci = 0; ci < kCosBins; ++ci) {
            const double c0 = -1.0 + (2.0 * ci / kCosBins);
            const double c1 = -1.0 + (2.0 * (ci + 1.0) / kCosBins);
            for (int pi = 0; pi < kPhiBins; ++pi) {
                const double p0 = 2.0 * kPiDouble * pi / kPhiBins;
                const double p1 = 2.0 * kPiDouble * (pi + 1.0) / kPhiBins;
                const double integral = simpson(c0, c1, kPanels, [&](double cosTheta) {
                    const double sinTheta = std::sqrt(std::max(0.0, 1.0 - (cosTheta * cosTheta)));
                    return simpson(p0, p1, kPanels, [&](double phi) {
                        const glm::vec3 wi(static_cast<float>(sinTheta * std::cos(phi)),
                                            static_cast<float>(sinTheta * std::sin(phi)),
                                            static_cast<float>(cosTheta));
                        return static_cast<double>(pathtracer::scene::pdfBsdf(params, wo, wi));
                    });
                });
                expected[static_cast<std::size_t>((ci * kPhiBins) + pi)] = integral * kSampleCount;
                mass += integral;
            }
        }
        expected.back() = std::max(1.0 - mass, 0.0) * kSampleCount;

        std::vector<double> observed(expected.size(), 0.0);
        std::mt19937 rng(kSeed);
        for (int i = 0; i < kSampleCount; ++i) {
            pathtracer::scene::Sampler sampler(0, 0, 0, 1, rng());
            const std::optional<pathtracer::scene::BsdfSample> sample = pathtracer::scene::sampleBsdf(params, wo, sampler);
            if (!sample.has_value() || sample->pdf <= 0.0F) {
                observed.back() += 1.0;
                continue;
            }
            const glm::vec3 wi = sample->wiLocal;
            const int ci = std::min(static_cast<int>((wi.z + 1.0F) * 0.5F * kCosBins), kCosBins - 1);
            float phi = std::atan2(wi.y, wi.x);
            if (phi < 0.0F) { phi += static_cast<float>(2.0 * kPiDouble); }
            const int pi = std::min(static_cast<int>(phi / static_cast<float>(2.0 * kPiDouble) * kPhiBins), kPhiBins - 1);
            observed[static_cast<std::size_t>((ci * kPhiBins) + pi)] += 1.0;
        }

        // Cells too sparse to trust individually are merged into one, which is what keeps the statistic chi-square distributed rather than merely chi-square shaped.
        double chiSquare = 0.0;
        int cells = 0;
        double pooledExpected = 0.0;
        double pooledObserved = 0.0;
        for (std::size_t i = 0; i < expected.size(); ++i) {
            if (expected[i] < kMinExpected) {
                pooledExpected += expected[i];
                pooledObserved += observed[i];
                continue;
            }
            const double delta = observed[i] - expected[i];
            chiSquare += (delta * delta) / expected[i];
            ++cells;
        }
        if (pooledExpected >= kMinExpected) {
            const double delta = pooledObserved - pooledExpected;
            chiSquare += (delta * delta) / pooledExpected;
            ++cells;
        }
        const int dof = cells - 1;
        const double p = tools::stats::chiSquareUpperTail(chiSquare, dof);
        worstP = std::min(worstP, p);
        if (p >= perCase) {
            continue;
        }
        std::cerr << "bsdf_validate: FAILED sampling chi-square at roughness=" << testCase.roughness
                  << " metallic=" << testCase.metallic << " transmission=" << testCase.transmission
                  << " ndotV=" << testCase.ndotV << " chi2=" << chiSquare << " dof=" << dof << " p=" << p
                  << " (threshold " << perCase << ", sampled mass " << mass << ")\n";
        ok = false;
    }
    std::cout << "bsdf_validate: sampling chi-square over " << cases.size() << " configurations, "
              << kSampleCount << " draws each, worst p-value " << worstP << " against a Sidak threshold of "
              << perCase << "\n";
    finish(ctx, ok, "sampling_chi_square failed; see the rows above");
    return;
}

// fresnelAtMicrofacet (bsdf.h) is the path-traced Fresnel AOV's whole estimator, and it makes two claims this pins.
// (1) At the roughness floor the visible-normal distribution collapses onto the macro normal, so its expectation must
// return fresnelAtViewAngle -- that is what makes the AOV a strict generalisation of the single macro-normal sample it
// replaced rather than a different quantity wearing the same name.
// (2) Above the floor it must NOT return that value at grazing: D_vis spreads the half-vector over a lobe whose width
// grows with alpha, and F is convex in cos, so the mean over the lobe falls below the macro value exactly where the
// macro ramp is steepest. A version that ignored roughness -- or sampled the wrong distribution -- would pass (1) and
// fail (2), which is why both are asserted together.
PT_CHECK(microfacet_fresnel, Slow, Statistical) {
    ctx.plan(3);
    constexpr int kDraws = 200000;
    constexpr std::uint32_t kSeed = 7919U;
    // Expectation of F over D_vis at this roughness and view angle, by the same draws the renderer makes.
    const auto expectation = [](const BsdfParams& params, float ndotV) {
        const glm::vec3 wo(std::sqrt(std::max(0.0F, 1.0F - (ndotV * ndotV))), 0.0F, ndotV);
        glm::dvec3 sum(0.0);
        for (int i = 0; i < kDraws; ++i) {
            pathtracer::scene::Sampler sampler(0, 0, i, kDraws, kSeed);
            sum += glm::dvec3(pathtracer::scene::fresnelAtMicrofacet(params, wo, sampler.next2D()));
        }
        return glm::vec3(sum / static_cast<double>(kDraws));
    };

    // 0.02 is the roughness kMinAlpha floors alpha at (bsdf.cpp), so this is the smoothest surface the BSDF admits.
    bool smoothOk = true;
    float worstSmooth = 0.0F;
    for (const float ndotV : {0.1F, 0.4F, 0.7F, 1.0F}) {
        for (const float metallic : {0.0F, 1.0F}) {
            const BsdfParams params = makeParams(0.02F, metallic, 0.0F);
            const glm::vec3 mean = expectation(params, ndotV);
            const glm::vec3 macro = fresnelAtViewAngle(params, ndotV);
            for (int c = 0; c < 3; ++c) {
                worstSmooth = std::max(worstSmooth, std::abs(mean[c] - macro[c]));
            }
        }
    }
    // The residual is the half-vector's O(alpha) tilt at alpha = 4e-4, not sampling noise; 1e-3 is an order above it.
    smoothOk = worstSmooth < 1e-3F;
    char smoothDetail[192];
    std::snprintf(smoothDetail, sizeof(smoothDetail),
                  "worst |E[F(wo.wh)] - F(n.wo)| at the roughness floor is %.3e, which must be below 1e-3",
                  static_cast<double>(worstSmooth));
    PT_EXPECT(ctx, smoothOk, smoothDetail);

    // Grazing, where the macro ramp is steepest and the lobe average therefore departs from it most.
    constexpr float kGrazing = 0.1F;
    const BsdfParams rough = makeParams(0.6F, 0.0F, 0.0F);
    const glm::vec3 roughMean = expectation(rough, kGrazing);
    const glm::vec3 roughMacro = fresnelAtViewAngle(rough, kGrazing);
    char flatDetail[192];
    std::snprintf(flatDetail, sizeof(flatDetail),
                  "at roughness 0.6, grazing: E[F] = %.4f against a macro F of %.4f, which it must fall below",
                  static_cast<double>(roughMean.x), static_cast<double>(roughMacro.x));
    PT_EXPECT(ctx, roughMean.x < roughMacro.x, flatDetail);

    // A coloured conductor: the reason the lane carries RGB rather than the (F, 1-F, 0) packing it replaced. Gulbrandsen
    // 2014's edgeTint inverts to a per-channel complex IOR, so the expectation is chromatic and a single channel cannot
    // stand in for it. Grazing again, where the edge tint acts.
    const BsdfParams tinted = makeColoredMetalParams(0.2F, glm::vec3(0.9F, 0.6F, 0.3F));
    const glm::vec3 tintedMean = expectation(tinted, kGrazing);
    const float spread = std::max({tintedMean.x, tintedMean.y, tintedMean.z}) -
                          std::min({tintedMean.x, tintedMean.y, tintedMean.z});
    char chromaDetail[192];
    std::snprintf(chromaDetail, sizeof(chromaDetail),
                  "edge-tinted conductor spans %.4f across RGB (%.4f, %.4f, %.4f); a greyscale lane would report one",
                  static_cast<double>(spread), static_cast<double>(tintedMean.x),
                  static_cast<double>(tintedMean.y), static_cast<double>(tintedMean.z));
    PT_EXPECT(ctx, spread > 1e-3F, chromaDetail);
}

PT_CHECK_MAIN("bsdf")
