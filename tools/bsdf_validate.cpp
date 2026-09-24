// Standalone correctness check for pathtracer::scene::bsdf: the combined pdf never integrates to more than the total
// lobe-selection mass, and a furnace test never returns more energy than it received. Upper bound only, VNDF
// reflection discarding below-horizon samples. Same standalone-CLI convention as the other validators.

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

// A coloured, dark conductor (f0=0.5, not the white f0=1 makeParams gives at metallic=1). A white f0 clamps
// specularProb to 0.95, leaving only 5% diffuse selection mass to hide a diffuse-pdf error under this tolerance;
// f0=0.5 leaves ~50%, enough for the same error to fail loudly.
BsdfParams makeColoredMetalParams(float roughness, glm::vec3 edgeTint = glm::vec3(1.0F)) {
    const glm::vec3 baseColor(1.0F);
    return BsdfParams{baseColor,    1.0F, roughness, glm::vec3(0.5F), edgeTint,
                       /*ior=*/1.5F, /*transmissionFactor=*/0.0F, /*diffuseRoughness=*/0.0F,
                       pathtracer::scene::eonAlbedoInversion(baseColor, 0.0F),
                       /*transmissionTint=*/glm::vec3(1.0F)};
}

// Uniform-solid-angle hemisphere sample (PBRT-style inversion): z=u1, r=sqrt(1-u1^2), phi=2*pi*u2. Used to integrate
// pdfBsdf over the hemisphere, MIS-combined (Veach 1997 sec. 9.2) with sampleBsdf's own density, which IS the
// integrand. Upper bound only; diffuseRoughness is swept because at 0 the uniform-mix branch goes untested.
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
                        // pdfBsdf mirrors wi into wo's hemisphere, so for a below-surface wo the density over the +z
                        // hemisphere is identically zero. Integrating there passed the assertion vacuously, so the
                        // exiting-side rows tested nothing: flip the sampled hemisphere to match wo's side.
                        if (ndotV < 0.0F) {
                            wi.z = -wi.z;
                        }
                        const double p = pathtracer::scene::pdfBsdf(params, wo, wi);
                        integral += p / combinedDensity(p);
                    }
                    // sampleBsdf returns wi in woLocal's own convention and reports the density it drew from, so no
                    // second pdfBsdf evaluation is needed and none of the mirroring above applies.
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

// sampleBsdf's reported density must equal pdfBsdf re-evaluated at the direction it returned, the contract
// BsdfSample::pdf states and the one nothing else asserts. Exact equality, not a tolerance: both sides are the same
// arithmetic over the same LobeProbabilities. Swept over checkPdfNormalization's grid plus the exiting rows.
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

// Asserts the pass condition rather than the failure condition: NaN compares false against every ordered operator,
// so a `min < lo || max > hi` form would be satisfied by a NaN and report success.
bool withinBand(const glm::vec3& value, float centre, float tolerance) {
    return minChannel(value) >= centre - tolerance && maxChannel(value) <= centre + tolerance;
}

// Support coverage: every direction the BSDF has value at must carry mixture density, the condition under which the
// one-sample MIS estimator is unbiased (Veach 1997 sec. 9.2). Exact, not a tolerance -- f > 0 with pdf == 0 is a
// support defect at any magnitude. Closes the gap where a strategy gated off entirely passes shape and total tests.
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

// Furnace test through sampleBsdf: uniform L0=1 from every direction, both hemispheres since transmission can
// receive from the far side. The bound is 1.0 except on the exiting side below the critical angle, where the
// non-symmetric eta^2 compression makes eta^2 the correct answer. The sweep reaches past the ior=1.5 critical angle.
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

    // Coloured conductor (f0=0.5): a white f0's clamped 95% specular probability leaves too little diffuse selection
    // mass for this tolerance to resolve a diffuse-pdf error.
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

// Two-sided white furnace: a white, non-absorbing surface under uniform L0=1 must return exactly 1.0. Single-scatter
// GGX loses what Smith G2 masks away (Heitz et al. 2016) -- measured 0.307 at roughness 1.0 -- so this is what
// Kulla-Conty has to return. Half the rows sit off the albedo table's grid to exercise its bilinear blend.
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
        // Off-grid values, re-placed whenever the table's resolution changes: a row that drifted onto a node would
        // measure nothing. Each is the exact float landing on index k+0.5, so the lookup blends two nodes at one
        // half. Nothing derives them from albedoGridRes(), which would follow the grid and could never fail.
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

    // EON rough-diffuse energy preservation: the two-sided white furnace swept over diffuseRoughness. Classical
    // Oren-Nayar variants lose energy as it rises, which EON's multiple-scattering term fixes, so this reads 1.0
    // everywhere. Conductors are swept too and asserted exactly invariant, diffuseKd being zeroed at metallic=1.
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
                // Seeded by (metallic, ndotV) only, never by diffuseRoughness: the assertion below is exact
                // equality, so the two readings it compares must draw the identical direction sequence.
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

// Paper Listing 1's E_EON at normal incidence, the EON directional albedo rho*E_F + rho_ms*(1-E_F), transcribed
// independently of evaluateEon so the two cannot share a mistake. Normal incidence needs no FON G-term: the exact
// albedo's G and the renderer's quartic fit both vanish at mu=1, leaving AF = 1/(1+c1*r) on either path.
glm::vec3 referenceEonAlbedo(const glm::vec3& rho, float r) {
    const float c1 = 0.5F - (2.0F / (3.0F * kPi));
    const float c2 = (2.0F / 3.0F) - (28.0F / (15.0F * kPi));
    const float eFon = 1.0F / (1.0F + (c1 * r));
    const float avgEFon = eFon * (1.0F + (c2 * r));
    const glm::vec3 rhoMs = (rho * rho) * avgEFon / (glm::vec3(1.0F) - (rho * (1.0F - avgEFon)));
    return (rho * eFon) + (rhoMs * (1.0F - eFon));
}

// A bare EON diffuse surface. ior=1 makes the measurement exact rather than approximate: exact dielectric Fresnel is
// identically zero, so the coat contributes nothing, which checkIndexMatchedCoat asserts rather than assumes.
// The specular roughness and f0 are inert here by the same argument, and swept to prove it.
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

// Cosine-weighted hemispherical integral of the shipped diffuse lobe at normal incidence -- the directional albedo
// the renderer produces, not a reimplementation. The second moment is accumulated alongside the first so the
// assertion band is the estimator's own standard error, computed from the run rather than picked by hand.
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

// The observed albedo must equal the authored albedo: the property EON's Appendix A inversion exists to provide.
// Two assertions per row: the analytic one is closed form against closed form and catches an algebra error, the
// Monte Carlo one runs through the shipped lobe. The chromatic row carries the weight, the saturation being per-channel.
PT_CHECK(eon_albedo_inversion, Slow, Statistical) {
    constexpr int kSampleCount = 400000;
    // Two named, bounded residuals and nothing else. kSigmaBand is a confidence level on the estimator's own measured
    // standard error; kFitTolerance is the model residual. Neither is a tolerance picked to make a run pass.
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

// EON BRDF value (paper eq. 16-19) in double, transcribed independently of evaluateEon with c1 and c2 re-derived
// from their literals. The quartic albedo fit's coefficients are the paper's data, so quoting them is transcription.
// The 1e-7 floors are reproduced rather than dropped: they are the model's guarded evaluation at r=0.
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

// The instrument for coatAlbedo's fresnelAvg argument: checkAverageFresnel pins dielectricFresnelAvg as a function,
// this pins what coatAlbedo passes it. ior=1 is the only point resolvable without the albedo table at all, and the
// tolerance is exactly zero from x*1.0F == x. The sweep reaches mu 1e-5, where the old Snell form falsely hit TIR.
PT_CHECK(index_matched_coat, Fast, Exact) {
    // Exact, from the collapse above. The second bound is a float32-against-double residual on the same closed form,
    // about 15 operations deep.
    constexpr float kInvarianceTolerance = 0.0F;
    constexpr float kValueTolerance = 1e-6F;
    // 0.0 is the reference row every other is compared against. 0.3661 and 0.92 sit deliberately off the table's grid,
    // the same device checkWhiteFurnaceTwoSided uses.
    const std::array<float, 8> roughnesses = {0.0F, 0.05F, 0.25F, 0.3661F, 0.5F, 0.75F, 0.92F, 1.0F};
    // The tail below 2.44e-4 (2^-12, where 1.0F-mu*mu rounds to 1.0F) is where the old Snell transcription falsely
    // reported TIR.
    const std::array<float, 11> cosines = {1.0F,   0.8F,         0.6F,    0.4F,    0.2F, 0.05F,
                                           1e-2F, 1e-3F, 2.44e-4F, 1.7263349e-4F, 1e-5F};
    const std::array<float, 3> diffuseRoughnesses = {0.0F, 0.5F, 1.0F};
    // The chromatic row carries the weight for the value assertion, for the reason checkEonAlbedoInversion gives: at
    // baseColor 1 the inversion is the identity and proves nothing.
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

// Mean throughput through sampleBsdf with every transmitted draw converted back from radiance to energy: dividing
// those draws by eta^2 puts every sample in one domain with an analytic answer. Accumulated in double, since at
// 200k samples a float sum carries an ulp of 0.008 and values just above 1 would round down into it.
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

// Two-sided energy balance for a transmissive interface, the counterpart to checkWhiteFurnaceTwoSided; in the energy
// domain 1.0 is correct everywhere. Gates two failure modes the radiance checks cannot see: compensation delivered
// over the wrong hemisphere, and an escape budget that double-counts. metallic=1 rows must transmit nothing.
PT_CHECK(transmissive_energy_balance, Slow, Statistical) {
    constexpr int kSampleCount = 200000;
    // Same tolerance as the opaque white furnace: 1.0 is a correctness target, not a baseline, and the residual under
    // it is deterministic model error rather than sampling noise. Measured worst 0.0036, at transmissionFactor 0.5
    // entering, where the diffuse coat coupling renormalises by an averaged rescale.
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

// Round-trip energy closure of a rough dielectric: a white, non-absorbing slab reads exactly 1.0 under a uniform
// environment. Unlike transmissive_energy_balance it integrates the escape table over every transmitted direction,
// through BSDF sampling alone. The floor is the table's accuracy against the generator, 1e-3 over a worst 6.2e-4.
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

// A white, non-absorbing transmissive dielectric with an explicit baseColor and transmissionTint -- the one
// configuration this suite never had, since makeParams leaves both at their defaults.
BsdfParams makeTransmissiveTintParams(float roughness, const glm::vec3& baseColor,
                                       const glm::vec3& transmissionTint) {
    return BsdfParams{baseColor,          /*metallic=*/0.0F,            roughness,
                       glm::vec3(0.04F),   /*edgeTint=*/glm::vec3(1.0F), /*ior=*/1.5F,
                       /*transmissionFactor=*/1.0F, /*diffuseRoughness=*/0.0F,
                       pathtracer::scene::eonAlbedoInversion(baseColor, 0.0F), transmissionTint};
}

// Snell refraction of wo about +z, transcribed independently of bsdf.cpp: the direction the interface actually
// refracts into, so a sign or eta error in either is visible as a disagreement.
glm::vec3 refractAboutZ(const glm::vec3& wo, float eta) {
    const float sin2ThetaT = eta * eta * std::max(0.0F, 1.0F - (wo.z * wo.z));
    const float cosThetaT = std::sqrt(std::max(0.0F, 1.0F - sin2ThetaT));
    return {-eta * wo.x, -eta * wo.y, -cosThetaT};
}

// Throughput of sampleBsdf's smooth delta transmission branch, which carries its tint on a different code path from
// the continuous lobes. pdf == 0 identifies that branch, a rough sample returning a real density.
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

// The transmission-tint convention, and the only instrument here that can see it: every other transmissive case runs
// at baseColor 1, where the conventions agree. Two assertions per row -- independence of baseColor, exact, and
// linearity in the tint at a few ULP -- across both the rough continuous lobe and the smooth delta branch.
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

    // One row's pair of assertions, shared by the rough and smooth paths: T must not move with baseColor at all, and
    // must scale exactly with the tint.
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
        // The delta branch does not depend on the roughness sweep -- it is selected by being below the smooth
        // threshold -- so it is measured once rather than per row.
        const auto smooth = [&](const glm::vec3& bc, const glm::vec3& tn) {
            return deltaTransmitThroughput(makeTransmissiveTintParams(kSmoothRoughness, bc, tn), wo,
                                            seed)
                .value_or(glm::vec3(0.0F));
        };
        assertRow("smooth", kSmoothRoughness, ndotV, smooth(white, white), smooth(baseColour, white),
                   smooth(white, tint), smooth(baseColour, tint));
    }

    // Backstop: every assertion above is skipped where the interface transmits nothing, so a change that silently
    // zeroed the transmission lobe would pass them all vacuously.
    if (measured == 0) {
        std::cerr << "bsdf_validate: FAILED transmission tint -- no row transmitted anything, so "
                     "nothing was asserted\n";
        ok = false;
    }
    finish(ctx, ok, "transmission_tint failed; see the rows above");
    return;
}

// Exact unpolarized dielectric Fresnel, entering orientation, in double: the reference dielectricFresnelAvg's
// quadrature rule is measured against.
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

// The instrument the suite never had: F_avg attenuates every repeated bounce of the Kulla-Conty lobe, and no energy
// test resolves an error in it, the furnaces running where every candidate agrees. Truth is this file's own
// reference. The conductor tolerance is the fit's measured bound, 4.0e-4; Karis' Schlick mean fails it by 216x.
PT_CHECK(average_fresnel, Fast, Exact) {
    constexpr double kConductorTolerance = 5e-4;
    // The working band, everything any material actually authors: measured worst 5.5e-5 at ior 1.0575 over a
    // 0.0025-step scan of [1.05, 3.0].
    constexpr double kDielectricTolerance = 1e-4;
    // The near-index-match band, stated separately rather than absorbed into the one above, which it would loosen 8x
    // over a region no material occupies.
    constexpr double kNearIndexMatchTolerance = 8e-4;
    constexpr double kNearIndexMatchIor = 1.05;
    const std::array<double, 12> reflectivities = {1e-4, 0.01, 0.1,  0.25, 0.4,  0.48,
                                                    0.555, 0.7, 0.85, 0.95, 0.99, 1.0};
    const std::array<double, 8> edgeTints = {0.0, 0.1, 0.25, 0.5, 0.6, 0.75, 0.9, 1.0};
    // Both regimes, and the two iors that ship (glass.json 1.5168, clay.json 1.55). ior=1 is where an index-matched
    // interface reflects nothing at all.
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

// --- Independent reference for the reflect-side albedo table. Domain and measure are the generator's, which is why
// a fixed rule works: with tan(theta_h) = alpha*tan(psi) (Walter 2007) the integrand flattens. The phi split at pi/2
// resolves a boundary layer, and the inner rule is Gauss-Legendre because Simpson converges an order slower.
double referenceSmithG2OverCosO(double cosO, double cosI, double alpha) {
    const double alpha2 = alpha * alpha;
    const auto radical = [&](double c) { return std::sqrt(alpha2 + ((1.0 - alpha2) * c * c)); };
    return 2.0 * cosI / ((cosI * radical(cosO)) + (cosO * radical(cosI)));
}

constexpr double kPiDouble = 3.14159265358979324;

// Gauss-Legendre nodes and weights mapped to [0,1], by Newton iteration on P_n through Bonnet's recurrence
// (Press et al., Numerical Recipes 3rd ed.).
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

// bsdf.cpp's roughness floor, mirrored so every reference below evaluates the alpha the lobe actually ships at
// rather than an unclamped one the renderer never sees.
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

// Cosine-weighted mean, 2*int_0^1 E(mu)*mu dmu. The mu=0 endpoint contributes exactly 0, the mu weight killing a
// bounded E.
glm::dvec2 referenceAverageAlbedo(double alpha) {
    constexpr int kPanels = 64;
    return 2.0 * simpson(0.0, 1.0, kPanels,
                          [&](double mu) { return referenceDirectionalAlbedo(mu, alpha) * mu; });
}


// --- The instrument for albedo_table.inc's interpolation error, the table's second error source, which nothing else
// resolves: the two-sided furnace bounds it only to 2%. Four measurements reported separately, each bound the
// measured worst plus ~1.7x, against this file's own double-precision reference rather than the shipped lookup.
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

    // Evenly spread node indices over [first, last], endpoints included: the "held on exact nodes" coordinate, where
    // that axis contributes no interpolation error and the other is isolated. The mu axis sweeps from node 0, a real
    // node at mu = 0 where E = 1 is an exact identity, so it is the sharpest column rather than one to skip.
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

    // Control, listed first because every row below is only as trustworthy as this one: both axes on exact nodes, so
    // the lookup returns a stored value verbatim and no interpolation happens. What is left is this file's Simpson
    // against the generator's Gauss-Legendre. If it is not small, the instrument is the limit, not the table.
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

// The albedo-table reference is this check's whole cost -- a 96x96 Simpson and a 64-panel Simpson per row -- so both
// are hoisted out of every loop that does not change them.
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

// The diffuse channel's full coupling, (1 - coat(wo))/(1 - coatAvg) * (1 - coat(wi)), as a function of the fresnelAvg
// the three call sites pass.
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

// The instrument for coatAlbedo's fresnelAvg value at working ior, which checkIndexMatchedCoat cannot supply: that
// pins the argument, this pins what it evaluates to. Recovered by inverting the coat coupling -- never as
// diffuse(ior)/diffuse(ior=1), which cancels the term being measured. Full ior range, every candidate mutated.
PT_CHECK(coat_fresnel_average, Slow, Exact) {
    constexpr double kTolerance = 6e-5;
    // Residual of the recovered root, not an accuracy claim: it catches a coupling the model cannot reproduce at any
    // fresnelAvg, such as a lost 1/(1-coatAvg) renormalisation.
    constexpr double kResidualTolerance = 1e-6;
    const std::array<double, 9> iors = {1.1, 1.33, 1.5, 1.5168, 1.55, 1.8, 2.0, 2.5, 3.0};
    // Roughness 0 is excluded: 1-E is ~0 there, so F_avg reaches nothing and is not observable. Nothing here is
    // aligned to the table's grid, so the rows exercise its interpolation too.
    const std::array<double, 4> roughnesses = {0.25, 0.5, 0.75, 1.0};
    const std::array<double, 3> cosines = {0.4, 0.7, 1.0};
    const glm::vec3 albedo(0.8F, 0.3F, 0.1F);
    constexpr float kDiffuseRoughness = 0.5F;

    // Hoisted out of the ior loop as well as the row loop: the albedo reference does not depend on ior, so the whole
    // sweep costs four reference evaluations rather than four per ior.
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

                    // The bracket is the model's own monotone branch: referenceCoupling is unimodal in fresnelAvg,
                    // so the extremum is located by ternary search and bisection then runs on the monotone side.
                    // Monotone increasing, which is not obvious: F_avg raises coatAvg faster than coatAlbedo.
                    const CoatGeometry geometry = coatGeometry(ior, albedosByRoughness[r],
                                                                static_cast<int>(o),
                                                                static_cast<int>(i), muO, muI);
                    // Wide enough to contain every candidate a revert could install.
                    constexpr double kBracketCeiling = 0.6;
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
                    // Central difference, wide enough to clear the bisection's own resolution and narrow enough that
                    // the coupling is locally linear across it.
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

// Cauchy dispersion, asserted against the contract it exists to satisfy rather than a restatement of its own formula:
// (ior, abbe) means n_d at the d line with V_d = (n_d-1)/(n_F-n_C), so those two identities are the specification.
// Nothing else in the suite can see an error here -- every render-based transmissive check runs at ior 1.0.
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

// Conductor Fresnel, verified through the public API. At metallic=1 evaluateBsdf is the specular lobe alone,
// f(g) = K*F_g(woDotNh) + M, and M does not cancel across edgeTint since conductorFresnelAvg makes F_avg a function
// of it. So the two halves split by roughness: 0.05 pins the ratio identity, 0.6 pins monotonicity in edgeTint.
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
    // Strict, with no epsilon: the guard above conditions the whole g=0->1 span, so the compare is between two
    // readings of the same well-conditioned quantity.
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
    // The quotient below divides one span by another, so it only carries signal where the denominator span is large
    // enough to be resolved above float32 noise.
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

// The specular lobe's closed form at the mirrored pair, in double: nh is exactly +z there, so sin(theta_h) is 0 and
// the GGX denominator collapses to alpha^2, leaving D = 1/(pi*alpha^2) exactly. G2 is Smith height-correlated with
// both lambdas at the same cosine.
double specularGeometry(double alpha, double cosine) {
    const double alpha2 = alpha * alpha;
    const double d = 1.0 / (kPiDouble * alpha2);
    const double tan2 = (1.0 - (cosine * cosine)) / (cosine * cosine);
    const double lambda = 0.5 * (-1.0 + std::sqrt(1.0 + (alpha2 * tan2)));
    const double g2 = 1.0 / (1.0 + (2.0 * lambda));
    return (d * g2) / (4.0 * cosine * cosine);
}

// The only instrument in this file that reads the specular lobe's absolute magnitude, and the only one that can:
// sampleBsdf returns f/pdf, where a common factor cancels. That gap let distributionGGX ship two errors invisible in
// throughput. Dividing the measured lobe by the reference K isolates F; the grazing rows run to cos 1e-7.
PT_CHECK(dielectric_fresnel, Fast, Exact) {
    // Float32 round-off in the shipped lobe against a double reference, relative to F since rounding is. M is not
    // resolvable at these roughnesses.
    constexpr double kFresnelTolerance = 4.7e-7;
    // Normal incidence is an exact identity, not a fit: referenceDielectricFresnel(1, n) is ((n-1)/(n+1))^2 with both
    // polarisations equal.
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
                // Strict, no epsilon: unpolarized external reflection is monotone in theta for every n > 1, and the
                // smallest real step here is comfortably above float32 noise.
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
    // Index match, kept out of the sweep: at ior 1 the true curve is identically zero, so the monotonicity assertion
    // would have nothing to compare. Asserted at exactly zero, F being the only ior-dependent factor. The cosines run
    // to 1e-5 because below 2^-12 the old Snell form rounded to 1.0F and falsely reported TIR -- 6 of 8 rows failed.
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
    // No anti-vacuity guard here, unlike checkIndexMatchedCoat and checkCoatFresnelAvg: both arrays are non-empty at
    // compile time, every row is compared, and the count is fixed.
    std::cout << "  dielectric Fresnel: " << rowsChecked << " points vs reference, "
              << indexMatchedRows << " index-matched rows at exactly zero\n";
    finish(ctx, ok, "dielectric_fresnel failed; see the rows above");
    return;
}

// Helmholtz reciprocity: f(wo->wi) == f(wi->wo), symmetric by construction once the directional-albedo diffuse
// coupling is applied on both sides. It fails hard on the pre-coupling code, which the furnace passed throughout.
// Transmission is excluded: radiance transport across a refracting interface is genuinely non-symmetric.
PT_CHECK(reciprocity, Fast, Exact) {
    constexpr float kRelativeTolerance = 1e-4F;
    const std::array<float, 4> roughnesses = {0.05F, 0.25F, 0.5F, 1.0F};
    const std::array<float, 3> metallics = {0.0F, 0.5F, 1.0F};
    const std::array<float, 4> cosines = {1.0F, 0.7F, 0.4F, 0.15F};
    // At 0 the diffuse lobe is Lambertian and reciprocal for free, so the sweep is what actually puts EON under this
    // check.
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

// eta^2-corrected reciprocity for the transmission lobe: f_t(wo->wi)*eta_wi^2 == f_t(wi->wo)*eta_wo^2. Single
// scatter only, permanently -- transmitMultiScatter reads its factors at orientations the swap exchanges, inherent
// rather than a gap (ROADMAP.md transport #1). wi is constructed, not sampled, or the check passes vacuously.
PT_CHECK(transmission_reciprocity, Fast, Exact) {
    // Not checkReciprocity's 1e-4: D is sharply peaked at these alphas and the two queries build ht from differently
    // scaled sums, so the float32 paths diverge more than a reflection pair does.
    constexpr float kRelativeTolerance = 1e-2F;
    const std::array<float, 2> roughnesses = {0.05F, 0.10F};
    const std::array<float, 3> iors = {1.2F, 1.5F, 2.0F};
    const std::array<float, 4> cosines = {1.0F, 0.9F, 0.7F, 0.5F};
    const std::array<float, 5> offsets = {0.0F, 0.5F, 1.0F, 2.0F, 4.0F};  // multiples of alpha
    // Perturbation axes. Out-of-plane and diagonal put wo and wi at different azimuths, so a swapped-phi error
    // cannot hide the way it would in a coplanar sweep.
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
                        // A zero offset lands on the refracted direction whatever the axis is, so only the first
                        // pass over it is a distinct pair.
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

// Round trip through a transmissive interface: the non-symmetric eta^2 compression (Veach 1997 sec. 5.2) applies
// entering and exiting, and the two must cancel exactly. checkFurnace tests each side separately against a per-side
// bound, which passes even if they do not cancel; this asserts the product.
PT_CHECK(transmission_round_trip, Slow, Statistical) {
    constexpr int kSampleCount = 200000;
    // Not tight to 1.0: each side's furnace value also contains that interface's reflected lobe, so the product
    // carries a Fresnel cross-term that does not cancel. The band still discriminates: compounding factors would
    // land near ior^2, under-cancelling ones well below 1.
    constexpr float kTolerance = 0.08F;
    constexpr float kIorRoundTrip = 1.5F;  // matches makeParams
    const std::array<float, 3> ndotVs = {1.0F, 0.8F, 0.5F};

    bool ok = true;
    std::uint32_t seed = 4000;
    for (float ndotV : ndotVs) {
        ++seed;
        // Snell-correct pairing: the exit angle is not the entry angle, and reusing thetaI would put the exit past
        // the critical angle where no round trip exists. Two-sided deliberately: an upper bound alone is blind to
        // the factors under-cancelling, which loses energy just as wrongly.
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
        // Two-sided deliberately: an upper bound alone catches the eta^2 factors compounding but is blind to them
        // under-cancelling, which loses energy just as wrongly.
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

// Transmission through an index-matched interface, the one configuration needing no reference: at ior 1 a ray passes
// straight through carrying only the tint. The only check reaching the Snell block's cos^2 TIR predicate at ior 1,
// and the only one pinning it at every roughness. The existence assertion is binary and cannot be tuned.
PT_CHECK(index_matched_transmission, Fast, Exact) {
    // Straight-through is algebraic at r == 1 -- cos(thetaT) == cos(thetaI) and the tangential components scale by
    // exactly 1 -- but it is reached through the shipped code path, not asserted of it.
    constexpr float kDirectionTolerance = 1.2e-7F;
    // Not throughput == tint: sampleBsdf returns f/pdf, so a draw carries tint/P. The noise-free invariant is
    // chromaticity instead -- at ior 1 the only surviving factor is the tint, so throughput is a positive multiple
    // of it. Chromatic on purpose: a white tint cannot tell preservation from mere brightness.
    constexpr float kChromaticityTolerance = 1.2e-7F;
    constexpr int kRoughDraws = 4096;
    constexpr float kSmoothRoughness = 0.02F;   // alpha 4e-4, below bsdf.cpp's kSmoothAlpha: the delta branch
    constexpr float kRoughRoughness = 0.3F;     // alpha 0.09, comfortably above it: the refractAbout branch
    // Reaches 1e-5 for the same reason checkIndexMatchedCoat does: 2.44e-4 is 2^-12, and every row at or below it
    // returned nullopt on the pre-fix code.
    const std::array<float, 8> cosines = {1.0F,     0.7F,          0.1F,   1e-3F,
                                          2.44e-4F, 1.7263349e-4F, 1e-4F, 1e-5F};
    // Chromatic on purpose: a white tint cannot distinguish a preserved throughput from one that merely kept its
    // brightness.
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

        // The rough draws must select that same delta branch and carry that same throughput. Every draw is asserted,
        // not merely counted.
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
        // Nothing may reflect: exact dielectric Fresnel is identically zero at ior 1, so the reflection lobe has no
        // value to carry.
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

        // The same two backstops the smooth row carries, at the same tolerances: existing and finite is not enough,
        // the sample must also point the right way.
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
        // The count of transmission samples, not of rejections: at ior 1 the reflection lobe is identically zero, so
        // every draw is a transmission, and every row draws the identical sequence and selection.
        if (transmitted != transmittedAtNormal) {
            std::cerr << "bsdf_validate: FAILED index-matched rough transmission at cos=" << cosine
                      << " -- " << transmitted << " transmission samples against "
                      << transmittedAtNormal
                      << " at normal incidence; the lobe selection is angle-independent at ior 1, so a "
                         "deficit is refraction failing at an interface whose critical angle does not exist\n";
            ok = false;
        }
    }
    // The rough assertions are all per-draw, so a change that stopped the interface transmitting at all would satisfy
    // every one of them vacuously.
    if (transmittedAtNormal <= 0) {
        std::cerr << "bsdf_validate: FAILED index-matched transmission -- no rough draw transmitted, so "
                     "every rough assertion above was made about nothing\n";
        ok = false;
    }
    // No anti-vacuity guard on the angle count: cosines is non-empty at compile time and the counter increments
    // unconditionally.
    std::cout << "  " << rowsChecked << " angles asserted, worst direction error " << worstDirection
              << ", worst chromaticity spread " << worstChromaticity << ", rough rejections "
              << roughRejections << " of " << kRoughDraws * static_cast<int>(cosines.size()) << '\n';
    finish(ctx, ok, "index_matched_transmission failed; see the rows above");
    return;
}


// --- Total internal reflection at a real critical angle: past it the interface must reflect exactly all of the
// energy, cosThetaT being non-positive so no transmitted direction exists.
namespace {

// cos of the critical angle for a ray leaving the denser medium: sin(thetaC) = etaT/etaI, so cos(thetaC) =
// sqrt(1 - (etaT/etaI)^2). Stated in this file independently of bsdf.cpp's cos2Transmitted, so a fault in that
// predicate cannot define away the angle it is being measured against.
double criticalCosine(double iorDense) {
    const double ratio = 1.0 / iorDense;
    return std::sqrt(std::max(0.0, 1.0 - (ratio * ratio)));
}

}  // namespace

    // Inside the TIR cone the interface must reflect exactly all of it: at and past the critical angle there is no
    // transmitted direction at all, so a draw that reports one is energy arriving where Snell cannot reach.
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

// The two sites deciding TIR from the macro normal now share one cos2Transmitted predicate rather than two
// separately-rounded transcriptions. Deliberately not asserted against the rough branch: refractAbout decides per
// microfacet, and the transmit-side multiple-scattering lobe is not a refraction at all (115/4096 draws inside).
PT_CHECK(tir_predicate_agreement, Fast, Exact) {
    // Enough draws that "transmission is reachable" is not a statement about one lucky lobe selection: just outside the
    // cone the transmitted share is already ~26% (measured 1068/4096 at ior 1.33), so a zero count over this many draws
    // is a structural absence rather than a sampling accident.
    constexpr int kDraws = 4096;
    // Angular resolution is set by the finest offset below: a divergence moving the critical cosine by less than
    // that would fall between the rows and go unseen.
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

// --- Goodness of fit between where sampleBsdf's draws actually land and the density pdfBsdf reports for them:
// Pearson's chi-square over equal-solid-angle bins of the sphere, with rejected draws a cell of their own, so this
// tests the total sampling mass as well as its shape. Fresh scramble per sample, a chi-square needing iid draws.
struct ChiSquareCase {
    float roughness;
    float metallic;
    float transmission;
    float ndotV;
};

PT_CHECK(sampling_chi_square, Slow, Statistical) {
    constexpr int kCosBins = 16;
    constexpr int kPhiBins = 8;
    // Even, for Simpson, per axis per bin. Measured, not guessed: the peaked refraction lobe at roughness 0.2 reports
    // p=1e-78 on correct code at 48 panels, still fails at 64, passes from 96, and stops moving past this.
    constexpr int kPanels = 256;
    // Sized for power at the suite's family-wise rate: 200000 draws, scaled by the noncentrality ratio so the power
    // against the same alternative is unchanged.
    constexpr int kSampleCount = 280000;
    // Cochran's rule: the count below which a cell's chi-square term is not trustworthy and must be pooled.
    constexpr double kMinExpected = 5.0;
    constexpr std::uint32_t kSeed = 0x9E3779B9U;

    // Transmissive rows sit either side of the interface so the transmitted multiple-scattering lobe is drawn at
    // both eta orientations.
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

        // Cells too sparse to trust individually are merged into one, which is what keeps the statistic
        // chi-square distributed rather than merely chi-square shaped.
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

// fresnelAtMicrofacet is the path-traced Fresnel AOV's whole estimator, and it makes two claims nothing else checks:
// that the half-vector comes from the same VNDF sampleBsdf draws from, and that E[F(wo.wh)] over it converges to a
// roughness-dependent mean a single macro-normal evaluation cannot express.
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
