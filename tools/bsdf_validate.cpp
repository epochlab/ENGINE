// Standalone correctness check for engine::scene::bsdf (bsdf.h): verifies the combined specular+diffuse pdf never integrates to more than the total lobe-selection mass.
// Upper bound only, deliberately: VNDF reflection sampling is not normalized over the hemisphere (samples reflecting below the horizon are discarded), so the true integral is the horizon-clipped mass, which has no closed form and is measured instead by the white furnace test below.
// Also runs a furnace test (uniform incident radiance from every direction, including through transmission) via BSDF importance sampling: must never reflect/transmit more energy than received.
// Same standalone-CLI convention as embree_validate.cpp/furnace_test.cpp: no test framework, non-zero exit on failure.

#include <array>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <random>

#include <glm/glm.hpp>

#include "engine/scene/bsdf.h"
#include "engine/scene/sampler.h"

namespace {

using engine::scene::BsdfParams;

constexpr float kPi = 3.14159265F;

// edgeTint defaults to white, the no-dip edge Schlick always produced, so every pre-existing case here
// is a strict subset of the swept coverage rather than a shifted version of it.
BsdfParams makeParams(float roughness, float metallic, float transmissionFactor,
                       float diffuseRoughness = 0.0F, glm::vec3 edgeTint = glm::vec3(1.0F)) {
    const glm::vec3 baseColor(1.0F);  // worst case: full white albedo
    const glm::vec3 f0 = glm::mix(glm::vec3(0.04F), baseColor, metallic);
    return BsdfParams{baseColor,          metallic, roughness,          f0, edgeTint,
                       /*ior=*/1.5F, transmissionFactor, diffuseRoughness,
                       engine::scene::eonAlbedoInversion(baseColor, diffuseRoughness),
                       /*transmissionTint=*/glm::vec3(1.0F)};
}

// A colored/dark conductor (f0=0.5, not the white f0=baseColor=1 makeParams gives at metallic=1): the case that caught evaluateDiffuseLobe's pdf-gating bug.
// A white f0 clamps specularProb to 0.95, leaving only 5% diffuse selection mass to hide a diffuse-pdf bug under this test's tolerance; f0=0.5 leaves ~50%, large enough for the same bug to fail loudly.
BsdfParams makeColoredMetalParams(float roughness, glm::vec3 edgeTint = glm::vec3(1.0F)) {
    const glm::vec3 baseColor(1.0F);
    return BsdfParams{baseColor,    1.0F, roughness, glm::vec3(0.5F), edgeTint,
                       /*ior=*/1.5F, /*transmissionFactor=*/0.0F, /*diffuseRoughness=*/0.0F,
                       engine::scene::eonAlbedoInversion(baseColor, 0.0F),
                       /*transmissionTint=*/glm::vec3(1.0F)};
}

// Uniform-solid-angle hemisphere sample (PBRT-style inversion, same as furnace_test.cpp): z=u1, r=sqrt(1-u1^2), phi=2*pi*u2, pdf=1/(2*pi).
glm::vec3 sampleUniformHemisphere(std::mt19937& rng) {
    std::uniform_real_distribution<float> unit(0.0F, 1.0F);
    const float cosTheta = unit(rng);
    const float sinTheta = std::sqrt(std::max(0.0F, 1.0F - (cosTheta * cosTheta)));
    const float phi = 2.0F * kPi * unit(rng);
    return {sinTheta * std::cos(phi), sinTheta * std::sin(phi), cosTheta};
}

// Integrates pdfBsdf(wo, .) over the hemisphere; for an opaque material (transmissionFactor=0) this must not exceed 1.0, since all sampling probability mass is in the two continuous lobes the pdf covers.
// Multiple importance sampling with the balance heuristic (Veach 1997 sec. 9.2) over two proposals that between them cover both regimes: uniform hemisphere for the tails, sampleBsdf's own density for the lobe. Uniform alone stops working once the lobe is narrow -- at roughness 0.05 it spans ~2e-5 sr, which 200k uniform draws hit a handful of times at O(100) weight each, and the estimate is then too noisy to bound at all (measured 1.67 against a truth of <= 1, and still 1.17 at a hundred times the samples).
// sampleBsdf draws from exactly pdfBsdf, so the second proposal's density IS the integrand and the combined balance-heuristic density collapses to N1/(2*pi) + N2*p(x): one pdf evaluation per sample, bounded below by the uniform term and above by N2*p, so it is well conditioned at every roughness. Unbiased despite that density being sub-normalised (sampleBsdf returns nullopt below the horizon), because the heuristic needs only the expected sample count per solid angle, which is N2*q2 either way.
// Upper-bound only, not an equality: VNDF reflection sampling discards samples reflected below the horizon, so the true integral is the horizon-clipped mass, which has no closed form. The estimator is now tight enough that a real double-counted pdf shows up as an excess rather than drowning in variance.
// diffuseRoughness is swept because at 0 -- the only value this used to test, and the one principled.json ships -- eonUniformMixWeight is pow(0, 0.1) = 0 exactly, so pdfEon degenerates to cltcPdf alone and CLTC itself degenerates to plain cosine. Neither the LTC fit's own normalisation nor the uniform/CLTC one-sample MIS mixture was reached at all. The metallic=1 rows matter most here: metallic zeroes diffuseKd but NOT diffuseProb, so a conductor still carries the full CLTC density with none of its value, and a mis-normalised fit shows up in the mixture denominator rather than in any picture.
bool checkPdfNormalization() {
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
                        const double p = engine::scene::pdfBsdf(params, wo, wi);
                        integral += p / combinedDensity(p);
                    }
                    // sampleBsdf returns wi in woLocal's own convention and reports the density it drew from, so no second pdfBsdf evaluation is needed and none of the mirroring above applies.
                    for (int i = 0; i < kBsdfSamples; ++i) {
                        engine::scene::Sampler sampler(0, 0, i, kBsdfSamples, kBsdfSeed);
                        const std::optional<engine::scene::BsdfSample> sample =
                            engine::scene::sampleBsdf(params, wo, sampler);
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
    return ok;
}

// sampleBsdf's reported density must equal pdfBsdf re-evaluated at the direction it returned -- the contract BsdfSample::pdf states in bsdf.h ("exactly what pdfBsdf would return for it") and that nothing asserted. checkPdfNormalization above reads sample->pdf, the density the sampler reports about itself, and never re-evaluates it, so that contract had no instrument at all.
// What it covers was measured, not assumed. NOT a mixture term missing from the density: sampleBsdf reports evaluateContinuousLobes' own pdf, so both sides are then wrong together and this stays green -- deleting the msReflect density term leaves 0 failures here and fails checkFurnace at Lo=1.28 instead. What it does cover is the sign-mirroring round trip, since sampleBsdf returns wi in woLocal's convention and pdfBsdf re-mirrors it and nothing else in the suite closes that loop, and any future strategy reporting a hand-computed density beside the mixture rather than through it -- the usual optimisation once a lobe's own pdf is already in hand.
// Exact equality, not a tolerance: both sides are the same arithmetic over the same deterministic LobeProbabilities, so any difference is a broken round trip rather than drift. Delta transmission reports 0 on both sides and is asserted like every other row.
// Swept over checkPdfNormalization's grid plus transmissionFactor, including the ndotV<0 exiting rows the mirroring claim rests on, so every strategy in the ladder is drawn: VNDF reflection, EON, both multiple-scattering cosine lobes, rough and smooth refraction.
bool checkSampleDensityConsistency() {
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
                            engine::scene::Sampler sampler(0, 0, i, kSampleCount, kSeed);
                            const std::optional<engine::scene::BsdfSample> sample =
                                engine::scene::sampleBsdf(params, wo, sampler);
                            if (!sample.has_value()) {
                                continue;
                            }
                            ++compared;
                            const float reevaluated =
                                engine::scene::pdfBsdf(params, wo, sample->wiLocal);
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
    return ok;
}

glm::vec3 furnaceLo(const BsdfParams& params, const glm::vec3& wo, int sampleCount, std::uint32_t seed) {
    glm::vec3 accum(0.0F);
    for (int i = 0; i < sampleCount; ++i) {
        engine::scene::Sampler sampler(0, 0, i, sampleCount, seed);
        const std::optional<engine::scene::BsdfSample> sample =
            engine::scene::sampleBsdf(params, wo, sampler);
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

// Furnace test through sampleBsdf: uniform radiance L0=1 from every direction (both hemispheres, since transmission can receive from the far side); estimator Lo = mean(throughputWeight) since throughputWeight already folds in f*cosTheta/pdf.
// ndotV sweep includes negative values (woLocal.z<0, the exiting side of a transmissive dielectric) and a value past the ior=1.5 critical angle (~41.8deg, cosTheta~0.745) to force total internal reflection.
// Energy bound is 1.0 (L0) everywhere except the exiting side (ndotV<0) of a transmissive material below the critical angle, where sampleBsdf's eta^2 non-symmetric radiance-compression factor (Veach 1997 sec. 5.2, see bsdf.cpp's transmission branch) legitimately raises Lo above L0: L/n^2 is the invariant along a ray, so radiance increases going from a denser medium (ior=1.5, inside) into a rarer one (1.0, outside) by up to ior^2.
// The naive Lo<=1 bound only holds for eta==1 interfaces (pure reflection) or the entering side, where this same factor is <1, exactly compensating so a round trip through the surface loses no net energy.
bool checkFurnace() {
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
    return ok;
}

// TWO-SIDED white furnace: a white, non-absorbing surface under uniform L0=1 radiance must return exactly 1.0 (every photon it receives leaves again); checkFurnace above only ever asserts Lo<=bound, so it cannot see energy loss, this BSDF's actual failure mode.
// Restricted to cases where 1.0 is analytically correct: white base color, no transmission, entering side. A colored conductor (f0=0.5) legitimately absorbs with no closed-form expectation, so it stays upper-bound-only in checkFurnace.
// Single-scatter GGX loses the energy smithG2 masks away (Heitz, Hanika, d'Eon, Dachsbacher 2016): a white conductor at roughness 1.0 measured 0.307, under a third of the light received. Kulla-Conty multiple-scattering compensation plus the directional-albedo diffuse coupling (bsdf.cpp) return it, making 1.0 a correctness target, not a regression baseline: both bounds share the same tolerance and a shortfall is a bug.
// Half the rows sit deliberately off the 32x32 albedo table's grid; there the measured value is E_true + (1 - E_interpolated), so these rows test the table's interpolation error directly, which is why bsdf.cpp needs no public accessor for the table itself.
struct WhiteFurnaceCase {
    float roughness;
    float ndotV;
    bool offGrid;
};

bool checkWhiteFurnaceTwoSided() {
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
        // Off-grid: table rows/columns land on k/31, so these fall mid-cell on both axes.
        {0.37F, 0.565F, true},
        {0.37F, 0.31F, true},
        {0.63F, 0.565F, true},
        {0.63F, 0.31F, true},
        {0.82F, 0.565F, true},
        {0.82F, 0.31F, true},
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
    return ok;
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
bool checkEonDiffuseFurnace() {
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
    return ok;
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
                              float roughness = 0.5F) {
    return BsdfParams{baseColor,          /*metallic=*/0.0F,          roughness,
                       glm::vec3(0.0F),    /*edgeTint=*/glm::vec3(1.0F), /*ior=*/1.0F,
                       /*transmissionFactor=*/0.0F, diffuseRoughness,
                       engine::scene::eonAlbedoInversion(baseColor, diffuseRoughness),
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
            engine::scene::evaluateBsdfSplit(params, wo, wi).diffuse * wi.z * 2.0F * kPi;
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
bool checkEonAlbedoInversion() {
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
    return ok;
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

// The instrument for coatAlbedo's fresnelAvg ARGUMENT, which nothing else in this suite can see. checkAverageFresnel pins dielectricFresnelAvg the function; reverting the three call sites (bsdf.cpp:538, :805, :808) to Karis' schlickFresnelAvg(coatF0) leaves all six validators green while moving 1882 of the 1886 channels cornell changed, because clay.json's ior 1.55 is every wall, the floor and the ceiling.
// ior=1 is the ONLY point where the question is resolvable at all. At every ior>1 each term carrying F_avg is multiScatterTint(F_avg, Eavg)*(1-E(mu, alpha)), and E/Eavg come only from the private 32x32 table whose own quadrature error is ~1.5e-3 (bsdf.cpp:169) against a 2.0e-4 spread between the two candidates: the reference is already 7.5x coarser than the signal, and resolving it at 10:1 needs an E 75x more accurate than the table being checked. At ior=1 the table's coefficients are multiplied by exact zeros and drop out of the expression entirely.
// The collapse is exact in float32, term by term: dielectricF0(1)=+0, dielectricFresnelAvg(1)=(1-1)/5.08638=+0 so multiScatterTint(0,E)=+0, fresnelDielectric(mu,1,1)=+0 so coatFresnelRatio=+0, hence coatAlbedo=+0, diffuseCoupling=(1-0)/(1-0)=1, diffuseKdAt=1, and .diffuse is evaluateEon multiplied by exactly 1.0F. evaluateEon reads only diffuseRho/diffuseRoughness/wi/wo, and alpha reaches .specular alone, so params.roughness has no other route into the diffuse channel: an index-matched interface is optically absent, and its roughness cannot be observable.
// Hence tolerance exactly zero, from x*1.0F == x -- an algebraic guarantee, not a measured run. The Karis revert breaks it by a measured 7.8e-4 relative, worst at roughness 0.92 / mu_o = mu_i = 1, about 8000 ULP at the diffuse value's magnitude.
// Two facts that set the sweep, both against instinct. The deviation peaks at NORMAL incidence, not grazing: coat = msTint*(1-E(mu)) and E rises toward grazing at high roughness (0.31 at mu=1, 0.89 at mu=1/31), so a grazing-first sweep is ~6x weaker. And it changes sign below mu~0.3 at roughness 1, so only |delta|==0 is a safe predicate; any signed bound breaks.
// Do NOT widen mu below 1e-3: fresnelDielectric(mu, 1, 1) returns 1.0F, not 0, for mu <= 1.7263349e-4, because etaI==etaT makes the ratio exactly 1 and 1.0F-mu*mu rounds to 1.0F, tripping the total-internal-reflection early-out (bsdf.cpp:45). The collapse is genuinely false in that sliver; it is recorded in README section 5.1 rather than worked around silently. Do NOT extend the sweep to ior>1 either -- the identity is exact only at index match.
// Residual, deliberate: any g(ior) with g(1)=0 passes here, notably 2*dielectricFresnelAvg(ior). This check pins the argument's collapse; checkAverageFresnel pins the function's value. Neither alone is sufficient and both are cheap.
bool checkIndexMatchedCoat() {
    // Exact, from the collapse above. The second bound is a float32-vs-double residual on the same closed form, ~15 operations deep, measured worst 1.74e-7 -- 5.7x under, thin on purpose. It is not the instrument; it is the backstop that stops a roughness-INDEPENDENT corruption (a pinned diffuseKd, a lost 1/(1-coatAvg) normalisation, a channel swap) from passing as bit-identical, which the invariance assertion alone cannot see.
    constexpr float kInvarianceTolerance = 0.0F;
    constexpr float kValueTolerance = 1e-6F;
    // 0.0 is the reference row every other is compared against. 0.37 and 0.92 sit deliberately off the table's 32-row k/31 grid, the device checkWhiteFurnaceTwoSided's off-grid cases use; 0.92 is where the revert's deviation is largest.
    const std::array<float, 8> roughnesses = {0.0F, 0.05F, 0.25F, 0.37F, 0.5F, 0.75F, 0.92F, 1.0F};
    const std::array<float, 6> cosines = {1.0F, 0.8F, 0.6F, 0.4F, 0.2F, 0.05F};
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
                        engine::scene::evaluateBsdfSplit(
                            makeDiffuseParams(albedo, diffuseRoughness, roughnesses[0]), wo, wi)
                            .diffuse;
                    const glm::vec3 analytic = referenceEon(
                        engine::scene::eonAlbedoInversion(albedo, diffuseRoughness),
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
                            engine::scene::evaluateBsdfSplit(
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
    return ok;
}

// Mean throughput through sampleBsdf with every transmitted draw converted back from radiance to energy. sampleBsdf applies the non-symmetric eta^2 radiance compression on refraction (Veach 1997 sec. 5.2), so a transmitted sample carries radiance and a raw mean is bounded by ior^2, not 1.0, which is why checkFurnace can only assert an upper bound on its transmissive rows and never sees energy loss there.
// Dividing those draws by eta^2 puts every sample in one domain with an analytic answer; LobeType::Transmission is exactly the far-hemisphere draws, delta and rough alike.
glm::vec3 transmissiveEnergyLo(const BsdfParams& params, const glm::vec3& wo, int sampleCount,
                                std::uint32_t seed) {
    const float eta = wo.z < 0.0F ? params.ior : 1.0F / params.ior;  // etaI/etaT, exiting vs entering
    const float etaSq = eta * eta;
    glm::vec3 accum(0.0F);
    for (int i = 0; i < sampleCount; ++i) {
        engine::scene::Sampler sampler(0, 0, i, sampleCount, seed);
        const std::optional<engine::scene::BsdfSample> sample =
            engine::scene::sampleBsdf(params, wo, sampler);
        if (!sample.has_value()) {
            continue;
        }
        accum += sample->type == engine::scene::LobeType::Transmission
                      ? sample->throughputWeight / etaSq
                      : sample->throughputWeight;
    }
    return accum / static_cast<float>(sampleCount);
}

// TWO-SIDED energy balance for a transmissive interface: the counterpart to checkWhiteFurnaceTwoSided, which is restricted to "no transmission, entering side" since those are the only rows where 1.0 is correct in the radiance domain.
// In the energy domain 1.0 is correct everywhere: a white, non-absorbing interface reflects, refracts, or hands the rest to the diffuse substrate, and the multiple-scattering lobes return what smithG2 masked; nothing is absorbed at any roughness, side, or transmissionFactor.
// Gates two failure modes the radiance-domain checks structurally cannot see: multiple-scattering compensation delivered over the refraction-reachable cone only rather than the whole far hemisphere, and a transmission lobe whose value drops transmissionFactor or (1-metallic) while its selection probability keeps them (the factors cancel out of throughput, so only an absolute bound catches it).
// metallic=1 rows cover a conductor, which must transmit nothing however its transmissionFactor is set.
bool checkTransmissiveEnergyBalance() {
    constexpr int kSampleCount = 200000;
    // Same tolerance as the opaque white furnace: 1.0 is a correctness target, not a baseline. Residual is albedo-table interpolation error, worst across the TIR boundary where the transmitted channel steps in mu and eta=1.5 falls between two table slices.
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
    return ok;
}

// A white, non-absorbing transmissive dielectric with an explicit baseColor and transmissionTint -- the one configuration this suite never had, makeParams above hardcoding baseColor 1 and every transmissive row leaving the tint white.
BsdfParams makeTransmissiveTintParams(float roughness, const glm::vec3& baseColor,
                                       const glm::vec3& transmissionTint) {
    return BsdfParams{baseColor,          /*metallic=*/0.0F,            roughness,
                       glm::vec3(0.04F),   /*edgeTint=*/glm::vec3(1.0F), /*ior=*/1.5F,
                       /*transmissionFactor=*/1.0F, /*diffuseRoughness=*/0.0F,
                       engine::scene::eonAlbedoInversion(baseColor, 0.0F), transmissionTint};
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
        engine::scene::Sampler sampler(0, 0, i, kAttempts, seed);
        const std::optional<engine::scene::BsdfSample> sample =
            engine::scene::sampleBsdf(params, wo, sampler);
        if (sample.has_value() && sample->type == engine::scene::LobeType::Transmission &&
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
bool checkTransmissionTint() {
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
                return engine::scene::evaluateBsdfSplit(makeTransmissiveTintParams(roughness, bc, tn),
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
    return ok;
}

// Gulbrandsen 2014 eq 12 and eq 2 exactly as the paper's Appendix A prints them -- the LITERAL k^2, not
// the factored form bsdf.cpp ships -- followed by the textbook complex-arithmetic Fresnel, all in double.
// Deliberately the other implementation of both departures: bsdf.cpp uses (nMax-n)(n-nLow) for k^2 and the
// real-arithmetic unpolarized form, so one comparison against this reference tests both at once. Double is
// what makes the literal k^2 usable here; it is what fails in float32 near r=1, which is why bsdf.cpp
// factors it.
std::complex<double> referenceConductorIor(double r, double g) {
    r = std::clamp(r, 1e-4, 0.9999);   // matches bsdf.cpp's kMinReflectivity/kMaxReflectivity
    g = std::clamp(g, 0.0, 1.0);
    const double sqrtR = std::sqrt(r);
    const double nMin = (1.0 - r) / (1.0 + r);
    const double nMax = (1.0 + sqrtR) / (1.0 - sqrtR);
    const double n = (nMin * g) + ((1.0 - g) * nMax);
    const double k2 = ((((n + 1.0) * (n + 1.0)) * r) - ((n - 1.0) * (n - 1.0))) / (1.0 - r);
    return {n, std::sqrt(std::max(0.0, k2))};
}

double referenceConductorFresnelAt(const std::complex<double>& eta, double cosTheta) {
    const double c = std::clamp(cosTheta, 0.0, 1.0);
    const std::complex<double> cosThetaT = std::sqrt(1.0 - ((1.0 - (c * c)) / (eta * eta)));
    const std::complex<double> rParallel = ((eta * c) - cosThetaT) / ((eta * c) + cosThetaT);
    const std::complex<double> rPerpendicular = (c - (eta * cosThetaT)) / (c + (eta * cosThetaT));
    return 0.5 * (std::norm(rParallel) + std::norm(rPerpendicular));
}

double referenceConductorFresnel(double r, double g, double cosTheta) {
    return referenceConductorFresnelAt(referenceConductorIor(r, g), cosTheta);
}

// Exact unpolarized dielectric Fresnel, entering orientation, in double -- the reference dielectricFresnelAvg's rational fit is measured against below.
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

// Cosine-weighted average Fresnel, 2*int_0^1 F(mu)*mu dmu, by composite Simpson. The integrand is analytic on [0,1] for every (n, k) the Gulbrandsen domain reaches, so the O(h^4) error at this width is ~1e-13 -- ten orders under the tolerances it is used to police, and doubling the panel count moves no printed digit.
// The mu=0 endpoint contributes nothing (the mu weight kills it) whatever F does there, which is what keeps the rule insensitive to the grazing limit both Fresnels approach differently.
template <typename Fresnel>
double cosineAverageFresnel(Fresnel fresnel) {
    constexpr int kPanels = 4000;   // even, for Simpson
    const double h = 1.0 / kPanels;
    double sum = fresnel(1.0);
    for (int i = 1; i < kPanels; ++i) {
        const double mu = i * h;
        sum += (i % 2 == 1 ? 4.0 : 2.0) * fresnel(mu) * mu;
    }
    return 2.0 * (h / 3.0) * sum;
}

// The instrument the suite never had. F_avg attenuates every repeated bounce of the Kulla-Conty multiple-scattering lobe, and NO energy test in this file can see an error in it: checkWhiteFurnaceTwoSided runs at f0=1, where Schlick's mean, the quadrature rule and the truth all agree to 1e-4, and checkFurnace's coloured-metal rows are upper-bound-only and so blind to a loss. Measuring F_avg directly is the honest fix; testing around it is not available, because a two-sided grey-conductor furnace has no closed form.
// Truth is this file's own independent reference -- literal k^2, complex arithmetic, double -- not the shipped Fresnel, so a transcription error in bsdf.cpp shows up here instead of cancelling. The two inversions agree to 4e-12 (checkConductorFresnel pins that), twelve orders under the tolerance, so what this measures is the quadrature rule's own fit error and nothing else.
// Conductor tolerance is the fit's measured bound (max 4.0e-4 over the full clamped domain, worst near r=0.48) plus headroom. Karis' Schlick mean fails it by 216x at r=0.255 g=1, which is the point: reverting bsdf.cpp to schlickFresnelAvg must fail this test, and must NOT fail at g=1 r=1 where the old suite did all its conductor energy checking.
bool checkAverageFresnel() {
    constexpr double kConductorTolerance = 5e-4;
    // dielectricFresnelAvg's own long-standing claim, now asserted rather than only written down. Loose next to the conductor rule because it is a two-constant rational fit, not a fitted quadrature. Headroom is thin and deliberately not widened: measured worst is 0.00597 at ior 3.0 and 0.00588 at ior 1.1, ~9% under the bound, so any refit or tolerance change trips this rather than passing silently.
    constexpr double kDielectricTolerance = 0.0065;
    const std::array<double, 12> reflectivities = {1e-4, 0.01, 0.1,  0.25, 0.4,  0.48,
                                                    0.555, 0.7, 0.85, 0.95, 0.99, 1.0};
    const std::array<double, 8> edgeTints = {0.0, 0.1, 0.25, 0.5, 0.6, 0.75, 0.9, 1.0};
    // dielectricFresnelAvg's stated range, plus ior=1 where an index-matched interface reflects nothing and the fit must return exactly 0 -- the collapse Karis' mean of coatF0 does not have (it returns 1/21 there).
    const std::array<double, 7> iors = {1.0, 1.1, 1.33, 1.5, 2.0, 2.5, 3.0};

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
            const glm::vec3 rule = engine::scene::conductorFresnelAvg(
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
        const double fit = engine::scene::dielectricFresnelAvg(static_cast<float>(ior));
        const double error = std::abs(fit - truth);
        std::cout << "    ior " << ior << "   fit " << fit << "   truth " << truth << "   error "
                  << error << '\n';
        if (!(error <= kDielectricTolerance)) {
            std::cerr << "bsdf_validate: FAILED dielectric F_avg at ior=" << ior << " fit=" << fit
                      << " vs quadrature " << truth << " (error " << error << ", tolerance "
                      << kDielectricTolerance << ")\n";
            ok = false;
        }
    }
    return ok;
}

// Cauchy dispersion (bsdf.cpp's cauchyIor), asserted against the contract it exists to satisfy rather than
// against a restatement of its own formula: (ior, abbe) means "index n_d at the d line, and an Abbe number
// V_d = (n_d-1)/(n_F-n_C)", so those two identities ARE the specification, and a test that recomputed
// A + B/lambda^2 here would only prove the expression was typed twice.
// Nothing else in the suite can see an error here. Every render-based transmissive check runs at ior 1.0,
// where B is exactly 0 and dispersion is identically the no-op -- deliberately, since that is what lets
// checkBeerLambert isolate the channel estimator from the refraction geometry. This check carries the
// whole proof of the optics.
bool checkCauchyDispersion() {
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
        const float nD = engine::scene::cauchyIor(glass.iorD, glass.abbe, kLambdaDNm);
        const float nF = engine::scene::cauchyIor(glass.iorD, glass.abbe, kLambdaFNm);
        const float nC = engine::scene::cauchyIor(glass.iorD, glass.abbe, kLambdaCNm);
        const float measuredAbbeDifference = nF - nC;
        // The Abbe number's definition, rearranged. Computed here from the authored inputs alone.
        const float expectedAbbeDifference = (glass.iorD - 1.0F) / glass.abbe;

        const float nRed = engine::scene::cauchyIor(glass.iorD, glass.abbe,
                                                     engine::scene::kRgbWavelengthsNm.x);
        const float nGreen = engine::scene::cauchyIor(glass.iorD, glass.abbe,
                                                       engine::scene::kRgbWavelengthsNm.y);
        const float nBlue = engine::scene::cauchyIor(glass.iorD, glass.abbe,
                                                      engine::scene::kRgbWavelengthsNm.z);

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
        for (float lambda : {kLambdaFNm, engine::scene::kRgbWavelengthsNm.z, kLambdaDNm,
                             engine::scene::kRgbWavelengthsNm.x, kLambdaCNm}) {
            const float n = engine::scene::cauchyIor(glass.iorD, 0.0F, lambda);
            if (n != glass.iorD) {
                std::cerr << "bsdf_validate: FAILED abbe=0 no-op at ior " << glass.iorD << " lambda "
                          << lambda << " -- returned " << n
                          << ", expected the authored ior bit-for-bit.\n";
                ok = false;
            }
        }
    }
    return ok;
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
bool checkConductorFresnel() {
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
                               engine::scene::eonAlbedoInversion(glm::vec3(1.0F), 0.0F),
                               /*transmissionTint=*/glm::vec3(1.0F)};
        };
        // R(theta=0) == r, independent of edgeTint: wo == wi == +z puts woDotNh at exactly 1.
        const glm::vec3 normalIncidence(0.0F, 0.0F, 1.0F);
        for (float roughness : roughnesses) {
            std::array<float, 5> atNormal{};
            for (std::size_t i = 0; i < edgeTints.size(); ++i) {
                atNormal[i] = maxChannel(engine::scene::evaluateBsdf(
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
                    maxChannel(engine::scene::evaluateBsdf(params(roughness, glm::vec3(1.0F)), wo, wi));
                const float atBlack =
                    maxChannel(engine::scene::evaluateBsdf(params(roughness, glm::vec3(0.0F)), wo, wi));
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
                        engine::scene::evaluateBsdf(params(roughness, glm::vec3(edgeTint)), wo, wi));
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
                maxChannel(engine::scene::evaluateBsdf(params(0.05F, glm::vec3(1.0F)), wo, wi));
            const float atBlack =
                maxChannel(engine::scene::evaluateBsdf(params(0.05F, glm::vec3(0.0F)), wo, wi));
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
    return ok;
}

// The specular lobe's closed form at the mirrored pair, in double: nh is exactly +z there, so sin(theta_h) is 0 and the GGX denominator collapses to alpha^2, giving D = 1/(pi*alpha^2) without evaluating the shipped D at all.
// G2 is Smith height-correlated with both lambdas at the same cosine, matching bsdf.cpp's smithLambda/smithG2.
double specularGeometry(double alpha, double cosine) {
    constexpr double kPiDouble = 3.14159265358979324;
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
// The grazing end stops at cos=0.02, where singleScatter's 4*muO*muI is 1.6e-3 -- three orders above its own 1e-6 floor, so the reference K's unfloored 4*c^2 is the divisor the lobe actually used and the row is a real comparison rather than a clamped one.
bool checkDielectricFresnel() {
    // Float32 round-off in the shipped lobe against a double reference, nothing else: M is not resolvable at these roughnesses. Measured worst 2.57e-7 at ior 1.5, roughness 0.1, cos 0.08, plus ~17% headroom. A fit-shaped error cannot hide under a bound this tight -- Schlick misses by 0.02 at ior 1.5168 cos 0.5, five orders above it.
    constexpr double kFresnelTolerance = 3e-7;
    // Normal incidence is an exact identity, not a fit: referenceDielectricFresnel(1, n) is ((n-1)/(n+1))^2 with both polarisations equal, and nh, woDotNh and G2 are all exactly 1 there, so the only residual is float32 evaluation of F itself. Measured worst 2.54e-8 at ior 2.5, plus ~18% headroom.
    constexpr double kNormalIncidenceTolerance = 3e-8;
    constexpr float kMinAlpha = 0.02F * 0.02F;   // bsdf.cpp's roughness floor, mirrored so K uses the alpha the lobe actually used
    const std::array<double, 4> iors = {1.1, 1.5, 1.5168, 2.5};   // 1.5168 is glass.json's own N-BK7 value
    const std::array<float, 3> roughnesses = {0.02F, 0.05F, 0.1F};
    const std::array<float, 9> cosines = {1.0F, 0.9F, 0.7F, 0.5F, 0.35F, 0.25F, 0.15F, 0.08F, 0.02F};

    bool ok = true;
    int rowsChecked = 0;
    std::cout << "bsdf_validate: dielectric Fresnel, absolute lobe magnitude vs exact unpolarized\n";
    std::cout << "  ior      rough   worst |err|   F(cos=0.02) measured / exact\n";
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
                                     engine::scene::eonAlbedoInversion(baseColor, 0.0F),
                                     /*transmissionTint=*/glm::vec3(1.0F)};
            double worstError = 0.0;
            double previous = -1.0;
            double atGrazing = 0.0;
            for (float cosine : cosines) {
                const float sine = std::sqrt(std::max(0.0F, 1.0F - (cosine * cosine)));
                const glm::vec3 wo(sine, 0.0F, cosine);
                const glm::vec3 wi(-sine, 0.0F, cosine);
                const double measured =
                    static_cast<double>(engine::scene::evaluateBsdfSplit(params, wo, wi).specular.x) /
                    specularGeometry(alpha, cosine);
                const double expected = referenceDielectricFresnel(cosine, ior);
                const double tolerance = cosine == 1.0F ? kNormalIncidenceTolerance : kFresnelTolerance;
                ++rowsChecked;
                worstError = std::max(worstError, std::abs(measured - expected));
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
    // No conditioning skip anywhere above, unlike checkConductorFresnel's normalised ratio: every row of the sweep is compared, and the count is printed so that stays visible.
    std::cout << "  dielectric Fresnel: " << rowsChecked << " points vs reference\n";
    return ok;
}

// Helmholtz reciprocity: f(wo->wi) == f(wi->wo). The continuous lobes are symmetric by construction after the directional-albedo diffuse coupling landed: D and G2 are symmetric, Fresnel is evaluated at the shared half-vector, and both the coupling and the multiple-scattering lobe are products of matching wo-side and wi-side factors, so this is an equality to float precision, not a statistical bound.
// It fails hard on the pre-coupling code, where the diffuse lobe carried (1 - F(mu_o)) alone; not an energy error (the furnace passed throughout) but a misdistribution across view/light geometry, and the blocker for every bidirectional transport algorithm (BDPT, VCM, light tracing, photon mapping), all of which require symmetric f.
// Transmission is excluded (transmissionFactor=0, both cosines positive): radiance transport across a refracting interface is genuinely non-symmetric, so f(wo->wi)==f(wi->wo) is the wrong invariant there; the eta^2-corrected one it does satisfy lives in checkTransmissionReciprocity below.
bool checkReciprocity() {
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
                        const glm::vec3 forward = engine::scene::evaluateBsdf(params, wo, wi);
                        const glm::vec3 reverse = engine::scene::evaluateBsdf(params, wi, wo);
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
    return ok;
}

// eta^2-corrected reciprocity for the transmission lobe: f_t(wo->wi)*eta_wi^2 == f_t(wi->wo)*eta_wo^2. Every term in evaluateTransmissionLobe is symmetric under the swap except denom = (wo.h) + etaR*(wi.h), which the reversed frame rescales by etaI/etaT; squared, that is exactly the eta ratio above. With wo outside and wi inside it reads f(wo->wi)*ior^2 == f(wi->wo).
// Catches a misplaced etaR^2, a flipped denom orientation or an un-flipped ht -- O(1) errors (a stray eta^2 is 2.25x or 0.44x at ior 1.5) invisible to the furnace and round-trip tests, which assert only totals and in which the two sides' errors cancel.
// SINGLE SCATTER ONLY, permanently -- not a symptom of a fixable bug. multiScatterShape (bsdf.cpp) now looks up escapeWi at the correct per-branch eta (reciprocal for a transmitted wi, matching wo's orientation for a reflected one, each paired with its own averageEscapeAlbedo normalisation), which restores the total-energy identity (checkTransmissiveEnergyBalance sweeps this at roughness up to 1.0) but does not and cannot make the multi-scatter transmit lobe itself reciprocal: its numerator is symmetric under the wo/wi swap but its denominator (deficitAvg, tied to each side's own physical eta orientation) is not, which is inherent to transmissive multiple scattering, not an implementation gap. Known limitation (README sec. 4): it blocks bidirectional transport through rough glass, not this unidirectional integrator. Confirmed empirically: extending this sweep to roughness 0.20 still passes (the multi-scatter term stays under kMinDeficit there), but 0.40 fails hard (up to 5x forward/reverse mismatch) for exactly this reason -- do not chase that by widening the sweep.
// Isolated with no new accessor by staying under bsdf.cpp's kMinDeficit, where the multiple-scattering term switches itself off: 1 - escapeAvg measures 1.4e-4 at roughness 0.10 and 1.8e-3 at 0.20 against a 1e-3 gate, so 0.15 upward is not safe. Both roughnesses stay above kSmoothAlpha or there is no continuous lobe to test at all.
// wi is CONSTRUCTED, not sampled: at these roughnesses the lobe is a fraction of a degree wide, so an arbitrary far-side direction returns zero on both sides and the check passes having tested nothing. Refract wo through the macro normal, then perturb by a multiple of alpha for off-peak pairs; the non-zero-pair count is asserted for the same reason.
// transmissionFactor is pinned at 1.0. Between 0 and 1 the entering side scales the lobe by it and the exiting side by 1.0 -- a modelling asymmetry (inside the medium there is no substrate to withhold anything), not a Jacobian error, so sweeping it would test the convention rather than the invariant.
bool checkTransmissionReciprocity() {
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
                                     engine::scene::eonAlbedoInversion(glm::vec3(1.0F), 0.0F),
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
                            engine::scene::evaluateBsdf(params, wo, wi) * ior * ior;
                        const glm::vec3 reverse = engine::scene::evaluateBsdf(params, wi, wo);
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
    return ok;
}

// Round trip through a transmissive interface: sampleBsdf applies a non-symmetric eta^2 compression on refraction (Veach 1997 sec. 5.2), entering scales by (1/ior)^2, exiting by ior^2, so a ray that enters and leaves the same surface must lose no net energy.
// checkFurnace tests each side separately against a per-side bound (1.0 entering, ior^2 exiting), which passes even if the two factors do not actually cancel; this asserts the invariant bsdf.cpp's own comment claims.
bool checkTransmissionRoundTrip() {
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
    return ok;
}

}  // namespace

int main() {
    const bool pdfOk = checkPdfNormalization();
    const bool densityOk = checkSampleDensityConsistency();
    const bool furnaceOk = checkFurnace();
    const bool whiteFurnaceOk = checkWhiteFurnaceTwoSided();
    const bool eonDiffuseOk = checkEonDiffuseFurnace();
    const bool eonInversionOk = checkEonAlbedoInversion();
    const bool indexMatchedCoatOk = checkIndexMatchedCoat();
    const bool transmissiveEnergyOk = checkTransmissiveEnergyBalance();
    const bool transmissionTintOk = checkTransmissionTint();
    const bool conductorFresnelOk = checkConductorFresnel();
    const bool dielectricFresnelOk = checkDielectricFresnel();
    const bool averageFresnelOk = checkAverageFresnel();
    const bool dispersionOk = checkCauchyDispersion();
    const bool reciprocityOk = checkReciprocity();
    const bool transmissionReciprocityOk = checkTransmissionReciprocity();
    const bool roundTripOk = checkTransmissionRoundTrip();

    if (!pdfOk || !densityOk || !furnaceOk || !whiteFurnaceOk || !eonDiffuseOk || !eonInversionOk || !indexMatchedCoatOk ||
        !transmissiveEnergyOk || !transmissionTintOk || !conductorFresnelOk || !dielectricFresnelOk || !averageFresnelOk || !dispersionOk ||
        !reciprocityOk || !transmissionReciprocityOk || !roundTripOk) {
        std::cerr << "bsdf_validate: FAILED\n";
        return EXIT_FAILURE;
    }
    std::cout << "bsdf_validate: PASSED\n";
    return EXIT_SUCCESS;
}
