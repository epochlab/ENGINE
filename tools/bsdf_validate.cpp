// Standalone correctness check for engine::scene::bsdf (bsdf.h): verifies the combined specular+diffuse pdf never integrates to more than the total lobe-selection mass.
// Upper bound only, deliberately: VNDF reflection sampling is not normalized over the hemisphere (samples reflecting below the horizon are discarded), so the true integral is the horizon-clipped mass, which has no closed form and is measured instead by the white furnace test below.
// Also runs a furnace test (uniform incident radiance from every direction, including through transmission) via BSDF importance sampling: must never reflect/transmit more energy than received.
// Same standalone-CLI convention as embree_validate.cpp/furnace_test.cpp: no test framework, non-zero exit on failure.

#include <array>
#include <cmath>
#include <complex>
#include <cstdlib>
#include <iostream>
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
                       /*ior=*/1.5F, transmissionFactor, diffuseRoughness};
}

// A colored/dark conductor (f0=0.5, not the white f0=baseColor=1 makeParams gives at metallic=1): the case that caught evaluateDiffuseLobe's pdf-gating bug.
// A white f0 clamps specularProb to 0.95, leaving only 5% diffuse selection mass to hide a diffuse-pdf bug under this test's tolerance; f0=0.5 leaves ~50%, large enough for the same bug to fail loudly.
BsdfParams makeColoredMetalParams(float roughness, glm::vec3 edgeTint = glm::vec3(1.0F)) {
    return BsdfParams{glm::vec3(1.0F), 1.0F, roughness, glm::vec3(0.5F), edgeTint,
                       /*ior=*/1.5F, /*transmissionFactor=*/0.0F, /*diffuseRoughness=*/0.0F};
}

// Uniform-solid-angle hemisphere sample (PBRT-style inversion, same as furnace_test.cpp): z=u1, r=sqrt(1-u1^2), phi=2*pi*u2, pdf=1/(2*pi).
glm::vec3 sampleUniformHemisphere(std::mt19937& rng) {
    std::uniform_real_distribution<float> unit(0.0F, 1.0F);
    const float cosTheta = unit(rng);
    const float sinTheta = std::sqrt(std::max(0.0F, 1.0F - (cosTheta * cosTheta)));
    const float phi = 2.0F * kPi * unit(rng);
    return {sinTheta * std::cos(phi), sinTheta * std::sin(phi), cosTheta};
}

// Integrates pdfBsdf(wo, .) over the hemisphere via uniform-hemisphere Monte Carlo; for an opaque material (transmissionFactor=0) this must not exceed 1.0, since all sampling probability mass is in the two continuous lobes the pdf covers.
// Upper-bound only, not a tight equality check: uniform hemisphere sampling under-samples a sharp GGX lobe at low roughness (same accepted limitation as furnace_test.cpp's checkPunctualSweep), so a low reading there is expected noise; only exceeding 1.0 indicates a real double-counted pdf.
// diffuseRoughness is swept because at 0 -- the only value this used to test, and the one principled.json ships -- eonUniformMixWeight is pow(0, 0.1) = 0 exactly, so pdfEon degenerates to cltcPdf alone and CLTC itself degenerates to plain cosine. Neither the LTC fit's own normalisation nor the uniform/CLTC one-sample MIS mixture was reached at all. The metallic=1 rows matter most here: metallic zeroes diffuseKd but NOT diffuseProb, so a conductor still carries the full CLTC density with none of its value, and a mis-normalised fit shows up in the mixture denominator rather than in any picture.
bool checkPdfNormalization() {
    std::mt19937 rng(7);
    constexpr int kSampleCount = 200000;
    constexpr float kTolerance = 0.05F;
    constexpr float kUniformPdf = 1.0F / (2.0F * kPi);
    const std::array<float, 4> roughnesses = {0.05F, 0.25F, 0.5F, 1.0F};
    const std::array<float, 2> metallics = {0.0F, 1.0F};
    const std::array<float, 4> ndotVs = {0.2F, 0.6F, 1.0F, -0.6F};
    const std::array<float, 3> diffuseRoughnesses = {0.0F, 0.5F, 1.0F};

    bool ok = true;
    for (float roughness : roughnesses) {
        for (float metallic : metallics) {
            for (float diffuseRoughness : diffuseRoughnesses) {
                for (float ndotV : ndotVs) {
                    const BsdfParams params = makeParams(roughness, metallic, 0.0F, diffuseRoughness);
                    const glm::vec3 wo(std::sqrt(std::max(0.0F, 1.0F - (ndotV * ndotV))), 0.0F, ndotV);
                    double integral = 0.0;
                    for (int i = 0; i < kSampleCount; ++i) {
                        glm::vec3 wi = sampleUniformHemisphere(rng);
                        // pdfBsdf mirrors wi into wo's hemisphere, so for a below-surface wo the density over the +z hemisphere is identically zero.
                        // Integrating the +z hemisphere there measured 0 and passed the <=1.0 assertion vacuously, so the exiting-side rows tested nothing; flip the sampled hemisphere to match wo's side.
                        if (ndotV < 0.0F) {
                            wi.z = -wi.z;
                        }
                        integral += engine::scene::pdfBsdf(params, wo, wi) / kUniformPdf;
                    }
                    integral /= kSampleCount;
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
    return ok;
}

glm::vec3 furnaceLo(const BsdfParams& params, const glm::vec3& wo, int sampleCount, std::uint32_t seed) {
    glm::vec3 accum(0.0F);
    for (int i = 0; i < sampleCount; ++i) {
        engine::scene::Sampler sampler(0, 0, i, seed);
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
// correctness target as the specular sweep. Conductors have no diffuse substrate (diffuseKd is zeroed
// at metallic=1 in computeLobeProbabilities), so this is dielectric-only; roughness is fixed at a
// mid-range value since it is the specular lobe's own parameter, orthogonal to diffuseRoughness.
bool checkEonDiffuseFurnace() {
    constexpr int kSampleCount = 400000;
    constexpr float kTolerance = 0.02F;
    constexpr float kRoughness = 0.5F;
    const std::array<float, 5> diffuseRoughnesses = {0.0F, 0.25F, 0.5F, 0.75F, 1.0F};
    const std::array<float, 3> ndotVs = {1.0F, 0.6F, 0.2F};

    bool ok = true;
    std::uint32_t seed = 20000;
    std::cout << "bsdf_validate: EON diffuse-roughness furnace energy (1.0 = perfectly energy-conserving)\n";
    std::cout << "  diffuseRoughness  ndotV  Lo\n";
    for (float diffuseRoughness : diffuseRoughnesses) {
        for (float ndotV : ndotVs) {
            ++seed;
            const BsdfParams params = makeParams(kRoughness, 0.0F, 0.0F, diffuseRoughness);
            const glm::vec3 wo(std::sqrt(std::max(0.0F, 1.0F - (ndotV * ndotV))), 0.0F, ndotV);
            const glm::vec3 lo = furnaceLo(params, wo, kSampleCount, seed);
            std::cout << "  " << diffuseRoughness << "              " << ndotV << "    "
                      << minChannel(lo) << '\n';
            if (!withinBand(lo, 1.0F, kTolerance)) {
                std::cerr << "bsdf_validate: FAILED EON diffuse furnace energy at diffuseRoughness="
                          << diffuseRoughness << " ndotV=" << ndotV << " Lo=[" << minChannel(lo) << ", "
                          << maxChannel(lo) << "] (expected 1.0 +/- " << kTolerance << ")\n";
                ok = false;
            }
        }
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
        engine::scene::Sampler sampler(0, 0, i, seed);
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
                               /*transmissionFactor=*/0.0F, /*diffuseRoughness=*/0.0F};
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
                                     /*transmissionFactor=*/1.0F, /*diffuseRoughness=*/0.0F};
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
    const bool furnaceOk = checkFurnace();
    const bool whiteFurnaceOk = checkWhiteFurnaceTwoSided();
    const bool eonDiffuseOk = checkEonDiffuseFurnace();
    const bool transmissiveEnergyOk = checkTransmissiveEnergyBalance();
    const bool conductorFresnelOk = checkConductorFresnel();
    const bool averageFresnelOk = checkAverageFresnel();
    const bool reciprocityOk = checkReciprocity();
    const bool transmissionReciprocityOk = checkTransmissionReciprocity();
    const bool roundTripOk = checkTransmissionRoundTrip();

    if (!pdfOk || !furnaceOk || !whiteFurnaceOk || !eonDiffuseOk || !transmissiveEnergyOk ||
        !conductorFresnelOk || !averageFresnelOk || !reciprocityOk || !transmissionReciprocityOk ||
        !roundTripOk) {
        std::cerr << "bsdf_validate: FAILED\n";
        return EXIT_FAILURE;
    }
    std::cout << "bsdf_validate: PASSED\n";
    return EXIT_SUCCESS;
}
