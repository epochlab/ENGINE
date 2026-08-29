// Standalone correctness check for engine::scene::bsdf (bsdf.h): verifies the combined specular+diffuse pdf never integrates to MORE than the total lobe-selection mass (upper bound only, deliberately: VNDF reflection sampling is not normalized over the hemisphere -- samples reflecting below the horizon are discarded -- so the true integral is the horizon-clipped mass, which has no closed form and is measured instead by the white furnace test below), and runs a furnace test (uniform incident radiance from every direction, including through transmission) via BSDF importance sampling -- must never reflect/transmit more energy than received. Same standalone-CLI convention as embree_validate.cpp/furnace_test.cpp: no test framework, non-zero exit on failure.

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <random>

#include <glm/glm.hpp>

#include "engine/scene/bsdf.h"
#include "engine/scene/sampler.h"

namespace {

using engine::scene::BsdfParams;

constexpr float kPi = 3.14159265F;

BsdfParams makeParams(float roughness, float metallic, float transmissionFactor) {
    const glm::vec3 baseColor(1.0F);  // worst case: full white albedo
    const glm::vec3 f0 = glm::mix(glm::vec3(0.04F), baseColor, metallic);
    return BsdfParams{baseColor, metallic, roughness, f0, 1.5F, transmissionFactor};
}

// A colored/dark conductor (f0=0.5, not the white f0=baseColor=1 makeParams gives at metallic=1) -- specifically the case that caught evaluateDiffuseLobe's pdf-gating bug: a white f0 clamps specularProb to 0.95, leaving only 5% diffuse selection mass to hide a diffuse-pdf bug under this test's tolerance; f0=0.5 leaves ~50%, large enough for the same bug to fail loudly.
BsdfParams makeColoredMetalParams(float roughness) {
    return BsdfParams{glm::vec3(1.0F), 1.0F, roughness, glm::vec3(0.5F), 1.5F, 0.0F};
}

// Uniform-solid-angle hemisphere sample (PBRT-style inversion, same as furnace_test.cpp): z=u1, r=sqrt(1-u1^2), phi=2*pi*u2, pdf=1/(2*pi).
glm::vec3 sampleUniformHemisphere(std::mt19937& rng) {
    std::uniform_real_distribution<float> unit(0.0F, 1.0F);
    const float cosTheta = unit(rng);
    const float sinTheta = std::sqrt(std::max(0.0F, 1.0F - (cosTheta * cosTheta)));
    const float phi = 2.0F * kPi * unit(rng);
    return {sinTheta * std::cos(phi), sinTheta * std::sin(phi), cosTheta};
}

// Integrates pdfBsdf(wo, .) over the hemisphere via uniform-hemisphere Monte Carlo; for an opaque material (transmissionFactor=0) this must not exceed 1.0 -- all sampling probability mass is in the two continuous lobes the pdf covers. Upper-bound only, not a tight equality check: uniform hemisphere sampling under-samples a sharp GGX lobe at low roughness (same accepted limitation as furnace_test.cpp's checkPunctualSweep), so a low reading there is expected noise, not a bug -- only exceeding 1.0 would indicate a real double-counted pdf.
bool checkPdfNormalization() {
    std::mt19937 rng(7);
    constexpr int kSampleCount = 200000;
    constexpr float kTolerance = 0.05F;
    constexpr float kUniformPdf = 1.0F / (2.0F * kPi);
    const std::array<float, 4> roughnesses = {0.05F, 0.25F, 0.5F, 1.0F};
    const std::array<float, 2> metallics = {0.0F, 1.0F};
    const std::array<float, 4> ndotVs = {0.2F, 0.6F, 1.0F, -0.6F};

    bool ok = true;
    for (float roughness : roughnesses) {
        for (float metallic : metallics) {
            for (float ndotV : ndotVs) {
                const BsdfParams params = makeParams(roughness, metallic, 0.0F);
                const glm::vec3 wo(std::sqrt(std::max(0.0F, 1.0F - (ndotV * ndotV))), 0.0F, ndotV);
                double integral = 0.0;
                for (int i = 0; i < kSampleCount; ++i) {
                    glm::vec3 wi = sampleUniformHemisphere(rng);
                    // pdfBsdf mirrors wi into wo's hemisphere, so for a below-surface wo the density
                    // over the +z hemisphere is identically zero. Integrating the +z hemisphere there
                    // measured 0 and passed the <= 1.0 assertion vacuously -- the exiting-side rows
                    // tested nothing at all. Flip the sampled hemisphere to match wo's side.
                    if (ndotV < 0.0F) {
                        wi.z = -wi.z;
                    }
                    integral += engine::scene::pdfBsdf(params, wo, wi) / kUniformPdf;
                }
                integral /= kSampleCount;
                if (integral > 1.0 + kTolerance) {
                    std::cerr << "bsdf_validate: FAILED pdf normalization UPPER bound at roughness="
                              << roughness << " metallic=" << metallic << " ndotV=" << ndotV
                              << " integral=" << integral << " (expected <= 1.0)\n";
                    ok = false;
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

// Furnace test through sampleBsdf: uniform radiance L0=1 from every direction (both hemispheres -- transmission can receive from the far side), estimator Lo = mean(throughputWeight) since throughputWeight already folds in f*cosTheta/pdf. ndotV sweep includes negative values (woLocal.z<0, the exiting side of a transmissive dielectric) and a value past the ior=1.5 critical angle (~41.8deg, cosTheta~0.745) to force total internal reflection. Energy bound: 1.0 (L0) everywhere EXCEPT the exiting side (ndotV<0) of a transmissive material below the critical angle, where sampleBsdf's eta^2 non-symmetric radiance-compression factor (Veach 1997 sec. 5.2 -- see bsdf.cpp's transmission branch) legitimately raises Lo above L0: L/n^2 is the invariant along a ray, so radiance increases going from a denser medium (ior=1.5, inside) into a rarer one (1.0, outside) by up to ior^2. The naive Lo<=1 bound only holds for eta==1 interfaces (pure reflection) or the entering side, where this same factor is < 1 -- exactly compensating so a round trip through the surface loses no net energy.
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
                    if (maxLo > energyBound + kTolerance) {
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
            if (maxLo > 1.0F + kTolerance) {
                std::cerr << "bsdf_validate: FAILED colored-metal furnace test at roughness="
                          << roughness << " ndotV=" << ndotV << " Lo=" << maxLo
                          << " (expected <= 1.0)\n";
                ok = false;
            }
        }
    }
    return ok;
}

// TWO-SIDED white furnace. A white, non-absorbing surface under uniform L0=1 radiance must return
// exactly 1.0: every photon it receives leaves again. checkFurnace above only ever asserts Lo <= bound,
// so it cannot see energy LOSS -- which is the failure mode this BSDF actually has. Restricted to the
// cases where 1.0 is the analytically correct answer: white base colour, no transmission, entering side.
// A coloured conductor (f0 = 0.5) legitimately absorbs and has no closed-form expectation, so it stays
// upper-bound-only in checkFurnace.
//
// Single-scatter GGX loses the energy smithG2 masks away (Heitz, Hanika, d'Eon, Dachsbacher 2016) -- a
// white conductor at roughness 1.0 measured 0.307 here, returning under a third of the light it received.
// Kulla-Conty multiple-scattering compensation plus the directional-albedo diffuse coupling (bsdf.cpp)
// return it, so 1.0 is now a CORRECTNESS TARGET, not a regression baseline: both bounds are the same
// tolerance and a shortfall is a bug, not an accepted approximation.
//
// Half the rows sit deliberately OFF the 32x32 albedo table's grid. At an off-grid point the measured
// value is E_true + (1 - E_interpolated), so these rows test the table's interpolation error directly --
// which is why bsdf.cpp needs no public accessor for the table itself.
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
            if (minChannel(value) < 1.0F - kTolerance || maxChannel(value) > 1.0F + kTolerance) {
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

// Mean throughput through sampleBsdf with every transmitted draw converted back from radiance to ENERGY.
// sampleBsdf applies the non-symmetric eta^2 radiance compression on refraction (Veach 1997 sec. 5.2), so a
// transmitted sample carries radiance and a raw mean is bounded by ior^2, not 1.0 -- which is why
// checkFurnace can only assert an upper bound on its transmissive rows and never sees energy LOSS there.
// Dividing those draws by eta^2 puts every sample in one domain with an analytic answer.
// LobeType::Transmission is exactly the far-hemisphere draws, delta and rough alike.
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

// TWO-SIDED energy balance for a TRANSMISSIVE interface -- the counterpart to checkWhiteFurnaceTwoSided,
// which is restricted to "no transmission, entering side" because those are the only rows where 1.0 is
// correct in the radiance domain. In the energy domain 1.0 is correct everywhere: a white, non-absorbing
// interface reflects, refracts, or hands the rest to the diffuse substrate, and the multiple-scattering
// lobes return what smithG2 masked. Nothing is absorbed at any roughness, side or transmissionFactor.
//
// Gates two failure modes the radiance-domain checks structurally cannot see: multiple-scattering
// compensation delivered over the refraction-reachable cone only rather than the whole far hemisphere,
// and a transmission lobe whose value drops transmissionFactor or (1-metallic) while its selection
// probability keeps them -- the factors cancel out of throughput, so only an absolute bound catches it.
// metallic=1 rows cover a conductor, which must transmit nothing however its transmissionFactor is set.
bool checkTransmissiveEnergyBalance() {
    constexpr int kSampleCount = 200000;
    // Same tolerance as the opaque white furnace -- 1.0 is a correctness target, not a baseline. Residual
    // is albedo-table interpolation error, worst across the TIR boundary where the transmitted channel
    // steps in mu and eta=1.5 falls between two table slices.
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
                    if (minChannel(lo) < 1.0F - kTolerance || maxChannel(lo) > 1.0F + kTolerance) {
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

// Helmholtz reciprocity: f(wo->wi) == f(wi->wo). The continuous lobes are symmetric by construction after
// the directional-albedo diffuse coupling landed -- D and G2 are symmetric, Fresnel is evaluated at the
// shared half-vector, and both the coupling and the multiple-scattering lobe are products of matching
// wo-side and wi-side factors. So this is an equality to float precision, not a statistical bound.
//
// It fails hard on the pre-coupling code, where the diffuse lobe carried (1 - F(mu_o)) alone. Not an
// energy error -- the furnace passed throughout -- but a misdistribution across view/light geometry, and
// the blocker for every bidirectional transport algorithm (BDPT, VCM, light tracing, photon mapping),
// all of which require symmetric f.
bool checkReciprocity() {
    constexpr float kRelativeTolerance = 1e-4F;
    const std::array<float, 4> roughnesses = {0.05F, 0.25F, 0.5F, 1.0F};
    const std::array<float, 3> metallics = {0.0F, 0.5F, 1.0F};
    const std::array<float, 4> cosines = {1.0F, 0.7F, 0.4F, 0.15F};

    bool ok = true;
    for (float roughness : roughnesses) {
        for (float metallic : metallics) {
            const BsdfParams params = makeParams(roughness, metallic, 0.0F);
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
                    if (maxChannel(glm::abs(forward - reverse)) >
                        kRelativeTolerance * std::max(scale, 1e-4F)) {
                        std::cerr << "bsdf_validate: FAILED reciprocity at roughness=" << roughness
                                  << " metallic=" << metallic << " muO=" << muA << " muI=" << muB
                                  << " f(wo->wi)=" << forward.x << " f(wi->wo)=" << reverse.x << '\n';
                        ok = false;
                    }
                }
            }
        }
    }
    return ok;
}

// Round trip through a transmissive interface. sampleBsdf applies a non-symmetric eta^2 radiance
// compression on refraction (Veach 1997 sec. 5.2): entering scales by (1/ior)^2, exiting by ior^2, so a
// ray that enters and leaves the same surface must lose no net energy. checkFurnace tests each side
// separately against a per-side bound (1.0 entering, ior^2 exiting), which passes even if the two
// factors do not actually cancel -- this asserts the invariant bsdf.cpp's own comment claims.
bool checkTransmissionRoundTrip() {
    constexpr int kSampleCount = 200000;
    // Not tight to 1.0: each side's furnace value also contains that interface's REFLECTED
    // lobe, so the product carries a Fresnel cross-term that grows toward grazing (measured
    // 1.027 at normal incidence, 1.058 at 60 degrees). The band still has large discriminating
    // power -- factors that compounded rather than cancelled would land near ior^2 = 2.25, and
    // ones that under-cancelled near 1/2.25 = 0.44.
    constexpr float kTolerance = 0.08F;
    constexpr float kIorRoundTrip = 1.5F;  // matches makeParams
    const std::array<float, 3> ndotVs = {1.0F, 0.8F, 0.5F};

    bool ok = true;
    std::uint32_t seed = 4000;
    for (float ndotV : ndotVs) {
        ++seed;
        // Snell-correct pairing. A ray entering at cos(thetaI) travels inside the medium at
        // cos(thetaT), sin(thetaT) = sin(thetaI)/ior -- so the exiting leg must be probed at thetaT,
        // not at thetaI. Reusing thetaI puts the exit past the critical angle (cos ~0.745 at ior 1.5)
        // where the interface totally internally reflects and no round trip exists at all.
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
        // Two-sided deliberately. An upper bound alone catches the eta^2 factors COMPOUNDING but is
        // blind to them under-cancelling, which loses energy on every round trip through glass -- the
        // same one-sided blind spot the white furnace test above exists to close.
        if (roundTrip > 1.0F + kTolerance || roundTrip < 1.0F - kTolerance) {
            std::cerr << "bsdf_validate: FAILED transmission round trip at ndotV=" << ndotV
                      << " -- entering " << entering << " x exiting " << exiting << " = " << roundTrip
                      << "; the eta^2 radiance-compression factors must cancel over a round trip "
                         "(expected 1.0 +/- " << kTolerance << ").\n";
            ok = false;
        }
    }
    return ok;
}

// Regression check for the DirectSpecular/IndirectSpecular AOV albedo-leak bug: BsdfSample::rawThroughputWeight for a specular-sampled draw must be independent of baseColor. Drives sampleBsdf with two metallic=0 materials differing only in baseColor (white vs. saturated red) through identically-seeded Samplers -- lobe selection and wi depend only on wo/roughness/metallic/f0/ior (identical between the two), never baseColor, so both draws follow the same branch deterministically; wherever both land on LobeType::SpecularReflection, their rawThroughputWeight must match. Before the fix, rawThroughputWeight for a specular sample was the combined specular+diffuse value, which does depend on baseColor via the diffuse term -- this check would have caught that.
bool checkSpecularRawThroughputIsolation() {
    using engine::scene::LobeType;
    constexpr int kSampleCount = 2000;
    constexpr float kTolerance = 1e-4F;
    const std::array<float, 4> roughnesses = {0.05F, 0.25F, 0.5F, 1.0F};
    const std::array<float, 4> ndotVs = {0.2F, 0.6F, 1.0F, -0.6F};

    bool ok = true;
    int specularSamplesSeen = 0;
    std::uint32_t seed = 0;
    for (float roughness : roughnesses) {
        for (float ndotV : ndotVs) {
            ++seed;
            const BsdfParams white = makeParams(roughness, 0.0F, 0.0F);
            const BsdfParams red{glm::vec3(1.0F, 0.0F, 0.0F), 0.0F, roughness, glm::vec3(0.04F), 1.5F, 0.0F};
            const glm::vec3 wo(std::sqrt(std::max(0.0F, 1.0F - (ndotV * ndotV))), 0.0F, ndotV);
            for (int i = 0; i < kSampleCount; ++i) {
                engine::scene::Sampler samplerWhite(0, 0, i, seed);
                engine::scene::Sampler samplerRed(0, 0, i, seed);
                const std::optional<engine::scene::BsdfSample> sampleWhite =
                    engine::scene::sampleBsdf(white, wo, samplerWhite);
                const std::optional<engine::scene::BsdfSample> sampleRed =
                    engine::scene::sampleBsdf(red, wo, samplerRed);
                if (!sampleWhite.has_value() || !sampleRed.has_value()) {
                    continue;
                }
                if (sampleWhite->type != sampleRed->type) {
                    std::cerr << "bsdf_validate: FAILED specular isolation lobe-selection parity at "
                                 "roughness="
                              << roughness << " ndotV=" << ndotV << " sample=" << i << "\n";
                    ok = false;
                    continue;
                }
                if (sampleWhite->type != LobeType::SpecularReflection) {
                    continue;
                }
                ++specularSamplesSeen;
                const glm::vec3 diff = sampleWhite->rawThroughputWeight - sampleRed->rawThroughputWeight;
                if (std::max({std::fabs(diff.x), std::fabs(diff.y), std::fabs(diff.z)}) > kTolerance) {
                    std::cerr << "bsdf_validate: FAILED specular rawThroughputWeight depends on baseColor "
                                 "at roughness="
                              << roughness << " ndotV=" << ndotV << " white=(" << sampleWhite->rawThroughputWeight.x
                              << "," << sampleWhite->rawThroughputWeight.y << "," << sampleWhite->rawThroughputWeight.z
                              << ") red=(" << sampleRed->rawThroughputWeight.x << ","
                              << sampleRed->rawThroughputWeight.y << "," << sampleRed->rawThroughputWeight.z << ")\n";
                    ok = false;
                }
            }
        }
    }
    if (specularSamplesSeen == 0) {
        std::cerr << "bsdf_validate: FAILED specular isolation check observed zero SpecularReflection samples\n";
        ok = false;
    }
    return ok;
}

}  // namespace

int main() {
    const bool pdfOk = checkPdfNormalization();
    const bool furnaceOk = checkFurnace();
    const bool whiteFurnaceOk = checkWhiteFurnaceTwoSided();
    const bool transmissiveEnergyOk = checkTransmissiveEnergyBalance();
    const bool reciprocityOk = checkReciprocity();
    const bool roundTripOk = checkTransmissionRoundTrip();
    const bool specularIsolationOk = checkSpecularRawThroughputIsolation();

    if (!pdfOk || !furnaceOk || !whiteFurnaceOk || !transmissiveEnergyOk || !reciprocityOk ||
        !roundTripOk || !specularIsolationOk) {
        std::cerr << "bsdf_validate: FAILED\n";
        return EXIT_FAILURE;
    }
    std::cout << "bsdf_validate: PASSED\n";
    return EXIT_SUCCESS;
}
