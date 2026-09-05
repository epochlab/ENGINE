// Standalone correctness check: path_tracer.cpp's NEE+MIS combination (Veach & Guibas 1995 power heuristic) against EnvironmentMap's importance sampling.
// Exercises the exact estimator tracePath uses: NEE via importanceSampleDirection + evaluateBsdf/pdfBsdf, BSDF-sampled env hit via sampleBsdf + EnvironmentMap::pdf, MIS-weighted by the power heuristic.
// Needs no scene geometry/BVH/GPU resources (integrator_validate.cpp covers the full renderPathTraced path, including Material/MeshInstance construction; both are plain data, no GL context required).
// Reference: a flat unoccluded surface under a uniform-radiance (L0=1) environment has exact Lo(wo) = integral over the hemisphere of evaluateBsdf(wo,wi)*cos(wi) dwi, computed via independent uniform-hemisphere Monte Carlo (same convention as furnace_test.cpp/bsdf_validate.cpp).
// The MIS-combined estimator mirrors tracePath's single-bounce case (no occlusion, so NEE's shadow ray and the BSDF-sampled continuation both always reach the environment) and must converge to the reference; a mismatch means a double- or under-counting bug in the MIS weighting.
// Restricted to opaque materials (transmissionFactor=0): the delta transmission lobe has no continuous pdf, so it's excluded from evaluateBsdf/NEE and always takes MIS weight 1.0 on the BSDF-sampled side (path_tracer.cpp); no double-counting question there, only in the two continuous lobes this check covers.

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <random>

#include <glm/glm.hpp>

#include "engine/gfx/hdr_image.h"
#include "engine/scene/bsdf.h"
#include "engine/scene/environment_map.h"
#include "engine/scene/sampler.h"

namespace {

using engine::scene::BsdfParams;
using engine::scene::EnvironmentMap;
using engine::scene::LobeType;
using engine::scene::Sampler;

constexpr float kPi = 3.14159265F;

BsdfParams makeParams(float roughness, float metallic) {
    const glm::vec3 baseColor(1.0F);  // worst case: full white albedo
    const glm::vec3 f0 = glm::mix(glm::vec3(0.04F), baseColor, metallic);
    return BsdfParams{baseColor,   metallic, roughness, f0, /*edgeTint=*/glm::vec3(1.0F),
                       /*ior=*/1.5F, /*transmissionFactor=*/0.0F, /*diffuseRoughness=*/0.0F,
                       engine::scene::eonAlbedoInversion(baseColor, 0.0F),
                       /*transmissionTint=*/glm::vec3(1.0F)};
}

glm::vec3 sampleUniformHemisphere(std::mt19937& rng) {
    std::uniform_real_distribution<float> unit(0.0F, 1.0F);
    const float cosTheta = unit(rng);
    const float sinTheta = std::sqrt(std::max(0.0F, 1.0F - (cosTheta * cosTheta)));
    const float phi = 2.0F * kPi * unit(rng);
    return {sinTheta * std::cos(phi), sinTheta * std::sin(phi), cosTheta};
}

// Structured environment: a dim background with one small bright patch, so the luminance CDF has real
// structure to invert (a uniform map makes both marginal and conditional CDFs linear, which would let a
// mis-scaled Jacobian or an off-by-one bin lookup pass unnoticed).
EnvironmentMap makeStructuredEnvironment() {
    engine::gfx::HdrImage image;
    image.width = 64;
    image.height = 32;
    image.rgba.assign(static_cast<std::size_t>(image.width) * static_cast<std::size_t>(image.height) * 4,
                       0.05F);
    for (int y = 8; y < 12; ++y) {
        for (int x = 20; x < 26; ++x) {
            const std::size_t idx =
                ((static_cast<std::size_t>(y) * static_cast<std::size_t>(image.width)) +
                 static_cast<std::size_t>(x)) *
                4;
            image.rgba[idx + 0] = 400.0F;
            image.rgba[idx + 1] = 380.0F;
            image.rgba[idx + 2] = 300.0F;
        }
    }
    return EnvironmentMap(std::move(image));
}

// importanceSampleDirection returns a direction AND the solid-angle density it was drawn with; pdf()
// independently recovers that density from a direction. MIS divides by the first and weights by the
// second, so any disagreement between them silently corrupts every MIS weight in the renderer while
// leaving each function looking individually reasonable. Nothing tested this before.
bool checkEnvPdfConsistency() {
    constexpr int kSampleCount = 20000;
    constexpr float kTolerance = 1e-3F;
    const EnvironmentMap env = makeStructuredEnvironment();
    std::mt19937 rng(1234);
    std::uniform_real_distribution<float> unit(0.0F, 1.0F);

    // Non-zero rotation: the sample path rotates by +angle and the query path by -angle, so a sign slip
    // between them cancels at 0 and only shows up here.
    constexpr float kRotation = 0.7F;
    bool ok = true;
    int worstIndex = -1;
    float worstRelative = 0.0F;
    for (int i = 0; i < kSampleCount; ++i) {
        const EnvironmentMap::EnvSample sample =
            env.importanceSampleDirection(glm::vec2(unit(rng), unit(rng)), kRotation);
        const float queried = env.pdf(sample.direction, kRotation);
        const float relative = std::fabs(queried - sample.pdf) / std::max(sample.pdf, 1e-6F);
        if (relative > worstRelative) {
            worstRelative = relative;
            worstIndex = i;
        }
    }
    if (worstRelative > kTolerance) {
        std::cerr << "nee_validate: FAILED env pdf consistency -- worst relative mismatch "
                  << worstRelative << " at sample " << worstIndex
                  << "; importanceSampleDirection's own pdf and pdf() must agree for the same "
                     "direction, or every MIS weight is wrong.\n";
        ok = false;
    }
    return ok;
}

// Uniform-radiance (L0=1) equirect environment: constant regardless of resolution, but a real image so EnvironmentMap's CDF machinery runs its normal (non-degenerate) path, not the all-black fallback.
EnvironmentMap makeUniformEnvironment() {
    engine::gfx::HdrImage image;
    image.width = 64;
    image.height = 32;
    image.rgba.assign(static_cast<std::size_t>(image.width) * static_cast<std::size_t>(image.height) * 4,
                       1.0F);
    return EnvironmentMap(std::move(image));
}

// Independent ground truth: Lo(wo) = integral_hemisphere evaluateBsdf(wo,wi) * wi.z dwi, L0=1.
float referenceLo(const BsdfParams& params, const glm::vec3& wo, int sampleCount, std::mt19937& rng) {
    constexpr float kUniformPdf = 1.0F / (2.0F * kPi);
    glm::vec3 accum(0.0F);
    for (int i = 0; i < sampleCount; ++i) {
        const glm::vec3 wi = sampleUniformHemisphere(rng);
        accum += engine::scene::evaluateBsdf(params, wo, wi) * wi.z / kUniformPdf;
    }
    return std::max({accum.x, accum.y, accum.z}) / static_cast<float>(sampleCount);
}

// MIS-combined NEE + BSDF-sampled estimator, mirroring path_tracer.cpp's tracePath exactly: power heuristic, no occlusion since there's no geometry here, so both strategies always reach the environment (matching a flat unoccluded surface).
float misCombinedLo(const BsdfParams& params, const glm::vec3& wo, const EnvironmentMap& env,
                     int sampleCount, std::uint32_t seed) {
    glm::vec3 accum(0.0F);
    for (int i = 0; i < sampleCount; ++i) {
        Sampler sampler(0, 0, i, seed);

        // NEE.
        const EnvironmentMap::EnvSample lightSample =
            env.importanceSampleDirection(sampler.next2D(), 0.0F);
        if (lightSample.direction.z > 0.0F) {
            const glm::vec3 bsdfValue = engine::scene::evaluateBsdf(params, wo, lightSample.direction);
            const float bsdfPdf = engine::scene::pdfBsdf(params, wo, lightSample.direction);
            if (bsdfPdf > 0.0F && (bsdfValue.x > 0.0F || bsdfValue.y > 0.0F || bsdfValue.z > 0.0F)) {
                const float lightPdf2 = lightSample.pdf * lightSample.pdf;
                const float bsdfPdf2 = bsdfPdf * bsdfPdf;
                const float misWeight = lightPdf2 / (lightPdf2 + bsdfPdf2);
                accum += bsdfValue * lightSample.direction.z * misWeight / lightSample.pdf;
            }
        }

        // BSDF-sampled.
        const std::optional<engine::scene::BsdfSample> sample = engine::scene::sampleBsdf(params, wo, sampler);
        if (sample.has_value() && sample->type != LobeType::Transmission) {
            const float bsdfPdf = engine::scene::pdfBsdf(params, wo, sample->wiLocal);
            const float lightPdf = env.pdf(sample->wiLocal, 0.0F);
            const float bsdfPdf2 = bsdfPdf * bsdfPdf;
            const float lightPdf2 = lightPdf * lightPdf;
            const float misWeight = bsdfPdf2 / (bsdfPdf2 + lightPdf2);
            accum += sample->throughputWeight * misWeight;  // L0=1 folded in via the uniform environment already
        }
    }
    return std::max({accum.x, accum.y, accum.z}) / static_cast<float>(sampleCount);
}

bool checkMisAgreement() {
    constexpr int kSampleCount = 100000;
    constexpr float kTolerance = 0.05F;
    // Excludes low roughness (e.g. 0.05): uniform-hemisphere sampling under-samples a sharp GGX peak there (same limitation as furnace_test.cpp/bsdf_validate.cpp), biasing the reference low.
    // Those two files only check an upper bound for that reason; this test needs a tight two-sided equality, so it needs roughness values where the reference itself converges reliably.
    const std::array<float, 3> roughnesses = {0.25F, 0.5F, 1.0F};
    const std::array<float, 2> metallics = {0.0F, 1.0F};
    const std::array<float, 3> ndotVs = {0.2F, 0.6F, 1.0F};

    bool ok = true;
    std::uint32_t seed = 0;
    std::mt19937 referenceRng(99);
    const EnvironmentMap env = makeUniformEnvironment();

    for (float roughness : roughnesses) {
        for (float metallic : metallics) {
            for (float ndotV : ndotVs) {
                ++seed;
                const BsdfParams params = makeParams(roughness, metallic);
                const glm::vec3 wo(std::sqrt(std::max(0.0F, 1.0F - (ndotV * ndotV))), 0.0F, ndotV);

                const float reference = referenceLo(params, wo, kSampleCount, referenceRng);
                const float combined = misCombinedLo(params, wo, env, kSampleCount, seed);

                if (std::fabs(combined - reference) > kTolerance * std::max(reference, 0.1F)) {
                    std::cerr << "nee_validate: FAILED MIS agreement at roughness=" << roughness
                              << " metallic=" << metallic << " ndotV=" << ndotV
                              << " reference=" << reference << " misCombined=" << combined << '\n';
                    ok = false;
                }
            }
        }
    }
    return ok;
}

}  // namespace

int main() {
    const bool envPdfOk = checkEnvPdfConsistency();
    const bool misOk = checkMisAgreement();
    if (!envPdfOk || !misOk) {
        std::cerr << "nee_validate: FAILED\n";
        return EXIT_FAILURE;
    }
    std::cout << "nee_validate: PASSED\n";
    return EXIT_SUCCESS;
}
