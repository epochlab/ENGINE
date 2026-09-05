// Standalone correctness check: path_tracer.cpp's NEE+MIS combination (Veach & Guibas 1995 power heuristic) against EnvironmentMap's importance sampling, and (below) against LightSet's rectangular-emitter sampling.
// Exercises the exact estimator tracePath uses: NEE via importanceSampleDirection + evaluateBsdf/pdfBsdf, BSDF-sampled env hit via sampleBsdf + EnvironmentMap::pdf, MIS-weighted by the power heuristic.
// Needs no scene geometry/BVH/GPU resources (integrator_validate.cpp covers the full renderPathTraced path, including Material/MeshInstance construction; both are plain data, no GL context required). The quad-light checks below need no BVH either: directionHitsQuad is a closed-form ray/rectangle test.
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
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>

#include "engine/gfx/hdr_image.h"
#include "engine/scene/bsdf.h"
#include "engine/scene/environment_map.h"
#include "engine/scene/light.h"
#include "engine/scene/sampler.h"

namespace {

using engine::scene::BsdfParams;
using engine::scene::EnvironmentMap;
using engine::scene::LightSample;
using engine::scene::LightSet;
using engine::scene::LobeType;
using engine::scene::QuadLight;
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

// Uniform over the FULL sphere (unlike sampleUniformHemisphere above), for the quad-light checks
// below: a rectangle can subtend solid angle on either side of the shading point's tangent plane once
// it's a light rather than a BSDF lobe, so the independent oracle must search the whole sphere.
glm::vec3 sampleUniformSphere(std::mt19937& rng) {
    std::uniform_real_distribution<float> unit(0.0F, 1.0F);
    const float z = 1.0F - (2.0F * unit(rng));
    const float r = std::sqrt(std::max(0.0F, 1.0F - (z * z)));
    const float phi = 2.0F * kPi * unit(rng);
    return {r * std::cos(phi), r * std::sin(phi), z};
}

// Independent of SphericalRectangle's own internal frame: re-derives the ray/rectangle intersection
// directly from the quad's world-space geometry (assumes edge0 perpendicular to edge1, true of every
// rectangle this file authors), so a sign/axis bug in buildSphericalRectangle's local frame cannot
// also be present here by construction. This is the oracle checkQuadLightSolidAngle measures against.
bool directionHitsQuad(const QuadLight& quad, const glm::vec3& p, const glm::vec3& dir) {
    const glm::vec3 normal = glm::normalize(glm::cross(quad.edge0, quad.edge1));
    const float denom = glm::dot(dir, normal);
    if (std::fabs(denom) < 1e-8F) {
        return false;  // parallel to the plane
    }
    const float t = glm::dot(quad.origin - p, normal) / denom;
    if (t <= 0.0F) {
        return false;  // plane is behind p
    }
    const glm::vec3 hit = (p + (t * dir)) - quad.origin;
    const float u = glm::dot(hit, quad.edge0) / glm::dot(quad.edge0, quad.edge0);
    const float v = glm::dot(hit, quad.edge1) / glm::dot(quad.edge1, quad.edge1);
    return u >= 0.0F && u <= 1.0F && v >= 0.0F && v <= 1.0F;
}

// buildSphericalRectangle's solid angle (Girard's theorem on the four internal angles) checked against
// an independent Monte Carlo oracle -- uniform-sphere direction sampling plus directionHitsQuad, which
// shares no code with the analytic formula -- and every direction SphericalRectangle::sample() itself
// draws checked against the same oracle, which catches a bug in the xu/yv inversion even if the scalar
// solid angle above happens to come out right.
bool checkQuadLightSolidAngle() {
    const QuadLight quad{glm::vec3(-0.5F, 1.0F, -0.5F), glm::vec3(1.0F, 0.0F, 0.0F),
                         glm::vec3(0.0F, 0.0F, 1.0F), glm::vec3(1.0F), false};
    const glm::vec3 p(0.0F, 0.0F, 0.0F);
    const std::optional<engine::scene::SphericalRectangle> rect =
        engine::scene::buildSphericalRectangle(quad, p);
    if (!rect.has_value()) {
        std::cerr << "nee_validate: FAILED quad solid angle -- buildSphericalRectangle returned "
                     "nullopt for a valid configuration\n";
        return false;
    }

    constexpr int kSampleCount = 2000000;
    std::mt19937 rng(42);
    int hits = 0;
    for (int i = 0; i < kSampleCount; ++i) {
        if (directionHitsQuad(quad, p, sampleUniformSphere(rng))) {
            ++hits;
        }
    }
    const float hitFraction = static_cast<float>(hits) / static_cast<float>(kSampleCount);
    const float mcSolidAngle = 4.0F * kPi * hitFraction;
    // Binomial standard error on the hit fraction, propagated to solid angle -- the tolerance comes
    // from the estimator's own statistics, not a hand-picked constant.
    const float stderrSolidAngle =
        4.0F * kPi * std::sqrt(hitFraction * (1.0F - hitFraction) / static_cast<float>(kSampleCount));
    const float tolerance = 6.0F * stderrSolidAngle;  // ~6 sigma

    bool ok = true;
    if (std::fabs(mcSolidAngle - rect->solidAngle) > tolerance) {
        std::cerr << "nee_validate: FAILED quad solid angle -- analytic " << rect->solidAngle
                  << " vs Monte Carlo " << mcSolidAngle << " (tolerance " << tolerance << ")\n";
        ok = false;
    }

    std::uniform_real_distribution<float> unit(0.0F, 1.0F);
    constexpr int kDrawCount = 20000;
    for (int i = 0; i < kDrawCount; ++i) {
        const glm::vec2 u(unit(rng), unit(rng));
        const glm::vec3 point = rect->sample(u);
        const glm::vec3 dir = glm::normalize(point - p);
        if (!directionHitsQuad(quad, p, dir)) {
            std::cerr << "nee_validate: FAILED quad solid angle -- sample() at u=(" << u.x << ", "
                      << u.y << ") produced a direction missing the rectangle\n";
            ok = false;
            break;
        }
    }
    return ok;
}

// Same brute-force reference method as referenceLo, restricted to the hemisphere directions the quad
// actually subtends (via directionHitsQuad) rather than a uniform environment -- Lo(wo) = integral
// over the hemisphere of evaluateBsdf(wo,wi) * Le(wi) * cos(wi) dwi, Le being the quad's constant
// radiance where it's visible and 0 elsewhere.
float referenceLoQuad(const BsdfParams& params, const glm::vec3& wo, const QuadLight& quad,
                       const glm::vec3& p, int sampleCount, std::mt19937& rng) {
    constexpr float kUniformPdf = 1.0F / (2.0F * kPi);
    glm::vec3 accum(0.0F);
    for (int i = 0; i < sampleCount; ++i) {
        const glm::vec3 wi = sampleUniformHemisphere(rng);
        if (directionHitsQuad(quad, p, wi)) {
            accum += engine::scene::evaluateBsdf(params, wo, wi) * wi.z * quad.radiance / kUniformPdf;
        }
    }
    return std::max({accum.x, accum.y, accum.z}) / static_cast<float>(sampleCount);
}

// Mirrors misCombinedLo's structure exactly, with LightSet(nullptr, ..., {quad}) in place of the
// uniform environment and directionHitsQuad in place of "always reaches the environment" (a
// BSDF-sampled ray can miss a finite rectangle where it could never miss an infinite environment).
float misCombinedLoQuad(const BsdfParams& params, const glm::vec3& wo, const QuadLight& quad,
                         const glm::vec3& p, int sampleCount, std::uint32_t seed) {
    const std::vector<QuadLight> quads{quad};
    const LightSet lights(nullptr, 0.0F, 1.0F, quads);
    glm::vec3 accum(0.0F);
    for (int i = 0; i < sampleCount; ++i) {
        Sampler sampler(0, 0, i, seed);

        // NEE.
        const std::optional<LightSample> lightSample = lights.sample(p, sampler);
        if (lightSample.has_value() && lightSample->direction.z > 0.0F) {
            const glm::vec3 bsdfValue = engine::scene::evaluateBsdf(params, wo, lightSample->direction);
            const float bsdfPdf = engine::scene::pdfBsdf(params, wo, lightSample->direction);
            if (bsdfPdf > 0.0F && (bsdfValue.x > 0.0F || bsdfValue.y > 0.0F || bsdfValue.z > 0.0F)) {
                const float lightPdf2 = lightSample->pdf * lightSample->pdf;
                const float bsdfPdf2 = bsdfPdf * bsdfPdf;
                const float misWeight = lightPdf2 / (lightPdf2 + bsdfPdf2);
                accum += bsdfValue * lightSample->direction.z * misWeight * lightSample->radiance /
                         lightSample->pdf;
            }
        }

        // BSDF-sampled.
        const std::optional<engine::scene::BsdfSample> sample =
            engine::scene::sampleBsdf(params, wo, sampler);
        if (sample.has_value() && sample->type != LobeType::Transmission &&
            directionHitsQuad(quad, p, sample->wiLocal)) {
            const float bsdfPdf = engine::scene::pdfBsdf(params, wo, sample->wiLocal);
            const float lightPdf = lights.pdfQuad(0, p);
            const float bsdfPdf2 = bsdfPdf * bsdfPdf;
            const float lightPdf2 = lightPdf * lightPdf;
            const float misWeight = bsdfPdf2 / (bsdfPdf2 + lightPdf2);
            accum += sample->throughputWeight * lights.quadRadianceToward(0, sample->wiLocal) * misWeight;
        }
    }
    return std::max({accum.x, accum.y, accum.z}) / static_cast<float>(sampleCount);
}

bool checkQuadLightMisAgreement() {
    constexpr int kSampleCount = 200000;
    constexpr float kTolerance = 0.05F;
    const std::array<float, 3> roughnesses = {0.25F, 0.5F, 1.0F};
    const std::array<float, 2> metallics = {0.0F, 1.0F};
    const std::array<float, 3> ndotVs = {0.2F, 0.6F, 1.0F};

    // Directly overhead: a 1x1 rectangle at height 2, subtending a modest solid angle -- large enough
    // that BSDF sampling alone finds it often enough to converge in this many samples, small enough
    // that light sampling still matters at grazing wo, so both one-strategy estimators (folded into
    // misCombinedLoQuad above) are load-bearing here, not just the combined one.
    const QuadLight quad{glm::vec3(-0.5F, -0.5F, 2.0F), glm::vec3(0.0F, 1.0F, 0.0F),
                         glm::vec3(1.0F, 0.0F, 0.0F), glm::vec3(3.0F), false};
    const glm::vec3 p(0.0F);

    bool ok = true;
    std::uint32_t seed = 1000;
    std::mt19937 referenceRng(7);
    for (float roughness : roughnesses) {
        for (float metallic : metallics) {
            for (float ndotV : ndotVs) {
                ++seed;
                const BsdfParams params = makeParams(roughness, metallic);
                const glm::vec3 wo(std::sqrt(std::max(0.0F, 1.0F - (ndotV * ndotV))), 0.0F, ndotV);

                const float reference = referenceLoQuad(params, wo, quad, p, kSampleCount, referenceRng);
                const float combined = misCombinedLoQuad(params, wo, quad, p, kSampleCount, seed);

                if (std::fabs(combined - reference) > kTolerance * std::max(reference, 0.1F)) {
                    std::cerr << "nee_validate: FAILED quad MIS agreement at roughness=" << roughness
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
    const bool quadSolidAngleOk = checkQuadLightSolidAngle();
    const bool quadMisOk = checkQuadLightMisAgreement();
    if (!envPdfOk || !misOk || !quadSolidAngleOk || !quadMisOk) {
        std::cerr << "nee_validate: FAILED\n";
        return EXIT_FAILURE;
    }
    std::cout << "nee_validate: PASSED\n";
    return EXIT_SUCCESS;
}
