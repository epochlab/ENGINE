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

#include "pathtracer/gfx/hdr_image.h"
#include "check.h"
#include "fixtures.h"
#include "stats.h"
#include "pathtracer/scene/bsdf.h"
#include "pathtracer/scene/environment_map.h"
#include "pathtracer/scene/light.h"
#include "pathtracer/scene/sampler.h"

namespace {

using pathtracer::scene::BsdfParams;
using pathtracer::scene::EnvironmentMap;
using pathtracer::scene::LightSample;
using pathtracer::scene::LightSet;
using pathtracer::scene::LobeType;
using pathtracer::scene::QuadLight;
using pathtracer::scene::Sampler;

using tools::fixtures::kPi;
using tools::fixtures::makeUniformEnvironment;
using tools::fixtures::referenceLo;
using tools::fixtures::sampleUniformHemisphere;

BsdfParams makeParams(float roughness, float metallic) {
    const glm::vec3 baseColor(1.0F);  // worst case: full white albedo
    const glm::vec3 f0 = glm::mix(glm::vec3(0.04F), baseColor, metallic);
    return BsdfParams{baseColor,   metallic, roughness, f0, /*edgeTint=*/glm::vec3(1.0F),
                       /*ior=*/1.5F, /*transmissionFactor=*/0.0F, /*diffuseRoughness=*/0.0F,
                       pathtracer::scene::eonAlbedoInversion(baseColor, 0.0F),
                       /*transmissionTint=*/glm::vec3(1.0F)};
}

// Structured environment: a dim background with one small bright patch, so the luminance CDF has real
// structure to invert (a uniform map makes both marginal and conditional CDFs linear, which would let a
// mis-scaled Jacobian or an off-by-one bin lookup pass unnoticed).
EnvironmentMap makeStructuredEnvironment(pathtracer::gfx::ScalarType type) {
    constexpr int kWidth = 64;
    constexpr int kHeight = 32;
    std::vector<float> rgba(static_cast<std::size_t>(kWidth) * kHeight * 4, 0.05F);
    for (int y = 8; y < 12; ++y) {
        for (int x = 20; x < 26; ++x) {
            const std::size_t idx = ((static_cast<std::size_t>(y) * kWidth) + static_cast<std::size_t>(x)) * 4;
            rgba[idx + 0] = 400.0F;
            rgba[idx + 1] = 380.0F;
            rgba[idx + 2] = 300.0F;
        }
    }
    return EnvironmentMap(tools::fixtures::makeImageTexture(kWidth, kHeight, rgba, type));
}

// importanceSampleDirection returns a direction AND the solid-angle density it was drawn with; pdf()
// independently recovers that density from a direction. MIS divides by the first and weights by the
// second, so any disagreement between them silently corrupts every MIS weight in the renderer while
// leaving each function looking individually reasonable. Nothing tested this before.
PT_CHECK(environment_pdf_consistency, Fast, Exact) {
    constexpr int kSampleCount = 20000;
    constexpr float kTolerance = 1e-3F;
    // Non-zero rotation: the sample path rotates by +angle and the query path by -angle, so a sign slip
    // between them cancels at 0 and only shows up here.
    constexpr float kRotation = 0.7F;
    constexpr std::array<pathtracer::gfx::ScalarType, 2> kTypes = {pathtracer::gfx::ScalarType::Float32,
                                                               pathtracer::gfx::ScalarType::Float16};
    ctx.plan(static_cast<int>(kTypes.size()));
    for (const pathtracer::gfx::ScalarType type : kTypes) {
        const EnvironmentMap env = makeStructuredEnvironment(type);
        std::mt19937 rng(static_cast<std::mt19937::result_type>(ctx.seed()));
        std::uniform_real_distribution<float> unit(0.0F, 1.0F);
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
        // An agreement between two code paths over the same direction, so this is an exactness assertion with a
        // floating-point tolerance, not a statistical one: the two must agree or every MIS weight is wrong.
        char detail[224];
        std::snprintf(detail, sizeof(detail),
                      "%s: worst relative mismatch %.3e at sample %d, between importanceSampleDirection's own pdf and pdf()",
                      pathtracer::gfx::scalarTypeName(type), static_cast<double>(worstRelative), worstIndex);
        PT_EXPECT(ctx, worstRelative <= kTolerance, detail);
    }
}

// The CDFs must be built from the values the map returns, not the source they were rounded from, or the sampling density stops being proportional to the radiance it weights. Background 1.0 and one texel at 1 + 2^-11, the exact binary16 midpoint above 1.0: round-to-nearest-even stores it as 1.0, so at Float16 the pdf ratio must sit nearer the stored luminance ratio (1) than the source ratio (1 + 2^-11), and at Float32 the reverse. Discriminating by nearest hypothesis needs no tolerance.
PT_CHECK(environment_pdf_tracks_stored_luminance, Fast, Exact) {
    constexpr int kWidth = 64;
    constexpr int kHeight = 32;
    constexpr int kPatchX = 20;
    constexpr int kPatchY = 16;
    constexpr int kBackgroundX = 40;
    const float midpoint = 1.0F + pathtracer::gfx::kHalfUnitRoundoff;
    std::vector<float> rgba(static_cast<std::size_t>(kWidth) * kHeight * 4, 1.0F);
    const std::size_t patch = ((static_cast<std::size_t>(kPatchY) * kWidth) + kPatchX) * 4;
    rgba[patch + 0] = midpoint;
    rgba[patch + 1] = midpoint;
    rgba[patch + 2] = midpoint;

    // Direction through a texel's centre, inverting equirectTexelOf's (u, v) = (phi / 2pi + 1/2, theta / pi).
    const auto centre = [](int x, int y) {
        const float phi = ((((static_cast<float>(x) + 0.5F) / kWidth) - 0.5F) * 2.0F * glm::pi<float>());
        const float theta = ((static_cast<float>(y) + 0.5F) / kHeight) * glm::pi<float>();
        return glm::vec3(std::sin(theta) * std::sin(phi), std::cos(theta), std::sin(theta) * std::cos(phi));
    };
    ctx.plan(2);
    for (const pathtracer::gfx::ScalarType type : {pathtracer::gfx::ScalarType::Float16, pathtracer::gfx::ScalarType::Float32}) {
        const pathtracer::gfx::ImageTexture image = tools::fixtures::makeImageTexture(kWidth, kHeight, rgba, type);
        const float storedRatio = image.texel(kPatchX, kPatchY).g / image.texel(kBackgroundX, kPatchY).g;
        const float sourceRatio = midpoint;
        const EnvironmentMap env(image);
        // Same row, so sin(theta) cancels and the solid-angle pdf ratio is the luminance ratio.
        const float pdfRatio = env.pdf(centre(kPatchX, kPatchY), 0.0F) / env.pdf(centre(kBackgroundX, kPatchY), 0.0F);
        const bool tracksStored = type == pathtracer::gfx::ScalarType::Float16
                                      ? std::fabs(pdfRatio - storedRatio) < std::fabs(pdfRatio - sourceRatio)
                                      : std::fabs(pdfRatio - sourceRatio) < std::fabs(pdfRatio - 1.0F);
        char detail[224];
        std::snprintf(detail, sizeof(detail), "%s: pdf ratio %.9g, stored luminance ratio %.9g, source ratio %.9g",
                      pathtracer::gfx::scalarTypeName(type), static_cast<double>(pdfRatio),
                      static_cast<double>(storedRatio), static_cast<double>(sourceRatio));
        PT_EXPECT(ctx, tracksStored, detail);
    }
}

// MIS-combined NEE + BSDF-sampled estimator, mirroring path_tracer.cpp's tracePath exactly: power heuristic, no occlusion since there's no geometry here, so both strategies always reach the environment (matching a flat unoccluded surface).
float misCombinedLo(const BsdfParams& params, const glm::vec3& wo, const EnvironmentMap& env,
                     int sampleCount, std::uint32_t seed) {
    glm::vec3 accum(0.0F);
    for (int i = 0; i < sampleCount; ++i) {
        Sampler sampler(0, 0, i, sampleCount, seed);

        // NEE.
        const EnvironmentMap::EnvSample lightSample =
            env.importanceSampleDirection(sampler.next2D(), 0.0F);
        if (lightSample.direction.z > 0.0F) {
            const glm::vec3 bsdfValue = pathtracer::scene::evaluateBsdf(params, wo, lightSample.direction);
            const float bsdfPdf = pathtracer::scene::pdfBsdf(params, wo, lightSample.direction);
            if (bsdfPdf > 0.0F && (bsdfValue.x > 0.0F || bsdfValue.y > 0.0F || bsdfValue.z > 0.0F)) {
                const float lightPdf2 = lightSample.pdf * lightSample.pdf;
                const float bsdfPdf2 = bsdfPdf * bsdfPdf;
                const float misWeight = lightPdf2 / (lightPdf2 + bsdfPdf2);
                accum += bsdfValue * lightSample.direction.z * misWeight / lightSample.pdf;
            }
        }

        // BSDF-sampled.
        const std::optional<pathtracer::scene::BsdfSample> sample = pathtracer::scene::sampleBsdf(params, wo, sampler);
        if (sample.has_value() && sample->type != LobeType::Transmission) {
            const float bsdfPdf = pathtracer::scene::pdfBsdf(params, wo, sample->wiLocal);
            const float lightPdf = env.pdf(sample->wiLocal, 0.0F);
            const float bsdfPdf2 = bsdfPdf * bsdfPdf;
            const float lightPdf2 = lightPdf * lightPdf;
            const float misWeight = bsdfPdf2 / (bsdfPdf2 + lightPdf2);
            accum += sample->throughputWeight * misWeight;  // L0=1 folded in via the uniform environment already
        }
    }
    return std::max({accum.x, accum.y, accum.z}) / static_cast<float>(sampleCount);
}

// The MIS estimator and the brute-force reference must agree. BOTH sides are noisy, which is what the old
// "5% of max(reference, 0.1)" band got wrong twice over: it modelled the reference as exact, and its 0.1 floor was
// standing in for the fact that a relative band is undefined as the denominator approaches zero.
// Replaced by the difference of two independent estimators, whose variances add (Welch 1947). The reference is drawn
// with std::mt19937 and is genuinely iid, so its replicates are iid trivially; the MIS side is Sampler-driven and is
// replicated over independent scramble seeds, which is what makes ITS replicate means iid (see stats.h kReplicates).
// Total sample budget is unchanged -- the same count, redistributed across replicates.
PT_CHECK(mis_agreement_environment, Slow, Statistical) {
    constexpr int kTotalSamples = 100000;
    constexpr int kPerReplicate = kTotalSamples / tools::stats::kReplicates;
    // Excludes low roughness (e.g. 0.05): uniform-hemisphere sampling under-samples a sharp GGX peak there (the same
    // limitation bsdf_validate documents), biasing the reference low. Those checks bound one side only for that
    // reason; this one needs a tight two-sided equality, so it needs roughness values where the reference converges.
    const std::array<float, 3> roughnesses = {0.25F, 0.5F, 1.0F};
    const std::array<float, 2> metallics = {0.0F, 1.0F};
    const std::array<float, 3> ndotVs = {0.2F, 0.6F, 1.0F};

    ctx.plan(static_cast<int>(roughnesses.size() * metallics.size() * ndotVs.size()));
    const EnvironmentMap env = makeUniformEnvironment();

    for (float roughness : roughnesses) {
        for (float metallic : metallics) {
            for (float ndotV : ndotVs) {
                char label[96];
                std::snprintf(label, sizeof(label), "env r=%g m=%g n=%g", static_cast<double>(roughness),
                              static_cast<double>(metallic), static_cast<double>(ndotV));
                const BsdfParams params = makeParams(roughness, metallic);
                const glm::vec3 wo(std::sqrt(std::max(0.0F, 1.0F - (ndotV * ndotV))), 0.0F, ndotV);

                const std::uint64_t rowSeed = ctx.subSeed(label);
                std::mt19937 referenceRng(static_cast<std::mt19937::result_type>(rowSeed));
                tools::stats::Welford reference;
                tools::stats::Welford combined;
                for (int r = 0; r < tools::stats::kReplicates; ++r) {
                    reference.add(referenceLo(params, wo, kPerReplicate, referenceRng));
                    // A fresh scramble seed per replicate: that independence is the whole basis of the band.
                    combined.add(misCombinedLo(params, wo, env, kPerReplicate,
                                                static_cast<std::uint32_t>(rowSeed + r)));
                }

                const double difference = combined.mean() - reference.mean();
                const tools::stats::Band band = tools::stats::differenceBand(reference, combined, ctx.alpha());
                char detail[256];
                std::snprintf(detail, sizeof(detail),
                              "%s: reference %.5f, MIS %.5f, difference %+.3e vs +/-%.3e", label,
                              reference.mean(), combined.mean(), difference, band.halfWidth());
                PT_EXPECT(ctx, band.contains(difference), detail);
            }
        }
    }
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
PT_CHECK(quad_light_solid_angle, Slow, Statistical) {
    const QuadLight quad{glm::vec3(-0.5F, 1.0F, -0.5F), glm::vec3(1.0F, 0.0F, 0.0F),
                          glm::vec3(0.0F, 0.0F, 1.0F), glm::vec3(1.0F), false};
    const glm::vec3 p(0.0F, 0.0F, 0.0F);
    const std::optional<pathtracer::scene::SphericalRectangle> rect =
        pathtracer::scene::buildSphericalRectangle(quad, p);
    ctx.plan(3);
    if (!rect.has_value()) {
        PT_EXPECT(ctx, false, "buildSphericalRectangle returned nullopt for a valid configuration");
        ctx.plan(1);
        return;
    }

    constexpr int kSampleCount = 2000000;
    std::mt19937 rng(static_cast<std::mt19937::result_type>(ctx.seed()));
    long long hits = 0;
    for (int i = 0; i < kSampleCount; ++i) {
        hits += directionHitsQuad(quad, p, sampleUniformSphere(rng)) ? 1 : 0;
    }
    // Wilson score interval on the hit fraction rather than the Wald interval the 6-sigma form used: Wald's width
    // collapses toward zero as the fraction approaches 0 or 1, which is exactly where a small light lands. Scaled to
    // solid angle by the same 4*pi the estimator uses, so the band is the estimator's own statistics throughout.
    const tools::stats::Band fraction = tools::stats::wilsonBand(hits, kSampleCount, ctx.alpha());
    const tools::stats::Band solidAngle{4.0 * kPi * fraction.lo, 4.0 * kPi * fraction.hi};
    char detail[224];
    std::snprintf(detail, sizeof(detail), "analytic %.6f vs Monte Carlo interval [%.6f, %.6f] from %lld/%d hits",
                  static_cast<double>(rect->solidAngle), solidAngle.lo, solidAngle.hi, hits, kSampleCount);
    PT_EXPECT(ctx, solidAngle.contains(static_cast<double>(rect->solidAngle)), detail);

    // Every direction sample() draws must land on the rectangle: an exact assertion on the xu/yv inversion, which the
    // scalar solid angle above could be right about while the mapping is wrong.
    std::uniform_real_distribution<float> unit(0.0F, 1.0F);
    constexpr int kDrawCount = 20000;
    int misses = 0;
    glm::vec2 firstMiss(0.0F);
    for (int i = 0; i < kDrawCount; ++i) {
        const glm::vec2 u(unit(rng), unit(rng));
        const glm::vec3 dir = glm::normalize(rect->sample(u) - p);
        if (!directionHitsQuad(quad, p, dir)) {
            if (misses == 0) {
                firstMiss = u;
            }
            ++misses;
        }
    }
    char missDetail[192];
    std::snprintf(missDetail, sizeof(missDetail), "%d of %d sample() directions missed the rectangle (first at u=%g,%g)",
                  misses, kDrawCount, static_cast<double>(firstMiss.x), static_cast<double>(firstMiss.y));
    PT_EXPECT(ctx, misses == 0, missDetail);
    PT_EXPECT(ctx, rect->solidAngle > 0.0F, "a valid rectangle must subtend positive solid angle");
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
            accum += pathtracer::scene::evaluateBsdf(params, wo, wi) * wi.z * quad.radiance / kUniformPdf;
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
        Sampler sampler(0, 0, i, sampleCount, seed);

        // NEE.
        const std::optional<LightSample> lightSample = lights.sample(p, sampler);
        if (lightSample.has_value() && lightSample->direction.z > 0.0F) {
            const glm::vec3 bsdfValue = pathtracer::scene::evaluateBsdf(params, wo, lightSample->direction);
            const float bsdfPdf = pathtracer::scene::pdfBsdf(params, wo, lightSample->direction);
            if (bsdfPdf > 0.0F && (bsdfValue.x > 0.0F || bsdfValue.y > 0.0F || bsdfValue.z > 0.0F)) {
                const float lightPdf2 = lightSample->pdf * lightSample->pdf;
                const float bsdfPdf2 = bsdfPdf * bsdfPdf;
                const float misWeight = lightPdf2 / (lightPdf2 + bsdfPdf2);
                accum += bsdfValue * lightSample->direction.z * misWeight * lightSample->radiance /
                         lightSample->pdf;
            }
        }

        // BSDF-sampled.
        const std::optional<pathtracer::scene::BsdfSample> sample =
            pathtracer::scene::sampleBsdf(params, wo, sampler);
        if (sample.has_value() && sample->type != LobeType::Transmission &&
            directionHitsQuad(quad, p, sample->wiLocal)) {
            const float bsdfPdf = pathtracer::scene::pdfBsdf(params, wo, sample->wiLocal);
            const float lightPdf = lights.pdfQuad(0, p);
            const float bsdfPdf2 = bsdfPdf * bsdfPdf;
            const float lightPdf2 = lightPdf * lightPdf;
            const float misWeight = bsdfPdf2 / (bsdfPdf2 + lightPdf2);
            accum += sample->throughputWeight * lights.quadRadianceToward(0, sample->wiLocal) * misWeight;
        }
    }
    return std::max({accum.x, accum.y, accum.z}) / static_cast<float>(sampleCount);
}

// Same identity as mis_agreement_environment, against an area light rather than the environment, and converted the
// same way and for the same reason: both sides are estimators, so their variances add.
PT_CHECK(mis_agreement_quad_light, Slow, Statistical) {
    constexpr int kTotalSamples = 200000;
    constexpr int kPerReplicate = kTotalSamples / tools::stats::kReplicates;
    const std::array<float, 3> roughnesses = {0.25F, 0.5F, 1.0F};
    const std::array<float, 2> metallics = {0.0F, 1.0F};
    const std::array<float, 3> ndotVs = {0.2F, 0.6F, 1.0F};

    // Directly overhead: a 1x1 rectangle at height 2, subtending a modest solid angle -- large enough that BSDF
    // sampling alone finds it often enough to converge in this many samples, small enough that light sampling still
    // matters at grazing wo, so both one-strategy estimators are load-bearing here, not just the combined one.
    const QuadLight quad{glm::vec3(-0.5F, -0.5F, 2.0F), glm::vec3(0.0F, 1.0F, 0.0F),
                          glm::vec3(1.0F, 0.0F, 0.0F), glm::vec3(3.0F), false};
    const glm::vec3 p(0.0F);

    ctx.plan(static_cast<int>(roughnesses.size() * metallics.size() * ndotVs.size()));
    for (float roughness : roughnesses) {
        for (float metallic : metallics) {
            for (float ndotV : ndotVs) {
                char label[96];
                std::snprintf(label, sizeof(label), "quad r=%g m=%g n=%g", static_cast<double>(roughness),
                              static_cast<double>(metallic), static_cast<double>(ndotV));
                const BsdfParams params = makeParams(roughness, metallic);
                const glm::vec3 wo(std::sqrt(std::max(0.0F, 1.0F - (ndotV * ndotV))), 0.0F, ndotV);

                const std::uint64_t rowSeed = ctx.subSeed(label);
                std::mt19937 referenceRng(static_cast<std::mt19937::result_type>(rowSeed));
                tools::stats::Welford reference;
                tools::stats::Welford combined;
                for (int r = 0; r < tools::stats::kReplicates; ++r) {
                    reference.add(referenceLoQuad(params, wo, quad, p, kPerReplicate, referenceRng));
                    combined.add(misCombinedLoQuad(params, wo, quad, p, kPerReplicate,
                                                    static_cast<std::uint32_t>(rowSeed + r)));
                }

                const double difference = combined.mean() - reference.mean();
                const tools::stats::Band band = tools::stats::differenceBand(reference, combined, ctx.alpha());
                char detail[256];
                std::snprintf(detail, sizeof(detail),
                              "%s: reference %.5f, MIS %.5f, difference %+.3e vs +/-%.3e", label,
                              reference.mean(), combined.mean(), difference, band.halfWidth());
                PT_EXPECT(ctx, band.contains(difference), detail);
            }
        }
    }
}

}  // namespace

PT_CHECK_MAIN("nee")
