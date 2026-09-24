#include "pathtracer/scene/path_tracer.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <vector>

#include "pathtracer/scene/bsdf.h"
#include "pathtracer/scene/gbuffer_shading.h"
#include "pathtracer/scene/sampler.h"
#include "pathtracer/scene/shading_scene.h"

namespace pathtracer::scene {

namespace {

constexpr float kRayEpsilon = 1e-4F;

// Offsets the AO sampler's seed from the path sampler's so the two streams are independent and the eight pre-existing
// images stay bit-identical. 2^32/phi, the standard decorrelating odd constant (Knuth); any fixed offset would do.
constexpr std::uint32_t kAoSeedOffset = 0x9E3779B9U;
// The same for the Fresnel AOV's stream: frac(sqrt 2)*2^32, kAoSeedOffset's sibling. Distinct, non-zero and odd is the whole requirement.
constexpr std::uint32_t kFresnelSeedOffset = 0x6A09E667U;

// pbrt's ShadowEpsilon convention (PBR Sec 6.8.6): a relative back-off on a finite shadow ray's tMax, needed now that
// a light can be real geometry in the BVH -- an unshortened tMax lets the light's own front face occlude it.
// A no-op for the environment, where FLT_MAX * (1 - 1e-3) is still effectively unbounded, so one formula covers both.
constexpr float kShadowDistanceEpsilon = 1e-3F;

// Beer-Lambert absorption (Arnold standard_surface / OpenPBR): sigma_a = -ln(transmissionColor)/transmissionDepth.
// depth == 0 means no interior medium at all, so absorption is exactly zero and the enter/exit toggle stays
// symmetric. The colour floor stays: color == 0 is a reachable input and -log(0) = +inf would meet t = +inf as 0*inf.
glm::vec3 sigmaAFromTransmission(const glm::vec3& color, float depth) {
    if (depth <= 0.0F) {
        return glm::vec3(0.0F);
    }
    return -glm::log(glm::max(color, glm::vec3(1e-6F))) / depth;
}

// Transmission needs a curvature-scaled ray offset, not the flat kRayEpsilon reflection uses: the refraction
// direction can clip a neighbouring facet a genuinely non-zero distance away, measured at ~1e-3 on a 500-triangle
// sphere. See DERIVATIONS.md "Transmission ray offset" for why it scales by curvature and not by facet size.
float transmissionOffsetEpsilon(const ShadingTriangle& tri) {
    const glm::vec3 n0 = glm::normalize(tri.v0.normal);
    const glm::vec3 n1 = glm::normalize(tri.v1.normal);
    const glm::vec3 n2 = glm::normalize(tri.v2.normal);
    const float curvature = std::max({glm::length(glm::cross(n0, n1)), glm::length(glm::cross(n1, n2)),
                                       glm::length(glm::cross(n2, n0))});
    const float e0 = glm::length(tri.v1.position - tri.v0.position);
    const float e1 = glm::length(tri.v2.position - tri.v1.position);
    const float e2 = glm::length(tri.v0.position - tri.v2.position);
    return std::max(kRayEpsilon, std::max({e0, e1, e2}) * curvature);
}

// Blackman-Harris at Arnold's default 1.5px radius: support wider than one pixel, so footprints overlap and each
// sample reconstructs several pixels rather than only the one it was drawn in.
constexpr float kFilterRadius = 1.5F;
// How many pixels either side of a sample its splat can reach: a sample sits at most 1.0 past its own pixel's far
// centre, so a destination two pixels away is at least kFilterRadius off and weighs exactly zero.
constexpr int kFilterExtent = 1;
constexpr int kFilterTableSize = 64;
// Per-tile accumulator lanes: beauty.rgb, termination bounce, shadow, the five transport buckets, ambient occlusion,
// then the microfacet Fresnel. Scalars take one lane each and broadcast to RGB at write-out, matching writeTexel.
constexpr int kSampleLanes = 24;
constexpr int kTileLanes = kSampleLanes + 1;  // plus the per-pixel filter weight the lanes above are normalised by

// Sampled at |x| = i/(kFilterTableSize-1) * kFilterRadius and read back by truncating lookup, the table trick PBRT
// uses: the filter is smooth over 1.5px, and this replaces three cos() per tap on the renderer's hottest loop.
std::array<float, kFilterTableSize> buildFilterTable() {
    constexpr float kA0 = 0.35875F;
    constexpr float kA1 = 0.48829F;
    constexpr float kA2 = 0.14128F;
    constexpr float kA3 = 0.01168F;
    constexpr float kPi = 3.14159265F;
    std::array<float, kFilterTableSize> table{};
    for (int i = 0; i < kFilterTableSize; ++i) {
        // Blackman-Harris is defined over [0,1] with its centre at t = 0.5, so |x| = 0 maps there and the radius maps
        // to the effectively zero end of the window.
        const float t = 0.5F + (0.5F * static_cast<float>(i) / static_cast<float>(kFilterTableSize - 1));
        table[static_cast<std::size_t>(i)] = kA0 - (kA1 * std::cos(2.0F * kPi * t)) +
                                              (kA2 * std::cos(4.0F * kPi * t)) -
                                              (kA3 * std::cos(6.0F * kPi * t));
    }
    return table;
}

const std::array<float, kFilterTableSize> kFilterTable = buildFilterTable();

float filterWeight(float distance) {
    const float t = std::abs(distance) / kFilterRadius;
    if (t >= 1.0F) {
        return 0.0F;
    }
    return kFilterTable[static_cast<std::size_t>(t * static_cast<float>(kFilterTableSize - 1))];
}

struct TraceResult {
    glm::vec3 radiance;
    int terminationBounce;  // bounce index the path stopped at (== maxBounces + 1 if depth-capped)
    float shadow;           // 1.0 = shadowed/occluded, 0.0 = lit or no primary hit at all (background)
    float ao;  // 1.0 = unoccluded, 0.0 = occluded within aoMaxDistance -- inverted relative to shadow above
    // One VNDF draw's Fresnel at the primary hit; 0 where there is no BSDF vertex at bounce 0.
    glm::vec3 fresnel{0.0F};

    // Transport-component breakdown -- see PathTraceResult's doc comment for the bucketing rule.
    glm::vec3 directDiffuse{0.0F};
    glm::vec3 indirectDiffuse{0.0F};
    glm::vec3 directSpecular{0.0F};
    glm::vec3 indirectSpecular{0.0F};
    glm::vec3 refraction{0.0F};
};

// Which of the five transport buckets a path's contribution belongs to: set once at bounce 0's lobe, stickily
// overridden to Refraction the moment any bounce samples a transmission lobe. Direct vs indirect falls out of which
// bounce the radiance lands on, so it is not tracked here.
enum class PathBucket { Diffuse, SpecularReflection, Refraction };

TraceResult tracePath(const Ray& primaryRay, const EmbreeAccel& accel,
                       const std::vector<ShadingTriangle>& shadingTriangles,
                       const std::vector<MeshInstance>& instances,
                       const std::vector<int>& instanceLightIndex, const LightSet& lights,
                       bool showSky, const PathTraceSettings& settings,
                       const std::vector<PathTraceSettings>& perInstanceSettings,
                       Sampler& sampler, glm::vec2 aoSample, glm::vec2 fresnelSample,
                       pathtracer::debug::RayCounts& __restrict rays) {
    glm::vec3 radiance(0.0F);
    glm::vec3 throughput(1.0F);
    Ray ray = primaryRay;
    int bounce = 0;
    std::optional<PathBucket> pathBucket;  // unset until bounce 0 successfully samples a lobe
    // Single-level medium stack: nullopt = vacuum, set = the sigmaA of the dielectric the ray is inside, toggled on
    // every Transmission sample. Sufficient for one glass object; a second overlapping one would need a real stack.
    std::optional<glm::vec3> mediumSigmaA;
    // The RGB channel this path has committed to, once it reaches a dispersive interface -- unset means
    // full RGB transport, which is every path in a scene authoring no Abbe number. See the selection
    // block below for the estimator, and gbuffer_shading.cpp's resolveBsdfParams for what it selects.
    std::optional<int> heroChannel;
    glm::vec3 directDiffuseAccum(0.0F);
    glm::vec3 indirectDiffuseAccum(0.0F);
    glm::vec3 directSpecularAccum(0.0F);
    glm::vec3 indirectSpecularAccum(0.0F);
    glm::vec3 refractionAccum(0.0F);
    // Routes a radiance contribution into the path's bucket. A no-op when pathBucket is unset -- the camera ray missed
    // on bounce 0 -- since that background radiance is real but deliberately unbucketed.
    const auto addToBucket = [&](const glm::vec3& contribution, bool isDirect) {
        if (!pathBucket.has_value()) {
            return;
        }
        switch (*pathBucket) {
            case PathBucket::Refraction:
                refractionAccum += contribution;
                break;
            case PathBucket::Diffuse:
                (isDirect ? directDiffuseAccum : indirectDiffuseAccum) += contribution;
                break;
            case PathBucket::SpecularReflection:
                (isDirect ? directSpecularAccum : indirectSpecularAccum) += contribution;
                break;
        }
    };
    float gShadow = 0.0F;  // default: no surface hit at all -- not "shadowed", just background
    float gAo = 1.0F;      // default: background is fully unoccluded, matching the polarity in PathTraceResult
    // Default: no BSDF vertex at bounce 0 -- the camera ray missed, or hit an emitter, which returns before shading.
    glm::vec3 gFresnel(0.0F);

    // MIS state for the previous bounce's BSDF sample, the one that produced `ray`: reweights this bounce's miss
    // against NEE's light-sampling pdf so a direction both strategies reach is not double-counted.
    float lastBsdfPdf = 0.0F;
    // A delta lobe has no density for NEE to double-count against, so its miss takes full weight. Derived from
    // lastBsdfPdf rather than the lobe type: pdfBsdf returns exactly 0 only for the smooth-glass branch.
    bool lastSampleWasDelta = false;
    // The previous vertex's shading position, not ray.origin: an emitter hit's MIS weight must use the pdf NEE would
    // actually have had, and NEE samples from shading.position while ray.origin carries the epsilon offset.
    glm::vec3 lastShadingPosition(0.0F);

    // bounce 0, the camera ray with its NEE direct lighting, always traces regardless of maxBounces, which counts
    // secondary bounces beyond it. maxBounces==0 therefore means direct lighting only, with no continuation rays.
    for (; bounce <= settings.maxBounces + 1; ++bounce) {
        (bounce == 0 ? rays.primary : rays.bounce) += 1;
        const std::optional<Hit> hit = accel.intersect(ray);

        // Beer-Lambert attenuation for the segment just travelled, gated on medium state from the previous iteration.
        // hit->t is genuine Euclidean distance, every ray direction here being unit-length. A miss is unbounded, so
        // it is written per channel: exp(-0 * inf) is NaN and cannot express the non-absorbing case.
        if (mediumSigmaA.has_value()) {
            const glm::vec3& sigmaA = *mediumSigmaA;
            throughput *= hit.has_value()
                               ? glm::exp(-sigmaA * hit->t)
                               : glm::vec3(sigmaA.x > 0.0F ? 0.0F : 1.0F,
                                            sigmaA.y > 0.0F ? 0.0F : 1.0F,
                                            sigmaA.z > 0.0F ? 0.0F : 1.0F);
        }

        if (!hit.has_value()) {
            // showSky gates only the primary ray's own miss. Indirect bounces and NEE always sample real environment
            // radiance, so hiding the background does not unlight the scene.
            if (bounce == 0 && !showSky) {
                break;
            }
            const glm::vec3 envRadiance = lights.environmentRadiance(ray.dir, /*nearest=*/false);
            // Power heuristic (Veach 1997): full weight for bounce 0, which is not part of the MIS estimator, and
            // after a delta sample, where NEE has no density to balance against. A rough transmission sample is
            // MIS-eligible like any other continuous lobe.
            float misWeight = 1.0F;
            if (bounce > 0 && !lastSampleWasDelta) {
                const float lightPdf = lights.pdfEnvironment(ray.dir);
                const float bsdfPdf2 = lastBsdfPdf * lastBsdfPdf;
                misWeight = bsdfPdf2 / (bsdfPdf2 + (lightPdf * lightPdf));
            }
            const glm::vec3 missRadiance = throughput * envRadiance * misWeight;
            radiance += missRadiance;
            addToBucket(missRadiance, /*isDirect=*/bounce == 1);
            break;
        }

        const ShadingTriangle& triangle =
            shadingTriangles[static_cast<std::size_t>(hit->triangleIndex)];

        // An emitter hit -- a quad light's own triangles, injected into the BVH so they occlude and are BSDF-hittable:
        // Le, MIS-weighted exactly like the environment miss, then terminate, a pure emitter having no BSDF. Checked
        // before the depth cap, symmetric with the miss branch: an emitter hit is exactly as eligible as a miss.
        const int lightIndex = instanceLightIndex[static_cast<std::size_t>(triangle.instanceIndex)];
        if (lightIndex >= 0) {
            const glm::vec3 emitted = lights.quadRadianceToward(lightIndex, ray.dir);
            float misWeight = 1.0F;
            if (bounce > 0 && !lastSampleWasDelta) {
                const float lightPdf = lights.pdfQuad(lightIndex, lastShadingPosition);
                const float bsdfPdf2 = lastBsdfPdf * lastBsdfPdf;
                misWeight = bsdfPdf2 / (bsdfPdf2 + (lightPdf * lightPdf));
            }
            const glm::vec3 hitRadiance = throughput * emitted * misWeight;
            radiance += hitRadiance;
            addToBucket(hitRadiance, /*isDirect=*/bounce == 1);
            break;
        }

        // Depth cap. The extra iteration past maxBounces exists only so the final BSDF-sampled ray can collect its
        // MIS-weighted contribution in the miss and emitter-hit branches; a ray reaching ordinary geometry here adds
        // nothing and terminates.
        if (bounce > settings.maxBounces) {
            break;
        }

        const Material& material =
            instances[static_cast<std::size_t>(triangle.instanceIndex)].material;
        const PathTraceSettings& instanceSettings =
            perInstanceSettings[static_cast<std::size_t>(triangle.instanceIndex)];

        // Dispersion: commit the path to one RGB channel on first reaching a dispersive interface, before any BSDF
        // work at this vertex, since the whole interaction is wavelength dependent. One-sample estimator weighted by
        // throughput, so the result is unbiased. See DERIVATIONS.md "Dispersion channel commitment".
        if (!heroChannel.has_value() && instanceSettings.abbe > 0.0F &&
            instanceSettings.transmissionFactor > 0.0F) {
            const float sum = throughput.x + throughput.y + throughput.z;
            if (sum > 0.0F) {
                const float u = sampler.next1D() * sum;
                const int channel = u < throughput.x ? 0 : (u < throughput.x + throughput.y ? 1 : 2);
                throughput = glm::vec3(0.0F);
                throughput[channel] = sum;
                heroChannel = channel;
            }
        }

        const ShadingVertex shading = interpolateShading(triangle, hit->u, hit->v);
        const ShadingFrame frame = buildShadingFrame(shading, material, instanceSettings);
        const BsdfParams params =
            resolveBsdfParams(material, shading.uv, shading.colour, instanceSettings, heroChannel);
        const glm::vec3 woWorld = -ray.dir;
        // True flat per-triangle plane normal, for the normal-map light-leak rejection and for offsetting ray origins
        // off the surface: both need the actual geometry, not the interpolated or normal-mapped shading normal.
        const glm::vec3 geoNormal = geometricNormalOf(triangle);

        const glm::vec3 woLocal = frame.toLocal(woWorld);

        if (bounce == 0) {
            gShadow = 1.0F;  // assume shadowed once we know there's a real surface; the NEE check below may clear this
            // The Fresnel the microfacet lobe evaluates at this vertex, one VNDF draw per sample. Its own stream, for
            // the same reason AO has one. No ray and no BVH query: the half-vector is drawn analytically.
            gFresnel = fresnelAtMicrofacet(params, woLocal, fresnelSample);
            // Cosine-weighted obscurance (Zhukov 1998; Iones 2003): W = (1/pi) * int rho(t(w)) cos(theta) dw, and
            // sampling at pdf = cos/pi cancels both factors, so the estimator is the mean of rho alone. Negating the
            // sampled direction points a back-facing primary hit outward; the cosine density's symmetry makes it exact.
            const bool frontSide = glm::dot(geoNormal, woWorld) > 0.0F;
            const glm::vec3 aoDir =
                frame.toWorld(sampleCosineHemisphere(aoSample)) * (frontSide ? 1.0F : -1.0F);
            // NEE's near-side origin verbatim: the shading-terminator offset plus a geometric-normal back-off, on whichever side wo is.
            const glm::vec3 aoOrigin = shadowTerminatorOffset(triangle, hit->u, hit->v, frontSide) +
                                        (geoNormal * kRayEpsilon * (frontSide ? 1.0F : -1.0F));
            ++rays.ao;
            // Closest-hit rather than any-hit because rho needs the distance; measured under 1.3% of frame time, a
            // ray bounded this short leaving any-hit almost nothing to early-terminate out of.
            const std::optional<Hit> aoHit =
                accel.intersect(Ray{aoOrigin, aoDir, kRayEpsilon, settings.aoMaxDistance});
            // rho(x) = 1 - (1-x)^2 over x = t/D is the lowest-degree polynomial meeting the three conditions the
            // bounded ray imposes: rho(0) = 0, rho(1) = 1 and rho'(1) = 0, so neither value nor gradient steps at D.
            // Embree clamps t to tfar, so an ulp of overshoot only makes k*k a tiny positive: no clamp needed.
            if (aoHit.has_value()) {
                const float k = 1.0F - (aoHit->t / settings.aoMaxDistance);
                gAo = 1.0F - (k * k);
            } else {
                gAo = 1.0F;
            }
        }

        // A failed sample must not skip the NEE block below: NEE and the continuing ray are independent estimators of
        // independent directions, sharing only this vertex's params and frame, so one failing says nothing about the
        // other.
        const std::optional<BsdfSample> sample = sampleBsdf(params, woLocal, sampler);

        // Bucket assignment: bounce 0 sets the path's bucket from scratch; any later bounce only ever overrides it to
        // Refraction, stickily, once the path passes through a transmission lobe.
        if (sample.has_value()) {
            if (bounce == 0) {
                pathBucket = sample->type == LobeType::Transmission ? PathBucket::Refraction
                             : sample->type == LobeType::Diffuse            ? PathBucket::Diffuse
                                                                             : PathBucket::SpecularReflection;
            } else if (sample->type == LobeType::Transmission) {
                pathBucket = PathBucket::Refraction;
            }
        }

        // Medium-state toggle, co-located with the bucket assignment since both key off the same condition: a
        // Transmission sample crossed the interface. Entering starts absorbing at this instance's sigmaA, exiting
        // returns to vacuum. Reflection bounces, TIR included, leave the state untouched.
        if (sample.has_value() && sample->type == LobeType::Transmission) {
            mediumSigmaA = mediumSigmaA.has_value()
                               ? std::nullopt
                               : std::make_optional(sigmaAFromTransmission(
                                     instanceSettings.transmissionColor,
                                     instanceSettings.transmissionDepth));
        }

        // Next-event estimation: sample a light from this vertex, evaluate the combined BSDF value and pdf toward it,
        // and add the MIS-weighted contribution if unoccluded. Independent of whichever lobe `sample` drew, and fired
        // unconditionally so both sides of a transmissive vertex are light-sampled rather than left to BSDF sampling.
        const std::optional<LightSample> lightSample = lights.sample(shading.position, sampler);
        if (lightSample.has_value()) {
            const float geoCos = glm::dot(lightSample->direction, geoNormal);
            const float shadingCos = glm::dot(lightSample->direction, frame.normal);
            // Both sides, not just wo's: on a transmissive surface a light behind the vertex reaches the eye through
            // the transmission lobe, so restricting NEE to the near side left rough glass lit by BSDF sampling alone.
            const bool nearSide = geoCos > 0.0F && shadingCos > 0.0F;
            const bool farSide = geoCos < 0.0F && shadingCos < 0.0F && params.transmissionFactor > 0.0F;
            if (nearSide || farSide) {
                const glm::vec3 wiLocalLight = frame.toLocal(lightSample->direction);
                const float lightCos = std::abs(shadingCos);  // far-side samples carry a negative cosine
                // One evaluation for the value, the pdf and the per-lobe split the transport AOVs need. The four
                // separate calls this replaced each recomputed the same lobe probabilities and table lookups.
                const BsdfEval eval = evaluateBsdfSplit(params, woLocal, wiLocalLight);
                const glm::vec3 bsdfValue = eval.total();
                if (eval.pdf > 0.0F &&
                    (bsdfValue.x > 0.0F || bsdfValue.y > 0.0F || bsdfValue.z > 0.0F)) {
                    // Offset along geoNormal toward whichever side the light sample is on -- the far side for a
                    // transmissive vertex lit from behind. That side crosses the interface like a transmission
                    // continuation ray, so it takes the same curvature-scaled offset.
                    const float shadowEpsilon =
                        farSide ? transmissionOffsetEpsilon(triangle) : kRayEpsilon;
                    const glm::vec3 shadowOrigin =
                        shadowTerminatorOffset(triangle, hit->u, hit->v, geoCos > 0.0F) +
                        (geoNormal * shadowEpsilon * (geoCos > 0.0F ? 1.0F : -1.0F));
                    const Ray shadowRay{shadowOrigin, lightSample->direction, shadowEpsilon,
                                         lightSample->distance * (1.0F - kShadowDistanceEpsilon)};
                    ++rays.shadow;
                    if (!accel.occluded(shadowRay)) {
                        if (bounce == 0) {
                            gShadow = 0.0F;
                        }
                        const float lightPdf2 = lightSample->pdf * lightSample->pdf;
                        const float bsdfPdf2 = eval.pdf * eval.pdf;
                        const float misWeightLight = lightPdf2 / (lightPdf2 + bsdfPdf2);
                        const glm::vec3 common = throughput * lightSample->radiance * lightCos *
                                                  misWeightLight / lightSample->pdf;
                        const glm::vec3 neeContribution = bsdfValue * common;
                        radiance += neeContribution;
                        if (bounce == 0) {
                            // Bounce 0's NEE contribution is split across the buckets by the lobe that carried it,
                            // deterministically and at its own physical value -- never by sample->type, which names
                            // the lobe the continuation ray drew.
                            directDiffuseAccum += eval.diffuse * common;
                            directSpecularAccum += eval.specular * common;
                            refractionAccum += eval.transmission * common;
                        } else {
                            // Deeper bounces keep the path's sticky bucket at the full physical contribution: which
                            // lobe carries the light here no longer names the transport type of a path already
                            // bucketed at bounce 0.
                            addToBucket(neeContribution, /*isDirect=*/false);
                        }
                    }
                }
            }
        }

        // No continuation direction could be sampled: terminate here, NEE having already taken this vertex's direct
        // lighting.
        if (!sample.has_value()) {
            break;
        }

        lastBsdfPdf = sample->pdf;
        lastSampleWasDelta = lastBsdfPdf <= 0.0F;
        lastShadingPosition = shading.position;

        const glm::vec3 wiWorld = frame.toWorld(sample->wiLocal);

        // Geometric-normal-consistency rejection, a simpler stand-in for Schussler et al. 2017's two-facet
        // microsurface reconstruction: a reflection or diffuse sample crossing to the wrong side of the true triangle
        // plane is rejected rather than shaded, which is what a normal map can otherwise make it do.
        if (sample->type != LobeType::Transmission) {
            const bool woAbove = glm::dot(woWorld, geoNormal) > 0.0F;
            const bool wiAbove = glm::dot(wiWorld, geoNormal) > 0.0F;
            if (woAbove != wiAbove) {
                break;
            }
        }

        throughput *= sample->throughputWeight;

        if (bounce >= settings.russianRouletteStartBounce) {
            const float continueProb = std::clamp(
                std::max({throughput.x, throughput.y, throughput.z}), settings.rrMinProb,
                settings.rrMaxProb);
            if (sampler.next1D() >= continueProb) {
                break;
            }
            throughput /= continueProb;
        }

        // Chiang/Li/Burley shadow-terminator-corrected origin, nudged off the true triangle plane along the geometric
        // normal toward wi's side, so an inward-going ray takes the opposite sign: the unmirrored form pushes the
        // origin back through the interface, desynchronising the medium stack on 0.93% of glass entries in cornell.
        const bool leavingOnNormalSide = glm::dot(wiWorld, geoNormal) > 0.0F;
        const float offsetEpsilon = sample->type == LobeType::Transmission
                                         ? transmissionOffsetEpsilon(triangle)
                                         : kRayEpsilon;
        const glm::vec3 offsetOrigin =
            shadowTerminatorOffset(triangle, hit->u, hit->v, leavingOnNormalSide) +
            (geoNormal * offsetEpsilon * (leavingOnNormalSide ? 1.0F : -1.0F));
        ray = Ray{offsetOrigin, wiWorld, offsetEpsilon, std::numeric_limits<float>::max()};
    }

    return {radiance,           bounce,               gShadow,             gAo,
            gFresnel,           directDiffuseAccum,   indirectDiffuseAccum,
            directSpecularAccum, indirectSpecularAccum, refractionAccum};
}

}  // namespace

PathTraceResult makePathTraceResult(int width, int height) {
    // 10 images, in PathTraceResult's declaration order, which this positional init must match. The trailing
    // overRange field is deliberately left default.
    return {makeImage(width, height), makeImage(width, height), makeImage(width, height),
            makeImage(width, height), makeImage(width, height), makeImage(width, height),
            makeImage(width, height), makeImage(width, height), makeImage(width, height),
            makeImage(width, height), OverRangeStats{}};
}

void renderPathTraced(const Camera& camera, const EmbreeAccel& accel,
                       const std::vector<ShadingTriangle>& shadingTriangles,
                       const std::vector<MeshInstance>& instances,
                       const std::vector<int>& instanceLightIndex, const LightSet& lights,
                       int width, int height, bool showSky, const PathTraceSettings& settings,
                       const std::vector<PathTraceSettings>& perInstanceSettings,
                       std::uint32_t scrambleSeed, int sampleBase, int sampleCount,
                       const std::atomic<std::uint64_t>& generation,
                       std::uint64_t requestedGeneration, ThreadPool& threadPool,
                       pathtracer::debug::PassStats& stats, PathTraceResult& out) {
    const float aspect = static_cast<float>(width) / static_cast<float>(height);
    // Constant for the whole pass, so built once here: the aspect-taking primaryRay rebuilds it every call, which at
    // samplesPerPixel rays per pixel is millions of identical reconstructions.
    const Camera::ViewBasis basis = camera.viewBasis(aspect);
    const int tilesX = (width + kPathTraceTileSize - 1) / kPathTraceTileSize;
    const int tilesY = (height + kPathTraceTileSize - 1) / kPathTraceTileSize;

    // One worker owns every output pixel of one tile and traces every pixel within the filter radius of it -- the
    // kFilterExtent-wide halo, whose samples are therefore traced twice, once by each tile they splat into.
    const auto renderTile = [&](int tileIndex) {
        const int tileX0 = (tileIndex % tilesX) * kPathTraceTileSize;
        const int tileY0 = (tileIndex / tilesX) * kPathTraceTileSize;
        const int tileX1 = std::min(tileX0 + kPathTraceTileSize, width);
        const int tileY1 = std::min(tileY0 + kPathTraceTileSize, height);

        // Stack-local, not thread_local like the accumulator below: zeroed by construction, so the reset boundary is
        // the tile boundary with no bookkeeping. A thread_local would outlive both the tile and the pass.
        pathtracer::debug::RayCounts tileRays;

        // Reused for the worker thread's life, so a pass allocates nothing; sized for a full tile even at the image
        // edge so the stride stays kPathTraceTileSize. The check below targets namespace scope, not a local.
        // NOLINTNEXTLINE(misc-use-internal-linkage)
        thread_local std::vector<float> accumulator;
        accumulator.assign(static_cast<std::size_t>(kPathTraceTileSize) * kPathTraceTileSize * kTileLanes, 0.0F);

        for (int y = std::max(tileY0 - kFilterExtent, 0);
             y < std::min(tileY1 + kFilterExtent, height); ++y) {
            for (int x = std::max(tileX0 - kFilterExtent, 0);
                 x < std::min(tileX1 + kFilterExtent, width); ++x) {
                for (int s = 0; s < settings.samplesPerPixel; ++s) {
                    // sampleBase + s is this sample's position in the pixel's accumulated sequence, not a per-pass
                    // seed: it must advance across passes for the Sobol points to stratify against the samples already
                    // accumulated. scrambleSeed stays fixed for the whole accumulation -- see sampler.h.
                    const int sampleIndex = sampleBase + s;
                    Sampler sampler(x, y, sampleIndex, sampleCount, scrambleSeed);
                    const glm::vec2 jitter = sampler.next2D();
                    const float filmX = static_cast<float>(x) + jitter.x;
                    const float filmY = static_cast<float>(y) + jitter.y;
                    const float ndcX = ((filmX / static_cast<float>(width)) * 2.0F) - 1.0F;
                    // HdrImage row 0 is the top (EXR/glTF convention); NDC +Y is up -- flip.
                    const float ndcY = 1.0F - ((filmY / static_cast<float>(height)) * 2.0F);
                    const Ray primary = camera.primaryRay(basis, ndcX, ndcY);
                    // AO and the Fresnel AOV each draw from their own stream, not `sampler`: taking dimensions from
                    // the path sampler would shift every later dimension. The offset goes on the scramble seed, never
                    // the sample index, which would only walk the same Sobol points a few steps along.
                    Sampler aoSampler(x, y, sampleIndex, sampleCount, scrambleSeed ^ kAoSeedOffset);
                    const glm::vec2 aoSample = aoSampler.next2D();
                    Sampler fresnelSampler(x, y, sampleIndex, sampleCount, scrambleSeed ^ kFresnelSeedOffset);
                    const glm::vec2 fresnelSample = fresnelSampler.next2D();
                    const TraceResult trace =
                        tracePath(primary, accel, shadingTriangles, instances, instanceLightIndex,
                                  lights, showSky, settings, perInstanceSettings, sampler,
                                  aoSample, fresnelSample, tileRays);
                    const std::array<float, kSampleLanes> values{
                        trace.radiance.x,          trace.radiance.y,
                        trace.radiance.z,          static_cast<float>(trace.terminationBounce),
                        trace.shadow,              trace.directDiffuse.x,
                        trace.directDiffuse.y,     trace.directDiffuse.z,
                        trace.indirectDiffuse.x,   trace.indirectDiffuse.y,
                        trace.indirectDiffuse.z,   trace.directSpecular.x,
                        trace.directSpecular.y,    trace.directSpecular.z,
                        trace.indirectSpecular.x,  trace.indirectSpecular.y,
                        trace.indirectSpecular.z,  trace.refraction.x,
                        trace.refraction.y,        trace.refraction.z,
                        trace.ao,                  trace.fresnel.x,
                        trace.fresnel.y,           trace.fresnel.z};

                    // Clipped to this tile: taps outside it belong to a neighbouring tile, which traces this same
                    // sample itself rather than receiving it.
                    const int splatX0 = std::max(tileX0, static_cast<int>(std::ceil(filmX - 0.5F - kFilterRadius)));
                    const int splatX1 = std::min(tileX1 - 1, static_cast<int>(std::floor(filmX - 0.5F + kFilterRadius)));
                    const int splatY0 = std::max(tileY0, static_cast<int>(std::ceil(filmY - 0.5F - kFilterRadius)));
                    const int splatY1 = std::min(tileY1 - 1, static_cast<int>(std::floor(filmY - 0.5F + kFilterRadius)));
                    for (int splatY = splatY0; splatY <= splatY1; ++splatY) {
                        // Separable: the 2D weight is the product of the two 1D lookups, so a row's own factor is
                        // hoisted out of the inner loop.
                        const float weightY = filterWeight(filmY - (static_cast<float>(splatY) + 0.5F));
                        if (weightY <= 0.0F) {
                            continue;
                        }
                        for (int splatX = splatX0; splatX <= splatX1; ++splatX) {
                            const float weight =
                                weightY * filterWeight(filmX - (static_cast<float>(splatX) + 0.5F));
                            if (weight <= 0.0F) {
                                continue;
                            }
                            float* lanes =
                                accumulator.data() +
                                ((static_cast<std::size_t>(splatY - tileY0) * kPathTraceTileSize) +
                                 static_cast<std::size_t>(splatX - tileX0)) * kTileLanes;
                            for (std::size_t lane = 0; lane < kSampleLanes; ++lane) {
                                lanes[lane] += weight * values[lane];
                            }
                            lanes[kSampleLanes] += weight;
                        }
                    }
                }
            }
        }

        for (int y = tileY0; y < tileY1; ++y) {
            for (int x = tileX0; x < tileX1; ++x) {
                const float* lanes = accumulator.data() +
                                      ((static_cast<std::size_t>(y - tileY0) * kPathTraceTileSize) +
                                       static_cast<std::size_t>(x - tileX0)) * kTileLanes;
                // Always positive: a pixel's own samples land within half a pixel of its centre, well inside the
                // 1.5px support, and samplesPerPixel is at least 1.
                const float invWeight = 1.0F / lanes[kSampleLanes];
                writeTexel(out.beauty, x, y, glm::vec3(lanes[0], lanes[1], lanes[2]) * invWeight);
                writeTexel(out.bounceHeatmap, x, y, glm::vec3(lanes[3] * invWeight));
                writeTexel(out.shadow, x, y, glm::vec3(lanes[4] * invWeight));
                writeTexel(out.directDiffuse, x, y, glm::vec3(lanes[5], lanes[6], lanes[7]) * invWeight);
                writeTexel(out.indirectDiffuse, x, y, glm::vec3(lanes[8], lanes[9], lanes[10]) * invWeight);
                writeTexel(out.directSpecular, x, y, glm::vec3(lanes[11], lanes[12], lanes[13]) * invWeight);
                writeTexel(out.indirectSpecular, x, y, glm::vec3(lanes[14], lanes[15], lanes[16]) * invWeight);
                writeTexel(out.refraction, x, y, glm::vec3(lanes[17], lanes[18], lanes[19]) * invWeight);
                writeTexel(out.ao, x, y, glm::vec3(lanes[20] * invWeight));
                writeTexel(out.fresnel, x, y, glm::vec3(lanes[21], lanes[22], lanes[23]) * invWeight);
            }
        }

        stats.addTile(tileRays);
    };

    threadPool.parallelFor(tilesX * tilesY, [&renderTile, &generation, requestedGeneration,
                                              &stats](int tileIndex) {
        if (generation.load(std::memory_order_relaxed) != requestedGeneration) {
            stats.addCancelledTile();  // counted, not ignored: a cancelled pass must not read as a short one
            return;  // stale -- caller discards this pass's result entirely
        }
        renderTile(tileIndex);
    });
}

}  // namespace pathtracer::scene
