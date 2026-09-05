#include "engine/scene/path_tracer.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <vector>

#include "engine/scene/bsdf.h"
#include "engine/scene/gbuffer_shading.h"
#include "engine/scene/sampler.h"
#include "engine/scene/shading_scene.h"

namespace engine::scene {

namespace {

constexpr float kRayEpsilon = 1e-4F;

// pbrt's ShadowEpsilon convention (Pharr/Jakob/Humphreys Sec 6.8.6): a relative back-off on a finite
// shadow ray's own tMax, needed now that a light can be real geometry sitting in the BVH -- an
// unshortened tMax lets the light's own front face register as its own occluder at t == distance.
// A no-op for the environment (distance == FLT_MAX): FLT_MAX * (1 - 1e-3) is still a normal,
// effectively-unbounded float, so one formula covers both light kinds with no branch.
constexpr float kShadowDistanceEpsilon = 1e-3F;

// Beer-Lambert absorption coefficient (Arnold standard_surface / OpenPBR convention): sigma_a = -ln(transmissionColor)/transmissionDepth, transmissionColor being the colour white light reaches after travelling transmissionDepth inside the medium.
// depth == 0 means there is no interior medium at all, not an infinitely dense one: transmissionColor is then the on-surface tint BsdfParams::transmissionTint carries, so absorption here is exactly zero rather than the -ln(c)/1e-4 a floored divide used to return, which rendered any coloured depth-0 material black.
// Zero rather than an absent medium, so tracePath's enter/exit toggle below stays symmetric across a depth == 0 interface.
// The colour floor stays: color == 0 is a reachable material-file input and -log(0) = +inf would meet a t = +inf as 0*inf on a miss.
glm::vec3 sigmaAFromTransmission(const glm::vec3& color, float depth) {
    if (depth <= 0.0F) {
        return glm::vec3(0.0F);
    }
    return -glm::log(glm::max(color, glm::vec3(1e-6F))) / depth;
}

// Reflection/diffuse continuation rays stay close to the geometric normal's hemisphere, where
// kRayEpsilon (a floating-point-scale constant) has always been sufficient. Transmission bends sharply
// away from it, and on curved geometry approximated by flat facets, the shading-normal-derived refraction
// direction can clip a NEIGHBOURING facet a small-but-nonzero distance away -- a genuine geometric
// intersection, not floating-point noise (measured on a 500-triangle sphere: self-intersections at ~1e-3,
// an order of magnitude below a facet's own edge length).
// Scaled by curvature, not raw facet size: the mechanism is the smooth shading normal diverging from the
// flat facet's true geometric normal across the facet, so the fix must vanish wherever that divergence
// does. sin(angle) between each pair of vertex normals (cross product of two unit vectors) is exactly
// zero on any planar patch -- coplanar vertex normals, whatever the facet's absolute size -- and grows
// with tessellation coarseness on genuinely curved geometry. Scaling raw edge length alone (an earlier,
// broken version of this fix) has no such zero: it blew up on a flat 2000-unit slab quad, pushing the
// continuation ray origin far past the geometry it needed to traverse (caught by integrator_validate).
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

// Blackman-Harris at Arnold's default 1.5px radius: support wider than one pixel, so neighbouring footprints overlap and each sample reconstructs several pixels instead of only the one it was drawn in -- which is where nearly all of a reconstruction filter's benefit over the 1px box comes from. Non-negative everywhere, so no pixel can end up with a zero or negative total weight and no ringing appears around highlights.
constexpr float kFilterRadius = 1.5F;
constexpr int kFilterExtent = 1;  // how many pixels either side of a sample its splat can reach: a sample sits at most 1.0 past its own pixel's far centre, so a destination two pixels away is at least kFilterRadius off and weighs exactly zero
constexpr int kFilterTableSize = 64;
// Square destination tiles, each owned outright by one worker: splatting crosses pixel boundaries, so the row-disjoint invariant the rasterizer still relies on cannot hold here. Size trades halo waste against load-balancing granularity -- the halo re-traces (size+2*kFilterExtent)^2/size^2 of a tile, 4.2% here against 6.3% at 64, while doubling to 128 quarters the number of work items a small render has to spread across its workers. 96 and 128 measured indistinguishable at 1080p; 64 measurably worse.
constexpr int kTileSize = 96;
// Per-tile accumulator lanes: beauty.rgb, termination bounce, shadow, then the five transport buckets' rgb -- the scalars take one lane each and are broadcast to RGB at write-out, matching writeTexel's convention.
constexpr int kSampleLanes = 20;
constexpr int kTileLanes = kSampleLanes + 1;  // plus the per-pixel filter weight the lanes above are normalised by

// Sampled at |x| = i/(kFilterTableSize-1) * kFilterRadius and read back by truncating lookup, the same table trick PBRT uses: the filter is smooth over 1.5px, and this replaces three cos() per tap on the renderer's hottest inner loop.
std::array<float, kFilterTableSize> buildFilterTable() {
    constexpr float kA0 = 0.35875F;
    constexpr float kA1 = 0.48829F;
    constexpr float kA2 = 0.14128F;
    constexpr float kA3 = 0.01168F;
    constexpr float kPi = 3.14159265F;
    std::array<float, kFilterTableSize> table{};
    for (int i = 0; i < kFilterTableSize; ++i) {
        // Blackman-Harris is defined over [0,1]; the window's own centre is t = 0.5, so a sample at |x| = 0 maps there and one at the radius maps to the (effectively zero) end of the window.
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

    // Transport-component breakdown -- see PathTraceResult's doc comment for the bucketing rule.
    glm::vec3 directDiffuse{0.0F};
    glm::vec3 indirectDiffuse{0.0F};
    glm::vec3 directSpecular{0.0F};
    glm::vec3 indirectSpecular{0.0F};
    glm::vec3 refraction{0.0F};
};

// Which of the five transport-component AOV buckets a path's contribution belongs to -- set once at bounce 0's lobe, stickily overridden to Refraction the moment any bounce samples a transmission lobe. Direct-vs-indirect for Diffuse/SpecularReflection isn't tracked here; it falls out of which bounce index the radiance-contributing miss lands on (see tracePath).
enum class PathBucket { Diffuse, SpecularReflection, Refraction };

TraceResult tracePath(const Ray& primaryRay, const EmbreeAccel& accel,
                       const std::vector<ShadingTriangle>& shadingTriangles,
                       const std::vector<MeshInstance>& instances,
                       const std::vector<int>& instanceLightIndex, const LightSet& lights,
                       bool showSky, const PathTraceSettings& settings,
                       const std::vector<PathTraceSettings>& perInstanceSettings,
                       Sampler& sampler) {
    glm::vec3 radiance(0.0F);
    glm::vec3 throughput(1.0F);
    Ray ray = primaryRay;
    int bounce = 0;
    std::optional<PathBucket> pathBucket;  // unset until bounce 0 successfully samples a lobe
    // Single-level medium stack: nullopt = vacuum, set = the sigmaA of the dielectric the ray is
    // currently inside. Toggled below on every Transmission-lobe sample (entering sets it, exiting
    // clears it) -- sufficient for one glass object; would need generalising to a real stack before a
    // second, overlapping transmissive object entered the scene.
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
    // Routes a radiance contribution into the path's bucket -- a no-op when pathBucket is unset, i.e. the camera ray missed all geometry on bounce 0 (background seen directly): that contribution is real (added to `radiance`/beauty by the caller) but isn't attributed to any of the five transport-component AOVs, the same way a "background" AOV is conventionally kept separate from surface-interaction AOVs in production renderers. isDirect: "exactly one surface vertex between camera and light" -- for a BSDF-sampled miss, that's bounce==1 (one hit at bounce 0, then straight to the environment); for NEE firing at the vertex reached at bounce 0 (querying the light directly from the first surface hit, no extra bounce needed), that's bounce==0. Both describe the same physical path length; see call sites.
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

    // MIS state for the *previous* bounce's BSDF sample (the one that produced `ray`) -- used to reweight this bounce's miss contribution against NEE's light-sampling pdf, so a direction reachable by both strategies isn't double-counted. Meaningless at bounce==0 (ray is the primary/camera ray, not a BSDF sample -- its miss is a pure camera-visibility event, not part of the two-strategy light-transport estimator NEE/MIS balances).
    float lastBsdfPdf = 0.0F;
    // A delta lobe has no density for NEE to double-count against, so its miss takes full weight. Derived from lastBsdfPdf rather than the lobe type: pdfBsdf returns exactly 0 only for the smooth-glass transmission branch, and sampleBsdf rejects pdf <= 1e-8 on every other lobe.
    bool lastSampleWasDelta = false;
    // The previous vertex's SHADING position, not ray.origin: an emitter hit's MIS weight must use the pdf NEE would actually have had, and NEE samples from shading.position (see the NEE block below) while ray.origin carries the epsilon offset that keeps the continuation ray off the surface. Solid angle measured from two points ~1e-4 apart differs, so weighting from ray.origin leaves the two strategies' weights not summing to 1 -- a small but real bias. pbrt carries the previous interaction for the same reason.
    glm::vec3 lastShadingPosition(0.0F);

    // bounce 0 (the primary/camera ray, direct lighting via NEE) always traces regardless of maxBounces -- maxBounces counts secondary/indirect bounces beyond it, so maxBounces==0 means direct lighting only, no continuation rays. The loop runs one iteration PAST maxBounces so the final BSDF-sampled ray can still collect its MIS-weighted environment contribution via the miss branch below; that extra iteration breaks at the depth guard before any surface interaction -- see the guard for why the terminal ray must be traced rather than dropped.
    for (; bounce <= settings.maxBounces + 1; ++bounce) {
        const std::optional<Hit> hit = accel.intersect(ray);

        // Beer-Lambert attenuation for the segment just traveled (ray.origin to this hit/miss), gated on
        // medium state carried over from the previous iteration's Transmission sample. hit->t is genuine
        // Euclidean distance: every ray direction reaching this point is unit-length (primary rays via
        // Camera::primaryRay, continuation rays via ShadingFrame::toWorld of a normalized local sample).
        // A miss is an unbounded segment: transmittance is exp(-sigmaA * inf), which is 0 in any channel that absorbs and 1 in any that does not. Written per channel because 0 * inf is NaN, so the exp form cannot express the non-absorbing case. Reachable whenever a transmissive object is not watertight -- integrator_validate's own two-quad slab has open sides -- where the previous hit-only gate contributed unattenuated environment radiance and gained energy.
        if (mediumSigmaA.has_value()) {
            const glm::vec3& sigmaA = *mediumSigmaA;
            throughput *= hit.has_value()
                               ? glm::exp(-sigmaA * hit->t)
                               : glm::vec3(sigmaA.x > 0.0F ? 0.0F : 1.0F,
                                            sigmaA.y > 0.0F ? 0.0F : 1.0F,
                                            sigmaA.z > 0.0F ? 0.0F : 1.0F);
        }

        if (!hit.has_value()) {
            // showSky gates only the primary ray's own miss (the camera seeing the background directly) -- indirect bounces and NEE (below) always sample real environment radiance regardless of showSky, so hiding the background doesn't unlight the scene.
            if (bounce == 0 && !showSky) {
                break;
            }
            const glm::vec3 envRadiance = lights.environmentRadiance(ray.dir, /*nearest=*/false);
            // Power heuristic (Veach 1997): full weight for bounce 0 (camera ray, not part of the MIS estimator) and after a delta sample, where NEE has zero density and there is nothing to balance against. A ROUGH transmission sample is MIS-eligible like any other lobe and takes the heuristic -- NEE reaches the far side of a transmissive vertex (see the NEE block below), so weighting it at 1.0 double-counted their overlap.
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

        // An emitter hit (a quad light's own two triangles, injected into the BVH so they occlude and
        // are BSDF-hittable): Le, MIS-weighted exactly like the environment miss above, then terminate --
        // a pure emitter has no BSDF to continue sampling from (Arnold quad_light semantics). Checked
        // BEFORE the depth cap below, symmetric with the miss branch above: the extra iteration past
        // maxBounces exists so the final BSDF-sampled ray can still collect ITS light contribution,
        // whichever light it reaches, and an emitter hit is exactly as eligible as a miss is.
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

        // Depth cap. The extra iteration past maxBounces exists solely so the final BSDF-sampled ray can collect its MIS-weighted light contribution in the miss/emitter-hit branches above; a ray reaching ordinary (non-emitting) geometry here contributes nothing and must not shade. Without it, NEE at the final vertex is MIS-weighted down against a BSDF-sampling counterpart that never fires, losing bsdfPdf^2/(bsdfPdf^2 + lightPdf^2) of that vertex's direct lighting -- approaching 100% where bsdfPdf >> lightPdf, and applying to every second surface vertex at maxBounces==1. Breaking here keeps terminationBounce == maxBounces + 1 for a depth-capped path, unchanged from before.
        if (bounce > settings.maxBounces) {
            break;
        }

        const Material& material =
            instances[static_cast<std::size_t>(triangle.instanceIndex)].material;
        const PathTraceSettings& instanceSettings =
            perInstanceSettings[static_cast<std::size_t>(triangle.instanceIndex)];

        // Dispersion: commit the path to one RGB channel on first reaching a dispersive interface, before
        // any BSDF work at this vertex, since it is the whole interaction that is wavelength dependent --
        // Fresnel and the lobe probabilities as much as the refraction direction -- and every later vertex
        // then stays on that wavelength, medium included. Committing here rather than at path start, as a
        // spectral hero-wavelength renderer must (Wilkie et al. 2014), is strictly cheaper: a path that
        // never meets dispersive glass keeps full RGB and pays nothing.
        // One-sample channel estimator with probabilities proportional to the throughput carried so far
        // (OpenPBR implementation note, arXiv:2512.23696): the surviving channel takes T_c/p_c, which is
        // sum(T) for every c, so the estimator is unbiased AND the path's magnitude -- hence Russian
        // roulette's continuation probability below -- no longer depends on which channel was drawn.
        // The heroChannel guard is a draw the estimator does not need, not a correctness gate: masking
        // leaves two channels exactly zero, so a throughput-weighted redraw at a later dispersive vertex
        // can only return the same channel (measured: 400k redraws, zero changed). What it buys is one
        // fewer sampler dimension per later crossing. Under uniform 1/3 selection it WOULD be load-bearing.
        // sum == 0 is reachable, not impossible: rrMinProb floors the roulette, so a zero-throughput path
        // survives rather than being killed. It carries no energy to any channel, so there is nothing to
        // commit and the draw is skipped.
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
        // True flat per-triangle plane normal -- used below for the normal-map light-leak rejection and for offsetting shadow/continuation ray origins off the surface, both of which need the actual geometry rather than the interpolated or normal-mapped shading normal.
        const glm::vec3 geoNormal = geometricNormalOf(triangle);

        if (bounce == 0) {
            gShadow = 1.0F;  // assume shadowed once we know there's a real surface; the NEE check below may clear this
        }

        const glm::vec3 woLocal = frame.toLocal(woWorld);
        // A failed sample must NOT skip the NEE block below: NEE and the continuing ray are independent estimators of independent directions, sharing only this vertex's params/frame, so the failure of one says nothing about the other. sampleBsdf returns nullopt on a below-horizon VNDF reflection, an underflowed mixture pdf, or a transmission lobe with no mass (bsdf.cpp) -- none of which say anything about the BSDF's value toward the light. The terminating break is therefore deferred to after NEE, matching the geometric-consistency rejection further down, which already breaks there.
        const std::optional<BsdfSample> sample = sampleBsdf(params, woLocal, sampler);

        // Bucket assignment: bounce 0 sets the path's bucket from scratch; any later bounce only ever overrides it to Refraction (sticky -- once a path passes through a transmission lobe, its remaining contribution is refraction transport regardless of what it was before). Skipped entirely when no lobe could be sampled, leaving pathBucket unset at bounce 0 -- the same convention addToBucket already applies to an unbucketed background contribution.
        if (sample.has_value()) {
            if (bounce == 0) {
                pathBucket = sample->type == LobeType::Transmission ? PathBucket::Refraction
                             : sample->type == LobeType::Diffuse            ? PathBucket::Diffuse
                                                                             : PathBucket::SpecularReflection;
            } else if (sample->type == LobeType::Transmission) {
                pathBucket = PathBucket::Refraction;
            }
        }

        // Medium-state toggle, co-located with the bucket assignment above since both key off the same
        // condition: a Transmission sample crossed the interface. Entering (currently vacuum) starts
        // absorbing at this instance's sigmaA; exiting (already inside) returns to vacuum. Reflection
        // bounces, including TIR, leave the state untouched -- the ray stays in whichever medium it was
        // already in.
        if (sample.has_value() && sample->type == LobeType::Transmission) {
            mediumSigmaA = mediumSigmaA.has_value()
                               ? std::nullopt
                               : std::make_optional(sigmaAFromTransmission(
                                     instanceSettings.transmissionColor,
                                     instanceSettings.transmissionDepth));
        }

        // Next-event estimation: sample a light directly from this vertex (LightSet::sample -- the
        // environment map and/or any rectangular emitters, selected uniformly and then importance-
        // sampled within whichever was picked, see light.h), evaluate the combined BSDF value/pdf
        // toward it, and add the MIS-weighted contribution if unoccluded. Independent of whichever lobe
        // `sample` above drew for the continuing bounce -- NEE and the continuing ray are two separate
        // estimators for two separate directions from the same vertex, only sharing this vertex's
        // params/frame. Firing unconditionally (no lobe-type check) is deliberate: the guard below
        // admits both sides of a transmissive vertex, so refraction is light-sampled rather than left
        // to BSDF sampling alone, and the MIS weights above and below now balance the same two
        // strategies over the same directions. nullopt (no light in the set, or a degenerate quad at
        // this exact vertex) simply skips NEE for this vertex -- the continuing ray still fires below.
        const std::optional<LightSample> lightSample = lights.sample(shading.position, sampler);
        if (lightSample.has_value()) {
            const float geoCos = glm::dot(lightSample->direction, geoNormal);
            const float shadingCos = glm::dot(lightSample->direction, frame.normal);
            // Both sides, not just wo's: on a transmissive surface a light behind the vertex reaches the eye through the transmission lobe, so restricting NEE to the near side left rough glass lit by BSDF sampling alone. Requiring geoCos and shadingCos to agree in sign keeps the normal-map light-leak rejection intact on either side. The lobe itself decides whether the far side carries anything -- a delta interface returns pdfBsdf == 0 there and the guard below drops it, which is correct since a delta lobe cannot be light-sampled.
            const bool nearSide = geoCos > 0.0F && shadingCos > 0.0F;
            const bool farSide = geoCos < 0.0F && shadingCos < 0.0F && params.transmissionFactor > 0.0F;
            if (nearSide || farSide) {
                const glm::vec3 wiLocalLight = frame.toLocal(lightSample->direction);
                const float lightCos = std::abs(shadingCos);  // far-side samples carry a negative cosine
                // One evaluation for the value, the pdf and the per-lobe split the transport AOVs need -- the four separate calls this replaced (evaluateBsdf, pdfBsdf, and one isolating call per lobe) each recomputed the same lobe probabilities, GGX/Fresnel terms and albedo-table lookups.
                const BsdfEval eval = evaluateBsdfSplit(params, woLocal, wiLocalLight);
                const glm::vec3 bsdfValue = eval.total();
                if (eval.pdf > 0.0F &&
                    (bsdfValue.x > 0.0F || bsdfValue.y > 0.0F || bsdfValue.z > 0.0F)) {
                    // Offset along geoNormal toward whichever side the light sample is on -- the far side for a transmissive vertex lit from behind, the near side otherwise.
                    // Far side crosses the interface like a transmission continuation ray, so it takes the same curvature-scaled offset: kRayEpsilon does not clear the neighbouring facet on curved geometry, leaking env radiance through a surface that should occlude it. Near side keeps kRayEpsilon; flat geometry collapses to it.
                    const float shadowEpsilon =
                        farSide ? transmissionOffsetEpsilon(triangle) : kRayEpsilon;
                    const glm::vec3 shadowOrigin =
                        shadowTerminatorOffset(triangle, hit->u, hit->v, geoCos > 0.0F) +
                        (geoNormal * shadowEpsilon * (geoCos > 0.0F ? 1.0F : -1.0F));
                    const Ray shadowRay{shadowOrigin, lightSample->direction, shadowEpsilon,
                                         lightSample->distance * (1.0F - kShadowDistanceEpsilon)};
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
                            // Bounce 0's NEE contribution is split across the buckets by the LOBE THAT CARRIED IT, deterministically and at its own physical value -- never routed by sample->type, which is the lobe the continuation ray happened to draw and has nothing to do with NEE. The three components partition eval.total() exactly (bsdf.h), so this writes the same energy `radiance` just took, only attributed. `common` carries throughput as a factor shared by `radiance` and all three accumulators, so the partition holds whatever throughput is -- including the two exactly-zero channels a dispersive path is masked to, which the hero-channel block above can set before this point at bounce 0 (measured: 132461 of 2000000 bounce-0 NEE splits reach here non-unit).
                            directDiffuseAccum += eval.diffuse * common;
                            directSpecularAccum += eval.specular * common;
                            refractionAccum += eval.transmission * common;
                        } else {
                            // Deeper bounces keep the path's sticky bucket at the full physical contribution: which lobe carries the light at THIS vertex no longer names the transport type of a path already bucketed at bounce 0, and this vertex's own colour legitimately tints indirect transport.
                            addToBucket(neeContribution, /*isDirect=*/false);
                        }
                    }
                }
            }
        }

        // No continuation direction could be sampled -- terminate here, after NEE has already taken this vertex's direct lighting (see sampleBsdf's call site above for why the two are independent).
        if (!sample.has_value()) {
            break;
        }

        lastBsdfPdf = sample->pdf;
        lastSampleWasDelta = lastBsdfPdf <= 0.0F;
        lastShadingPosition = shading.position;

        const glm::vec3 wiWorld = frame.toWorld(sample->wiLocal);

        // Geometric-normal-consistency rejection (normal-map robustness -- simpler stand-in for Schussler et al. 2017's full two-facet microsurface reconstruction): a reflection/diffuse sample crossing to the wrong side of the true triangle plane is a normal-map light-leak artifact, not a physical bounce.
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

        // Chiang/Li/Burley shadow-terminator-corrected origin, nudged off the true triangle plane along the geometric normal (toward wi's side) to avoid self-intersection. Transmission gets a facet-scaled epsilon instead of the fixed one -- see transmissionOffsetEpsilon.
        // The correction is projected toward whichever side wi leaves on: it pulls the hit onto the vertex tangent planes, which moves it strictly along +normal, so on an inward-going ray (refraction entering, or TIR inside a dielectric) the unmirrored form pushes the origin back through the interface. That crossing registers no transmission event, desynchronising the medium stack -- measured at 0.93% of glass entries in cornell, each contributing unattenuated environment radiance on a later miss.
        const bool leavingOnNormalSide = glm::dot(wiWorld, geoNormal) > 0.0F;
        const float offsetEpsilon = sample->type == LobeType::Transmission
                                         ? transmissionOffsetEpsilon(triangle)
                                         : kRayEpsilon;
        const glm::vec3 offsetOrigin =
            shadowTerminatorOffset(triangle, hit->u, hit->v, leavingOnNormalSide) +
            (geoNormal * offsetEpsilon * (leavingOnNormalSide ? 1.0F : -1.0F));
        ray = Ray{offsetOrigin, wiWorld, offsetEpsilon, std::numeric_limits<float>::max()};
    }

    return {radiance,           bounce,               gShadow,
            directDiffuseAccum, indirectDiffuseAccum, directSpecularAccum,
            indirectSpecularAccum, refractionAccum};
}

}  // namespace

PathTraceResult makePathTraceResult(int width, int height) {
    // 8 fields (beauty/bounceHeatmap/shadow + 5 transport-component AOVs) -- see PathTraceResult's declaration order in path_tracer.h, which this positional init must match.
    return {makeImage(width, height), makeImage(width, height), makeImage(width, height),
            makeImage(width, height), makeImage(width, height), makeImage(width, height),
            makeImage(width, height), makeImage(width, height)};
}

void renderPathTraced(const Camera& camera, const EmbreeAccel& accel,
                       const std::vector<ShadingTriangle>& shadingTriangles,
                       const std::vector<MeshInstance>& instances,
                       const std::vector<int>& instanceLightIndex, const LightSet& lights,
                       int width, int height, bool showSky, const PathTraceSettings& settings,
                       const std::vector<PathTraceSettings>& perInstanceSettings,
                       std::uint32_t runSeed, const std::atomic<std::uint64_t>& generation,
                       std::uint64_t requestedGeneration, ThreadPool& threadPool,
                       PathTraceResult& out) {
    const float aspect = static_cast<float>(width) / static_cast<float>(height);
    // Constant for the whole pass, so it is built once here rather than per primary ray: the aspect-taking primaryRay rebuilds it every call, which at samplesPerPixel rays per pixel is millions of identical reconstructions per pass. rasterizer.cpp already hoists it the same way.
    const Camera::ViewBasis basis = camera.viewBasis(aspect);
    const int tilesX = (width + kTileSize - 1) / kTileSize;
    const int tilesY = (height + kTileSize - 1) / kTileSize;

    // One worker owns every output pixel of one tile, and traces every pixel within the filter radius of it -- the kFilterExtent-wide halo, whose samples are therefore traced twice, once by each of the two tiles they splat into. Sampler is seeded per (x, y, s, runSeed), so both tiles compute the identical sample; the cost is ~13% more rays at this tile size, and what it buys is that no splat ever crosses into another worker's pixels, so the whole pass needs no locks, no atomics and no merge phase.
    const auto renderTile = [&](int tileIndex) {
        const int tileX0 = (tileIndex % tilesX) * kTileSize;
        const int tileY0 = (tileIndex / tilesX) * kTileSize;
        const int tileX1 = std::min(tileX0 + kTileSize, width);
        const int tileY1 = std::min(tileY0 + kTileSize, height);

        // Reused for the life of the worker thread, so a pass allocates nothing: sized for a full tile even at the image edge, which keeps the row stride a constant kTileSize.
        thread_local std::vector<float> accumulator;
        accumulator.assign(static_cast<std::size_t>(kTileSize) * kTileSize * kTileLanes, 0.0F);

        for (int y = std::max(tileY0 - kFilterExtent, 0);
             y < std::min(tileY1 + kFilterExtent, height); ++y) {
            for (int x = std::max(tileX0 - kFilterExtent, 0);
                 x < std::min(tileX1 + kFilterExtent, width); ++x) {
                for (int s = 0; s < settings.samplesPerPixel; ++s) {
                    Sampler sampler(x, y, s, runSeed);
                    const glm::vec2 jitter = sampler.next2D();
                    const float filmX = static_cast<float>(x) + jitter.x;
                    const float filmY = static_cast<float>(y) + jitter.y;
                    const float ndcX = ((filmX / static_cast<float>(width)) * 2.0F) - 1.0F;
                    // HdrImage row 0 is the top (EXR/glTF convention); NDC +Y is up -- flip.
                    const float ndcY = 1.0F - ((filmY / static_cast<float>(height)) * 2.0F);
                    const Ray primary = camera.primaryRay(basis, ndcX, ndcY);
                    const TraceResult trace =
                        tracePath(primary, accel, shadingTriangles, instances, instanceLightIndex,
                                  lights, showSky, settings, perInstanceSettings, sampler);
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
                        trace.refraction.y,        trace.refraction.z};

                    // Clipped to this tile: the taps falling outside it belong to a neighbouring tile, which traces this same sample itself rather than receiving it.
                    const int splatX0 = std::max(tileX0, static_cast<int>(std::ceil(filmX - 0.5F - kFilterRadius)));
                    const int splatX1 = std::min(tileX1 - 1, static_cast<int>(std::floor(filmX - 0.5F + kFilterRadius)));
                    const int splatY0 = std::max(tileY0, static_cast<int>(std::ceil(filmY - 0.5F - kFilterRadius)));
                    const int splatY1 = std::min(tileY1 - 1, static_cast<int>(std::floor(filmY - 0.5F + kFilterRadius)));
                    for (int splatY = splatY0; splatY <= splatY1; ++splatY) {
                        // Separable: the 2D weight is the product of the two 1D lookups, so a row's own factor is hoisted out of the inner loop.
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
                                ((static_cast<std::size_t>(splatY - tileY0) * kTileSize) +
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
                                      ((static_cast<std::size_t>(y - tileY0) * kTileSize) +
                                       static_cast<std::size_t>(x - tileX0)) * kTileLanes;
                // Always positive: a pixel's own samples land within half a pixel of its centre, well inside the 1.5px support, and samplesPerPixel is at least 1.
                const float invWeight = 1.0F / lanes[kSampleLanes];
                writeTexel(out.beauty, x, y, glm::vec3(lanes[0], lanes[1], lanes[2]) * invWeight);
                writeTexel(out.bounceHeatmap, x, y, glm::vec3(lanes[3] * invWeight));
                writeTexel(out.shadow, x, y, glm::vec3(lanes[4] * invWeight));
                writeTexel(out.directDiffuse, x, y, glm::vec3(lanes[5], lanes[6], lanes[7]) * invWeight);
                writeTexel(out.indirectDiffuse, x, y, glm::vec3(lanes[8], lanes[9], lanes[10]) * invWeight);
                writeTexel(out.directSpecular, x, y, glm::vec3(lanes[11], lanes[12], lanes[13]) * invWeight);
                writeTexel(out.indirectSpecular, x, y, glm::vec3(lanes[14], lanes[15], lanes[16]) * invWeight);
                writeTexel(out.refraction, x, y, glm::vec3(lanes[17], lanes[18], lanes[19]) * invWeight);
            }
        }
    };

    threadPool.parallelFor(tilesX * tilesY, [&renderTile, &generation, requestedGeneration](int tileIndex) {
        if (generation.load(std::memory_order_relaxed) != requestedGeneration) {
            return;  // stale -- caller discards this pass's result entirely
        }
        renderTile(tileIndex);
    });
}

}  // namespace engine::scene
