#include "engine/scene/path_tracer.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>

#include "engine/scene/bsdf.h"
#include "engine/scene/gbuffer_shading.h"
#include "engine/scene/sampler.h"
#include "engine/scene/shading_scene.h"

namespace engine::scene {

namespace {

constexpr float kRayEpsilon = 1e-4F;

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
                       const EnvironmentMap& environmentMap, float envRotationRadians,
                       bool showSky, float envExposure, const PathTraceSettings& settings,
                       Sampler& sampler) {
    glm::vec3 radiance(0.0F);
    glm::vec3 throughput(1.0F);
    Ray ray = primaryRay;
    int bounce = 0;
    std::optional<PathBucket> pathBucket;  // unset until bounce 0 successfully samples a lobe
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

    // bounce 0 (the primary/camera ray, direct lighting via NEE) always traces regardless of maxBounces -- maxBounces counts secondary/indirect bounces beyond it, so maxBounces==0 means direct lighting only, no continuation rays. The loop runs one iteration PAST maxBounces so the final BSDF-sampled ray can still collect its MIS-weighted environment contribution via the miss branch below; that extra iteration breaks at the depth guard before any surface interaction -- see the guard for why the terminal ray must be traced rather than dropped.
    for (; bounce <= settings.maxBounces + 1; ++bounce) {
        const std::optional<Hit> hit = accel.intersect(ray);
        if (!hit.has_value()) {
            // showSky gates only the primary ray's own miss (the camera seeing the background directly) -- indirect bounces and NEE (below) always sample real environment radiance regardless of showSky, so hiding the background doesn't unlight the scene.
            if (bounce == 0 && !showSky) {
                break;
            }
            const glm::vec3 envRadiance =
                environmentMap.sampleDirection(ray.dir, envRotationRadians) * envExposure;
            // Power heuristic (Veach 1997): full weight for bounce 0 (camera ray, not part of the MIS estimator) and after a delta sample, where NEE has zero density and there is nothing to balance against. A ROUGH transmission sample is MIS-eligible like any other lobe and takes the heuristic -- NEE reaches the far side of a transmissive vertex (see the NEE block below), so weighting it at 1.0 double-counted their overlap.
            float misWeight = 1.0F;
            if (bounce > 0 && !lastSampleWasDelta) {
                const float lightPdf = environmentMap.pdf(ray.dir, envRotationRadians);
                const float bsdfPdf2 = lastBsdfPdf * lastBsdfPdf;
                misWeight = bsdfPdf2 / (bsdfPdf2 + (lightPdf * lightPdf));
            }
            const glm::vec3 missRadiance = throughput * envRadiance * misWeight;
            radiance += missRadiance;
            addToBucket(missRadiance, /*isDirect=*/bounce == 1);
            break;
        }

        // Depth cap. The extra iteration past maxBounces exists solely so the final BSDF-sampled ray can collect its MIS-weighted environment contribution in the miss branch above; a ray reaching real geometry here contributes nothing (no emissive surfaces) and must not shade. Without it, NEE at the final vertex is MIS-weighted down against a BSDF-sampling counterpart that never fires, losing bsdfPdf^2/(bsdfPdf^2 + lightPdf^2) of that vertex's direct lighting -- approaching 100% where bsdfPdf >> lightPdf, and applying to every second surface vertex at maxBounces==1. Breaking here keeps terminationBounce == maxBounces + 1 for a depth-capped path, unchanged from before.
        if (bounce > settings.maxBounces) {
            break;
        }

        const ShadingTriangle& triangle =
            shadingTriangles[static_cast<std::size_t>(hit->triangleIndex)];
        const Material& material =
            instances[static_cast<std::size_t>(triangle.instanceIndex)].material;

        const ShadingVertex shading = interpolateShading(triangle, hit->u, hit->v);
        const ShadingFrame frame = buildShadingFrame(shading, material, settings);
        const BsdfParams params = resolveBsdfParams(material, shading.uv, settings);
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

        // Next-event estimation: sample the environment directly from this vertex (importance sampled by luminance, see EnvironmentMap::importanceSampleDirection), evaluate the combined BSDF value/pdf toward it, and add the MIS-weighted contribution if unoccluded. Independent of whichever lobe `sample` above drew for the continuing bounce -- NEE and the continuing ray are two separate estimators for two separate directions from the same vertex, only sharing this vertex's params/frame. Firing unconditionally (no lobe-type check) is deliberate: the guard below admits both sides of a transmissive vertex, so refraction is light-sampled rather than left to BSDF sampling alone, and the MIS weights above and below now balance the same two strategies over the same directions.
        {
            const EnvironmentMap::EnvSample lightSample =
                environmentMap.importanceSampleDirection(sampler.next2D(), envRotationRadians);
            const float geoCos = glm::dot(lightSample.direction, geoNormal);
            const float shadingCos = glm::dot(lightSample.direction, frame.normal);
            // Both sides, not just wo's: on a transmissive surface a light behind the vertex reaches the eye through the transmission lobe, so restricting NEE to the near side left rough glass lit by BSDF sampling alone. Requiring geoCos and shadingCos to agree in sign keeps the normal-map light-leak rejection intact on either side. The lobe itself decides whether the far side carries anything -- a delta interface returns pdfBsdf == 0 there and the guard below drops it, which is correct since a delta lobe cannot be light-sampled.
            const bool nearSide = geoCos > 0.0F && shadingCos > 0.0F;
            const bool farSide = geoCos < 0.0F && shadingCos < 0.0F && params.transmissionFactor > 0.0F;
            if (nearSide || farSide) {
                const glm::vec3 wiLocalLight = frame.toLocal(lightSample.direction);
                const float lightCos = std::abs(shadingCos);  // far-side samples carry a negative cosine
                // One evaluation for the value, the pdf and the per-lobe split the transport AOVs need -- the four separate calls this replaced (evaluateBsdf, pdfBsdf, and one isolating call per lobe) each recomputed the same lobe probabilities, GGX/Fresnel terms and albedo-table lookups.
                const BsdfEval eval = evaluateBsdfSplit(params, woLocal, wiLocalLight);
                const glm::vec3 bsdfValue = eval.total();
                if (eval.pdf > 0.0F &&
                    (bsdfValue.x > 0.0F || bsdfValue.y > 0.0F || bsdfValue.z > 0.0F)) {
                    // Offset along geoNormal toward whichever side the light sample is on -- the far side for a transmissive vertex lit from behind, the near side otherwise.
                    const glm::vec3 shadowOrigin =
                        shadowTerminatorOffset(triangle, hit->u, hit->v) +
                        (geoNormal * kRayEpsilon * (geoCos > 0.0F ? 1.0F : -1.0F));
                    const Ray shadowRay{shadowOrigin, lightSample.direction, kRayEpsilon,
                                         std::numeric_limits<float>::max()};
                    if (!accel.occluded(shadowRay)) {
                        if (bounce == 0) {
                            gShadow = 0.0F;
                        }
                        // Nearest, not bilinear: this radiance is divided by lightSample.pdf below, and that pdf is the density of one piecewise-constant texel. Bilinear here would bleed a bright texel's energy into neighbours whose density is correctly low, spiking f/pdf into fireflies at exactly the small bright features the luminance CDF exists to find. The miss path above keeps bilinear -- there the env pdf enters only a bounded MIS weight.
                        const glm::vec3 envRadiance =
                            environmentMap.sampleDirectionNearest(lightSample.direction,
                                                                   envRotationRadians) *
                            envExposure;
                        const float lightPdf2 = lightSample.pdf * lightSample.pdf;
                        const float bsdfPdf2 = eval.pdf * eval.pdf;
                        const float misWeightLight = lightPdf2 / (lightPdf2 + bsdfPdf2);
                        const glm::vec3 common =
                            throughput * envRadiance * lightCos * misWeightLight / lightSample.pdf;
                        const glm::vec3 neeContribution = bsdfValue * common;
                        radiance += neeContribution;
                        if (bounce == 0) {
                            // Bounce 0's NEE contribution is split across the buckets by the LOBE THAT CARRIED IT, deterministically and at its own physical value -- never routed by sample->type, which is the lobe the continuation ray happened to draw and has nothing to do with NEE. The three components partition eval.total() exactly (bsdf.h), so this writes the same energy `radiance` just took, only attributed. throughput is still (1,1,1) here -- it isn't multiplied until after this block -- so `common` is unaffected by it.
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

        // Chiang/Li/Burley shadow-terminator-corrected origin, nudged off the true triangle plane along the geometric normal (toward wi's side) to avoid self-intersection.
        const glm::vec3 offsetOrigin =
            shadowTerminatorOffset(triangle, hit->u, hit->v) +
            (geoNormal * kRayEpsilon * (glm::dot(wiWorld, geoNormal) > 0.0F ? 1.0F : -1.0F));
        ray = Ray{offsetOrigin, wiWorld, kRayEpsilon, std::numeric_limits<float>::max()};
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
                       const EnvironmentMap& environmentMap, int width, int height,
                       float envRotationRadians, bool showSky, float envExposure,
                       const PathTraceSettings& settings, std::uint32_t runSeed,
                       const std::atomic<std::uint64_t>& generation,
                       std::uint64_t requestedGeneration, RowThreadPool& threadPool,
                       PathTraceResult& out) {
    const float aspect = static_cast<float>(width) / static_cast<float>(height);
    const float sppInv = 1.0F / static_cast<float>(settings.samplesPerPixel);

    const auto renderRow = [&](int y) {
        for (int x = 0; x < width; ++x) {
            glm::vec3 colorAccum(0.0F);
            float bounceAccum = 0.0F;
            float shadowAccum = 0.0F;
            glm::vec3 directDiffuseAccum(0.0F);
            glm::vec3 indirectDiffuseAccum(0.0F);
            glm::vec3 directSpecularAccum(0.0F);
            glm::vec3 indirectSpecularAccum(0.0F);
            glm::vec3 refractionAccum(0.0F);
            for (int s = 0; s < settings.samplesPerPixel; ++s) {
                Sampler sampler(x, y, s, runSeed);
                const glm::vec2 jitter = sampler.next2D();
                const float ndcX =
                    (((static_cast<float>(x) + jitter.x) / static_cast<float>(width)) * 2.0F) - 1.0F;
                // HdrImage row 0 is the top (EXR/glTF convention); NDC +Y is up -- flip.
                const float ndcY =
                    1.0F -
                    (((static_cast<float>(y) + jitter.y) / static_cast<float>(height)) * 2.0F);
                const Ray primary = camera.primaryRay(ndcX, ndcY, aspect);
                const TraceResult trace = tracePath(primary, accel, shadingTriangles, instances,
                                                     environmentMap, envRotationRadians, showSky,
                                                     envExposure, settings, sampler);
                colorAccum += trace.radiance;
                bounceAccum += static_cast<float>(trace.terminationBounce);
                shadowAccum += trace.shadow;
                directDiffuseAccum += trace.directDiffuse;
                indirectDiffuseAccum += trace.indirectDiffuse;
                directSpecularAccum += trace.directSpecular;
                indirectSpecularAccum += trace.indirectSpecular;
                refractionAccum += trace.refraction;
            }
            writeTexel(out.beauty, x, y, colorAccum * sppInv);
            writeTexel(out.bounceHeatmap, x, y, glm::vec3(bounceAccum * sppInv));
            writeTexel(out.shadow, x, y, glm::vec3(shadowAccum * sppInv));
            writeTexel(out.directDiffuse, x, y, directDiffuseAccum * sppInv);
            writeTexel(out.indirectDiffuse, x, y, indirectDiffuseAccum * sppInv);
            writeTexel(out.directSpecular, x, y, directSpecularAccum * sppInv);
            writeTexel(out.indirectSpecular, x, y, indirectSpecularAccum * sppInv);
            writeTexel(out.refraction, x, y, refractionAccum * sppInv);
        }
    };

    threadPool.parallelForRows(height, [&renderRow, &generation, requestedGeneration](int y) {
        if (generation.load(std::memory_order_relaxed) != requestedGeneration) {
            return;  // stale -- caller discards this pass's result entirely
        }
        renderRow(y);
    });
}

}  // namespace engine::scene
