#include "engine/scene/path_tracer.h"

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <limits>
#include <optional>

#include "engine/scene/bsdf.h"
#include "engine/scene/false_color.h"
#include "engine/scene/gbuffer_shading.h"
#include "engine/scene/sampler.h"
#include "engine/scene/shading_scene.h"

namespace engine::scene {

namespace {

constexpr float kRayEpsilon = 1e-4F;

struct TraceResult {
    glm::vec3 radiance;
    float firstHitIor;      // -1 if the primary ray never hit geometry
    int terminationBounce;  // bounce index the path stopped at (== maxBounces + 1 if depth-capped)

    // Primary-hit (bounce==0) G-buffer data -- default-valued (primaryHit=false) on a primary miss.
    bool primaryHit = false;
    glm::vec3 worldPos{0.0F};
    glm::vec2 uv{0.0F};
    glm::vec3 normal{0.0F};
    glm::vec3 geomNormal{0.0F};
    glm::vec3 albedo{0.0F};
    float metallic = 0.0F;
    float roughness = 0.0F;
    glm::vec3 tangent{0.0F};
    int objectIndex = -1;
    float fresnel = 0.0F;
    float ao = 0.0F;
    float shadow = 0.0F;  // 1.0 = shadowed/occluded, 0.0 = lit or no primary hit at all (background)

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
    // Mirrors `throughput` exactly except every bounce's lobe pick multiplies in sample->rawThroughputWeight (that lobe's own isolated weight -- kd for Diffuse, F*G2/G1 for SpecularReflection, no admixture from the other lobe) instead of the physical throughputWeight -- used only to compute the delighted Direct/Indirect Diffuse/Specular AOV bucket writes below, never `radiance` itself. Diverges from `throughput` by the cumulative product of every traversed bounce's own-lobe isolation factor (not just bounce 0's).
    glm::vec3 lobeRawThroughput(1.0F);
    Ray ray = primaryRay;
    float firstHitIor = -1.0F;
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
    // Primary-hit (bounce==0) G-buffer locals, folded into the final TraceResult at the end of this function -- captured separately from radiance/firstHitIor/bounce since those keep changing for the rest of the loop after bounce 0, while the G-buffer is a one-shot snapshot.
    bool primaryHit = false;
    glm::vec3 gWorldPos(0.0F);
    glm::vec2 gUv(0.0F);
    glm::vec3 gNormal(0.0F);
    glm::vec3 gGeomNormal(0.0F);
    glm::vec3 gAlbedo(0.0F);
    float gMetallic = 0.0F;
    float gRoughness = 0.0F;
    glm::vec3 gTangent(0.0F);
    int gObjectIndex = -1;
    float gFresnel = 0.0F;
    float gAo = 0.0F;
    float gShadow = 0.0F;  // default: no surface hit at all -- not "shadowed", just background

    // MIS state for the *previous* bounce's BSDF sample (the one that produced `ray`) -- used to reweight this bounce's miss contribution against NEE's light-sampling pdf, so a direction reachable by both strategies isn't double-counted. Meaningless at bounce==0 (ray is the primary/camera ray, not a BSDF sample -- its miss is a pure camera-visibility event, not part of the two-strategy light-transport estimator NEE/MIS balances).
    float lastBsdfPdf = 0.0F;
    bool lastSampleWasTransmission = false;

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
            // Power heuristic (Veach 1997): full weight for bounce 0 (camera ray, not part of the MIS estimator) and after a transmission sample (delta lobe -- NEE has zero density there, so there's no double-counting risk to correct for).
            float misWeight = 1.0F;
            if (bounce > 0 && !lastSampleWasTransmission) {
                const float lightPdf = environmentMap.pdf(ray.dir, envRotationRadians);
                const float bsdfPdf2 = lastBsdfPdf * lastBsdfPdf;
                misWeight = bsdfPdf2 / (bsdfPdf2 + (lightPdf * lightPdf));
            }
            const glm::vec3 missRadiance = throughput * envRadiance * misWeight;
            radiance += missRadiance;
            const bool isolatedBucket = pathBucket == PathBucket::Diffuse ||
                                         pathBucket == PathBucket::SpecularReflection;
            addToBucket(isolatedBucket ? lobeRawThroughput * envRadiance * misWeight : missRadiance,
                        /*isDirect=*/bounce == 1);
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
        // Geometric (unmapped) normal -- needed both for the G-buffer snapshot below and for the normal-map light-leak rejection further down, computed once and reused for both.
        const glm::vec3 geoNormal = geometricNormalOf(triangle);

        if (bounce == 0) {
            firstHitIor = settings.ior;
            primaryHit = true;
            gWorldPos = shading.position;
            gUv = shading.uv;
            gNormal = frame.normal;
            // Smooth interpolated vertex normal, before normal-mapping -- distinct from `geoNormal` (the true flat per-triangle plane normal used below for shadow-ray offsetting and normal-map light-leak rejection, which needs the actual geometry, not this).
            gGeomNormal = glm::normalize(shading.normal);
            gAlbedo = params.baseColor;
            gMetallic = params.metallic;
            gRoughness = params.roughness;
            gTangent = frame.tangent;
            gObjectIndex = triangle.instanceIndex;
            const float ndotV = std::max(glm::dot(frame.normal, woWorld), 1e-4F);
            gFresnel = fresnelSchlick(ndotV, params.f0).x;  // scalar AOV -- see writeTexel's broadcast convention
            gAo = engine::gfx::sampleBilinear(material.aoTexture, shading.uv).r;
            gShadow = 1.0F;  // assume shadowed once we know there's a real surface; the NEE check below may clear this
        }

        const glm::vec3 woLocal = frame.toLocal(woWorld);
        // A failed sample must NOT skip the NEE block below: NEE and the continuing ray are independent estimators of independent directions, sharing only this vertex's params/frame, so the failure of one says nothing about the other. sampleBsdf returns nullopt on a below-horizon VNDF reflection, an underflowed mixture pdf, or a transmission lobe with no mass (bsdf.cpp) -- none of which say anything about the BSDF's value toward the light. The terminating break is therefore deferred to after NEE, matching the geometric-consistency rejection further down, which already breaks there.
        const std::optional<BsdfSample> sample = sampleBsdf(params, woLocal, sampler);

        // Bucket assignment: bounce 0 sets the path's bucket from scratch; any later bounce only ever overrides it to Refraction (sticky -- once a path passes through a transmission lobe, its remaining contribution is refraction transport regardless of what it was before). Skipped entirely when no lobe could be sampled, leaving pathBucket unset at bounce 0 -- the same convention addToBucket already applies to an unbucketed background contribution.
        if (sample.has_value()) {
            if (bounce == 0) {
                pathBucket = sample->type == LobeType::SpecularTransmission ? PathBucket::Refraction
                             : sample->type == LobeType::Diffuse            ? PathBucket::Diffuse
                                                                             : PathBucket::SpecularReflection;
            } else if (sample->type == LobeType::SpecularTransmission) {
                pathBucket = PathBucket::Refraction;
            }
        }

        // Next-event estimation: sample the environment directly from this vertex (importance sampled by luminance, see EnvironmentMap::importanceSampleDirection), evaluate the (combined, delta-transmission-lobe-excluded) BSDF value/pdf toward it, and add the MIS-weighted contribution if unoccluded. Independent of whichever lobe `sample` above drew for the continuing bounce -- NEE and the continuing ray are two separate estimators for two separate directions from the same vertex, only sharing this vertex's params/frame. Firing unconditionally (no lobe-type check) is deliberate: evaluateBsdf/pdfBsdf already return ~0 at a near-pure-transmissive vertex (both structurally exclude the delta transmission lobe), so NEE self-attenuates to negligible cost/contribution there with no special-casing.
        {
            const EnvironmentMap::EnvSample lightSample =
                environmentMap.importanceSampleDirection(sampler.next2D(), envRotationRadians);
            const float geoCos = glm::dot(lightSample.direction, geoNormal);
            const float shadingCos = glm::dot(lightSample.direction, frame.normal);
            if (geoCos > 0.0F && shadingCos > 0.0F) {
                const glm::vec3 wiLocalLight = frame.toLocal(lightSample.direction);
                const glm::vec3 bsdfValue = evaluateBsdf(params, woLocal, wiLocalLight);
                const float bsdfPdf = pdfBsdf(params, woLocal, wiLocalLight);
                if (bsdfPdf > 0.0F &&
                    (bsdfValue.x > 0.0F || bsdfValue.y > 0.0F || bsdfValue.z > 0.0F)) {
                    // Shadow ray offset toward geoNormal only -- geoCos>0 already established the light sample is on that side, unlike the continuing bounce ray below which can go either side depending on wi.
                    const glm::vec3 shadowOrigin =
                        shadowTerminatorOffset(triangle, hit->u, hit->v) + (geoNormal * kRayEpsilon);
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
                        const float bsdfPdf2 = bsdfPdf * bsdfPdf;
                        const float misWeightLight = lightPdf2 / (lightPdf2 + bsdfPdf2);
                        const glm::vec3 neeContribution = throughput * bsdfValue * envRadiance *
                                                            shadingCos * misWeightLight / lightSample.pdf;
                        radiance += neeContribution;
                        if (bounce == 0) {
                            // Bounce 0's NEE contribution is split deterministically across BOTH lobe buckets, never routed by sample->type. Which lobe the *continuation* ray happened to draw has nothing to do with NEE, and routing by it wrote only one bucket per sample -- thinning each in expectation by that lobe's selection probability (a Fresnel- and view-angle-dependent factor, clamped to [0.05,0.95] in computeLobeProbabilities), so an evenly-lit flat wall read as a Fresnel gradient -- while adding the variance of a binary coin flip to an estimator that need not be stochastic. Each lobe is isolated at its own value so the primary surface's own texture never enters the AOV (evaluateDiffuseRaw drops baseColor; the specular lobe's F*G2/G1 carries none at metallic=0). throughput and lobeRawThroughput are both still (1,1,1) here -- neither is multiplied until after this block -- so no multiplier is needed. Deliberately bypasses addToBucket: that routes by pathBucket, which is exactly what must not happen, and which is unset anyway when sampleBsdf failed.
                            const glm::vec3 common =
                                envRadiance * shadingCos * misWeightLight / lightSample.pdf;
                            directDiffuseAccum +=
                                evaluateDiffuseRaw(params, woLocal, wiLocalLight) * common;
                            directSpecularAccum +=
                                evaluateSpecularOnly(params, woLocal, wiLocalLight) * common;
                        } else {
                            // Deeper bounces keep the path's sticky bucket: this vertex's own colour legitimately tints indirect transport, so the full combined bsdfValue is used. Only Diffuse differs from neeContribution, via lobeRawThroughput (which keeps stripping each traversed bounce's own-lobe admixture); Specular and Refraction both reduce to neeContribution exactly.
                            glm::vec3 bucketContribution = neeContribution;
                            if (pathBucket == PathBucket::Diffuse) {
                                bucketContribution = lobeRawThroughput * bsdfValue * envRadiance *
                                                      shadingCos * misWeightLight / lightSample.pdf;
                            }
                            addToBucket(bucketContribution, /*isDirect=*/false);
                        }
                    }
                }
            }
        }

        // No continuation direction could be sampled -- terminate here, after NEE has already taken this vertex's direct lighting (see sampleBsdf's call site above for why the two are independent).
        if (!sample.has_value()) {
            break;
        }

        if (sample->type == LobeType::SpecularTransmission) {
            lastSampleWasTransmission = true;
        } else {
            lastSampleWasTransmission = false;
            lastBsdfPdf = pdfBsdf(params, woLocal, sample->wiLocal);
        }

        const glm::vec3 wiWorld = frame.toWorld(sample->wiLocal);

        // Geometric-normal-consistency rejection (normal-map robustness -- simpler stand-in for Schussler et al. 2017's full two-facet microsurface reconstruction): a reflection/diffuse sample crossing to the wrong side of the true triangle plane is a normal-map light-leak artifact, not a physical bounce.
        if (sample->type != LobeType::SpecularTransmission) {
            const bool woAbove = glm::dot(woWorld, geoNormal) > 0.0F;
            const bool wiAbove = glm::dot(wiWorld, geoNormal) > 0.0F;
            if (woAbove != wiAbove) {
                break;
            }
        }

        throughput *= sample->throughputWeight;
        lobeRawThroughput *= sample->rawThroughputWeight;

        if (bounce >= settings.russianRouletteStartBounce) {
            const float continueProb = std::clamp(
                std::max({throughput.x, throughput.y, throughput.z}), settings.rrMinProb,
                settings.rrMaxProb);
            if (sampler.next1D() >= continueProb) {
                break;
            }
            throughput /= continueProb;
            lobeRawThroughput /= continueProb;
        }

        // Chiang/Li/Burley shadow-terminator-corrected origin, nudged off the true triangle plane along the geometric normal (toward wi's side) to avoid self-intersection.
        const glm::vec3 offsetOrigin =
            shadowTerminatorOffset(triangle, hit->u, hit->v) +
            (geoNormal * kRayEpsilon * (glm::dot(wiWorld, geoNormal) > 0.0F ? 1.0F : -1.0F));
        ray = Ray{offsetOrigin, wiWorld, kRayEpsilon, std::numeric_limits<float>::max()};
    }

    return {radiance,   firstHitIor,          bounce,
            primaryHit, gWorldPos,            gUv,
            gNormal,    gGeomNormal,          gAlbedo,
            gMetallic,  gRoughness,           gTangent,
            gObjectIndex, gFresnel,           gAo,
            gShadow,
            directDiffuseAccum, indirectDiffuseAccum, directSpecularAccum,
            indirectSpecularAccum, refractionAccum};
}

}  // namespace

PathTraceResult renderPathTraced(const Camera& camera, const EmbreeAccel& accel,
                                  const std::vector<ShadingTriangle>& shadingTriangles,
                                  const std::vector<MeshInstance>& instances,
                                  const EnvironmentMap& environmentMap, int width, int height,
                                  float envRotationRadians, bool showSky, float envExposure,
                                  const PathTraceSettings& settings, std::uint32_t runSeed,
                                  const std::atomic<std::uint64_t>& generation,
                                  std::uint64_t requestedGeneration, RowThreadPool& threadPool) {
    // 22 fields (beauty/iorAov/bounceHeatmap + 14 G-buffer AOVs + 5 transport-component AOVs) -- see PathTraceResult's declaration order in path_tracer.h, which this positional init must match.
    PathTraceResult result{makeImage(width, height), makeImage(width, height),
                            makeImage(width, height), makeImage(width, height),
                            makeImage(width, height), makeImage(width, height),
                            makeImage(width, height), makeImage(width, height),
                            makeImage(width, height), makeImage(width, height),
                            makeImage(width, height), makeImage(width, height),
                            makeImage(width, height), makeImage(width, height),
                            makeImage(width, height), makeImage(width, height),
                            makeImage(width, height), makeImage(width, height),
                            makeImage(width, height), makeImage(width, height),
                            makeImage(width, height), makeImage(width, height)};
    const float aspect = static_cast<float>(width) / static_cast<float>(height);
    const float sppInv = 1.0F / static_cast<float>(settings.samplesPerPixel);
    const glm::vec3 cameraPos = camera.position();
    const glm::vec3 cameraForward = camera.forward();

    const auto renderRow = [&](int y) {
        for (int x = 0; x < width; ++x) {
            glm::vec3 colorAccum(0.0F);
            float iorSample = -1.0F;
            float bounceAccum = 0.0F;
            glm::vec3 directDiffuseAccum(0.0F);
            glm::vec3 indirectDiffuseAccum(0.0F);
            glm::vec3 directSpecularAccum(0.0F);
            glm::vec3 indirectSpecularAccum(0.0F);
            glm::vec3 refractionAccum(0.0F);
            TraceResult primarySample{};  // sample 0's G-buffer snapshot, see PathTraceResult's doc comment
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
                if (s == 0) {
                    iorSample = trace.firstHitIor;
                    primarySample = trace;
                }
                bounceAccum += static_cast<float>(trace.terminationBounce);
                directDiffuseAccum += trace.directDiffuse;
                indirectDiffuseAccum += trace.indirectDiffuse;
                directSpecularAccum += trace.directSpecular;
                indirectSpecularAccum += trace.indirectSpecular;
                refractionAccum += trace.refraction;
            }
            writeTexel(result.beauty, x, y, colorAccum * sppInv);
            writeTexel(result.iorAov, x, y, glm::vec3(iorSample));
            writeTexel(result.bounceHeatmap, x, y, glm::vec3(bounceAccum * sppInv));
            writeTexel(result.directDiffuse, x, y, directDiffuseAccum * sppInv);
            writeTexel(result.indirectDiffuse, x, y, indirectDiffuseAccum * sppInv);
            writeTexel(result.directSpecular, x, y, directSpecularAccum * sppInv);
            writeTexel(result.indirectSpecular, x, y, indirectSpecularAccum * sppInv);
            writeTexel(result.refraction, x, y, refractionAccum * sppInv);

            const float depth = primarySample.primaryHit
                                     ? glm::dot(primarySample.worldPos - cameraPos, cameraForward)
                                     : 0.0F;
            writeTexel(result.depth, x, y, glm::vec3(depth));
            writeTexel(result.worldPos, x, y, primarySample.worldPos);
            writeTexel(result.uv, x, y, glm::vec3(glm::fract(primarySample.uv), 0.0F));
            writeTexel(result.normal, x, y, primarySample.normal);
            writeTexel(result.geomNormal, x, y, primarySample.geomNormal);
            writeTexel(result.albedo, x, y, primarySample.albedo);
            writeTexel(result.metallic, x, y, glm::vec3(primarySample.metallic));
            writeTexel(result.roughness, x, y, glm::vec3(primarySample.roughness));
            writeTexel(result.tangent, x, y, primarySample.tangent);
            writeTexel(result.objectId, x, y,
                       primarySample.primaryHit ? falseColorForId(primarySample.objectIndex)
                                                 : glm::vec3(0.0F));
            writeTexel(result.alpha, x, y, glm::vec3(primarySample.primaryHit ? 1.0F : 0.0F));
            writeTexel(result.fresnel, x, y,
                       primarySample.primaryHit
                           ? glm::vec3(primarySample.fresnel, 1.0F - primarySample.fresnel, 0.0F)
                           : glm::vec3(0.0F));
            writeTexel(result.ao, x, y, glm::vec3(primarySample.ao));
            writeTexel(result.shadow, x, y, glm::vec3(primarySample.shadow));
        }
    };

    threadPool.parallelForRows(height, [&renderRow, &generation, requestedGeneration](int y) {
        if (generation.load(std::memory_order_relaxed) != requestedGeneration) {
            return;  // stale -- caller discards this pass's result entirely
        }
        renderRow(y);
    });

    return result;
}

}  // namespace engine::scene
