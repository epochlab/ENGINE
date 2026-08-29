#pragma once

#include <optional>

#include <glm/glm.hpp>

#include "engine/scene/sampler.h"

namespace engine::scene {

// Resolved shading parameters at a hit point (textures already sampled by the caller).
struct BsdfParams {
    glm::vec3 baseColor;
    float metallic;
    float roughness;            // perceptual; alpha = roughness^2, floored to avoid a delta lobe
    glm::vec3 f0;                // specular reflectance at normal incidence
    float ior;                   // dielectric IOR, non-metal lobes only
    float transmissionFactor;    // KHR_materials_transmission, 0 = opaque
};

// Local shading frame (z = shading normal) for world<->local direction transforms.
struct ShadingFrame {
    glm::vec3 tangent;
    glm::vec3 bitangent;
    glm::vec3 normal;
    [[nodiscard]] glm::vec3 toLocal(const glm::vec3& v) const {
        return {glm::dot(v, tangent), glm::dot(v, bitangent), glm::dot(v, normal)};
    }
    [[nodiscard]] glm::vec3 toWorld(const glm::vec3& v) const {
        return (v.x * tangent) + (v.y * bitangent) + (v.z * normal);
    }
};

// Which lobe sampleBsdf drew from -- used by path_tracer.cpp to bucket radiance into the AOV transport-component breakdown (Direct/Indirect Diffuse/Specular, Refraction) and to skip NEE's shadow ray at a delta vertex (SpecularTransmission has no continuous pdf, so a light sample can never land on it).
enum class LobeType { Diffuse, SpecularReflection, SpecularTransmission };

struct BsdfSample {
    glm::vec3 wiLocal;            // sampled direction, local shading frame
    glm::vec3 throughputWeight;   // f(wi)*|cosThetaI| / pdf(wi)
    LobeType type;
    // For Diffuse/SpecularReflection, the sampled lobe's OWN pdf-cancelled weight in isolation (its own f, its own pdf -- not evaluateContinuousLobes' mixture pdf), excluding the other lobe's admixture at the same wi: `kd` (no baseColor) for Diffuse, `F*G2/G1` plus the multiple-scattering term for SpecularReflection (Heitz 2018 VNDF weight identity, no baseColor at metallic=0). Identical to throughputWeight for SpecularTransmission. Lets path_tracer.cpp's delighted Direct/Indirect Diffuse/Specular AOVs access each lobe's own contribution without the other lobe's texture/energy bleeding in.
    // Known gap, AOV-only: the multiple-scattering lobe has no sampling strategy of its own, so it lands in BOTH raw weights imprecisely -- a Diffuse-typed sample carries multiple-scattering energy this field reports as diffuse, and a SpecularReflection-typed one divides it by the VNDF pdf, which is not the density it was drawn from. Beauty is exact either way (throughputWeight is the physical value, and the mixture pdf is the true sampling density); only the delighted split is approximate, and the physical transport buckets remove the mechanism entirely.
    glm::vec3 rawThroughputWeight;
};

// Schlick's approximation of Fresnel reflectance at normal-incidence reflectance f0, evaluated at cosTheta = dot(normal, direction). Shared with path_tracer.cpp's Fresnel G-buffer AOV, which wants the same reflectance term sampleBsdf/evaluateBsdf use internally, evaluated at the view angle rather than the sampled/shading direction.
[[nodiscard]] glm::vec3 fresnelSchlick(float cosTheta, const glm::vec3& f0);

// Combined pdf of the two continuous lobes (specular reflection + diffuse) at wiLocal; excludes the delta transmission lobe (zero-measure). woLocal.z sign: entering (>0) vs exiting (<0) a dielectric.
[[nodiscard]] float pdfBsdf(const BsdfParams& params, const glm::vec3& woLocal,
                            const glm::vec3& wiLocal);

// Combined value of the two continuous lobes at wiLocal -- see pdfBsdf.
[[nodiscard]] glm::vec3 evaluateBsdf(const BsdfParams& params, const glm::vec3& woLocal,
                                      const glm::vec3& wiLocal);

// The diffuse lobe's value at wiLocal with its baseColor factor excluded (kd/pi instead of baseColor*kd/pi) -- the "light before albedo" quantity path_tracer.cpp's delighted diffuse AOVs need. Zero if wiLocal is below the (sign-corrected) hemisphere, same convention as evaluateBsdf.
[[nodiscard]] glm::vec3 evaluateDiffuseRaw(const BsdfParams& params, const glm::vec3& woLocal,
                                            const glm::vec3& wiLocal);

// The specular lobe's value at wiLocal alone, excluding the diffuse term evaluateBsdf combines it with -- the DirectSpecular/IndirectSpecular AOV's counterpart to evaluateDiffuseRaw, isolating NEE's own-vertex specular contribution from the mixed diffuse+specular value. Unlike evaluateDiffuseRaw this needs no albedo-factor removal (the specular lobe is already physical, not texture-modulated the way baseColor modulates diffuse).
[[nodiscard]] glm::vec3 evaluateSpecularOnly(const BsdfParams& params, const glm::vec3& woLocal,
                                              const glm::vec3& wiLocal);

// Stochastically samples one of {rough specular reflection, diffuse, smooth specular transmission} by Fresnel-derived probability, returns the ready-to-multiply throughput weight. Diffuse+specular combine via the one-sample mixture estimator (both lobes evaluated at whichever wi was drawn, not just the sampled lobe -- required since a rough surface's lobes overlap). The specular selection probability is scaled by the GGX directional albedo E(mu_o, roughness), so VNDF sampling takes the single-scattering share and the cosine strategy takes the (cosine-shaped) multiple-scattering share. Transmission: Snell's law, TIR folded into the specular probability, single non-nested dielectric boundary. Returns nullopt if fully absorbed.
[[nodiscard]] std::optional<BsdfSample> sampleBsdf(const BsdfParams& params,
                                                    const glm::vec3& woLocal, Sampler& sampler);

}  // namespace engine::scene
