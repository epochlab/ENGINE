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

struct BsdfSample {
    glm::vec3 wiLocal;            // sampled direction, local shading frame
    glm::vec3 throughputWeight;   // f(wi)*|cosThetaI| / pdf(wi)
    bool specular;                 // true for the delta transmission lobe (no continuous pdf)
};

// Combined pdf of the two continuous lobes (specular reflection + diffuse) at wiLocal; excludes the delta transmission lobe (zero-measure). woLocal.z sign: entering (>0) vs exiting (<0) a dielectric.
[[nodiscard]] float pdfBsdf(const BsdfParams& params, const glm::vec3& woLocal,
                            const glm::vec3& wiLocal);

// Combined value of the two continuous lobes at wiLocal -- see pdfBsdf.
[[nodiscard]] glm::vec3 evaluateBsdf(const BsdfParams& params, const glm::vec3& woLocal,
                                      const glm::vec3& wiLocal);

// Stochastically samples one of {rough specular reflection, diffuse, smooth specular transmission}
// by Fresnel-derived probability, returns the ready-to-multiply throughput weight. Diffuse+specular
// combine via the one-sample mixture estimator (both lobes evaluated at whichever wi was drawn, not
// just the sampled lobe -- required since a rough surface's lobes overlap). Transmission: Snell's
// law, TIR folded into the specular probability, single non-nested dielectric boundary. Returns
// nullopt if fully absorbed.
[[nodiscard]] std::optional<BsdfSample> sampleBsdf(const BsdfParams& params,
                                                    const glm::vec3& woLocal, Sampler& sampler);

}  // namespace engine::scene
