#pragma once

#include <optional>

#include <glm/glm.hpp>

#include "pathtracer/scene/sampler.h"

namespace pathtracer::scene {

// Resolved shading parameters at a hit point (textures already sampled by the caller).
struct BsdfParams {
    // OpenPBR base_color, the normal-incidence reflection colour under uniform light; resolveBsdfParams derives f0 and diffuseRho from it.
    glm::vec3 baseColor;
    float metallic;
    float roughness;  // perceptual; alpha = roughness^2, floored to avoid a delta lobe
    // Specular reflectance at normal incidence; also Gulbrandsen reflectivity r, clamped to [1e-4, 0.9999] at use.
    glm::vec3 f0;
    // Gulbrandsen 2014 edgetint g: 1 = white edge (Schlick's value), 0 = max dip. Inverts with f0 to a complex IOR. Inert at metallic=0.
    glm::vec3 edgeTint;
    float ior;  // dielectric IOR, non-metal lobes only
    float transmissionFactor;  // KHR_materials_transmission, 0 = opaque
    // EON rough-diffuse r in [0,1] (Portsmouth, Kutz, Hill 2025, JCGT 14(1)); 0 = Lambertian. Not `roughness`, which drives specular.
    float diffuseRoughness;
    // EON single-scattering albedo rho, the diffuse lobe's own input, not the authored colour. See DERIVATIONS.md "EON albedo inversion".
    glm::vec3 diffuseRho;
    // The transmission lobe's only tint (OpenPBR/Arnold): carried by Beer-Lambert at transmissionDepth > 0, applied per crossing at 0.
    glm::vec3 transmissionTint;
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

// Which lobe sampleBsdf drew from; path_tracer.cpp buckets the transport AOVs by it. Transmission is delta only below the smooth threshold.
enum class LobeType { Diffuse, SpecularReflection, Transmission };

struct BsdfSample {
    glm::vec3 wiLocal;            // sampled direction, local shading frame
    glm::vec3 throughputWeight;   // f(wi)*|cosThetaI| / pdf(wi)
    LobeType type;
    // The mixture density wiLocal was drawn from, equal to pdfBsdf's. Zero for the smooth-glass delta branch, which MIS keys on.
    float pdf;
};

// The BSDF's continuous lobes at one wi, split by transport type in one pass. See DERIVATIONS.md "Transport AOV bucketing".
struct BsdfEval {
    glm::vec3 diffuse;
    glm::vec3 specular;
    glm::vec3 transmission;
    float pdf;
    [[nodiscard]] glm::vec3 total() const { return diffuse + specular + transmission; }
};

// Macro-surface Fresnel at cosTheta = dot(n, wo), dielectric and conductor mixed by metallic. Exists so the Fresnel AOV shows the curve.
[[nodiscard]] glm::vec3 fresnelAtViewAngle(const BsdfParams& params, float cosTheta);

// VNDF half-vector (Heitz 2018) through fresnelAtViewAngle at dot(wo, wh), per Walter 2007. One sample of E[F]: THE CALLER MUST AVERAGE.
[[nodiscard]] glm::vec3 fresnelAtMicrofacet(const BsdfParams& params, const glm::vec3& woLocal, glm::vec2 u);

// Cosine-weighted hemisphere direction about +z, pdf = cos(theta)/pi. The AO lane relies on the pdf cancelling the cosine (Miller 1994).
[[nodiscard]] glm::vec3 sampleCosineHemisphere(glm::vec2 u);

// Cosine-weighted average Fresnel, 2*int_0^1 F(mu)*mu dmu, the Kulla-Conty bounce tint. See DERIVATIONS.md "Average Fresnel quadrature".
[[nodiscard]] glm::vec3 conductorFresnelAvg(const glm::vec3& n, const glm::vec3& k);
[[nodiscard]] float dielectricFresnelAvg(float ior);

// Schlick-split directional albedo E(mu, roughness) = a+b and its mean Eavg. See DERIVATIONS.md "Kulla-Conty energy tables".
[[nodiscard]] glm::vec2 directionalAlbedoSplit(float mu, float roughness);
[[nodiscard]] glm::vec2 averageAlbedoSplit(float roughness);

// The grid those two index. mu is uniform in sqrt(mu), so never assume k/(res-1). See DERIVATIONS.md "Kulla-Conty energy tables".
[[nodiscard]] glm::ivec2 albedoGridRes();
[[nodiscard]] float albedoGridRoughness(float index);
[[nodiscard]] float albedoGridMu(float index);

// EON Appendix A: the rho whose normal-incidence directional albedo equals albedo under uniform light. Identity at r=0 and at albedo=1.
[[nodiscard]] glm::vec3 eonAlbedoInversion(const glm::vec3& albedo, float r);

// Representative wavelength per RGB channel (Adobe's OpenPBR reference); the resulting RGB banding is known, see ROADMAP.md transport #5.
inline constexpr glm::vec3 kRgbWavelengthsNm(620.0F, 540.0F, 450.0F);

// Cauchy n(lambda) from an authored (ior at the d line, Abbe V_d), per KHR_materials_dispersion. See DERIVATIONS.md "Cauchy dispersion".
[[nodiscard]] float cauchyIor(float iorD, float abbe, float lambdaNm);

// Value and pdf of the continuous lobes at wiLocal, split by transport type; one call, so GGX, Fresnel and albedo are computed once.
[[nodiscard]] BsdfEval evaluateBsdfSplit(const BsdfParams& params, const glm::vec3& woLocal,
                                          const glm::vec3& wiLocal);

// evaluateBsdfSplit's total() and pdf as named entry points for the validators; anything needing both calls evaluateBsdfSplit once instead.
[[nodiscard]] float pdfBsdf(const BsdfParams& params, const glm::vec3& woLocal,
                            const glm::vec3& wiLocal);
[[nodiscard]] glm::vec3 evaluateBsdf(const BsdfParams& params, const glm::vec3& woLocal,
                                      const glm::vec3& wiLocal);

// Samples one lobe by Fresnel/energy probability, returning its throughput weight or nullopt. See DERIVATIONS.md "BSDF lobe selection".
[[nodiscard]] std::optional<BsdfSample> sampleBsdf(const BsdfParams& params,
                                                    const glm::vec3& woLocal, Sampler& sampler);

}  // namespace pathtracer::scene
