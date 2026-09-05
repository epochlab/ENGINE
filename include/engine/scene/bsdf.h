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
    glm::vec3 f0;                // specular reflectance at normal incidence; also Gulbrandsen's reflectivity r for the conductor lobe, clamped to [1e-4, 0.9999] at the point of use
    // Gulbrandsen 2014 edgetint g, the conductor's grazing-angle colour bias: 1 = white edge (no reflectance dip, what Schlick forces), 0 = maximum dip. Together with f0 it inverts to a complex IOR -- see bsdf.cpp's conductorIorFromReflectivity. Inert at metallic=0.
    glm::vec3 edgeTint;
    float ior;                   // dielectric IOR, non-metal lobes only
    float transmissionFactor;    // KHR_materials_transmission, 0 = opaque
    // EON rough-diffuse parameter r in [0,1] (Portsmouth, Kutz, Hill 2025, "EON: A Practical
    // Energy-Preserving Rough Diffuse BRDF", JCGT 14(1)) -- distinct from `roughness`, which drives
    // the specular GGX lobe: these are different microsurface statistics even on the same material.
    // 0 = Lambertian (EON's exact r->0 limit), see evaluateDiffuseLobe.
    float diffuseRoughness;
    // EON's single-scattering albedo parameter rho, which is what the diffuse lobe evaluates -- NOT the
    // authored colour. baseColor is the albedo the surface is asked to be observed to have; rho is what
    // reproduces it once the multiple-scattering lobe's saturation is accounted for. Resolved once per
    // hit by gbuffer_shading.cpp's resolveBsdfParams via bsdf.cpp's eonAlbedoInversion, which is the
    // sole source of this value; the two coincide exactly at diffuseRoughness 0 and at baseColor 1.
    // Kept separate rather than folded into baseColor, which the transmission lobe and the rasterizer's
    // albedo AOV both still consume as the authored colour.
    glm::vec3 diffuseRho;
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

// Which lobe sampleBsdf drew from -- used by path_tracer.cpp to bucket radiance into the AOV transport-component breakdown (Direct/Indirect Diffuse/Specular, Refraction). Transmission is a delta lobe ONLY below the smooth-roughness threshold; above it the lobe has a real continuous pdf and is MIS-eligible like any other, so callers must not assume a light sample can never land on it.
enum class LobeType { Diffuse, SpecularReflection, Transmission };

struct BsdfSample {
    glm::vec3 wiLocal;            // sampled direction, local shading frame
    glm::vec3 throughputWeight;   // f(wi)*|cosThetaI| / pdf(wi)
    LobeType type;
    // The mixture density wiLocal was actually drawn from -- exactly what pdfBsdf would return for it, computed here because sampleBsdf already has it in hand. Zero for the smooth-glass delta branch, which is what MIS's delta test keys on (path_tracer.cpp).
    float pdf;
};

// The BSDF's continuous lobes at one wi, split by transport type and evaluated in a single pass. Reflection and transmission occupy disjoint hemispheres, so at most one of {diffuse+specular} and {transmission} is non-zero. total() is evaluateBsdf's value and pdf is pdfBsdf's, so the three components are a true partition of the value NEE divides by pdf -- which is what lets path_tracer.cpp's transport AOVs sum back to beauty exactly.
struct BsdfEval {
    glm::vec3 diffuse;
    glm::vec3 specular;
    glm::vec3 transmission;
    float pdf;
    [[nodiscard]] glm::vec3 total() const { return diffuse + specular + transmission; }
};

// Macro-surface Fresnel reflectance at cosTheta = dot(normal, viewDirection): the exact dielectric and exact complex-IOR conductor terms mixed by metallic, which is the same term evaluateSpecularLobe evaluates, in the entering orientation a primary-hit value wants.
// Sole consumer is the rasterizer's Fresnel G-buffer AOV (rasterizer.cpp). It exists so that AOV shows the Fresnel the renderer actually shades with: it used to be Schlick against f0, which for a metal is a different curve entirely -- Schlick is monotone in cos by construction, so it cannot show the reflectance dip an authored edgeTint produces.
[[nodiscard]] glm::vec3 fresnelAtViewAngle(const BsdfParams& params, float cosTheta);

// Cosine-weighted average Fresnel of each interface, 2*int_0^1 F(mu)*mu dmu -- what the Kulla-Conty multiple-scattering tint attenuates each repeated microfacet bounce by. Exported for tools/bsdf_validate.cpp's checkAverageFresnel, which is the only instrument that can see an error here: the two-sided white furnace runs at f0=1 where every candidate average agrees, and the coloured-metal furnace rows are upper-bound-only and so blind to a loss.
// conductorFresnelAvg takes the complex IOR rather than (reflectivity, edgeTint) because callers have already inverted it for the single-scatter term and must not invert twice; bsdf.cpp's conductorIorFromReflectivity is the inversion.
[[nodiscard]] glm::vec3 conductorFresnelAvg(const glm::vec3& n, const glm::vec3& k);
[[nodiscard]] float dielectricFresnelAvg(float ior);

// EON's Appendix A albedo inversion: the rho whose EON directional albedo at normal incidence equals albedo, under uniform illumination. The identity at r=0 and at albedo=1. Every BsdfParams::diffuseRho comes from here -- see that field, and bsdf.cpp for the derivation and the numerical form.
[[nodiscard]] glm::vec3 eonAlbedoInversion(const glm::vec3& albedo, float r);

// Representative wavelength of each RGB channel, from OpenPBR_BaseRgbWavelengths_nm in Adobe's OpenPBR BSDF reference implementation (openpbr_constants.h) -- the same standard this pipeline already takes edgeTint and EON from.
// Known error, in that implementation's own words: one fixed wavelength per channel makes dispersion "produce ... discrete RGB bands" rather than natural rainbow colours, because an RGB channel integrates a band and cannot be represented by a line. Its documented remedy is a stochastically drawn lambda per path, which needs only this lookup replaced by a draw -- see the README roadmap.
inline constexpr glm::vec3 kRgbWavelengthsNm(620.0F, 540.0F, 450.0F);

// Cauchy dispersion n(lambda) inverted from an authored (ior at the d line, Abbe number V_d), per Khronos KHR_materials_dispersion. abbe <= 0 returns iorD unchanged, which is how a non-dispersive material stays bit-identical. Exported for tools/bsdf_validate.cpp's checkCauchyDispersion; taken per-wavelength rather than per-channel so the Fraunhofer identities it is defined by are directly assertable.
[[nodiscard]] float cauchyIor(float iorD, float abbe, float lambdaNm);

// Value and pdf of the continuous lobes at wiLocal, split by transport type. Piecewise, since reflection and transmission occupy disjoint hemispheres: wiLocal on wo's side gives the specular-reflection + diffuse mixture, the far side gives the rough transmission lobe. Transmission is excluded only when it is a delta (roughness below the smooth threshold, zero-measure). woLocal.z sign: entering (>0) vs exiting (<0) a dielectric. One call rather than the four separate lobe evaluations NEE used to make -- the lobe probabilities, the GGX/Fresnel terms and the albedo-table lookups are all computed once and shared.
[[nodiscard]] BsdfEval evaluateBsdfSplit(const BsdfParams& params, const glm::vec3& woLocal,
                                          const glm::vec3& wiLocal);

// evaluateBsdfSplit's total() and pdf. Kept as named entry points for the validation tools, which want one or the other; anything needing both, or the split, should call evaluateBsdfSplit once instead.
[[nodiscard]] float pdfBsdf(const BsdfParams& params, const glm::vec3& woLocal,
                            const glm::vec3& wiLocal);
[[nodiscard]] glm::vec3 evaluateBsdf(const BsdfParams& params, const glm::vec3& woLocal,
                                      const glm::vec3& wiLocal);

// Stochastically samples one of {rough specular reflection, diffuse, refraction, multiple-scattering transmission} by Fresnel- and energy-derived probability, returns the ready-to-multiply throughput weight. Diffuse+specular combine via the one-sample mixture estimator (both lobes evaluated at whichever wi was drawn, not just the sampled lobe -- required since a rough surface's lobes overlap). The specular selection probability is scaled by the GGX directional albedo E(mu_o, roughness), so VNDF sampling takes the single-scattering share and the cosine strategy takes the (cosine-shaped) multiple-scattering share. Transmission: Walter et al. 2007 rough refraction about a VNDF-sampled microfacet normal, falling back to a pure-Snell delta lobe below the smooth-roughness threshold (matching PBRT's EffectivelySmooth, so smooth glass stays exact and noise-free); TIR folded into the specular probability; single non-nested dielectric boundary. The transmit-side multiple-scattering lobe is a fourth strategy, cosine-distributed over the far hemisphere, because refraction sampling reaches only directions some microfacet can refract into while that lobe spans the whole hemisphere. Returns nullopt if fully absorbed.
[[nodiscard]] std::optional<BsdfSample> sampleBsdf(const BsdfParams& params,
                                                    const glm::vec3& woLocal, Sampler& sampler);

}  // namespace engine::scene
