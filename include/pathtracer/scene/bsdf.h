#pragma once

#include <optional>

#include <glm/glm.hpp>

#include "pathtracer/scene/sampler.h"

namespace pathtracer::scene {

// Resolved shading parameters at a hit point (textures already sampled by the caller).
struct BsdfParams {
    // OpenPBR base_color: the observed reflection colour at normal incidence under uniform illumination.
    // resolveBsdfParams derives f0 and diffuseRho from it. It does not tint transmission -- transmissionTint does.
    glm::vec3 baseColor;
    float metallic;
    float roughness;  // perceptual; alpha = roughness^2, floored to avoid a delta lobe
    // Specular reflectance at normal incidence; also Gulbrandsen reflectivity r, clamped to [1e-4, 0.9999] at use.
    glm::vec3 f0;
    // Gulbrandsen 2014 edgetint g, the conductor's grazing-angle bias: 1 = white edge (what Schlick forces), 0 = max dip.
    // With f0 it inverts to a complex IOR (bsdf.cpp conductorIorFromReflectivity). Inert at metallic=0.
    glm::vec3 edgeTint;
    float ior;  // dielectric IOR, non-metal lobes only
    float transmissionFactor;  // KHR_materials_transmission, 0 = opaque
    // EON rough-diffuse parameter r in [0,1] (Portsmouth, Kutz, Hill 2025, JCGT 14(1)); 0 = Lambertian, EON's exact limit.
    // Distinct from `roughness`, which drives the specular lobe: different microsurface statistics on the same material.
    float diffuseRoughness;
    // EON single-scattering albedo rho, what the diffuse lobe evaluates -- not the authored colour. baseColor is what the
    // surface should be observed to have; rho reproduces it once multiple scattering is accounted for. The two coincide
    // at diffuseRoughness 0 and at baseColor 1. Sole source: resolveBsdfParams via eonAlbedoInversion.
    glm::vec3 diffuseRho;
    // The transmission lobe's only tint (OpenPBR/Arnold), the transmission-side counterpart to baseColor. Realized in the
    // volume or on the surface but never both, selected by transmissionDepth: at depth > 0 Beer-Lambert extinction carries
    // it and this is white; at depth == 0 it is the on-surface tint, applied per crossing. Sole source: resolveBsdfParams.
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

// Which lobe sampleBsdf drew from; path_tracer.cpp buckets radiance into the transport AOVs by it.
// Transmission is a delta lobe only below the smooth-roughness threshold -- above it, it is MIS-eligible like any other.
enum class LobeType { Diffuse, SpecularReflection, Transmission };

struct BsdfSample {
    glm::vec3 wiLocal;            // sampled direction, local shading frame
    glm::vec3 throughputWeight;   // f(wi)*|cosThetaI| / pdf(wi)
    LobeType type;
    // The mixture density wiLocal was drawn from, identical to pdfBsdf's value, computed here because sampleBsdf has it.
    // Zero for the smooth-glass delta branch, which is what MIS's delta test keys on (path_tracer.cpp).
    float pdf;
};

// The BSDF's continuous lobes at one wi, split by transport type in one pass. Reflection and transmission occupy disjoint
// hemispheres, so at most one of {diffuse+specular} and {transmission} is non-zero. total() is evaluateBsdf's value and
// pdf is pdfBsdf's, so the components partition what NEE divides by pdf -- which is why the transport AOVs sum to beauty.
struct BsdfEval {
    glm::vec3 diffuse;
    glm::vec3 specular;
    glm::vec3 transmission;
    float pdf;
    [[nodiscard]] glm::vec3 total() const { return diffuse + specular + transmission; }
};

// Macro-surface Fresnel at cosTheta = dot(n, wo): the exact dielectric and complex-IOR conductor terms mixed by metallic,
// the same term evaluateSpecularLobe evaluates. Reached only through fresnelAtMicrofacet, which supplies the microfacet
// angle instead. Exists so the Fresnel AOV shows the real curve: Schlick is monotone in cos and hides the edgeTint dip.
[[nodiscard]] glm::vec3 fresnelAtViewAngle(const BsdfParams& params, float cosTheta);

// One sample of the Fresnel the microfacet BSDF evaluates: a half-vector from the VNDF (Heitz 2018, the D_vis and alpha
// sampleBsdf draws from) through fresnelAtViewAngle at dot(wo, wh), the angle Walter 2007 makes correct (Karis 2013
// split-sum precedent). E[F] over that distribution, so THE CALLER MUST AVERAGE. u is a caller-owned uniform pair.
[[nodiscard]] glm::vec3 fresnelAtMicrofacet(const BsdfParams& params, const glm::vec3& woLocal, glm::vec2 u);

// Cosine-weighted hemisphere direction about +z, pdf = cos(theta)/pi. Exported for path_tracer.cpp's AO lane: the pdf
// cancels the cosine in Miller 1994's AO integral, collapsing the estimator to the mean of the visibility term.
[[nodiscard]] glm::vec3 sampleCosineHemisphere(glm::vec2 u);

// Cosine-weighted average Fresnel, 2*int_0^1 F(mu)*mu dmu: what the Kulla-Conty tint attenuates each repeated bounce by.
// One 3-node quadrature serves both. checkAverageFresnel is the only instrument that can see an error: the two-sided
// furnace runs at f0=1 where every candidate agrees. conductorFresnelAvg takes the complex IOR, already inverted.
[[nodiscard]] glm::vec3 conductorFresnelAvg(const glm::vec3& n, const glm::vec3& k);
[[nodiscard]] float dielectricFresnelAvg(float ior);

// Reflect-side Kulla-Conty lookups: Schlick-split directional albedo E(mu, roughness) as (a, b) with E = a+b, and its
// cosine-weighted mean Eavg(roughness). These are the lookups, not the table -- albedo_table.inc read through the axis
// warps and bilinear blend. The INTERPOLATION error has no instrument but checkAlbedoTableInterpolation (energy tests: 2%).
[[nodiscard]] glm::vec2 directionalAlbedoSplit(float mu, float roughness);
[[nodiscard]] glm::vec2 averageAlbedoSplit(float roughness);

// The grid those two index: resolutions, and each axis' node position at a fractional index, so an instrument can place
// samples exactly on and between nodes without transcribing the grid -- one that drifts onto a node measures nothing.
// Both axes are edge-aligned, but mu is uniform in sqrt(mu) and albedoGridMu inverts that warp: never assume k/(res-1).
[[nodiscard]] glm::ivec2 albedoGridRes();
[[nodiscard]] float albedoGridRoughness(float index);
[[nodiscard]] float albedoGridMu(float index);

// EON Appendix A inversion: the rho whose EON directional albedo at normal incidence equals albedo under uniform light.
// The identity at r=0 and at albedo=1. Every BsdfParams::diffuseRho comes from here; bsdf.cpp holds the numerical form.
[[nodiscard]] glm::vec3 eonAlbedoInversion(const glm::vec3& albedo, float r);

// Representative wavelength per RGB channel, from OpenPBR_BaseRgbWavelengths_nm in Adobe's OpenPBR reference impl.
// Known error, in its own words: one fixed wavelength per channel makes dispersion "produce ... discrete RGB bands",
// because a channel integrates a band and cannot be a line. Remedy is a drawn lambda per path (ROADMAP.md transport #5).
inline constexpr glm::vec3 kRgbWavelengthsNm(620.0F, 540.0F, 450.0F);

// Cauchy n(lambda) inverted from an authored (ior at the d line, Abbe V_d), per KHR_materials_dispersion. abbe <= 0
// returns iorD unchanged, which is how a non-dispersive material stays bit-identical. Per-wavelength rather than
// per-channel so checkCauchyDispersion can assert the Fraunhofer identities it is defined by directly.
[[nodiscard]] float cauchyIor(float iorD, float abbe, float lambdaNm);

// Value and pdf of the continuous lobes at wiLocal, split by transport type. Piecewise: wiLocal on wo's side gives the
// specular+diffuse mixture, the far side the rough transmission lobe, excluded only when it is a delta. woLocal.z sign
// is entering (>0) vs exiting (<0). One call, so lobe probabilities, GGX/Fresnel and albedo lookups are computed once.
[[nodiscard]] BsdfEval evaluateBsdfSplit(const BsdfParams& params, const glm::vec3& woLocal,
                                          const glm::vec3& wiLocal);

// evaluateBsdfSplit's total() and pdf, as named entry points for the validators, which want one or the other.
// Anything needing both, or the split, calls evaluateBsdfSplit once instead.
[[nodiscard]] float pdfBsdf(const BsdfParams& params, const glm::vec3& woLocal,
                            const glm::vec3& wiLocal);
[[nodiscard]] glm::vec3 evaluateBsdf(const BsdfParams& params, const glm::vec3& woLocal,
                                      const glm::vec3& wiLocal);

// Samples one of {rough specular, diffuse, refraction, multiple-scattering transmission} by Fresnel- and energy-derived
// probability and returns the ready-to-multiply throughput weight; nullopt if fully absorbed. Single non-nested
// dielectric boundary, TIR folded into the specular probability. See DERIVATIONS.md "BSDF lobe selection".
[[nodiscard]] std::optional<BsdfSample> sampleBsdf(const BsdfParams& params,
                                                    const glm::vec3& woLocal, Sampler& sampler);

}  // namespace pathtracer::scene
