#include "engine/scene/bsdf.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace engine::scene {

namespace {

constexpr float kPi = 3.14159265F;
constexpr float kMinAlpha = 0.02F * 0.02F;  // roughness floor, avoids a degenerate GGX delta lobe

float distributionGGX(float ndotH, float alpha) {
    const float alpha2 = alpha * alpha;
    const float d = ((ndotH * ndotH) * (alpha2 - 1.0F)) + 1.0F;
    return alpha2 / std::max(kPi * d * d, 1e-8F);
}

float smithLambda(float ndotV, float alpha) {
    const float ndotV2 = std::max(ndotV * ndotV, 1e-8F);
    const float tan2 = std::max(0.0F, 1.0F - ndotV2) / ndotV2;
    return 0.5F * (-1.0F + std::sqrt(1.0F + (alpha * alpha * tan2)));
}

float smithG1(float ndotV, float alpha) { return 1.0F / (1.0F + smithLambda(ndotV, alpha)); }

float smithG2(float ndotV, float ndotL, float alpha) {
    return 1.0F / (1.0F + smithLambda(ndotV, alpha) + smithLambda(ndotL, alpha));
}

// Exact unpolarized dielectric Fresnel reflectance (PBRT's FrDielectric); 1.0 past total internal reflection.
float fresnelDielectric(float cosThetaI, float etaI, float etaT) {
    cosThetaI = std::clamp(cosThetaI, -1.0F, 1.0F);
    if (cosThetaI < 0.0F) {
        std::swap(etaI, etaT);
        cosThetaI = -cosThetaI;
    }
    const float sinThetaI = std::sqrt(std::max(0.0F, 1.0F - (cosThetaI * cosThetaI)));
    const float sinThetaT = (etaI / etaT) * sinThetaI;
    if (sinThetaT >= 1.0F) {
        return 1.0F;
    }
    const float cosThetaT = std::sqrt(std::max(0.0F, 1.0F - (sinThetaT * sinThetaT)));
    const float rParallel =
        ((etaT * cosThetaI) - (etaI * cosThetaT)) / ((etaT * cosThetaI) + (etaI * cosThetaT));
    const float rPerpendicular =
        ((etaI * cosThetaI) - (etaT * cosThetaT)) / ((etaI * cosThetaI) + (etaT * cosThetaT));
    return ((rParallel * rParallel) + (rPerpendicular * rPerpendicular)) * 0.5F;
}

// Heitz 2018 VNDF sampling. wo.z > 0 required (caller pre-flips into the +z hemisphere).
glm::vec3 sampleGGXVNDF(const glm::vec3& wo, float alpha, glm::vec2 u) {
    const glm::vec3 vh = glm::normalize(glm::vec3(alpha * wo.x, alpha * wo.y, wo.z));
    const float lensq = (vh.x * vh.x) + (vh.y * vh.y);
    const glm::vec3 t1 = lensq > 0.0F ? glm::vec3(-vh.y, vh.x, 0.0F) * (1.0F / std::sqrt(lensq))
                                       : glm::vec3(1.0F, 0.0F, 0.0F);
    const glm::vec3 t2 = glm::cross(vh, t1);
    const float r = std::sqrt(u.x);
    const float phi = 2.0F * kPi * u.y;
    const float t1p = r * std::cos(phi);
    float t2p = r * std::sin(phi);
    const float s = 0.5F * (1.0F + vh.z);
    t2p = ((1.0F - s) * std::sqrt(std::max(0.0F, 1.0F - (t1p * t1p)))) + (s * t2p);
    const glm::vec3 nh = (t1p * t1) + (t2p * t2) +
                          (std::sqrt(std::max(0.0F, 1.0F - (t1p * t1p) - (t2p * t2p))) * vh);
    return glm::normalize(glm::vec3(alpha * nh.x, alpha * nh.y, std::max(0.0F, nh.z)));
}

glm::vec3 sampleCosineHemisphere(glm::vec2 u) {
    const float r = std::sqrt(u.x);
    const float phi = 2.0F * kPi * u.y;
    return {r * std::cos(phi), r * std::sin(phi), std::sqrt(std::max(0.0F, 1.0F - u.x))};
}

// Directional albedo of the single-scattering GGX lobe with Fresnel forced to 1 -- the fraction of energy
// smithG2 lets through, so 1-E is exactly what multiple scattering must return (Kulla & Conty 2017,
// "Revisiting Physically Based Shading at Imageworks"). Depends on nothing but (mu, alpha): Fresnel,
// metallic, baseColor and lobe-selection probabilities are all applied by the caller, never baked in here.
// Indexed by perceptual roughness rather than alpha -- E is far better distributed in sqrt(alpha), and it
// is what callers already hold. Grid is edge-aligned so roughness 0 / mu 1 are exact table entries.
constexpr int kAlbedoRes = 32;
constexpr int kAlbedoSamples = 16;  // per axis; 256 stratified samples/cell, ~1.5e-3 vs a 128x128 reference

// Split by Schlick's form F(c) = f0*(1 - (1-c)^5) + (1-c)^5 so one table serves any f0 (the standard
// environment-BRDF split): Ess(mu, f0) = f0*a + b, and with f0 = 1 that collapses to a + b = E, the
// Fresnel-free albedo the multiple-scattering lobe needs. Two channels, no third axis for ior.
//
// The TRANSMIT side needs a third axis. Its energy curve is not the reflect side's: the below-horizon
// reflections that drive E down are the *valid* side for refraction, so far fewer samples are discarded
// (measured 0.559 combined vs 0.307 reflect-only at roughness 1.0). And unlike the reflect side it
// genuinely depends on eta -- G2 uses the refracted |wi.z|, and TIR gates validity -- so the Schlick
// split cannot factor it out. One axis in log(eta) covers entering AND exiting, since the two are
// reciprocals of each other.
constexpr int kEtaRes = 16;
constexpr float kEtaMin = 1.0F / 2.5F;  // exiting a 2.5-ior medium; the reciprocal end is entering one
constexpr float kEtaMax = 2.5F;

struct AlbedoTable {
    std::array<float, static_cast<std::size_t>(kAlbedoRes) * kAlbedoRes> a;  // [roughnessIndex][muIndex]
    std::array<float, static_cast<std::size_t>(kAlbedoRes) * kAlbedoRes> b;
    std::array<float, kAlbedoRes> aavg;  // cosine-weighted means, 2*integral(.(mu)*mu dmu)
    std::array<float, kAlbedoRes> bavg;
    // Escaping fraction of a dielectric interface, split into the reflected and transmitted shares and
    // indexed [roughnessIndex][muIndex][etaIndex]. Both use EXACT dielectric Fresnel at build time rather
    // than the Schlick split above: inside the total-internal-reflection cone exact Fresnel is 1.0 while
    // Schlick reads ~0.1, so no rescale of a Schlick-basis number can stand in for it, and the escape
    // budget would under-count the reflected share by the whole TIR cone.
    std::array<float, static_cast<std::size_t>(kAlbedoRes) * kAlbedoRes * kEtaRes> r;
    std::array<float, static_cast<std::size_t>(kAlbedoRes) * kAlbedoRes * kEtaRes> t;
    std::array<float, static_cast<std::size_t>(kAlbedoRes) * kEtaRes> ravg;
    std::array<float, static_cast<std::size_t>(kAlbedoRes) * kEtaRes> tavg;
};

// log-spaced so eta and 1/eta are symmetric about index kEtaRes/2.
float etaAtIndex(int index) {
    const float u = static_cast<float>(index) / static_cast<float>(kEtaRes - 1);
    return std::exp(std::log(kEtaMin) + (u * (std::log(kEtaMax) - std::log(kEtaMin))));
}

// Refract wo about microfacet normal ht. Returns false on total internal reflection at that facet.
bool refractAbout(const glm::vec3& wo, const glm::vec3& ht, float eta, glm::vec3& wi) {
    const float cosI = glm::dot(wo, ht);
    if (cosI <= 0.0F) {
        return false;
    }
    const float sin2T = eta * eta * std::max(0.0F, 1.0F - (cosI * cosI));
    if (sin2T >= 1.0F) {
        return false;
    }
    wi = ((eta * cosI) - std::sqrt(1.0F - sin2T)) * ht - (eta * wo);
    return true;
}

// Deterministic stratified midpoint quadrature, not RNG Monte Carlo: the integrand is smooth, and a fixed
// grid keeps the table bit-identical across runs and machines (see the determinism note on -march=native).
// Below-horizon reflections contribute zero -- that discard is part of the energy loss being measured.
AlbedoTable buildAlbedoTable() {
    AlbedoTable table{};
    for (int ri = 0; ri < kAlbedoRes; ++ri) {
        const float roughness = static_cast<float>(ri) / static_cast<float>(kAlbedoRes - 1);
        const float alpha = std::max(roughness * roughness, kMinAlpha);
        double aWeighted = 0.0;
        double bWeighted = 0.0;
        std::array<double, kEtaRes> rWeighted{};
        std::array<double, kEtaRes> tWeighted{};
        for (int mi = 0; mi < kAlbedoRes; ++mi) {
            // mu=0 is a degenerate view direction (wo lies in the surface plane); nudge off it.
            const float mu = std::max(static_cast<float>(mi) / static_cast<float>(kAlbedoRes - 1), 1e-3F);
            const glm::vec3 wo(std::sqrt(std::max(0.0F, 1.0F - (mu * mu))), 0.0F, mu);
            const float g1 = smithG1(mu, alpha);
            double aSum = 0.0;
            double bSum = 0.0;
            std::array<double, kEtaRes> rSum{};
            std::array<double, kEtaRes> tSum{};
            for (int i = 0; i < kAlbedoSamples; ++i) {
                for (int j = 0; j < kAlbedoSamples; ++j) {
                    const glm::vec2 u((static_cast<float>(i) + 0.5F) / kAlbedoSamples,
                                       (static_cast<float>(j) + 0.5F) / kAlbedoSamples);
                    const glm::vec3 nh = sampleGGXVNDF(wo, alpha, u);
                    const glm::vec3 wi = glm::reflect(-wo, nh);
                    const float fc = std::pow(std::clamp(1.0F - glm::dot(wo, nh), 0.0F, 1.0F), 5.0F);
                    if (wi.z > 0.0F) {
                        const float weight = smithG2(mu, wi.z, alpha) / std::max(g1, 1e-8F);
                        aSum += weight * (1.0F - fc);
                        bSum += weight * fc;
                    }
                    // Same visible normal, reflected AND refracted -- the VNDF sample is the expensive
                    // part and is shared across every eta, so the third axis costs only the refraction.
                    // A facet reflects with probability F and refracts with 1-F, so the two shares are
                    // Fresnel-weighted complements of one throughput, never independent quantities.
                    const float woDotNh = glm::dot(wo, nh);
                    for (int ei = 0; ei < kEtaRes; ++ei) {
                        const float eta = etaAtIndex(ei);
                        const float fresnel = fresnelDielectric(woDotNh, eta, 1.0F);
                        if (wi.z > 0.0F) {
                            rSum[ei] += (smithG2(mu, wi.z, alpha) / std::max(g1, 1e-8F)) * fresnel;
                        }
                        glm::vec3 wt;
                        if (refractAbout(wo, nh, eta, wt) && wt.z < 0.0F) {
                            tSum[ei] +=
                                (smithG2(mu, -wt.z, alpha) / std::max(g1, 1e-8F)) * (1.0F - fresnel);
                        }
                    }
                }
            }
            const auto cells = static_cast<double>(kAlbedoSamples) * kAlbedoSamples;
            const float a = static_cast<float>(aSum / cells);
            const float b = static_cast<float>(bSum / cells);
            table.a[(ri * kAlbedoRes) + mi] = a;
            table.b[(ri * kAlbedoRes) + mi] = b;
            // Trapezoid over the mu axis: the grid is edge-aligned, so the two endpoints span half a
            // cell each and the step is 1/(kAlbedoRes-1), not 1/kAlbedoRes.
            const double endpoint = (mi == 0 || mi == kAlbedoRes - 1) ? 0.5 : 1.0;
            aWeighted += endpoint * 2.0 * a * mu;
            bWeighted += endpoint * 2.0 * b * mu;
            for (int ei = 0; ei < kEtaRes; ++ei) {
                const float r = static_cast<float>(rSum[ei] / cells);
                const float t = static_cast<float>(tSum[ei] / cells);
                table.r[(((ri * kAlbedoRes) + mi) * kEtaRes) + ei] = r;
                table.t[(((ri * kAlbedoRes) + mi) * kEtaRes) + ei] = t;
                rWeighted[ei] += endpoint * 2.0 * r * mu;
                tWeighted[ei] += endpoint * 2.0 * t * mu;
            }
        }
        table.aavg[ri] = static_cast<float>(aWeighted / (kAlbedoRes - 1));
        table.bavg[ri] = static_cast<float>(bWeighted / (kAlbedoRes - 1));
        for (int ei = 0; ei < kEtaRes; ++ei) {
            table.ravg[(ri * kEtaRes) + ei] = static_cast<float>(rWeighted[ei] / (kAlbedoRes - 1));
            table.tavg[(ri * kEtaRes) + ei] = static_cast<float>(tWeighted[ei] / (kAlbedoRes - 1));
        }
    }
    return table;
}

const AlbedoTable kAlbedo = buildAlbedoTable();

float lerp1(float a, float b, float t) { return a + ((b - a) * t); }

// Schlick-split albedo pair: Ess(f0) = f0*a + b, and a+b = E (the f0=1 case).
struct AlbedoSplit {
    float a;
    float b;
    [[nodiscard]] float total() const { return a + b; }                 // E, Fresnel-free
    [[nodiscard]] float at(float f0) const { return (f0 * a) + b; }
};

// Bilinear lookup. E climbs steeply as mu->0 (0.31 at mu=1, ~1.0 at grazing, alpha=1), so the first mu bin
// carries the largest interpolation error -- harmless, since every integral consuming E weights grazing by
// cos(theta).
AlbedoSplit directionalAlbedo(float mu, float roughness) {
    const float rf = std::clamp(roughness, 0.0F, 1.0F) * (kAlbedoRes - 1);
    const float mf = std::clamp(mu, 0.0F, 1.0F) * (kAlbedoRes - 1);
    const int r0 = std::min(static_cast<int>(rf), kAlbedoRes - 2);
    const int m0 = std::min(static_cast<int>(mf), kAlbedoRes - 2);
    const float rt = rf - static_cast<float>(r0);
    const float mt = mf - static_cast<float>(m0);
    const int i0 = (r0 * kAlbedoRes) + m0;
    const int i1 = ((r0 + 1) * kAlbedoRes) + m0;
    return {lerp1(lerp1(kAlbedo.a[i0], kAlbedo.a[i0 + 1], mt),
                   lerp1(kAlbedo.a[i1], kAlbedo.a[i1 + 1], mt), rt),
             lerp1(lerp1(kAlbedo.b[i0], kAlbedo.b[i0 + 1], mt),
                   lerp1(kAlbedo.b[i1], kAlbedo.b[i1 + 1], mt), rt)};
}

AlbedoSplit averageAlbedo(float roughness) {
    const float rf = std::clamp(roughness, 0.0F, 1.0F) * (kAlbedoRes - 1);
    const int r0 = std::min(static_cast<int>(rf), kAlbedoRes - 2);
    const float rt = rf - static_cast<float>(r0);
    return {lerp1(kAlbedo.aavg[r0], kAlbedo.aavg[r0 + 1], rt),
             lerp1(kAlbedo.bavg[r0], kAlbedo.bavg[r0 + 1], rt)};
}

// Fractional index into the log-spaced eta axis, clamped to the tabulated range.
float etaAxisCoord(float eta) {
    const float logMin = std::log(kEtaMin);
    const float u = (std::log(std::clamp(eta, kEtaMin, kEtaMax)) - logMin) / (std::log(kEtaMax) - logMin);
    return u * (kEtaRes - 1);
}

// Reflected and transmitted escaping shares of a dielectric interface, exact Fresnel already applied.
struct EscapeSplit {
    float reflect;
    float transmit;
    [[nodiscard]] float total() const { return reflect + transmit; }
};

// Trilinear over (roughness, mu, eta).
EscapeSplit escapeAlbedo(float mu, float roughness, float eta) {
    const float rf = std::clamp(roughness, 0.0F, 1.0F) * (kAlbedoRes - 1);
    const float mf = std::clamp(mu, 0.0F, 1.0F) * (kAlbedoRes - 1);
    const float ef = etaAxisCoord(eta);
    const int r0 = std::min(static_cast<int>(rf), kAlbedoRes - 2);
    const int m0 = std::min(static_cast<int>(mf), kAlbedoRes - 2);
    const int e0 = std::min(static_cast<int>(ef), kEtaRes - 2);
    const float rt = rf - static_cast<float>(r0);
    const float mt = mf - static_cast<float>(m0);
    const float et = ef - static_cast<float>(e0);
    const auto fetch = [&](const auto& channel, int r, int m) {
        const int base = (((r * kAlbedoRes) + m) * kEtaRes) + e0;
        return lerp1(channel[base], channel[base + 1], et);
    };
    const auto bilinear = [&](const auto& channel) {
        return lerp1(lerp1(fetch(channel, r0, m0), fetch(channel, r0, m0 + 1), mt),
                      lerp1(fetch(channel, r0 + 1, m0), fetch(channel, r0 + 1, m0 + 1), mt), rt);
    };
    return {bilinear(kAlbedo.r), bilinear(kAlbedo.t)};
}

EscapeSplit averageEscapeAlbedo(float roughness, float eta) {
    const float rf = std::clamp(roughness, 0.0F, 1.0F) * (kAlbedoRes - 1);
    const float ef = etaAxisCoord(eta);
    const int r0 = std::min(static_cast<int>(rf), kAlbedoRes - 2);
    const int e0 = std::min(static_cast<int>(ef), kEtaRes - 2);
    const float rt = rf - static_cast<float>(r0);
    const float et = ef - static_cast<float>(e0);
    const auto fetch = [&](const auto& channel, int r) {
        const int base = (r * kEtaRes) + e0;
        return lerp1(channel[base], channel[base + 1], et);
    };
    return {lerp1(fetch(kAlbedo.ravg, r0), fetch(kAlbedo.ravg, r0 + 1), rt),
             lerp1(fetch(kAlbedo.tavg, r0), fetch(kAlbedo.tavg, r0 + 1), rt)};
}

// Cosine-weighted average Fresnel, the normalisation both the multiple-scattering tint and the reciprocal
// diffuse coupling need. Schlick's average is exact in closed form (Karis); the dielectric one is the
// standard rational fit, accurate to 0.0065 absolute over ior in [1.1, 3.0] against exact quadrature --
// it enters only as the 1/(1-Favg) normalisation, a 0.25% effect at ior 1.5.
float dielectricFresnelAvg(float ior) { return (ior - 1.0F) / ((4.08567F + (1.00071F * ior))); }

glm::vec3 schlickFresnelAvg(const glm::vec3& f0) { return f0 + ((glm::vec3(1.0F) - f0) / 21.0F); }

// Normal-incidence reflectance implied by the ior -- the dielectric coat's own f0, independent of the
// (currently unrelated, see A5) f0 texture the conductor path uses.
float dielectricF0(float ior) {
    const float r = (ior - 1.0F) / (ior + 1.0F);
    return r * r;
}

// Kulla-Conty multiple-scattering tint: the share of the (1-E) energy that survives repeated bounces on
// the microsurface, each one attenuated by Favg. Equals 1 for a perfect reflector (Favg=1), so a white
// conductor conserves exactly.
float multiScatterTint(float fresnelAvg, float albedoAvg) {
    return (fresnelAvg * fresnelAvg * albedoAvg) /
           std::max(1.0F - (fresnelAvg * (1.0F - albedoAvg)), 1e-4F);
}

float schlickScalar(float cosTheta, float f0) {
    return f0 + ((1.0F - f0) * std::pow(std::clamp(1.0F - cosTheta, 0.0F, 1.0F), 5.0F));
}

// etaI/etaT rather than a bare ior: on the exiting side fresnelDielectric returns exactly 1.0 past the
// critical angle, and Schlick has no way to express total internal reflection at all. Hardcoding the
// entering orientation here under-reported the reflected share of an exiting ray by up to the whole TIR
// cone, which the compensation then tried to hand back as multiple scattering.
float coatFresnelRatio(float cosTheta, float etaI, float etaT, float f0) {
    return fresnelDielectric(cosTheta, etaI, etaT) / std::max(schlickScalar(cosTheta, f0), 1e-6F);
}

// Total directional albedo of the dielectric coat -- single scatter plus its own multiple-scattering
// lobe. This, NOT the macro-facet Fresnel F(mu_o), is what the coat actually reflects: at roughness 1
// and mu 0.4 the two differ by 4x (0.030 vs 0.129), and coupling the diffuse substrate to F(mu_o) hands
// that difference to neither lobe -- measured as a 10% energy loss before this was used.
//
// fresnelRatio rescales the single-scatter term by exact-dielectric / Schlick Fresnel at this direction.
// The table is built on Schlick's basis (so one table serves any f0) but the specular lobe evaluates
// exact fresnelDielectric, and Schlick under-predicts it at grazing -- leaving the substrate too much
// energy and creating ~1.4% at smooth grazing angles. The rescale makes the two agree exactly in the
// smooth limit, where the coat albedo IS the Fresnel term, and approximately as roughness widens the
// lobe away from the macro angle. It also collapses correctly at ior = 1, where exact Fresnel is
// identically zero but Schlick's (1-c)^5 tail is not.
float coatAlbedo(const AlbedoSplit& split, float albedoAvg, float f0, float fresnelRatio) {
    return (split.at(f0) * fresnelRatio) +
           (multiScatterTint(f0 + ((1.0F - f0) / 21.0F), albedoAvg) * (1.0F - split.total()));
}

struct LobeEval {
    glm::vec3 f;
    float pdf;
};

struct LobeProbabilities {
    float specular;
    float diffuse;
    float transmit;
    float etaI;
    float etaT;
    float diffuseKd;              // evaluateDiffuseLobe's wo-side energy factor, 0 on the exiting side
    float transmitPhysicalValue;  // transmission's true (1-F)*t energy fraction -- see below
    // Energy-compensation state, hoisted here so the wo-side table lookups happen once per evaluation
    // rather than once per lobe call.
    float albedoWo;         // E(mu_o, roughness), Fresnel-free
    float albedoAvg;        // Eavg(roughness)
    float coatF0;           // dielectric f0 implied by ior, for the diffuse coupling
    float coatAlbedoAvg;    // cosine-weighted mean coat albedo, the coupling's normalisation
    glm::vec3 fresnelAvg;
    // Multiple-scattering state for a transmissive interface, which needs its own deficit. A facet either
    // reflects or refracts, chosen by Fresnel, so the escaping fraction is inherently Fresnel-WEIGHTED --
    // R_ss + T_ss -- and cannot reuse the opaque path's Fresnel-free (1 - E). (Adding the two Fresnel-free
    // throughputs double-counts the same facets and drives the deficit negative.) The two formulations are
    // therefore blended by transmissionFactor rather than unified, so an opaque material keeps exactly the
    // measured behaviour the opaque compensation already has.
    float escapeWo;         // R_ss(mu_o) + T_ss(mu_o), the Fresnel-weighted escaping fraction
    float escapeAvg;
    float transmitShare;    // of the multiple-scattered energy, the fraction leaving refracted
    float etaSq;            // (etaI/etaT)^2, the radiance compression the transmit lobe must carry
    float transmitWeight;   // transmissionFactor*(1-metallic) -- how much transmission actually happens
};

// kd carries the wo-side (1-F)/(1-Favg) coupling; the matching wi-side (1-F) factor is applied here, so
// the lobe is reciprocal (A4) while its directional albedo still integrates to (1-F(mu_o)) -- same total
// energy as the old one-sided form, correctly distributed. pdf must NOT be gated on kd: sampleBsdf still
// selects this lobe with probability lobes.diffuse (independent of kd, see computeLobeProbabilities), so
// the pdf side of the MIS mixture must match that selection density regardless of how little/no value the
// lobe carries -- gating pdf on kd starves the mixture denominator and inflates throughput for metals
// (kd=0 but diffuseProb>0).
// Shared shape of the multiple-scattering lobe, on whichever hemisphere wi lies. Cosine-distributed and
// symmetric in wo/wi. Its reflected and transmitted shares are normalised to sum to the deficit
// (1 - escape(mu_o)) that single scattering left behind, but the transmitted share is only DELIVERED over
// the refraction-reachable cone: evaluateTransmissionLobe applies this after its geometric rejections, so
// directions no refraction can produce get nothing. That is deliberate -- those directions also have zero
// transmit pdf, so placing energy there would be unsamplable and bias the estimator. The consequence is
// that the compensation under-delivers on the transmit side (a slight darkening at high roughness,
// measured 0.68 of the correct energy at roughness 1.0 exiting), never over-delivers, and the one-sample
// mixture stays unbiased. Closing it needs this lobe to have its own cosine sampling strategy.
float multiScatterShape(const BsdfParams& params, float wiZ, const LobeProbabilities& lobes) {
    // Guarded rather than clamped. As roughness falls, every deficit tends to zero together and the ratio
    // stays finite -- but flooring only the denominator breaks that cancellation and turns a vanishing
    // lobe into a huge one (measured: a round trip reading 1.54 where it should read 1.02). Below this
    // there is no meaningful multiple scattering to return, so the lobe is simply off.
    const float deficitAvg = 1.0F - lobes.escapeAvg;
    if (deficitAvg <= 1e-3F) {
        return 0.0F;
    }
    const float mu = std::abs(wiZ);
    const float escapeWi = escapeAlbedo(mu, params.roughness, lobes.etaI / lobes.etaT).total();
    return (std::max(1.0F - lobes.escapeWo, 0.0F) * std::max(1.0F - escapeWi, 0.0F)) /
           (kPi * deficitAvg);
}

// The full reciprocal coupling factor at wi: the wo-side half is precomputed into lobes.diffuseKd, the
// wi-side half is the same (1 - coatAlbedo) evaluated here. Shared with evaluateDiffuseRaw and
// sampleBsdf's raw AOV weight so all three cannot drift apart.
float diffuseKdAt(const BsdfParams& params, const glm::vec3& wi, const LobeProbabilities& lobes) {
    const AlbedoSplit splitWi = directionalAlbedo(wi.z, params.roughness);
    const float coat = coatAlbedo(splitWi, lobes.albedoAvg, lobes.coatF0,
                                   coatFresnelRatio(wi.z, lobes.etaI, lobes.etaT, lobes.coatF0));
    return std::max(lobes.diffuseKd, 0.0F) * (1.0F - coat);
}

LobeEval evaluateDiffuseLobe(const BsdfParams& params, const glm::vec3& wi,
                              const LobeProbabilities& lobes) {
    if (wi.z <= 0.0F) {
        return {glm::vec3(0.0F), 0.0F};
    }
    return {params.baseColor * diffuseKdAt(params, wi, lobes) / kPi, wi.z / kPi};
}

// Single scatter D*G2*F/(4*ndotV*ndotL) plus the Kulla-Conty multiple-scattering lobe, and the VNDF pdf
// (Heitz 2018 eq.3, Jacobian 1/(4*dot(wo,nh))). The pdf covers the single-scattering term only -- the
// multiple-scattering term has no sampling strategy of its own and is picked up by whichever of the two
// existing strategies draws that wi, which leaves the one-sample mixture estimator unbiased (the mixture
// density is still the true density of the sampling procedure).
LobeEval evaluateSpecularLobe(const BsdfParams& params, const glm::vec3& wo, const glm::vec3& wi,
                               float alpha, const LobeProbabilities& lobes) {
    if (wo.z <= 0.0F || wi.z <= 0.0F) {
        return {glm::vec3(0.0F), 0.0F};
    }
    const glm::vec3 nh = glm::normalize(wo + wi);
    const float woDotNh = std::max(glm::dot(wo, nh), 0.0F);
    if (woDotNh <= 0.0F) {
        return {glm::vec3(0.0F), 0.0F};
    }
    const float d = distributionGGX(nh.z, alpha);
    const float g2 = smithG2(wo.z, wi.z, alpha);
    const float fDielectric = fresnelDielectric(woDotNh, lobes.etaI, lobes.etaT);
    const glm::vec3 f =
        glm::mix(glm::vec3(fDielectric), fresnelSchlick(woDotNh, params.f0), params.metallic);
    const glm::vec3 singleScatter = (d * g2 * f) / std::max(4.0F * wo.z * wi.z, 1e-6F);

    // Kulla & Conty 2017. Integrates to (1-E(mu_o)) at Favg=1 -- the energy smithG2 discarded -- so a
    // white conductor conserves, up to the table's own interpolation and quadrature error (measured
    // under 1% by the white furnace test). Symmetric in wo/wi, so it preserves reciprocity.
    // Reflected share of the multiple-scattering energy. The tint blends to 1 as the interface becomes
    // fully transmissive: a lossless dielectric returns all of it, tinted only by baseColor on the far
    // side, whereas a conductor's repeated bounces are attenuated by Favg each time.
    const glm::vec3 fms(multiScatterTint(lobes.fresnelAvg.x, lobes.albedoAvg),
                         multiScatterTint(lobes.fresnelAvg.y, lobes.albedoAvg),
                         multiScatterTint(lobes.fresnelAvg.z, lobes.albedoAvg));
    const float albedoWi = directionalAlbedo(wi.z, params.roughness).total();
    const glm::vec3 opaqueMs = fms * ((1.0F - lobes.albedoWo) * (1.0F - albedoWi)) /
                                (kPi * std::max(1.0F - lobes.albedoAvg, 1e-4F));
    const glm::vec3 transmissiveMs =
        glm::vec3((1.0F - lobes.transmitShare) * multiScatterShape(params, wi.z, lobes));
    const glm::vec3 multiScatter =
        glm::mix(opaqueMs, transmissiveMs, lobes.transmitWeight);

    const float g1 = smithG1(wo.z, alpha);
    const float pdf = (g1 * woDotNh * d) / std::max(wo.z, 1e-6F) / (4.0F * woDotNh);
    return {singleScatter + multiScatter, pdf};
}

// specular = Fresnel reflectance probability (exact dielectric via ior, Schlick via f0 for conductors, blended by metallic). "Exiting" (transmissive material, sign<0, already inside): no diffuse substrate, transmit takes everything specular didn't -- reflect internally or exit, no third option. Everything else -- entering (sign>0), or an opaque material's woLocal.z pushed negative by grazing-angle normal mapping -- uses the entering split: diffuse/transmit divide the remainder by transmissionFactor (0 for opaque, so transmit vanishes and this reduces to diffuse+specular regardless of which side of the interpolated normal wo landed on). transmitPhysicalValue != transmit: throughput = physicalValue/transmit, so physicalValue must independently carry the same transmissionFactor/metallic factors transmit's probability used, or they cancel out of the throughput and silently erase their effect on energy (caught by tools/bsdf_validate.cpp's furnace test).
LobeProbabilities computeLobeProbabilities(const BsdfParams& params, const glm::vec3& wo,
                                            float sign) {
    const bool exiting = sign < 0.0F && params.transmissionFactor > 0.0F;
    const float etaI = exiting ? params.ior : 1.0F;
    const float etaT = exiting ? 1.0F : params.ior;
    const float fresnelAtNormal = fresnelDielectric(wo.z, etaI, etaT);
    const glm::vec3 fresnelConductor = fresnelSchlick(wo.z, params.f0);
    const float conductorLuma = (fresnelConductor.x + fresnelConductor.y + fresnelConductor.z) / 3.0F;
    const AlbedoSplit splitWo = directionalAlbedo(wo.z, params.roughness);
    const AlbedoSplit splitAvg = averageAlbedo(params.roughness);
    // Scaled by E: the specular lobe now has two parts, and only the single-scattering part is drawn by
    // VNDF sampling. The multiple-scattering part is cosine-shaped and picked up by the diffuse strategy,
    // so its selection mass must move there too -- otherwise a rough white metal, whose Fresnel pins
    // specularProb to the 0.95 clamp, would sample 69% of its own reflectance only 5% of the time.
    const float specularProb = std::clamp(
        glm::mix(fresnelAtNormal, conductorLuma, params.metallic) * splitWo.total(), 0.05F, 0.95F);
    const float transmittance = (1.0F - fresnelAtNormal) * (1.0F - params.metallic);
    // Reciprocal diffuse coupling. The substrate receives what the coat did not reflect, on the way in
    // AND on the way out: evaluateDiffuseLobe applies the matching wi-side factor, and the pair is
    // renormalised by 1/(1-coatAlbedoAvg) so the directional albedo integrates back to (1-coatAlbedo(mu_o)).
    // Not exact -- coatAlbedoAvg evaluates coatAlbedo at the averaged split rather than averaging
    // coatAlbedo over wi, and the two differ because the Fresnel rescale varies with direction. The
    // white furnace test bounds the residual to under 1%. Symmetric in wo/wi, which the previous bare
    // (1-F(mu_o)) form was not -- and energy-complete, which it also was not.
    const float coatF0 = dielectricF0(params.ior);
    const float coatAlbedoAvg =
        coatAlbedo(splitAvg, splitAvg.total(), coatF0,
                    dielectricFresnelAvg(params.ior) /
                        std::max(schlickFresnelAvg(glm::vec3(coatF0)).x, 1e-6F));
    const float diffuseCoupling = (1.0F - coatAlbedo(splitWo, splitAvg.total(), coatF0,
                                                       coatFresnelRatio(wo.z, etaI, etaT, coatF0))) /
                                   std::max(1.0F - coatAlbedoAvg, 1e-4F);
    float diffuseProb = 0.0F;
    float transmitProb = 0.0F;
    float diffuseKd = 0.0F;
    float transmitPhysicalValue = 0.0F;
    if (!exiting) {
        diffuseProb = (1.0F - specularProb) * (1.0F - params.transmissionFactor);
        transmitProb = (1.0F - specularProb) * params.transmissionFactor;
        diffuseKd = diffuseCoupling * (1.0F - params.metallic) * (1.0F - params.transmissionFactor);
        transmitPhysicalValue = transmittance * params.transmissionFactor;
    } else {
        transmitProb = 1.0F - specularProb;
        transmitPhysicalValue = transmittance;
    }
    const glm::vec3 fresnelAvg = glm::mix(glm::vec3(dielectricFresnelAvg(params.ior)),
                                           schlickFresnelAvg(params.f0), params.metallic);
    // R_ss uses the same Schlick-split-with-exact-Fresnel-rescale as the coat; T_ss is the (1-fc) channel
    // scaled by (1-f0), which is Schlick's 1-F factored exactly.
    // transmitWeight, not transmissionFactor: the transmission lobe's own energy is gated by
    // (1-metallic) too (see transmittance above), so a metallic=1 material transmits nothing however its
    // transmissionFactor is set. Using the raw factor here credited the escape budget with transmission
    // that never happens, and the compensation handed the difference back as multiple scattering --
    // measured as Lo = 1.43 on a metallic=1, transmission=1 surface.
    const float eta = etaI / etaT;
    const float transmitWeight = params.transmissionFactor * (1.0F - params.metallic);
    // R + T, with no transmissionFactor weighting: energy the interface refracts but transmissionFactor
    // withholds from the transmit lobe enters the diffuse substrate instead (that is what diffuseKd's
    // (1-transmissionFactor) does), and escapes from there. Either way it leaves, so the microfacet
    // deficit the compensation must return is the same. Weighting this term by transmitWeight instead
    // over-reported the deficit at partial transmission -- measured Lo = 2.64 against a bound of 2.25.
    const EscapeSplit escapeWo = escapeAlbedo(wo.z, params.roughness, eta);
    const EscapeSplit escapeMean = averageEscapeAlbedo(params.roughness, eta);
    const float escape = escapeWo.total();
    const float escapeAvg = escapeMean.total();
    const float transmitSsAvg = transmitWeight * escapeMean.transmit;
    return {specularProb,
             diffuseProb,
             transmitProb,
             etaI,
             etaT,
             diffuseKd,
             transmitPhysicalValue,
             splitWo.total(),
             splitAvg.total(),
             coatF0,
             coatAlbedoAvg,
             fresnelAvg,
             escape,
             escapeAvg,
             transmitSsAvg / std::max(escapeAvg, 1e-4F),
             eta * eta,
             transmitWeight};
}

// Below this the GGX transmission lobe is treated as a delta (PBRT's TrowbridgeReitzDistribution::
// EffectivelySmooth). kMinAlpha (roughness 0.02) sits inside this region, so smooth glass keeps the exact,
// noise-free Snell path it has always had rather than becoming a stochastic estimate of the same thing.
constexpr float kSmoothAlpha = 1e-3F;

bool transmissionIsRough(const BsdfParams& params, float alpha) {
    return params.transmissionFactor > 0.0F && alpha >= kSmoothAlpha;
}

// Walter et al. 2007 rough transmission (value eq. 21, half-vector eq. 16, Jacobian eq. 17), in PBRT-v3's
// radiance-transport form. The non-symmetric eta^2 radiance compression (Veach 1997 sec. 5.2) is ALREADY
// folded in here -- PBRT's factor=1/eta, squared, cancels the explicit eta^2 in Walter's importance-mode
// value -- so unlike the delta branch in sampleBsdf this must not multiply by eta^2 again. The etaR^2 that
// survives in the pdf but not the value is the asymmetry, and the transmission round-trip test is what
// catches a double application.
LobeEval evaluateTransmissionLobe(const BsdfParams& params, const glm::vec3& wo, const glm::vec3& wi,
                                   float alpha, const LobeProbabilities& lobes) {
    if (wo.z <= 0.0F || wi.z >= 0.0F) {
        return {glm::vec3(0.0F), 0.0F};
    }
    const float etaR = lobes.etaT / lobes.etaI;
    glm::vec3 ht = glm::normalize(wo + (etaR * wi));
    if (ht.z < 0.0F) {
        ht = -ht;
    }
    const float woDotH = glm::dot(wo, ht);
    const float wiDotH = glm::dot(wi, ht);
    // Both on the same side of the microfacet means this pair is a reflection about ht, not a refraction.
    if (woDotH <= 0.0F || wiDotH >= 0.0F) {
        return {glm::vec3(0.0F), 0.0F};
    }
    const float denom = woDotH + (etaR * wiDotH);
    const float denom2 = denom * denom;
    if (denom2 < 1e-12F) {
        return {glm::vec3(0.0F), 0.0F};
    }
    const float d = distributionGGX(ht.z, alpha);
    const float g2 = smithG2(wo.z, -wi.z, alpha);
    const float fresnel = fresnelDielectric(woDotH, lobes.etaI, lobes.etaT);
    const float common = (d * g2 * std::abs(wiDotH) * woDotH) / (wo.z * -wi.z * denom2);
    const float g1 = smithG1(wo.z, alpha);
    const float vndfPdf = (g1 * woDotH * d) / std::max(wo.z, 1e-6F);
    // Transmitted share of the multiple-scattering energy, carrying the same eta^2 radiance compression
    // and baseColor tint the single-scatter transmission does.
    const glm::vec3 multiScatter = params.baseColor * lobes.transmitWeight * lobes.transmitShare *
                                    lobes.etaSq * multiScatterShape(params, wi.z, lobes);
    // (1-metallic), as the delta branch carries via transmitPhysicalValue: a conductor transmits nothing
    // however its transmissionFactor is set. Omitting it let a metallic=1 surface transmit at full
    // strength, measured as Lo = 1.43 against a bound of 1.0.
    return {(params.baseColor * (1.0F - fresnel) * (1.0F - params.metallic) * common) + multiScatter,
             vndfPdf * etaR * etaR * std::abs(wiDotH) / denom2};
}

glm::vec3 evaluateContinuousLobes(const BsdfParams& params, const glm::vec3& wo, const glm::vec3& wi,
                                   float alpha, const LobeProbabilities& lobes, float& outPdf) {
    // Reflection and transmission occupy disjoint hemispheres, so the mixture is piecewise -- no overlap
    // between the two to double-count.
    if (wi.z < 0.0F) {
        if (!transmissionIsRough(params, alpha)) {
            outPdf = 0.0F;
            return glm::vec3(0.0F);
        }
        const LobeEval transmission = evaluateTransmissionLobe(params, wo, wi, alpha, lobes);
        outPdf = lobes.transmit * transmission.pdf;
        return transmission.f;
    }
    const LobeEval specular = evaluateSpecularLobe(params, wo, wi, alpha, lobes);
    const LobeEval diffuse = evaluateDiffuseLobe(params, wi, lobes);
    outPdf = (lobes.specular * specular.pdf) + (lobes.diffuse * diffuse.pdf);
    return specular.f + diffuse.f;
}

}  // namespace

glm::vec3 fresnelSchlick(float cosTheta, const glm::vec3& f0) {
    return f0 + ((glm::vec3(1.0F) - f0) * std::pow(std::clamp(1.0F - cosTheta, 0.0F, 1.0F), 5.0F));
}

float pdfBsdf(const BsdfParams& params, const glm::vec3& woLocal, const glm::vec3& wiLocal) {
    const float sign = woLocal.z >= 0.0F ? 1.0F : -1.0F;
    const glm::vec3 wo(woLocal.x, woLocal.y, woLocal.z * sign);
    const glm::vec3 wi(wiLocal.x, wiLocal.y, wiLocal.z * sign);
    const float alpha = std::max(params.roughness * params.roughness, kMinAlpha);
    const LobeProbabilities lobes = computeLobeProbabilities(params, wo, sign);
    float pdf = 0.0F;
    evaluateContinuousLobes(params, wo, wi, alpha, lobes, pdf);
    return pdf;
}

glm::vec3 evaluateBsdf(const BsdfParams& params, const glm::vec3& woLocal, const glm::vec3& wiLocal) {
    const float sign = woLocal.z >= 0.0F ? 1.0F : -1.0F;
    const glm::vec3 wo(woLocal.x, woLocal.y, woLocal.z * sign);
    const glm::vec3 wi(wiLocal.x, wiLocal.y, wiLocal.z * sign);
    const float alpha = std::max(params.roughness * params.roughness, kMinAlpha);
    const LobeProbabilities lobes = computeLobeProbabilities(params, wo, sign);
    float pdf = 0.0F;
    return evaluateContinuousLobes(params, wo, wi, alpha, lobes, pdf);
}

glm::vec3 evaluateDiffuseRaw(const BsdfParams& params, const glm::vec3& woLocal,
                              const glm::vec3& wiLocal) {
    const float sign = woLocal.z >= 0.0F ? 1.0F : -1.0F;
    const glm::vec3 wo(woLocal.x, woLocal.y, woLocal.z * sign);
    const glm::vec3 wi(wiLocal.x, wiLocal.y, wiLocal.z * sign);
    if (wi.z <= 0.0F) {
        return glm::vec3(0.0F);
    }
    const LobeProbabilities lobes = computeLobeProbabilities(params, wo, sign);
    // Same kd as evaluateDiffuseLobe, minus the baseColor factor -- both halves of the reciprocal
    // coupling, so this AOV tracks the shaded value rather than drifting from it.
    return glm::vec3(diffuseKdAt(params, wi, lobes) / kPi);
}

glm::vec3 evaluateSpecularOnly(const BsdfParams& params, const glm::vec3& woLocal,
                                const glm::vec3& wiLocal) {
    const float sign = woLocal.z >= 0.0F ? 1.0F : -1.0F;
    const glm::vec3 wo(woLocal.x, woLocal.y, woLocal.z * sign);
    const glm::vec3 wi(wiLocal.x, wiLocal.y, wiLocal.z * sign);
    const float alpha = std::max(params.roughness * params.roughness, kMinAlpha);
    const LobeProbabilities lobes = computeLobeProbabilities(params, wo, sign);
    return evaluateSpecularLobe(params, wo, wi, alpha, lobes).f;
}

std::optional<BsdfSample> sampleBsdf(const BsdfParams& params, const glm::vec3& woLocal,
                                      Sampler& sampler) {
    const float sign = woLocal.z >= 0.0F ? 1.0F : -1.0F;
    const glm::vec3 wo(woLocal.x, woLocal.y, woLocal.z * sign);
    const float alpha = std::max(params.roughness * params.roughness, kMinAlpha);
    const LobeProbabilities lobes = computeLobeProbabilities(params, wo, sign);

    const float lobeU = sampler.next1D();

    if (lobeU < lobes.specular + lobes.diffuse) {
        const bool sampledSpecular = lobeU < lobes.specular;
        glm::vec3 wi;
        if (sampledSpecular) {
            const glm::vec3 nh = sampleGGXVNDF(wo, alpha, sampler.next2D());
            wi = glm::reflect(-wo, nh);
        } else {
            wi = sampleCosineHemisphere(sampler.next2D());
        }
        if (wi.z <= 0.0F) {
            return std::nullopt;
        }
        float pdf = 0.0F;
        const glm::vec3 f = evaluateContinuousLobes(params, wo, wi, alpha, lobes, pdf);
        if (pdf <= 1e-8F) {
            return std::nullopt;
        }
        const glm::vec3 throughput = (f * wi.z) / pdf;
        const glm::vec3 rawThroughput = [&]() -> glm::vec3 {
            if (!sampledSpecular) {
                return glm::vec3(diffuseKdAt(params, wi, lobes));
            }
            const LobeEval spec = evaluateSpecularLobe(params, wo, wi, alpha, lobes);
            return spec.pdf > 1e-8F ? spec.f * wi.z / spec.pdf : glm::vec3(0.0F);
        }();
        return BsdfSample{glm::vec3(wi.x, wi.y, wi.z * sign), throughput,
                           sampledSpecular ? LobeType::SpecularReflection : LobeType::Diffuse,
                           rawThroughput};
    }

    if (lobes.transmit <= 0.0F) {
        return std::nullopt;
    }

    // Rough transmission (Walter 2007): refract about a VNDF-sampled microfacet normal rather than the
    // macro normal. A microfacet steep enough to totally internally reflect yields no transmission sample.
    if (transmissionIsRough(params, alpha)) {
        const glm::vec3 ht = sampleGGXVNDF(wo, alpha, sampler.next2D());
        glm::vec3 wi;
        if (!refractAbout(wo, ht, lobes.etaI / lobes.etaT, wi) || wi.z >= 0.0F) {
            return std::nullopt;
        }
        float pdf = 0.0F;
        const glm::vec3 f = evaluateContinuousLobes(params, wo, wi, alpha, lobes, pdf);
        if (pdf <= 1e-8F) {
            return std::nullopt;
        }
        const glm::vec3 throughput = (f * -wi.z) / pdf;
        return BsdfSample{glm::vec3(wi.x, wi.y, wi.z * sign), throughput, LobeType::Transmission,
                           throughput};
    }

    // Smooth specular transmission (delta lobe): Snell's law, TIR already folded into lobes.specular via fresnelDielectric returning 1.0 past the critical angle.
    const float eta = lobes.etaI / lobes.etaT;
    const float sin2ThetaT = eta * eta * std::max(0.0F, 1.0F - (wo.z * wo.z));
    if (sin2ThetaT >= 1.0F) {
        return std::nullopt;
    }
    const float cosThetaT = std::sqrt(1.0F - sin2ThetaT);
    const glm::vec3 wt(-eta * wo.x, -eta * wo.y, -cosThetaT);
    // Non-symmetric radiance-compression factor for camera-originated (Veach 1997 sec. 5.2, PBRT's SpecularTransmission::Sample_f under TransportMode::Radiance) transport: eta^2 = (etaI/etaT)^2, the squared ratio of the medium the ray is leaving to the medium it's entering. Self-consistent under round trips -- entering (eta=1/ior) times exiting (eta=ior/1) squared multiplies to 1, so a ray that enters and exits the same surface loses no net energy (tools/bsdf_validate.cpp's furnace test).
    const glm::vec3 throughput =
        params.baseColor * (lobes.transmitPhysicalValue / lobes.transmit) * (eta * eta);
    return BsdfSample{glm::vec3(wt.x, wt.y, wt.z * sign), throughput, LobeType::Transmission,
                       throughput};
}

}  // namespace engine::scene
