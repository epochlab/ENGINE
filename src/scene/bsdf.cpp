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

// --- Conductor Fresnel: Gulbrandsen 2014, "Artist Friendly Metallic Fresnel", JCGT 3(4), ported
// from the paper's Appendix A listing. Replaces Schlick on the metal path, which is monotone in
// cos by construction ((1-c)^5 >= 0) and therefore forces every conductor to exactly white at
// grazing and cannot express the reflectance dip real metals have. Parameterised by reflectivity
// r (params.f0) and edgetint g (params.edgeTint), inverted to a complex IOR (n, k).

// Reflectivity is clamped, not asserted: f0 arrives from resolveBsdfParams as an unbounded texture
// product (baseColor * diffuseColour), so both ends are reachable from an asset.
// Upper end: at r=1 nMax is infinite and g=1 evaluates 0*inf = NaN. The listing clamps to 0.99, which
// costs 1% of normal-incidence reflectance at f0=1 -- enough to move the white furnace test onto its
// tolerance edge. 0.9999 is safe here because of the k^2 form below; measured in float32, n and k land
// within 1.4e-8 of their double values and the furnace rows shift by under 2e-4.
// Lower end: r=0 inverts to n=1, k=0 -- an index-matched interface, where the Fresnel below is 0/0 at
// cosTheta=0 exactly. Reachable from any black texel on a metal. At the 1e-4 floor k=0.02, F tends to 1
// at grazing and nothing is degenerate; the floor costs a pure-black metal 1e-4 of normal-incidence
// reflectance.
constexpr float kMinReflectivity = 1e-4F;
constexpr float kMaxReflectivity = 0.9999F;

struct ConductorIor {
    glm::vec3 n;
    glm::vec3 k;
};

// Paper eq 12 for n (a linear blend in g between the two ends of eq 11's range) and eq 2 for k.
// k^2 is evaluated as (nMax - n)(n - nLow) rather than the listing's ((n+1)^2*r - (n-1)^2)/(1-r).
// The two are the same expression -- those factors are eq 3's interval endpoints, i.e. eq 2's own roots
// -- and agree to 4e-12 relative in double, which tools/bsdf_validate.cpp asserts against the literal
// form. The literal form is what forces the listing's 0.99 clamp: it subtracts two large near-equal
// numbers, and in float32 at r=0.9999, g=0 it returns k^2 = -1.28e6 where the true value is exactly 0.
// The factored form returns exactly 0 there, since nMax - n is exactly zero at g=0.
// Authoring a real metal, for reference: reflectivity and edgeTint both come from measured n,k through the paper's eq 14/15, NOT from eyeballing a colour. Chromium at 615/550/465 nm from Johnson & Christy 1974 (refractiveindex.info main/Cr/nk/Johnson.yml) gives reflectivity [0.552, 0.555, 0.558], edgeTint [0.555, 0.558, 0.672] -- shipped in chrome.json until the preset was returned to an idealised near-white mirror, and reinstatable by pasting those two triples back.
// That look is 0.58x darker at normal incidence than the mirror it replaced, plus the grazing dip only edgeTint can carry -- the part Schlick cannot express at any f0. The cool cast is NOT the edge tint: it comes from the reflectivity triple already leaning blue (+0.006), since edgeTint acts at grazing and has ~zero effect at normal incidence.
// Round-tripping the two triples through this inversion recovers the source n,k to 0.0048, inside its three-significant-figure precision, so the 3-decimal rounding is the whole error budget. Representative wavelengths, not a spectral integration -- this renderer is RGB with no spectral upsampling, and across plausible wavelength triples reflectivity moves under 0.007 and edgeTint under 0.025.
ConductorIor conductorIorFromReflectivity(const glm::vec3& reflectivity, const glm::vec3& edgeTint) {
    const glm::vec3 r = glm::clamp(reflectivity, kMinReflectivity, kMaxReflectivity);
    const glm::vec3 g = glm::clamp(edgeTint, 0.0F, 1.0F);  // the paper's stated domain for g
    const glm::vec3 sqrtR = glm::sqrt(r);
    const glm::vec3 nMin = (1.0F - r) / (1.0F + r);
    const glm::vec3 nMax = (1.0F + sqrtR) / (1.0F - sqrtR);
    const glm::vec3 nLow = (1.0F - sqrtR) / (1.0F + sqrtR);
    const glm::vec3 n = (g * nMin) + ((1.0F - g) * nMax);
    // Both factors are non-negative across the clamped domain (nLow <= nMin <= n <= nMax); the max
    // absorbs float rounding on nMax - n at g -> 0, where the true value is zero.
    return {n, glm::sqrt(glm::max((nMax - n) * (n - nLow), 0.0F))};
}

// Exact unpolarized Fresnel reflectance of one channel of a conductor with complex IOR n + ik (Born &
// Wolf), in the standard real-arithmetic form: two hardware sqrts, no complex division, and no <complex>
// in this translation unit. NOT the paper's Appendix A rs/rp, which is the large-|eta| approximation
// (PBRT-v2's FrCond) and deviates from this by up to 0.094 absolute around r~0.25 -- mid reflectivity,
// which is where real metals sit -- chrome.json ships an idealised near-white mirror (r 0.95/0.95/0.97, edgeTint 1, no dip), well above that. bsdf_validate compares
// this against the complex-arithmetic definition, which the two forms match to 2.2e-12 over the clamped
// (r, g, cosTheta) domain.
// Deliberately free of clamps, unlike the inversion above, where a max() guards an exact-zero boundary
// (g=0) that float rounding can push slightly negative. Here no denominator can vanish once r is floored
// away from 0: a2b2 - t0 > 0 follows from a2b2 >= |t0| alone, and a2b2 itself is positive either because
// k >= 0.02 (as g drives n toward nMin) or because n = nMax > 1 forces t0 = n^2 - s2 > 0 at g=0, the one
// input where k is exactly 0. So a max() here could not prevent a fault, only hide one -- which is
// exactly what one did, see below.
float fresnelConductorChannel(float cosTheta, float n, float k) {
    const float c2 = cosTheta * cosTheta;
    const float s2 = 1.0F - c2;
    const float nk2 = (n * n) * (k * k);
    const float t0 = (n * n) - (k * k) - s2;
    const float a2b2 = std::sqrt((t0 * t0) + (4.0F * nk2));
    // a^2 = (a2b2 + t0)/2, taken through whichever of its two algebraically equal forms is a SUM.
    // Direct when t0 >= 0. When t0 < 0 the direct form subtracts two near-equal magnitudes: at f0=1,
    // n is ~5e-5 and it must resolve 2.5e-9 out of two numbers near 1.0, which in float32 collapses a to
    // zero and pins F at exactly 1.0. Measured, that is Schlick's own answer at f0=1, so the white
    // furnace's conductor rows came back byte-identical and the fault read as "the change is inert".
    // (a2b2 + t0)(a2b2 - t0) = 4*n^2*k^2 turns that branch into a division by a sum.
    const float a2 = t0 >= 0.0F ? (a2b2 + t0) * 0.5F : (2.0F * nk2) / (a2b2 - t0);
    const float a = std::sqrt(a2);
    const float t1 = a2b2 + c2;
    const float t2 = 2.0F * a * cosTheta;
    const float rPerpendicular = (t1 - t2) / (t1 + t2);
    const float t3 = (c2 * a2b2) + (s2 * s2);
    const float t4 = t2 * s2;
    // R = (rPerp + rPara)/2 with rPara = rPerp*(t3-t4)/(t3+t4), factored to drop a multiply.
    return 0.5F * rPerpendicular * (1.0F + ((t3 - t4) / (t3 + t4)));
}

// cosTheta is clamped to [0,1] rather than sign-swapped the way fresnelDielectric handles etaI/etaT: a
// conductor has no far side to enter, and grazing-angle normal mapping can push wo.z negative.
glm::vec3 fresnelConductor(float cosTheta, const glm::vec3& n, const glm::vec3& k) {
    const float c = std::clamp(cosTheta, 0.0F, 1.0F);
    return {fresnelConductorChannel(c, n.x, k.x), fresnelConductorChannel(c, n.y, k.y),
             fresnelConductorChannel(c, n.z, k.z)};
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

// Directional albedo of the single-scattering GGX lobe with Fresnel forced to 1, the fraction of energy smithG2 lets through, so 1-E is exactly what multiple scattering must return (Kulla & Conty 2017, "Revisiting Physically Based Shading at Imageworks").
// Depends on nothing but (mu, alpha): Fresnel, metallic, baseColor, and lobe-selection probabilities are all applied by the caller, never baked in here.
// Indexed by perceptual roughness rather than alpha: E is far better distributed in sqrt(alpha), and it is what callers already hold. Grid is edge-aligned so roughness 0 / mu 1 are exact table entries.
constexpr int kAlbedoRes = 32;
constexpr int kAlbedoSamples = 16;  // per axis; 256 stratified samples/cell, ~1.5e-3 vs a 128x128 reference

// Split by Schlick's form F(c) = f0*(1 - (1-c)^5) + (1-c)^5 so one table serves any f0 (the standard environment-BRDF split): Ess(mu, f0) = f0*a + b, and with f0=1 that collapses to a + b = E, the Fresnel-free albedo the multiple-scattering lobe needs. Two channels, no third axis for ior.
// The transmit side needs a third axis. Its energy curve is not the reflect side's: the below-horizon reflections that drive E down are the valid side for refraction, so far fewer samples are discarded (measured 0.559 combined vs 0.307 reflect-only at roughness 1.0).
// Unlike the reflect side it genuinely depends on eta (G2 uses the refracted |wi.z|, and TIR gates validity), so the Schlick split cannot factor it out. One axis in log(eta) covers entering and exiting, since the two are reciprocals of each other.
constexpr int kEtaRes = 16;
constexpr float kEtaMin = 1.0F / 2.5F;  // exiting a 2.5-ior medium; the reciprocal end is entering one
constexpr float kEtaMax = 2.5F;

struct AlbedoTable {
    std::array<float, static_cast<std::size_t>(kAlbedoRes) * kAlbedoRes> a;  // [roughnessIndex][muIndex]
    std::array<float, static_cast<std::size_t>(kAlbedoRes) * kAlbedoRes> b;
    std::array<float, kAlbedoRes> aavg;  // cosine-weighted means, 2*integral(.(mu)*mu dmu)
    std::array<float, kAlbedoRes> bavg;
    // Escaping fraction of a dielectric interface, split into the reflected and transmitted shares and indexed [roughnessIndex][muIndex][etaIndex]. Both use exact dielectric Fresnel at build time rather than the Schlick split above: inside the total-internal-reflection cone exact Fresnel is 1.0 while Schlick reads ~0.1, so no rescale of a Schlick-basis number can stand in for it, and the escape budget would under-count the reflected share by the whole TIR cone.
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

// Deterministic stratified midpoint quadrature, not RNG Monte Carlo: the integrand is smooth, and a fixed grid keeps the table bit-identical across runs and machines (see the determinism note on -march=native). Below-horizon reflections contribute zero; that discard is part of the energy loss being measured.
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
                    // Same visible normal, reflected and refracted: the VNDF sample is the expensive part and is shared across every eta, so the third axis costs only the refraction.
                    // A facet reflects with probability F and refracts with 1-F, so the two shares are Fresnel-weighted complements of one throughput, never independent quantities.
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
            // Trapezoid over the mu axis: the grid is edge-aligned, so the two endpoints span half a cell each and the step is 1/(kAlbedoRes-1), not 1/kAlbedoRes.
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

// Bilinear lookup. E climbs steeply as mu->0 (0.31 at mu=1, ~1.0 at grazing, alpha=1), so the first mu bin carries the largest interpolation error, harmless since every integral consuming E weights grazing by cos(theta).
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

}  // namespace

// The two average-Fresnel terms have external linkage: bsdf.h declares them for tools/bsdf_validate.cpp's checkAverageFresnel, the only instrument in the suite that can see an error in either (see the header comment). Everything around them stays internal.
// Cosine-weighted average Fresnel, the normalisation both the multiple-scattering tint and the reciprocal diffuse coupling need. The dielectric one is the standard rational fit, accurate to 0.0065 absolute over ior in [1.1, 3.0] against exact quadrature (measured, and asserted by checkAverageFresnel); it enters as the 1/(1-Favg) normalisation, a 0.25% effect at ior 1.5, and as the coat's own multiple-scattering attenuation in coatAlbedo.
float dielectricFresnelAvg(float ior) { return (ior - 1.0F) / ((4.08567F + (1.00071F * ior))); }

// Cosine-weighted average of the exact conductor Fresnel, as a 3-node quadrature rule sum(w_i*F(mu_i)) over the same fresnelConductorChannel the single scatter evaluates, so the average and the term it compensates describe one interface. Karis' mean is exact for Schlick and therefore the mean of a DIFFERENT function once the single scatter is complex-IOR; worse, its error changes sign with edgeTint, which f0 alone cannot express.
// Nodes and weights fitted by equality-constrained least squares against 128-point Gauss-Legendre over the whole clamped Gulbrandsen domain (r in [1e-4, 0.9999] x g in [0, 1]): max absolute error 4.0e-4, RMS 1.2e-4, measured in float32 through the shipped inversion. Karis is 216x worse there (max 0.086, at r=0.255 g=1).
// A rational fit over (r, g) was measured and rejected: at r=0.99 the inverted n collapses from 39.8 to 0.005 across the last tenth of g, a boundary layer no low-order form in that chart holds -- 49 terms reached only 9e-3. Sampling the function's own values sidesteps the chart, and (n, k) is what F_avg actually depends on.
// Weights sum to 1 (to 1e-9 in float32), so the rule is near-exact wherever F is constant in mu -- the r->1 mirror checkWhiteFurnaceTwoSided runs on, where it lands within 4e-8 -- and is bounded by F itself, so it cannot leave [0, 1] by more than that residual. multiScatterTint's 1/(1-Favg) survives the overshoot regardless: its f*f*a numerator goes to zero on the same approach.
constexpr float kFresnelAvgNodes[3] = {0.105319802F, 0.382154433F, 0.796427281F};
constexpr float kFresnelAvgWeights[3] = {0.038972482F, 0.280518736F, 0.680508783F};

glm::vec3 conductorFresnelAvg(const glm::vec3& n, const glm::vec3& k) {
    glm::vec3 sum(0.0F);
    for (int i = 0; i < 3; ++i) {
        sum += kFresnelAvgWeights[i] * fresnelConductor(kFresnelAvgNodes[i], n, k);
    }
    return sum;
}

// Fraunhofer d, F and C lines: the three wavelengths the Abbe number is defined at, V_d = (n_d-1)/(n_F-n_C), and the only wavelengths (ior, abbe) actually pins. Physical constants of the definition, not tuning.
constexpr float kLambdaDNm = 587.56F;
constexpr float kLambdaFNm = 486.13F;
constexpr float kLambdaCNm = 656.27F;

// Cauchy's two-term dispersion n(lambda) = A + B/lambda^2, with (A, B) inverted from the authored (n_d, V_d) -- the construction Khronos KHR_materials_dispersion specifies normatively. Two terms is the right order here: the material supplies exactly two numbers, so a Sellmeier form would have to invent its remaining coefficients.
// B follows from the Abbe definition applied to the Cauchy form, n_F - n_C = B*(lambda_F^-2 - lambda_C^-2), and A from pinning n(lambda_d) = n_d. Written in that general form rather than the spec's composite one, which pre-multiplies 1/(lambda_F^-2 - lambda_C^-2) into a literal 523655 and hides both Fraunhofer lines inside it.
// abbe <= 0 is the documented off switch (Arnold's transmission_dispersion_abbe and OpenPBR's dispersion scale use the same convention) and is also what keeps 1/abbe off the hot path for every non-dispersive material. No clamp on the result: n_d = 1 already gives B = 0 exactly, so an index-matched medium is non-dispersive out of the algebra rather than by special case.
float cauchyIor(float iorD, float abbe, float lambdaNm) {
    if (abbe <= 0.0F) {
        return iorD;
    }
    const float b = (iorD - 1.0F) /
                    (abbe * ((1.0F / (kLambdaFNm * kLambdaFNm)) - (1.0F / (kLambdaCNm * kLambdaCNm))));
    const float a = iorD - (b / (kLambdaDNm * kLambdaDNm));
    return a + (b / (lambdaNm * lambdaNm));
}

namespace {

glm::vec3 schlickFresnelAvg(const glm::vec3& f0) { return f0 + ((glm::vec3(1.0F) - f0) / 21.0F); }

// Normal-incidence reflectance implied by the ior -- the dielectric coat's own f0, independent of the
// (currently unrelated, see A5) f0 texture the conductor path uses.
float dielectricF0(float ior) {
    const float r = (ior - 1.0F) / (ior + 1.0F);
    return r * r;
}

// Kulla-Conty multiple-scattering tint: the share of the (1-E) energy that survives repeated bounces on the microsurface, each one attenuated by Favg. Equals 1 for a perfect reflector (Favg=1), so a white conductor conserves exactly.
float multiScatterTint(float fresnelAvg, float albedoAvg) {
    return (fresnelAvg * fresnelAvg * albedoAvg) /
           std::max(1.0F - (fresnelAvg * (1.0F - albedoAvg)), 1e-4F);
}

float schlickScalar(float cosTheta, float f0) {
    return f0 + ((1.0F - f0) * std::pow(std::clamp(1.0F - cosTheta, 0.0F, 1.0F), 5.0F));
}

// etaI/etaT rather than a bare ior: on the exiting side fresnelDielectric returns exactly 1.0 past the critical angle, and Schlick has no way to express total internal reflection at all. Hardcoding the entering orientation here under-reported the reflected share of an exiting ray by up to the whole TIR cone, which the compensation then tried to hand back as multiple scattering.
float coatFresnelRatio(float cosTheta, float etaI, float etaT, float f0) {
    return fresnelDielectric(cosTheta, etaI, etaT) / std::max(schlickScalar(cosTheta, f0), 1e-6F);
}

// Total directional albedo of the dielectric coat: single scatter plus its own multiple-scattering lobe. This, not the macro-facet Fresnel F(mu_o), is what the coat actually reflects: at roughness 1 and mu 0.4 the two differ by 4x (0.030 vs 0.129), and coupling the diffuse substrate to F(mu_o) hands that difference to neither lobe, measured as a 10% energy loss before this was used.
// fresnelRatio rescales the single-scatter term by exact-dielectric / Schlick Fresnel at this direction. The table is built on Schlick's basis (so one table serves any f0) but the specular lobe evaluates exact fresnelDielectric, and Schlick under-predicts it at grazing, leaving the substrate too much energy and creating ~1.4% at smooth grazing angles.
// The rescale makes the two agree exactly in the smooth limit, where the coat albedo is the Fresnel term, and approximately as roughness widens the lobe away from the macro angle. It also collapses correctly at ior=1, where exact Fresnel is identically zero but Schlick's (1-c)^5 tail is not.
// fresnelAvg is the coat's own cosine mean, dielectricFresnelAvg(ior), for the same reason fresnelRatio rescales the single scatter: the coat reflects by exact fresnelDielectric, so Karis' Schlick mean of coatF0 describes the wrong function here too (-0.0061 against -0.0023 at ior 1.5) and, unlike the exact mean, does not collapse to 0 at ior=1 where the interface reflects nothing.
float coatAlbedo(const AlbedoSplit& split, float albedoAvg, float f0, float fresnelRatio,
                  float fresnelAvg) {
    return (split.at(f0) * fresnelRatio) +
           (multiScatterTint(fresnelAvg, albedoAvg) * (1.0F - split.total()));
}

// Below this the GGX transmission lobe is treated as a delta (PBRT's TrowbridgeReitzDistribution::EffectivelySmooth). kMinAlpha (roughness 0.02) sits inside this region, so smooth glass keeps the exact, noise-free Snell path it has always had rather than becoming a stochastic estimate of the same thing.
constexpr float kSmoothAlpha = 1e-3F;

// Below this deficit there is no multiple scattering worth returning and the lobe switches off entirely: value (multiScatterShape) and selection probability (computeLobeProbabilities) must use the same test or the mixture allocates mass to a zero lobe.
constexpr float kMinDeficit = 1e-3F;

bool transmissionIsRough(const BsdfParams& params, float alpha) {
    return params.transmissionFactor > 0.0F && alpha >= kSmoothAlpha;
}

struct LobeEval {
    glm::vec3 f;
    float pdf;
};

struct LobeProbabilities {
    float specular;
    float diffuse;
    float transmit;     // single-scatter refraction, VNDF-sampled about a microfacet normal
    float msTransmit;   // multiple-scattering transmission, cosine-sampled over the far hemisphere
    float etaI;
    float etaT;
    float diffuseKd;              // evaluateDiffuseLobe's wo-side energy factor, 0 on the exiting side
    float transmitPhysicalValue;  // transmission's true (1-F)*t energy fraction -- see below
    // Energy-compensation state, hoisted here so the wo-side table lookups happen once per evaluation
    // rather than once per lobe call.
    float albedoWo;         // E(mu_o, roughness), Fresnel-free
    float albedoAvg;        // Eavg(roughness)
    float coatF0;           // dielectric f0 implied by ior, for the diffuse coupling
    glm::vec3 fresnelAvg;
    // Complex IOR inverted from (f0, edgeTint) once per evaluation rather than once per lobe call.
    // Set to the index-matched (1, 0) when metallic==0, where no consumer reads them: evaluateSpecularLobe
    // gates its conductor Fresnel on the SAME metallic>0 test computeLobeProbabilities gates this on, and
    // the two must stay identical -- (1, 0) is 0/0 at cosTheta=0 exactly, the one degenerate input.
    glm::vec3 conductorN;
    glm::vec3 conductorK;
    // Multiple-scattering state for a transmissive interface, which needs its own deficit. A facet either reflects or refracts, chosen by Fresnel, so the escaping fraction is inherently Fresnel-weighted (R_ss + T_ss) and cannot reuse the opaque path's Fresnel-free (1 - E); adding the two Fresnel-free throughputs double-counts the same facets and drives the deficit negative.
    // The two formulations are therefore blended by transmissionFactor rather than unified, so an opaque material keeps exactly the measured behaviour the opaque compensation already has.
    float escapeWo;         // R_ss(mu_o) + T_ss(mu_o), the Fresnel-weighted escaping fraction
    float escapeAvg;
    float escapeAvgRecip;   // averageEscapeAlbedo at the reciprocal eta (etaT/etaI) -- see multiScatterShape
    float transmitShare;    // of the multiple-scattered energy, the fraction leaving refracted
    float etaSq;            // (etaI/etaT)^2, the radiance compression the transmit lobe must carry
    // effectiveTransmission*(1-metallic): how much transmission actually happens. Scales both the single-scatter and the multiple-scattering transmit value; the delta branch carries the same factors through transmitPhysicalValue.
    float transmitWeight;
};

// kd carries the wo-side (1-F)/(1-Favg) coupling; the matching wi-side (1-F) factor is applied here, so the lobe is reciprocal (A4) while its directional albedo still integrates to (1-F(mu_o)), same total energy as the old one-sided form, correctly distributed.
// pdf must not be gated on kd: sampleBsdf still selects this lobe with probability lobes.diffuse (independent of kd, see computeLobeProbabilities), so the pdf side of the MIS mixture must match that selection density regardless of how little/no value the lobe carries; gating pdf on kd starves the mixture denominator and inflates throughput for metals (kd=0 but diffuseProb>0).
// Shared shape of the multiple-scattering lobe, on whichever hemisphere wi lies. Cosine-distributed and symmetric in wo/wi. eta/deficitAvg are passed in rather than read off lobes because the two callers below need different orientations: a reflected wi stays in wo's medium (eta = etaI/etaT, lobes.escapeAvg), but a transmitted wi has crossed into the far medium and its escape must be looked up in the reciprocal orientation (eta = etaT/etaI, lobes.escapeAvgRecip) -- averageEscapeAlbedo is only a self-normalising cosine mean of escapeAlbedo when both are evaluated at the same eta, so pairing the wrong eta with the wrong average would perturb the total-energy identity below rather than merely mis-shape it.
// Integrates over one full hemisphere to exactly (1 - escapeWo), since deficitAvg is the cosine-weighted mean of the same escape(mu) looked up here at the same eta, so the reflected share (1 - transmitShare) and the transmitted share transmitShare sum to the deficit across the two.
// Both shares are delivered over their whole hemisphere, which requires the transmitted one to sit outside evaluateTransmissionLobe's half-vector rejections. It can only live there because lobes.msTransmit gives it a sampling density over that whole hemisphere; without one, energy outside the refraction cone would be unsamplable and bias the estimator rather than merely darken it.
float multiScatterShape(const BsdfParams& params, float wiZ, float escapeWo, float eta,
                         float deficitAvg) {
    // Guarded rather than clamped: every deficit tends to zero together as roughness falls and the ratio
    // stays finite, but flooring the denominator alone breaks that cancellation and turns a vanishing lobe
    // into a huge one. computeLobeProbabilities gates selection mass on the identical test, so the mixture
    // never allocates to a lobe that is identically zero.
    if (deficitAvg <= kMinDeficit) {
        return 0.0F;
    }
    const float mu = std::abs(wiZ);
    const float escapeWi = escapeAlbedo(mu, params.roughness, eta).total();
    return (std::max(1.0F - escapeWo, 0.0F) * std::max(1.0F - escapeWi, 0.0F)) / (kPi * deficitAvg);
}

// The transmitted share of the multiple-scattering energy, for ANY wi on the far side -- deliberately
// free of evaluateTransmissionLobe's half-vector rejections, which describe single scattering only.
// Carries the same eta^2 radiance compression and baseColor tint the single-scatter transmission does.
// wi has crossed the interface, so its escape is looked up in the reciprocal orientation (etaT/etaI,
// escapeAvgRecip) -- see multiScatterShape's doc comment.
glm::vec3 transmitMultiScatter(const BsdfParams& params, float wiZ, const LobeProbabilities& lobes) {
    return params.baseColor * lobes.transmitWeight * lobes.transmitShare * lobes.etaSq *
           multiScatterShape(params, wiZ, lobes.escapeWo, lobes.etaT / lobes.etaI,
                              1.0F - lobes.escapeAvgRecip);
}

// The full reciprocal coupling factor at wi: the wo-side half is precomputed into lobes.diffuseKd, the
// wi-side half is the same (1 - coatAlbedo) evaluated here.
float diffuseKdAt(const BsdfParams& params, const glm::vec3& wi, const LobeProbabilities& lobes) {
    const AlbedoSplit splitWi = directionalAlbedo(wi.z, params.roughness);
    const float coat = coatAlbedo(splitWi, lobes.albedoAvg, lobes.coatF0,
                                   coatFresnelRatio(wi.z, lobes.etaI, lobes.etaT, lobes.coatF0),
                                   dielectricFresnelAvg(params.ior));
    return std::max(lobes.diffuseKd, 0.0F) * (1.0F - coat);
}

// --- EON rough-diffuse BRDF (Portsmouth, Kutz, Hill 2025, "EON: A Practical Energy-Preserving Rough
// Diffuse BRDF", JCGT 14(1)) -- replaces plain Lambertian as evaluateDiffuseLobe's base reflectance
// below, still wrapped by diffuseKdAt's Fresnel-coat coupling above. Builds on Fujii's FON model (a
// corrected qualitative Oren-Nayar) with an analytic multiple-scattering compensation term, the same
// Kulla & Conty 2017 philosophy this file's specular lobe already applies to GGX. Ported directly from
// the paper's reference GLSL listings; do not hand-derive replacement constants from memory.

constexpr float kConstant1Fon = 0.5F - (2.0F / (3.0F * kPi));
constexpr float kConstant2Fon = (2.0F / 3.0F) - (28.0F / (15.0F * kPi));

// FON directional albedo, quartic polynomial fit (paper eq. 14): accurate to <0.1% versus the exact
// trigonometric form and ~5x cheaper to evaluate -- used exclusively, the exact form has no consumer here.
float evalFonAlbedoApprox(float mu, float r) {
    const float muComplement = 1.0F - mu;
    constexpr float g1 = 0.0571085289F;
    constexpr float g2 = 0.491881867F;
    constexpr float g3 = -0.332181442F;
    constexpr float g4 = 0.0714429953F;
    const float gOverPi =
        muComplement * (g1 + (muComplement * (g2 + (muComplement * (g3 + (muComplement * g4))))));
    const float af = 1.0F / (1.0F + (kConstant1Fon * r));
    return (1.0F + (r * gOverPi)) * af;
}

}  // namespace

// Paper Appendix A: the rho achieving a desired observed albedo, so diffuseColour means what OpenPBR says base_color means -- "the observed reflection color (viewed at normal incidence under uniform illumination) in areas where the Fresnel reflection is negligible". OpenPBR declares that meaning but then sets rho = C directly, which does not deliver it at high diffuseRoughness; this closes the gap, measured at 9% relative for C=0.5 at r=1.
// Eq. 29 gives the normalised FON albedos at normal incidence, E_F(N) = 1/(1+c1*r) and <E_F> = (1+c2*r)/(1+c1*r). Setting eq. 28's E_EON(N) = C yields the quadratic a*rho^2 + b*rho - C = 0 with eq. 31's a = <E_F> - E_F(N) and b = E_F(N) + C*(1-<E_F>), both non-negative over the whole domain.
// Eq. 30 states the root as (-b + sqrt(b*b + 4ac))/(2a), which is the UNSTABLE one: a is proportional to r, so as r -> 0 a vanishing denominator divides a difference of near-equal quantities. The paper's remedy is a Taylor form switched in below some roughness; the conjugate-multiplied root used here is algebraically identical, needs no such threshold, and cancels nothing since b > 0 throughout (Press et al., Numerical Recipes 5.6). Constants and both coefficients are the appendix's, untouched.
// Hence the whole domain is one branch-free expression: r=0 gives a=0, b=1 and so rho=C -- the Lambertian identity, out of the algebra rather than special-cased -- and C=1 gives rho=1 exactly, leaving the white furnace untouched.
// Exported for tools/bsdf_validate.cpp's checkEonAlbedoInversion; resolved once per hit in gbuffer_shading.cpp rather than per lobe evaluation, since evaluateBsdfSplit runs repeatedly per vertex.
glm::vec3 eonAlbedoInversion(const glm::vec3& albedo, float r) {
    const float eFonNormal = 1.0F / (1.0F + (kConstant1Fon * r));
    const float avgEFon = eFonNormal * (1.0F + (kConstant2Fon * r));
    const float a = avgEFon - eFonNormal;
    const glm::vec3 b = glm::vec3(eFonNormal) + (albedo * (1.0F - avgEFon));
    return (2.0F * albedo) / (b + glm::sqrt((b * b) + (4.0F * a * albedo)));
}

namespace {

// EON BRDF value (paper eq. 16-19): FON single scatter plus an analytic multiple-scattering lobe. rho
// is the single-scattering albedo, NOT the authored colour: eonAlbedoInversion above maps one to the
// other, so that the albedo this lobe is observed to have is the albedo the material asked for.
glm::vec3 evaluateEon(const glm::vec3& rho, float r, const glm::vec3& wi, const glm::vec3& wo) {
    const float muI = wi.z;
    const float muO = wo.z;
    const float s = glm::dot(wi, wo) - (muI * muO);
    const float sOverT = s > 0.0F ? s / std::max(muI, muO) : s;
    const float af = 1.0F / (1.0F + (kConstant1Fon * r));
    const glm::vec3 singleScatter = (rho / kPi) * af * (1.0F + (r * sOverT));

    const float eFonO = evalFonAlbedoApprox(muO, r);
    const float eFonI = evalFonAlbedoApprox(muI, r);
    const float avgEFon = af * (1.0F + (kConstant2Fon * r));
    const glm::vec3 rhoMs = (rho * rho) * avgEFon / (glm::vec3(1.0F) - (rho * (1.0F - avgEFon)));
    constexpr float kEps = 1e-7F;
    const glm::vec3 multiScatter = (rhoMs / kPi) * std::max(kEps, 1.0F - eFonO) *
                                    std::max(kEps, 1.0F - eFonI) / std::max(kEps, 1.0F - avgEFon);
    return singleScatter + multiScatter;
}

// Uniform hemisphere direction (z = u.x directly, not remapped to [-1,1]): pdf = 1/(2*pi). EON's
// defensive-sampling companion to CLTC below (Owen & Zhou 2000's one-sample MIS), not a general utility
// -- sampleCosineHemisphere already covers the codebase's other uniform/cosine sampling needs.
glm::vec3 sampleUniformHemisphereEon(glm::vec2 u) {
    const float sinTheta = std::sqrt(std::max(0.0F, 1.0F - (u.x * u.x)));
    const float phi = 2.0F * kPi * u.y;
    return {sinTheta * std::cos(phi), sinTheta * std::sin(phi), u.x};
}

struct EonLtcCoeffs {
    float a;
    float b;
    float c;
    float d;
};

// Fitted Linearly-Transformed-Cosine matrix coefficients (paper Listing 2) that best match EON's
// cosine-weighted backscattering lobe for a given view angle/roughness -- the shape CLTC sampling below
// imports from.
EonLtcCoeffs eonLtcCoeffs(float mu, float r) {
    const float a = 1.0F + (r * (0.303392F + (((-0.518982F + (0.111709F * mu)) * mu) +
                                                ((-0.276266F + (0.335918F * mu)) * r))));
    const float b = (r * (-1.16407F + (1.15859F * mu) + ((0.150815F - (0.150105F * mu)) * r))) /
                    ((mu * mu * mu) - 1.43545F);
    const float c = 1.0F + (r * (0.20013F + ((-0.506373F + (0.261777F * mu)) * mu)));
    const float d = (r * (0.540852F + ((-1.01625F + (0.475392F * mu)) * mu))) /
                    (-1.0743F + ((0.0725628F + mu) * mu));
    return {a, b, c, d};
}

// Orthonormal frame aligning wLocal's azimuth to the x-axis, used to move into/out of the space the LTC
// fit (above) is expressed in.
glm::mat3 orthonormalBasisLtc(const glm::vec3& wLocal) {
    const float lenSq = (wLocal.x * wLocal.x) + (wLocal.y * wLocal.y);
    const glm::vec3 x = lenSq > 0.0F ? glm::vec3(wLocal.x, wLocal.y, 0.0F) * (1.0F / std::sqrt(lenSq))
                                       : glm::vec3(1.0F, 0.0F, 0.0F);
    const glm::vec3 y(-x.y, x.x, 0.0F);
    return glm::mat3(x, y, glm::vec3(0.0F, 0.0F, 1.0F));
}

// Clipped-LTC direction sample (paper Sec. 4, Listing 3): cosine-weighted sampling of the hemisphere
// clipped to the LTC lobe's positive half-space (the Nusselt-analog half-circle/half-ellipse
// projection), restricted to the positive hemisphere by construction -- no rejected below-surface
// samples, unlike naive LTC sampling.
glm::vec3 cltcSample(const glm::vec3& woLocal, float r, glm::vec2 u) {
    const EonLtcCoeffs m = eonLtcCoeffs(woLocal.z, r);
    const float radius = std::sqrt(u.x);
    const float phi = 2.0F * kPi * u.y;
    const float y = radius * std::sin(phi);
    const float vz = 1.0F / std::sqrt((m.d * m.d) + 1.0F);
    const float s = 0.5F * (1.0F + vz);
    const float x = -lerp1(std::sqrt(std::max(0.0F, 1.0F - (y * y))), radius * std::cos(phi), s);
    const glm::vec3 wh(x, y, std::sqrt(std::max(0.0F, 1.0F - (x * x) - (y * y))));
    const glm::vec3 wiUnnormalized((m.a * wh.x) + (m.b * wh.z), m.c * wh.y, (m.d * wh.x) + wh.z);
    return glm::normalize(orthonormalBasisLtc(woLocal) * wiUnnormalized);
}

// pdf of cltcSample's distribution at an arbitrary wiLocal (paper Listing 3's cltc_pdf) -- evaluated
// independently of how wiLocal was actually obtained, matching this file's existing convention of
// recomputing a lobe's pdf from evaluateBsdfSplit rather than threading it out of the sampler.
float cltcPdf(const glm::vec3& woLocal, const glm::vec3& wiLocal, float r) {
    const EonLtcCoeffs m = eonLtcCoeffs(woLocal.z, r);
    const glm::vec3 wi = glm::transpose(orthonormalBasisLtc(woLocal)) * wiLocal;
    const glm::vec3 wh(m.c * (wi.x - (m.b * wi.z)), (m.a - (m.b * m.d)) * wi.y,
                        -m.c * ((m.d * wi.x) - (m.a * wi.z)));
    const float lenSq = glm::dot(wh, wh);
    const float detM = m.c * (m.a - (m.b * m.d));
    const float vz = 1.0F / std::sqrt((m.d * m.d) + 1.0F);
    const float s = 0.5F * (1.0F + vz);
    return (detM * detM) / std::max(lenSq * lenSq, 1e-12F) * std::max(wh.z, 0.0F) / (kPi * s);
}

// Mixing weight between the CLTC lobe and a defensive uniform-hemisphere lobe (paper Sec. 4, fitted by
// minimizing the CLTC estimator's maximum throughput weight): CLTC alone has a residual bias/variance
// spike the uniform term corrects via one-sample MIS. Shared by sampleEon and pdfEon so the two always
// agree on which mixture they are drawing from/evaluating.
float eonUniformMixWeight(float mu, float r) {
    const float inner = 0.538233F - (0.290822F * mu);
    const float mid = -0.372058F + (inner * mu);
    return std::pow(r, 0.1F) * (0.162925F + (mid * mu));
}

// Samples EON's importance-sampling distribution (paper Sec. 4): one-sample MIS between the CLTC lobe
// and a uniform hemisphere lobe. Direction only -- pdfEon below is the single source of truth for the
// resulting density, called via evaluateDiffuseLobe regardless of which strategy produced wi.
glm::vec3 sampleEon(const glm::vec3& woLocal, float r, glm::vec2 u) {
    const float pUniform = eonUniformMixWeight(woLocal.z, r);
    // Strict: pUniform is exactly 0 at r=0 (pow(0,0.1)), where an inclusive test admits u.x==0 -- reachable from Sampler's Cranley-Patterson wrap -- and reshuffles it as 0/0. The resulting NaN direction passes every downstream guard (NaN fails all ordered comparisons) and poisons the pixel for the rest of the progressive render.
    if (u.x < pUniform) {
        u.x /= pUniform;
        return sampleUniformHemisphereEon(u);
    }
    u.x = (u.x - pUniform) / (1.0F - pUniform);
    return cltcSample(woLocal, r, u);
}

// pdf of sampleEon's distribution at an arbitrary wiLocal.
float pdfEon(const glm::vec3& woLocal, const glm::vec3& wiLocal, float r) {
    const float pUniform = eonUniformMixWeight(woLocal.z, r);
    constexpr float kUniformHemispherePdf = 1.0F / (2.0F * kPi);
    return (pUniform * kUniformHemispherePdf) + ((1.0F - pUniform) * cltcPdf(woLocal, wiLocal, r));
}

LobeEval evaluateDiffuseLobe(const BsdfParams& params, const glm::vec3& wo, const glm::vec3& wi,
                              const LobeProbabilities& lobes) {
    if (wi.z <= 0.0F || wo.z <= 0.0F) {
        return {glm::vec3(0.0F), 0.0F};
    }
    const glm::vec3 f = evaluateEon(params.diffuseRho, params.diffuseRoughness, wi, wo);
    return {f * diffuseKdAt(params, wi, lobes), pdfEon(wo, wi, params.diffuseRoughness)};
}

// Single scatter D*G2*F/(4*ndotV*ndotL) plus the Kulla-Conty multiple-scattering lobe, and the VNDF pdf (Heitz 2018 eq.3, Jacobian 1/(4*dot(wo,nh))).
// The pdf covers the single-scattering term only: the reflected multiple-scattering share has no sampling strategy of its own and is picked up by whichever of the two existing strategies draws that wi, which leaves the one-sample mixture estimator unbiased (the mixture density is still the true density of the sampling procedure).
// It needs none: unlike the transmitted share it has no geometric rejection to escape, so this hemisphere delivers all of it.
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
    // Gated, not mixed away at weight 0: glm::mix is a + t*(b-a), so a non-finite conductor term would
    // survive t=0 as NaN rather than cancel. Skipping the call keeps every dielectric bit-identical and
    // costs it nothing -- the same reasoning as the transmitWeight gate below.
    const glm::vec3 f =
        params.metallic > 0.0F
            ? glm::mix(glm::vec3(fDielectric),
                        fresnelConductor(woDotNh, lobes.conductorN, lobes.conductorK), params.metallic)
            : glm::vec3(fDielectric);
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
    // transmitWeight is exactly zero for every opaque material (transmissionFactor=0, or metallic=1
    // regardless of transmissionFactor) -- the common case. Skip multiScatterShape's own escape-table
    // lookup entirely rather than compute it and glm::mix it away at weight 0.
    const glm::vec3 multiScatter =
        lobes.transmitWeight > 0.0F
            ? glm::mix(opaqueMs,
                        glm::vec3((1.0F - lobes.transmitShare) *
                                  multiScatterShape(params, wi.z, lobes.escapeWo, lobes.etaI / lobes.etaT,
                                                     1.0F - lobes.escapeAvg)),
                        lobes.transmitWeight)
            : opaqueMs;

    const float g1 = smithG1(wo.z, alpha);
    const float pdf = (g1 * woDotNh * d) / std::max(wo.z, 1e-6F) / (4.0F * woDotNh);
    return {singleScatter + multiScatter, pdf};
}

// specular = Fresnel reflectance probability (exact dielectric via ior, Schlick via f0 for conductors, blended by metallic).
// "Exiting" (transmissive material, sign<0, already inside): no diffuse substrate, transmit takes everything specular didn't; reflect internally or exit, no third option.
// Everything else (entering, sign>0, or an opaque material's woLocal.z pushed negative by grazing-angle normal mapping) uses the entering split: diffuse/transmit divide the remainder by transmissionFactor (0 for opaque, so transmit vanishes and this reduces to diffuse+specular regardless of which side of the interpolated normal wo landed on).
// transmitPhysicalValue != transmit: throughput = physicalValue/transmit, so physicalValue must independently carry the same transmissionFactor/metallic factors transmit's probability used, or they cancel out of the throughput and silently erase their effect on energy (caught by tools/bsdf_validate.cpp's furnace test).
LobeProbabilities computeLobeProbabilities(const BsdfParams& params, const glm::vec3& wo, float sign,
                                            float alpha) {
    const bool exiting = sign < 0.0F && params.transmissionFactor > 0.0F;
    const float etaI = exiting ? params.ior : 1.0F;
    const float etaT = exiting ? 1.0F : params.ior;
    const float fresnelAtNormal = fresnelDielectric(wo.z, etaI, etaT);
    // Same metallic>0 gate as evaluateSpecularLobe's, for the same reason; see LobeProbabilities.
    ConductorIor conductor{glm::vec3(1.0F), glm::vec3(0.0F)};
    float conductorLuma = 0.0F;
    glm::vec3 conductorAvg(0.0F);
    if (params.metallic > 0.0F) {
        conductor = conductorIorFromReflectivity(params.f0, params.edgeTint);
        const glm::vec3 conductorF = fresnelConductor(wo.z, conductor.n, conductor.k);
        conductorLuma = (conductorF.x + conductorF.y + conductorF.z) / 3.0F;
        conductorAvg = conductorFresnelAvg(conductor.n, conductor.k);
    }
    const AlbedoSplit splitWo = directionalAlbedo(wo.z, params.roughness);
    const AlbedoSplit splitAvg = averageAlbedo(params.roughness);
    // Scaled by E: the specular lobe now has two parts, and only the single-scattering part is drawn by VNDF sampling. The multiple-scattering part is cosine-shaped and picked up by the diffuse strategy, so its selection mass must move there too; otherwise a rough white metal, whose Fresnel pins specularProb to the 0.95 clamp, would sample 69% of its own reflectance only 5% of the time.
    const float specularProb = std::clamp(
        glm::mix(fresnelAtNormal, conductorLuma, params.metallic) * splitWo.total(), 0.05F, 0.95F);
    const float transmittance = (1.0F - fresnelAtNormal) * (1.0F - params.metallic);
    // Reciprocal diffuse coupling. The substrate receives what the coat did not reflect, on the way in and on the way out: evaluateDiffuseLobe applies the matching wi-side factor, and the pair is renormalised by 1/(1-coatAlbedoAvg) so the directional albedo integrates back to (1-coatAlbedo(mu_o)).
    // Not exact: coatAlbedoAvg evaluates coatAlbedo at the averaged split rather than averaging coatAlbedo over wi, and the two differ because the Fresnel rescale varies with direction; the white furnace test bounds the residual to under 1%.
    // Symmetric in wo/wi, which the previous bare (1-F(mu_o)) form was not, and energy-complete, which it also was not.
    const float coatF0 = dielectricF0(params.ior);
    const float dielectricAvg = dielectricFresnelAvg(params.ior);
    const float coatAlbedoAvg =
        coatAlbedo(splitAvg, splitAvg.total(), coatF0,
                    dielectricAvg / std::max(schlickFresnelAvg(glm::vec3(coatF0)).x, 1e-6F),
                    dielectricAvg);
    const float diffuseCoupling = (1.0F - coatAlbedo(splitWo, splitAvg.total(), coatF0,
                                                       coatFresnelRatio(wo.z, etaI, etaT, coatF0),
                                                       dielectricAvg)) /
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
    // Each interface's own cosine mean, matching the Fresnel its single scatter evaluates: the quadrature rule for the conductor's complex IOR, the standard rational fit for the dielectric. conductorAvg is 0 off the metal path, where glm::mix at t=0 returns the dielectric term exactly.
    const glm::vec3 fresnelAvg = glm::mix(glm::vec3(dielectricAvg), conductorAvg, params.metallic);
    // R_ss uses the same Schlick-split-with-exact-Fresnel-rescale as the coat; T_ss is the (1-fc) channel scaled by (1-f0), which is Schlick's 1-F factored exactly.
    // transmitWeight, not transmissionFactor: the transmission lobe's own energy is gated by (1-metallic) too (see transmittance above), so a metallic=1 material transmits nothing however its transmissionFactor is set. Using the raw factor here credited the escape budget with transmission that never happens, and the compensation handed the difference back as multiple scattering, measured as Lo=1.43 on a metallic=1, transmission=1 surface.
    // effectiveTransmission: inside the medium there is no diffuse substrate to withhold anything (a ray must reflect internally or exit), so transmissionFactor gates the entering side only. transmitProb and transmitPhysicalValue above already did this; transmitWeight did not, leaving the exiting side's value and its escape budget disagreeing at 0 < transmissionFactor < 1.
    const float eta = etaI / etaT;
    const float effectiveTransmission = exiting ? 1.0F : params.transmissionFactor;
    const float transmitWeight = effectiveTransmission * (1.0F - params.metallic);
    // R + T, with no transmissionFactor weighting: energy the interface refracts but transmissionFactor withholds from the transmit lobe enters the diffuse substrate instead (that is what diffuseKd's (1-transmissionFactor) does), and escapes from there. Either way it leaves, so the microfacet deficit the compensation must return is the same. Weighting this term by transmitWeight instead over-reported the deficit at partial transmission, measured Lo=2.64 against a bound of 2.25.
    // Skipped entirely when transmissionFactor==0 (which also covers "not exiting", since exiting
    // requires transmissionFactor>0 by definition above): escapeAlbedo/averageEscapeAlbedo are trilinear
    // lookups over a log-spaced eta axis, and every value they'd feed here (transmitShare, msFraction)
    // only matters to the transmissive multi-scatter path, which evaluateSpecularLobe's transmitWeight
    // gate below already skips for every opaque material -- paying for the lookup anyway would be pure
    // waste on the common diffuse+specular case.
    float escape = 0.0F;
    float escapeAvg = 0.0F;
    float escapeAvgRecip = 0.0F;
    float transmitShare = 0.0F;
    float msFraction = 0.0F;
    if (params.transmissionFactor > 0.0F) {
        const EscapeSplit escapeWo = escapeAlbedo(wo.z, params.roughness, eta);
        const EscapeSplit escapeMean = averageEscapeAlbedo(params.roughness, eta);
        escape = escapeWo.total();
        escapeAvg = escapeMean.total();
        // The far medium's own average escape, at the reciprocal eta -- see multiScatterShape's doc comment.
        escapeAvgRecip = averageEscapeAlbedo(params.roughness, 1.0F / eta).total();
        const float transmitSsAvg = transmitWeight * escapeMean.transmit;
        transmitShare = transmitSsAvg / std::max(escapeAvg, 1e-4F);

        // Split of the transmit selection mass between the two far-hemisphere strategies, proportional to the energy each carries (transmitWeight cancels from both sides). Gated on the same kMinDeficit test multiScatterShape switches off at, so no mass reaches a lobe of identically zero value, also what leaves the smooth-glass rows bit-identical.
        if (transmissionIsRough(params, alpha) && (1.0F - escapeAvg) > kMinDeficit) {
            const float msEnergy = transmitShare * std::max(1.0F - escape, 0.0F);
            const float ssEnergy = escapeWo.transmit;
            if (msEnergy + ssEnergy > 1e-6F) {
                // Capped so the peaked single-scatter lobe always keeps a quarter of the mass.
                msFraction = std::clamp(msEnergy / (msEnergy + ssEnergy), 0.0F, 0.75F);
            }
        }
    }
    const float msTransmitProb = transmitProb * msFraction;

    return {.specular = specularProb,
             .diffuse = diffuseProb,
             .transmit = transmitProb - msTransmitProb,
             .msTransmit = msTransmitProb,
             .etaI = etaI,
             .etaT = etaT,
             .diffuseKd = diffuseKd,
             .transmitPhysicalValue = transmitPhysicalValue,
             .albedoWo = splitWo.total(),
             .albedoAvg = splitAvg.total(),
             .coatF0 = coatF0,
             .fresnelAvg = fresnelAvg,
             .conductorN = conductor.n,
             .conductorK = conductor.k,
             .escapeWo = escape,
             .escapeAvg = escapeAvg,
             .escapeAvgRecip = escapeAvgRecip,
             .transmitShare = transmitShare,
             .etaSq = eta * eta,
             .transmitWeight = transmitWeight};
}

// Walter et al. 2007 rough transmission, single scatter only (value eq. 21, half-vector eq. 16, Jacobian eq. 17), in PBRT-v3's radiance-transport form.
// The non-symmetric eta^2 radiance compression (Veach 1997 sec. 5.2) is already folded in here: PBRT's factor=1/eta, squared, cancels the explicit eta^2 in Walter's importance-mode value, so unlike the delta branch in sampleBsdf this must not multiply by eta^2 again.
// The etaR^2 that survives in the pdf but not the value is the asymmetry, and the transmission round-trip test is what catches a double application.
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
    // transmitWeight, the same factors the delta branch carries via transmitPhysicalValue: (1-metallic), since a conductor transmits nothing however its transmissionFactor is set, and the entering side's transmissionFactor, without which this refracted at full strength on top of a diffuse substrate already scaled by (1-transmissionFactor).
    return {params.baseColor * (1.0F - fresnel) * lobes.transmitWeight * common,
             vndfPdf * etaR * etaR * std::abs(wiDotH) / denom2};
}

BsdfEval evaluateContinuousLobes(const BsdfParams& params, const glm::vec3& wo, const glm::vec3& wi,
                                  float alpha, const LobeProbabilities& lobes) {
    // Reflection and transmission occupy disjoint hemispheres, so the mixture is piecewise: no overlap between the two to double-count.
    if (wi.z < 0.0F) {
        // The multiple-scattering term stays inside this gate. A delta interface keeps pdf=0 on the far side, and a non-zero value there would be silently discarded by path_tracer.cpp's bsdfPdf>0 guard: energy lost rather than gained.
        if (!transmissionIsRough(params, alpha)) {
            return {};
        }
        const LobeEval transmission = evaluateTransmissionLobe(params, wo, wi, alpha, lobes);
        return {glm::vec3(0.0F), glm::vec3(0.0F),
                transmission.f + transmitMultiScatter(params, wi.z, lobes),
                (lobes.transmit * transmission.pdf) + (lobes.msTransmit * -wi.z / kPi)};
    }
    const LobeEval specular = evaluateSpecularLobe(params, wo, wi, alpha, lobes);
    const LobeEval diffuse = evaluateDiffuseLobe(params, wo, wi, lobes);
    return {diffuse.f, specular.f, glm::vec3(0.0F),
            (lobes.specular * specular.pdf) + (lobes.diffuse * diffuse.pdf)};
}

}  // namespace

glm::vec3 fresnelAtViewAngle(const BsdfParams& params, float cosTheta) {
    // Entering orientation (etaI=1): a primary-hit view-angle value is always outside the surface, so
    // unlike evaluateSpecularLobe there is no exiting side to swap etaI/etaT for.
    const float fDielectric = fresnelDielectric(cosTheta, 1.0F, params.ior);
    if (params.metallic <= 0.0F) {
        return glm::vec3(fDielectric);
    }
    const ConductorIor conductor = conductorIorFromReflectivity(params.f0, params.edgeTint);
    return glm::mix(glm::vec3(fDielectric), fresnelConductor(cosTheta, conductor.n, conductor.k),
                     params.metallic);
}

BsdfEval evaluateBsdfSplit(const BsdfParams& params, const glm::vec3& woLocal,
                            const glm::vec3& wiLocal) {
    const float sign = woLocal.z >= 0.0F ? 1.0F : -1.0F;
    const glm::vec3 wo(woLocal.x, woLocal.y, woLocal.z * sign);
    const glm::vec3 wi(wiLocal.x, wiLocal.y, wiLocal.z * sign);
    const float alpha = std::max(params.roughness * params.roughness, kMinAlpha);
    const LobeProbabilities lobes = computeLobeProbabilities(params, wo, sign, alpha);
    return evaluateContinuousLobes(params, wo, wi, alpha, lobes);
}

float pdfBsdf(const BsdfParams& params, const glm::vec3& woLocal, const glm::vec3& wiLocal) {
    return evaluateBsdfSplit(params, woLocal, wiLocal).pdf;
}

glm::vec3 evaluateBsdf(const BsdfParams& params, const glm::vec3& woLocal, const glm::vec3& wiLocal) {
    return evaluateBsdfSplit(params, woLocal, wiLocal).total();
}

std::optional<BsdfSample> sampleBsdf(const BsdfParams& params, const glm::vec3& woLocal,
                                      Sampler& sampler) {
    const float sign = woLocal.z >= 0.0F ? 1.0F : -1.0F;
    const glm::vec3 wo(woLocal.x, woLocal.y, woLocal.z * sign);
    const float alpha = std::max(params.roughness * params.roughness, kMinAlpha);
    const LobeProbabilities lobes = computeLobeProbabilities(params, wo, sign, alpha);

    const float lobeU = sampler.next1D();

    if (lobeU < lobes.specular + lobes.diffuse) {
        const bool sampledSpecular = lobeU < lobes.specular;
        glm::vec3 wi;
        if (sampledSpecular) {
            const glm::vec3 nh = sampleGGXVNDF(wo, alpha, sampler.next2D());
            wi = glm::reflect(-wo, nh);
        } else {
            wi = sampleEon(wo, params.diffuseRoughness, sampler.next2D());
        }
        if (wi.z <= 0.0F) {
            return std::nullopt;
        }
        const BsdfEval eval = evaluateContinuousLobes(params, wo, wi, alpha, lobes);
        if (eval.pdf <= 1e-8F) {
            return std::nullopt;
        }
        const glm::vec3 throughput = (eval.total() * wi.z) / eval.pdf;
        return BsdfSample{glm::vec3(wi.x, wi.y, wi.z * sign), throughput,
                           sampledSpecular ? LobeType::SpecularReflection : LobeType::Diffuse,
                           eval.pdf};
    }

    // Top slice of the ladder: the multiple-scattering transmission lobe, cosine over the far hemisphere. It needs a strategy of its own because the refraction VNDF below reaches only directions some microfacet can refract into, while this lobe spans the whole hemisphere.
    // msTransmit tested first, not inside: the three probabilities below it sum to 1.0 only to float precision, so with no mass here a top-of-range lobeU must fall through to the transmit lobe it always belonged to rather than be rejected.
    if (lobes.msTransmit > 0.0F && lobeU >= lobes.specular + lobes.diffuse + lobes.transmit) {
        glm::vec3 wi = sampleCosineHemisphere(sampler.next2D());
        wi.z = -wi.z;
        const BsdfEval eval = evaluateContinuousLobes(params, wo, wi, alpha, lobes);
        if (eval.pdf <= 1e-8F) {
            return std::nullopt;
        }
        const glm::vec3 throughput = (eval.total() * -wi.z) / eval.pdf;
        return BsdfSample{glm::vec3(wi.x, wi.y, wi.z * sign), throughput, LobeType::Transmission,
                           eval.pdf};
    }

    if (lobes.transmit <= 0.0F) {
        return std::nullopt;
    }

    // Rough transmission (Walter 2007): refract about a VNDF-sampled microfacet normal rather than the macro normal. A microfacet steep enough to totally internally reflect yields no transmission sample.
    if (transmissionIsRough(params, alpha)) {
        const glm::vec3 ht = sampleGGXVNDF(wo, alpha, sampler.next2D());
        glm::vec3 wi;
        if (!refractAbout(wo, ht, lobes.etaI / lobes.etaT, wi) || wi.z >= 0.0F) {
            return std::nullopt;
        }
        const BsdfEval eval = evaluateContinuousLobes(params, wo, wi, alpha, lobes);
        if (eval.pdf <= 1e-8F) {
            return std::nullopt;
        }
        const glm::vec3 throughput = (eval.total() * -wi.z) / eval.pdf;
        return BsdfSample{glm::vec3(wi.x, wi.y, wi.z * sign), throughput, LobeType::Transmission,
                           eval.pdf};
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
    // pdf 0: a delta lobe has no density for NEE to double-count against, which is exactly the test path_tracer.cpp's MIS weighting makes.
    return BsdfSample{glm::vec3(wt.x, wt.y, wt.z * sign), throughput, LobeType::Transmission, 0.0F};
}

}  // namespace engine::scene
