#include "engine/scene/bsdf.h"
#include "engine/scene/fresnel_dielectric.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace engine::scene {

namespace {

constexpr float kPi = 3.14159265F;
constexpr float kMinAlpha = 0.02F * 0.02F;  // roughness floor, avoids a degenerate GGX delta lobe

// Trowbridge-Reitz/GGX D in the cancellation-free form (Filament 4.4.2, Google 2018): the textbook denominator ndotH^2*(alpha^2-1)+1 subtracts two near-equal numbers wherever the half-vector is near the normal, which at low roughness is the entire lobe -- measured 20% of D lost at the peak at alpha=4e-4, and 3.3e-4 at alpha=1e-2.
// nh is in the local shading frame (N = +z), so nh.x^2 + nh.y^2 IS sin^2(theta_h): every term of the sum is then non-negative and the peak value 1/(pi*alpha^2) is exact.
// The denominator needs no floor: d = alpha^2*cos^2 + sin^2 is minimised at alpha^2, and callers enforce alpha >= kMinAlpha, so kPi*d*d >= 8e-14 -- twenty-four orders above float32 underflow. The floor that used to stand here engaged for every roughness below 0.0867 and suppressed D by 124340x at 0.02 and 81.5x at 0.05.
float distributionGGX(const glm::vec3& nh, float alpha) {
    const float alpha2 = alpha * alpha;
    const float d = (alpha2 * nh.z * nh.z) + (nh.x * nh.x) + (nh.y * nh.y);
    return alpha2 / (kPi * d * d);
}

// cos*sqrt(1 + alpha^2*tan^2), the GGX Smith Lambda's radical scaled by the cosine: 1 + Lambda(c) = (c + radical)/(2c), and unlike tan it is finite at c = 0.
float smithRadical(float cosTheta, float alpha) {
    const float alpha2 = alpha * alpha;
    return std::sqrt(alpha2 + ((1.0F - alpha2) * cosTheta * cosTheta));
}

// G2/(4*cosO*cosI) for the height-correlated Smith G2 (Heitz 2014; Filament's V_SmithGGXCorrelated): the cosines multiply rather than divide, so it is exact to the silhouette and vanishes only where both cosines do.
float smithVisibility(float cosO, float cosI, float alpha) {
    return 0.5F / ((cosI * smithRadical(cosO, alpha)) + (cosO * smithRadical(cosI, alpha)));
}

// G1(c)/c, the VNDF pdf's projected-area factor, in the same division-free form: 2/alpha at grazing rather than 0/0.
float smithG1OverCos(float cosTheta, float alpha) { return 2.0F / (cosTheta + smithRadical(cosTheta, alpha)); }

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

// Kulla-Conty energy tables, baked offline by tools/albedo_table.cpp (Kulla & Conty 2017, "Revisiting Physically Based Shading at Imageworks"). The .inc defines kAlbedoRes/kTransmitRoughnessRes/kTransmitMuRes/kEtaRes/kEtaMin/kEtaMax alongside the arrays, so the grid the lookups below index is the grid the generator wrote and the two cannot drift apart.
// kAlbedoA/kAlbedoB are the directional albedo of the single-scattering GGX lobe with Fresnel forced to 1, the fraction of energy the height-correlated Smith G2 lets through, so 1-E is exactly what multiple scattering must return. Split by Schlick's form F(c) = f0*(1 - (1-c)^5) + (1-c)^5 so one table serves any f0 (the standard environment-BRDF split): Ess(mu, f0) = f0*a + b, and with f0=1 that collapses to a + b = E, the Fresnel-free albedo the multiple-scattering lobe needs.
// kEscapeReflect/kEscapeTransmit are the escaping fraction of a dielectric interface, split into the reflected and transmitted shares and indexed [roughnessIndex][muIndex][etaIndex], mu uniform in sqrt(mu). They use exact dielectric Fresnel rather than the Schlick split: inside the total-internal-reflection cone exact Fresnel is 1.0 while Schlick reads ~0.1, so no rescale of a Schlick-basis number can stand in for it, and the escape budget would under-count the reflected share by the whole TIR cone. Their third axis is why they keep their own grid, kTransmitRoughnessRes x kTransmitMuRes x kEtaRes.
// Building this at startup is what used to bound its accuracy: the grid and the quadrature were sized by load latency, not by what the energy tests need. See the generator for the rule and its measured residual.
#include "albedo_table.inc"

// Refract wo about microfacet normal ht. Returns false on total internal reflection at that facet.
bool refractAbout(const glm::vec3& wo, const glm::vec3& ht, float eta, glm::vec3& wi) {
    const float cosI = glm::dot(wo, ht);
    if (cosI <= 0.0F) {
        return false;
    }
    const float cos2T = cos2Transmitted(cosI, eta);
    if (cos2T < 0.0F) {
        return false;
    }
    wi = ((eta * cosI) - std::sqrt(cos2T)) * ht - (eta * wo);
    return true;
}

float lerp1(float a, float b, float t) { return a + ((b - a) * t); }

// Schlick-split albedo pair: Ess(f0) = f0*a + b, and a+b = E (the f0=1 case).
struct AlbedoSplit {
    float a;
    float b;
    [[nodiscard]] float total() const { return a + b; }                 // E, Fresnel-free
    [[nodiscard]] float at(float f0) const { return (f0 * a) + b; }
};

// Bilinear lookup. E climbs steeply as mu->0 (0.31 at mu=1, 0.99 at the first grid column, alpha=1), so the first mu bin carries the largest interpolation error -- measured 7.1e-3 there against 3.2e-5 for mu >= 0.3 -- harmless since every integral consuming E weights grazing by cos(theta).
AlbedoSplit directionalAlbedo(float mu, float roughness) {
    const float rf = std::clamp(roughness, 0.0F, 1.0F) * (kAlbedoRes - 1);
    const float mf = std::clamp(mu, 0.0F, 1.0F) * (kAlbedoRes - 1);
    const int r0 = std::min(static_cast<int>(rf), kAlbedoRes - 2);
    const int m0 = std::min(static_cast<int>(mf), kAlbedoRes - 2);
    const float rt = rf - static_cast<float>(r0);
    const float mt = mf - static_cast<float>(m0);
    const int i0 = (r0 * kAlbedoRes) + m0;
    const int i1 = ((r0 + 1) * kAlbedoRes) + m0;
    return {lerp1(lerp1(kAlbedoA[i0], kAlbedoA[i0 + 1], mt),
                   lerp1(kAlbedoA[i1], kAlbedoA[i1 + 1], mt), rt),
             lerp1(lerp1(kAlbedoB[i0], kAlbedoB[i0 + 1], mt),
                   lerp1(kAlbedoB[i1], kAlbedoB[i1 + 1], mt), rt)};
}

AlbedoSplit averageAlbedo(float roughness) {
    const float rf = std::clamp(roughness, 0.0F, 1.0F) * (kAlbedoRes - 1);
    const int r0 = std::min(static_cast<int>(rf), kAlbedoRes - 2);
    const float rt = rf - static_cast<float>(r0);
    return {lerp1(kAlbedoAvgA[r0], kAlbedoAvgA[r0 + 1], rt),
             lerp1(kAlbedoAvgB[r0], kAlbedoAvgB[r0 + 1], rt)};
}

// --- Sampling shape for the reflected multiple-scattering lobe, from kMsReflectDensity/kMsReflectCdf.
// The lobe's value is fms*(1-E(mu_o))*(1-E(mu_i))/(pi*(1-Eavg)), so the density that makes f*cos/pdf independent of wi is (1-E(mu_i))*cos/(pi*(1-Eavg)) -- the table holds exactly that shape as a piecewise-linear density over mu, normalised, with its exact prefix integrals.
// Cosine sampling, which this replaces, pays the ratio (1-E(mu_i))/(1-Eavg) as weight variance: measured relative variance 0.029 at roughness 1 but +1.66 at 0.25 and +17.3 at 0.126, where 1-Eavg and 1-E(mu) are both small and their quotient is not, with weights reaching 92x. The win is at LOW roughness, not high.
// Row blend is the albedo lookups' own, so the density the sampler inverts and the density the pdf evaluates are the same function of roughness, which is what keeps the estimator unbiased rather than merely close.
struct MsReflectRow {
    int base;
    float blend;
};

MsReflectRow msReflectRow(float roughness) {
    const float rf = std::clamp(roughness, 0.0F, 1.0F) * (kAlbedoRes - 1);
    const int r0 = std::min(static_cast<int>(rf), kAlbedoRes - 2);
    return {r0 * kAlbedoRes, rf - static_cast<float>(r0)};
}

float msReflectDensity(const MsReflectRow& row, int index) {
    return lerp1(kMsReflectDensity[row.base + index], kMsReflectDensity[row.base + kAlbedoRes + index],
                  row.blend);
}

float msReflectCdf(const MsReflectRow& row, int index) {
    return lerp1(kMsReflectCdf[row.base + index], kMsReflectCdf[row.base + kAlbedoRes + index], row.blend);
}

// Solid-angle density: the mu density spread over 2*pi of azimuth. Reduces to the cosine form mu/pi wherever the stored shape is 2*mu, which is what it becomes as the energy deficit goes flat.
float msReflectPdf(float mu, float roughness) {
    const MsReflectRow row = msReflectRow(roughness);
    const float mf = std::clamp(mu, 0.0F, 1.0F) * (kAlbedoRes - 1);
    const int m0 = std::min(static_cast<int>(mf), kAlbedoRes - 2);
    const float mt = mf - static_cast<float>(m0);
    return lerp1(msReflectDensity(row, m0), msReflectDensity(row, m0 + 1), mt) / (2.0F * kPi);
}

// Exact inversion of a tabulated piecewise-linear density over mu on an edge-aligned grid: binary search the prefix integrals for the segment, then take the positive root of its quadratic. Shared by both multiple-scattering lobes so the stable form below has exactly one implementation.
// Written as 2c/(q + sqrt(q^2 + 2*dq*c)) rather than the textbook (-q + sqrt(...))/dq, which is the algebraically identical form that stays finite as a segment flattens (dq -> 0, where it reduces to c/q) and at mu = 0, where q is exactly 0 and it reduces to sqrt(2c/dq).
// Both fetches must see the same normalised density, so a caller holding an unnormalised table scales both by its total rather than passing the raw entries: the 1e-9 floor below is a density-scale quantity, not a free epsilon.
template <typename Density, typename Cdf>
float invertPiecewiseLinearDensity(Density density, Cdf cdf, int resolution, float u) {
    int low = 0;
    int high = resolution - 1;
    while (high - low > 1) {
        const int mid = (low + high) / 2;
        (cdf(mid) <= u ? low : high) = mid;
    }
    const float step = 1.0F / static_cast<float>(resolution - 1);
    const float q0 = density(low);
    const float dq = density(low + 1) - q0;
    const float c = (u - cdf(low)) / step;
    const float root = std::sqrt(std::max((q0 * q0) + (2.0F * dq * c), 0.0F));
    const float t = std::clamp((2.0F * c) / std::max(q0 + root, 1e-9F), 0.0F, 1.0F);
    return (static_cast<float>(low) + t) * step;
}

glm::vec3 sampleMsReflect(float roughness, glm::vec2 u) {
    const MsReflectRow row = msReflectRow(roughness);
    const float mu = invertPiecewiseLinearDensity([&](int i) { return msReflectDensity(row, i); },
                                                   [&](int i) { return msReflectCdf(row, i); }, kAlbedoRes, u.x);
    const float r = std::sqrt(std::max(0.0F, 1.0F - (mu * mu)));
    const float phi = 2.0F * kPi * u.y;
    return {r * std::cos(phi), r * std::sin(phi), mu};
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
    const float rf = std::clamp(roughness, 0.0F, 1.0F) * (kTransmitRoughnessRes - 1);
    // The generator's escapeMu axis: nodes uniform in sqrt(mu), dense where E climbs from its grazing limit.
    const float mf = std::sqrt(std::clamp(mu, 0.0F, 1.0F)) * (kTransmitMuRes - 1);
    const float ef = etaAxisCoord(eta);
    const int r0 = std::min(static_cast<int>(rf), kTransmitRoughnessRes - 2);
    const int m0 = std::min(static_cast<int>(mf), kTransmitMuRes - 2);
    const int e0 = std::min(static_cast<int>(ef), kEtaRes - 2);
    const float rt = rf - static_cast<float>(r0);
    const float mt = mf - static_cast<float>(m0);
    const float et = ef - static_cast<float>(e0);
    const auto fetch = [&](const auto& channel, int r, int m) {
        const int base = (((r * kTransmitMuRes) + m) * kEtaRes) + e0;
        return lerp1(channel[base], channel[base + 1], et);
    };
    const auto bilinear = [&](const auto& channel) {
        return lerp1(lerp1(fetch(channel, r0, m0), fetch(channel, r0, m0 + 1), mt),
                      lerp1(fetch(channel, r0 + 1, m0), fetch(channel, r0 + 1, m0 + 1), mt), rt);
    };
    return {bilinear(kEscapeReflect), bilinear(kEscapeTransmit)};
}

EscapeSplit averageEscapeAlbedo(float roughness, float eta) {
    const float rf = std::clamp(roughness, 0.0F, 1.0F) * (kTransmitRoughnessRes - 1);
    const float ef = etaAxisCoord(eta);
    const int r0 = std::min(static_cast<int>(rf), kTransmitRoughnessRes - 2);
    const int e0 = std::min(static_cast<int>(ef), kEtaRes - 2);
    const float rt = rf - static_cast<float>(r0);
    const float et = ef - static_cast<float>(e0);
    const auto fetch = [&](const auto& channel, int r) {
        const int base = (r * kEtaRes) + e0;
        return lerp1(channel[base], channel[base + 1], et);
    };
    return {lerp1(fetch(kEscapeAvgReflect, r0), fetch(kEscapeAvgReflect, r0 + 1), rt),
             lerp1(fetch(kEscapeAvgTransmit, r0), fetch(kEscapeAvgTransmit, r0 + 1), rt)};
}

// --- Sampling shape for the transmitted multiple-scattering lobe, from kMsTransmitDensity/kMsTransmitCdf.
// The far-hemisphere twin of sampleMsReflect above, one axis wider because the escape it is built from is eta-dependent and the Schlick split that makes the reflect side Fresnel-free cannot factor that out.
// The density is (1-Escape(mu_i))*cos normalised by the row's own exact integral, and transmitMultiScatter's value is K*pdf/cos, so f*cos/pdf = K for every wi: value, density and sampler read one interpolant (Dupuy & Jakob 2018), and cosine sampling's (1-Escape(mu_i))/(1-EscapeAvg) weight variance (25 at roughness 0.13) is gone.
// One row per orientation: the reciprocal etaT/etaI for the transmitted share, whose wi has crossed into the far medium, and the forward etaI/etaT for the transmissive reflected share in evaluateSpecularLobe, whose wi has not.
struct MsTransmitRow {
    std::array<int, 4> base;
    std::array<float, 4> weight;
    float scale;
};

// Bilinear over (roughness, eta) of four rows, each entry a stride of kEtaRes apart along mu.
template <typename Table>
float msTransmitBlend(const Table& table, const MsTransmitRow& row, int index) {
    const int offset = index * kEtaRes;
    return (row.weight[0] * table[row.base[0] + offset]) + (row.weight[1] * table[row.base[1] + offset]) +
           (row.weight[2] * table[row.base[2] + offset]) + (row.weight[3] * table[row.base[3] + offset]);
}

// The table is stored unnormalised, so the blend is divided by its own blended total here rather than each row being normalised at bake time: integration is linear, so a blend of exact prefix integrals is the exact prefix integral of the blended density, and this is the interpolation of raw deficits escapeAlbedo itself performs -- blending four already-normalised rows would not commute with it, and their totals span seven orders across the roughness axis.
// It is also what makes a numerically dead row harmless: it contributes its own near-zero weight to the blend instead of a unit-mass shape of amplified noise, so no row needs a bake-time abort or a substituted fallback.
MsTransmitRow msTransmitRow(float roughness, float eta) {
    const float rf = std::clamp(roughness, 0.0F, 1.0F) * (kTransmitRoughnessRes - 1);
    const float ef = etaAxisCoord(eta);
    const int r0 = std::min(static_cast<int>(rf), kTransmitRoughnessRes - 2);
    const int e0 = std::min(static_cast<int>(ef), kEtaRes - 2);
    const float rt = rf - static_cast<float>(r0);
    const float et = ef - static_cast<float>(e0);
    const int base0 = (r0 * kTransmitMuRes * kEtaRes) + e0;
    const int base1 = base0 + (kTransmitMuRes * kEtaRes);
    MsTransmitRow row{{base0, base0 + 1, base1, base1 + 1},
                       {(1.0F - rt) * (1.0F - et), (1.0F - rt) * et, rt * (1.0F - et), rt * et},
                       0.0F};
    // Last prefix integral is the row's total energy deficit. Zero only if every clamped deficit in all four rows is zero, which leaves the lobe no energy to carry, so a zero scale correctly reports a zero density rather than dividing by it.
    const float total = msTransmitBlend(kMsTransmitCdf, row, kTransmitMuRes - 1);
    row.scale = total > 0.0F ? 1.0F / total : 0.0F;
    return row;
}

float msTransmitDensity(const MsTransmitRow& row, int index) {
    return msTransmitBlend(kMsTransmitDensity, row, index) * row.scale;
}

float msTransmitCdf(const MsTransmitRow& row, int index) {
    return msTransmitBlend(kMsTransmitCdf, row, index) * row.scale;
}

// Solid-angle density: the mu density spread over 2*pi of azimuth, mu measured from the far-side normal.
float msTransmitPdf(float mu, const MsTransmitRow& row) {
    const float mf = std::clamp(mu, 0.0F, 1.0F) * (kTransmitMuRes - 1);
    const int m0 = std::min(static_cast<int>(mf), kTransmitMuRes - 2);
    const float mt = mf - static_cast<float>(m0);
    return lerp1(msTransmitDensity(row, m0), msTransmitDensity(row, m0 + 1), mt) / (2.0F * kPi);
}

// Returns the near-hemisphere direction; the caller mirrors z, as the cosine draw it replaces did.
glm::vec3 sampleMsTransmit(const MsTransmitRow& row, glm::vec2 u) {
    const float mu = invertPiecewiseLinearDensity([&](int i) { return msTransmitDensity(row, i); },
                                                   [&](int i) { return msTransmitCdf(row, i); }, kTransmitMuRes, u.x);
    const float r = std::sqrt(std::max(0.0F, 1.0F - (mu * mu)));
    const float phi = 2.0F * kPi * u.y;
    return {r * std::cos(phi), r * std::sin(phi), mu};
}

}  // namespace

// Malley's method: a uniform point on the unit disk lifted to the hemisphere, which is exactly the cosine distribution (PBR 4th ed. 13.6.3). External linkage for path_tracer.cpp's AO lane, whose estimator is only the mean of visibility because this pdf cancels the cosine -- see the header.
glm::vec3 sampleCosineHemisphere(glm::vec2 u) {
    const float r = std::sqrt(u.x);
    const float phi = 2.0F * kPi * u.y;
    return {r * std::cos(phi), r * std::sin(phi), std::sqrt(std::max(0.0F, 1.0F - u.x))};
}

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

// ior == 1 is a delta at every roughness, not a rough interface: refractAbout returns -wo about every microfacet normal, so evaluateTransmissionLobe's half-vector normalize(wo + etaR*wi) normalises the zero vector and every guard below it is a NaN comparison -- measured NaN throughput on 7783 of 7783 transmission draws at roughness 0.1, 6666 of 7783 at roughness 1.0 (the remainder being the msTransmit draws, which do not form that half-vector). PBRT-v4's DielectricBxDF branches its value, pdf and sampler on the same `eta == 1 || EffectivelySmooth()`.
bool transmissionIsRough(const BsdfParams& params, float alpha) {
    return params.transmissionFactor > 0.0F && alpha >= kSmoothAlpha && params.ior != 1.0F;
}

struct LobeEval {
    glm::vec3 f;
    float pdf;
};

struct LobeProbabilities {
    float specular;
    float diffuse;
    float msReflect;    // multiple-scattering reflection, drawn from kMsReflectDensity over the near hemisphere
    float msReflectTransmissive;  // a transmissive interface's reflected multiple scattering, drawn from reflectShape
    float transmit;     // single-scatter refraction, VNDF-sampled about a microfacet normal
    float msTransmit;   // multiple-scattering transmission, drawn from kMsTransmitDensity over the far hemisphere
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
    MsTransmitRow transmitShape;  // escape-deficit shape at the reciprocal eta (etaT/etaI); scale 0 where no transmitted multiple scattering exists
    MsTransmitRow reflectShape;   // the same at the forward eta (etaI/etaT), for the reflected share whose wi stays in wo's medium
    float transmitShare;    // of the multiple-scattered energy, the fraction leaving refracted
    float etaSq;            // (etaI/etaT)^2, the radiance compression the transmit lobe must carry
    // effectiveTransmission*(1-metallic): how much transmission actually happens. Scales both the single-scatter and the multiple-scattering transmit value; the delta branch carries the same factors through transmitPhysicalValue.
    float transmitWeight;
    // Refraction's value per unit (1-F) in the VNDF strategy's per-facet reflect/refract split, transmitWeight; 0 where that strategy only reflects.
    float facetTransmit;
};

// kd carries the wo-side (1-F)/(1-Favg) coupling; the matching wi-side (1-F) factor is applied here, so the lobe is reciprocal (A4) while its directional albedo still integrates to (1-F(mu_o)), same total energy as the old one-sided form, correctly distributed.
// pdf must not be gated on kd: sampleBsdf selects this lobe with probability lobes.diffuse, which computeLobeProbabilities derives deterministically from params and wo, so the pdf side of the MIS mixture must match that selection density whatever value the lobe carries. Selection mass may depend on kd, but only by moving to another strategy of the same mixture (lobes.msReflect); deleting it starves the mixture denominator and inflates throughput.
// The transmitted share of the multiple-scattering energy, for ANY wi on the far side, free of evaluateTransmissionLobe's half-vector rejections, which describe single scattering only; lobes.msTransmit's density over the whole far hemisphere is what lets it live there without biasing the estimator.
// K*pdf/mu with pdf = msTransmitPdf, the density lobes.msTransmit draws: integrates to exactly K = tint*transmitWeight*transmitShare*etaSq*(1-escapeWo) for any table noise, and is non-zero exactly where that strategy has density, so no gate is needed to keep a vanishing deficit finite.
// mu > 0 strictly: the only caller is evaluateContinuousLobes' wi.z < 0 branch.
glm::vec3 transmitMultiScatter(const BsdfParams& params, float mu, float msPdf, const LobeProbabilities& lobes) {
    return params.transmissionTint * lobes.transmitWeight * lobes.transmitShare * lobes.etaSq *
           (std::max(1.0F - lobes.escapeWo, 0.0F) * msPdf / mu);
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

// Channel mean of the Fresnel evaluateSpecularLobe applies at a facet, metallic blend included.
float facetReflectance(const BsdfParams& params, float cosTheta, float fDielectric, const LobeProbabilities& lobes) {
    if (params.metallic <= 0.0F) {
        return fDielectric;
    }
    const glm::vec3 conductor = fresnelConductor(cosTheta, lobes.conductorN, lobes.conductorK);
    return glm::mix(fDielectric, (conductor.x + conductor.y + conductor.z) / 3.0F, params.metallic);
}

// Probability the VNDF strategy reflects about a facet on a rough transmissive interface: the facet's reflected value over its reflected plus refracted value (Walter et al. 2007 sec. 5.3; PBRT-v4 DielectricBxDF), so both branches weigh G2/G1 alone. Caller gates on facetTransmit > 0, where the denominator is positive.
float facetReflectProbability(const BsdfParams& params, float cosTheta, float fDielectric, const LobeProbabilities& lobes) {
    const float reflect = facetReflectance(params, cosTheta, fDielectric, lobes);
    return reflect / (reflect + ((1.0F - fDielectric) * lobes.facetTransmit));
}

// Single scatter D*G2*F/(4*ndotV*ndotL) plus the Kulla-Conty multiple-scattering lobe, and the VNDF pdf G1*D*dot(wo,nh)/ndotV (Heitz 2018 eq.3) times its reflection Jacobian 1/(4*dot(wo,nh)), which cancels dot(wo,nh).
// The pdf covers the single-scattering term only. The reflected multiple-scattering share is (1-E)cos-shaped and has a strategy of its own, lobes.msReflect, mirroring the transmitted share's lobes.msTransmit; evaluateContinuousLobes sums both densities into the mixture.
// Cosine was the standard practical choice (Kulla & Conty 2017); the zero-variance density here is (1-E(mu_i))cos/(pi*(1-Eavg)), and kMsReflectDensity holds exactly that shape, so lobes.msReflect draws it rather than paying the (1-E(mu_i))/(1-Eavg) weight ratio.
// A transmissive interface's reflected share has the forward-eta escape-deficit shape instead, drawn by lobes.msReflectTransmissive from the same row its value reads.
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
    const float d = distributionGGX(nh, alpha);
    const float fDielectric = fresnelDielectric(woDotNh, lobes.etaI, lobes.etaT);
    // Gated, not mixed away at weight 0: glm::mix is a + t*(b-a), so a non-finite conductor term would
    // survive t=0 as NaN rather than cancel. Skipping the call keeps every dielectric bit-identical and
    // costs it nothing -- the same reasoning as the transmitWeight gate below.
    const glm::vec3 f =
        params.metallic > 0.0F
            ? glm::mix(glm::vec3(fDielectric),
                        fresnelConductor(woDotNh, lobes.conductorN, lobes.conductorK), params.metallic)
            : glm::vec3(fDielectric);
    const glm::vec3 singleScatter = d * smithVisibility(wo.z, wi.z, alpha) * f;

    // Kulla & Conty 2017. Integrates to (1-E(mu_o)) at Favg=1 -- the energy G2 discarded -- so a
    // white conductor conserves, up to the table's own interpolation and quadrature error (measured
    // under 1% by the white furnace test). Symmetric in wo/wi, so it preserves reciprocity.
    // Reflected share of the multiple-scattering energy. The tint blends to 1 as the interface becomes
    // fully transmissive: a lossless dielectric returns all of it, tinted only by transmissionTint on the far
    // side, whereas a conductor's repeated bounces are attenuated by Favg each time.
    const glm::vec3 fms(multiScatterTint(lobes.fresnelAvg.x, lobes.albedoAvg),
                         multiScatterTint(lobes.fresnelAvg.y, lobes.albedoAvg),
                         multiScatterTint(lobes.fresnelAvg.z, lobes.albedoAvg));
    const float albedoWi = directionalAlbedo(wi.z, params.roughness).total();
    const glm::vec3 opaqueMs = fms * ((1.0F - lobes.albedoWo) * (1.0F - albedoWi)) /
                                (kPi * std::max(1.0F - lobes.albedoAvg, 1e-4F));
    // transmitWeight is exactly zero for every opaque material (transmissionFactor=0, or metallic=1 regardless of transmissionFactor) -- the common case. Skip the escape-shape row entirely rather than build it and glm::mix it away at weight 0.
    // The transmissive reflected share is transmitMultiScatter's near-side twin: (1-transmitShare)*(1-escapeWo)*pdf/mu over the forward-eta row (wi stays in wo's medium), integrating to exactly (1-transmitShare)*(1-escapeWo), so the two shares sum to the deficit with no normaliser of their own.
    const glm::vec3 multiScatter =
        lobes.transmitWeight > 0.0F
            ? glm::mix(opaqueMs,
                        glm::vec3((1.0F - lobes.transmitShare) * std::max(1.0F - lobes.escapeWo, 0.0F) *
                                  msTransmitPdf(wi.z, lobes.reflectShape) /
                                  wi.z),
                        lobes.transmitWeight)
            : opaqueMs;

    const float vndfPdf = 0.25F * d * smithG1OverCos(wo.z, alpha);
    return {singleScatter + multiScatter,
            lobes.facetTransmit > 0.0F ? vndfPdf * facetReflectProbability(params, woDotNh, fDielectric, lobes) : vndfPdf};
}

// Opaque: specular = Fresnel reflectance probability (exact dielectric via ior, Schlick via f0 for conductors, blended by metallic) times E, the rest diffuse and msReflect, whichever side of the interpolated normal wo landed on.
// Transmissive: every strategy by its energy at wo. "Exiting" (sign<0, already inside) has no diffuse substrate: reflect internally or exit, no third option.
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
    float diffuseKd = 0.0F;
    float transmitPhysicalValue = transmittance;
    if (!exiting) {
        diffuseKd = diffuseCoupling * (1.0F - params.metallic) * (1.0F - params.transmissionFactor);
        transmitPhysicalValue = transmittance * params.transmissionFactor;
    }
    // Each interface's own cosine mean, matching the Fresnel its single scatter evaluates: the quadrature rule for the conductor's complex IOR, the standard rational fit for the dielectric. conductorAvg is 0 off the metal path, where glm::mix at t=0 returns the dielectric term exactly.
    const glm::vec3 fresnelAvg = glm::mix(glm::vec3(dielectricAvg), conductorAvg, params.metallic);
    // msEnergy is exact -- evaluateSpecularLobe's opaqueMs integrates over the hemisphere to fms*(1-E(mu_o)), since int (1-E(mu_i)) cos = pi*(1-Eavg). diffuseEnergy drops evaluateDiffuseLobe's wi-side coat factor; selection mass need only be proportional to energy, not equal to it.
    const float msReflectEnergy = ((multiScatterTint(fresnelAvg.x, splitAvg.total()) +
                                     multiScatterTint(fresnelAvg.y, splitAvg.total()) +
                                     multiScatterTint(fresnelAvg.z, splitAvg.total())) /
                                    3.0F) *
                                   std::max(1.0F - splitWo.total(), 0.0F);
    const float diffuseEnergy =
        diffuseKd * (params.diffuseRho.x + params.diffuseRho.y + params.diffuseRho.z) / 3.0F;
    // R_ss uses the same Schlick-split-with-exact-Fresnel-rescale as the coat; T_ss is the (1-fc) channel scaled by (1-f0), which is Schlick's 1-F factored exactly.
    // transmitWeight, not transmissionFactor: the transmission lobe's own energy is gated by (1-metallic) too (see transmittance above), so a metallic=1 material transmits nothing however its transmissionFactor is set. Using the raw factor here credited the escape budget with transmission that never happens, and the compensation handed the difference back as multiple scattering, measured as Lo=1.43 on a metallic=1, transmission=1 surface.
    // effectiveTransmission: inside the medium there is no diffuse substrate to withhold anything (a ray must reflect internally or exit), so transmissionFactor gates the entering side only. transmitPhysicalValue above already did this; transmitWeight did not, leaving the exiting side's value and its escape budget disagreeing at 0 < transmissionFactor < 1.
    const float eta = etaI / etaT;
    const float effectiveTransmission = exiting ? 1.0F : params.transmissionFactor;
    const float transmitWeight = effectiveTransmission * (1.0F - params.metallic);
    LobeProbabilities lobes{.specular = 0.0F,
                            .diffuse = 0.0F,
                            .msReflect = 0.0F,
                            .msReflectTransmissive = 0.0F,
                            .transmit = 0.0F,
                            .msTransmit = 0.0F,
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
                            .escapeWo = 0.0F,
                            .transmitShape = {},
                            .reflectShape = {},
                            .transmitShare = 0.0F,
                            .etaSq = eta * eta,
                            .transmitWeight = transmitWeight,
                            .facetTransmit = 0.0F};

    if (params.transmissionFactor <= 0.0F) {
        // Scaled by E: the specular lobe has two parts, and only the single-scattering part is drawn by VNDF sampling. The multiple-scattering part is (1-E)cos-shaped and drawn by msReflect below, so its selection mass must move there; otherwise a rough white metal, whose Fresnel pins specularProb to the 0.95 clamp, would sample 69% of its own reflectance only 5% of the time.
        const float specularProb = std::clamp(
            glm::mix(fresnelAtNormal, conductorLuma, params.metallic) * splitWo.total(), 0.05F, 0.95F);
        const float diffuseProb = 1.0F - specularProb;
        // Split of the non-specular mass between the two strategies that share it, proportional to the energy each carries. Without it the Kulla-Conty reflection lobe borrows the diffuse slot and is drawn with a CLTC shape set by diffuseRoughness, a parameter of the lobe it is not: a conductor carries all of that slot's energy and none of its shape.
        // No deficit gate: this ratio already sends the share to zero continuously with the deficit, and gating would strand the mass on the diffuse strategy at exactly the low roughnesses where 1-Eavg is small.
        // All to cosine on underflow: the cosine strategy is strictly positive over the whole hemisphere at every parameter, so it is the safe recipient of mass whose split cannot be resolved.
        const float msReflectProb =
            diffuseProb *
            (msReflectEnergy + diffuseEnergy > 1e-6F ? msReflectEnergy / (msReflectEnergy + diffuseEnergy) : 1.0F);
        lobes.specular = specularProb;
        lobes.diffuse = diffuseProb - msReflectProb;
        lobes.msReflect = msReflectProb;
        return lobes;
    }

    // Transmissive: every strategy's selection mass is proportional to the energy it carries at wo, so f*cos/pdf is as flat across strategies as their shapes allow and no clamp or cap is needed; only a zero-energy strategy gets zero mass, and its value is zero, which is the MIS support condition (Veach 1997 sec. 9.2).
    // Flux terms, without the eta^2 radiance compression, as PBRT-v4's DielectricBxDF selects R/T: the compression cancels over the round trip, so weighting selection by it would oversample reflection entering and transmission exiting.
    // Untinted, so the sampled directions are independent of transmissionTint and a tinted estimate is exactly the white one times the tint (transmission_tint, rough_transmission_tint); a dark tint only spends draws on zero-valued paths, variance rather than bias.
    // Index-matched interfaces take the exact boundary rather than the table: Fresnel is identically 0, the delta branch transmits everything, and the log-spaced eta axis puts eta 1 halfway between its nodes, where the interpolated deficit is not 0.
    float reflectSs = 0.0F;
    float transmitSs = 1.0F;
    if (params.ior == 1.0F) {
        lobes.escapeWo = 1.0F;
        lobes.transmitShare = 1.0F;
    } else {
        // R + T, with no transmissionFactor weighting: energy the interface refracts but transmissionFactor withholds from the transmit lobe enters the diffuse substrate instead (that is what diffuseKd's (1-transmissionFactor) does), and escapes from there. Either way it leaves, so the microfacet deficit the compensation must return is the same. Weighting this term by transmitWeight instead over-reported the deficit at partial transmission, measured Lo=2.64 against a bound of 2.25.
        const EscapeSplit escapeWo = escapeAlbedo(wo.z, params.roughness, eta);
        const EscapeSplit escapeMean = averageEscapeAlbedo(params.roughness, eta);
        lobes.escapeWo = escapeWo.total();
        lobes.transmitShape = msTransmitRow(params.roughness, 1.0F / eta);
        lobes.reflectShape = msTransmitRow(params.roughness, eta);
        lobes.transmitShare = transmitWeight * escapeMean.transmit / std::max(escapeMean.total(), 1e-4F);
        reflectSs = escapeWo.reflect;
        transmitSs = escapeWo.transmit;
    }
    const float deficit = std::max(1.0F - lobes.escapeWo, 0.0F);
    const bool rough = transmissionIsRough(params, alpha);
    // The conductor's single scatter is macro Fresnel times E, as in the opaque split; the dielectric's is the table's R_ss.
    const float specularEnergy = glm::mix(reflectSs, conductorLuma * splitWo.total(), params.metallic);
    const float opaqueMsEnergy = (1.0F - transmitWeight) * msReflectEnergy;
    // A zero-scale row has no density to draw from, and its value reads the same zero.
    const float msReflectTransmissiveEnergy =
        lobes.reflectShape.scale > 0.0F ? transmitWeight * (1.0F - lobes.transmitShare) * deficit : 0.0F;
    // A rough interface's refraction is the VNDF strategy's other branch, chosen per facet, so its energy joins the specular mass; a delta keeps its own.
    const float transmitEnergy = rough ? transmitWeight * transmitSs : transmitPhysicalValue;
    // evaluateContinuousLobes drops the far-side multiple scattering of a delta interface, so it has no energy to select for.
    const float msTransmitEnergy = rough && lobes.transmitShape.scale > 0.0F
                                       ? transmitWeight * lobes.transmitShare * deficit
                                       : 0.0F;
    const float total = specularEnergy + diffuseEnergy + opaqueMsEnergy + msReflectTransmissiveEnergy +
                        transmitEnergy + msTransmitEnergy;
    // Zero only where every lobe's value is zero (a fully metallic interface whose conductor Fresnel is 0), leaving sampleBsdf nothing to draw.
    const float inverseTotal = total > 0.0F ? 1.0F / total : 0.0F;
    lobes.specular = (rough ? specularEnergy + transmitEnergy : specularEnergy) * inverseTotal;
    lobes.diffuse = diffuseEnergy * inverseTotal;
    lobes.msReflect = opaqueMsEnergy * inverseTotal;
    lobes.msReflectTransmissive = msReflectTransmissiveEnergy * inverseTotal;
    lobes.transmit = rough ? 0.0F : transmitEnergy * inverseTotal;
    lobes.msTransmit = msTransmitEnergy * inverseTotal;
    lobes.facetTransmit = rough ? transmitWeight : 0.0F;
    return lobes;
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
    const float d = distributionGGX(ht, alpha);
    const float fresnel = fresnelDielectric(woDotH, lobes.etaI, lobes.etaT);
    // D*G2*|wi.h|*(wo.h)/(wo.z*|wi.z|*denom^2), with G2/(wo.z*|wi.z|) taken as 4*smithVisibility.
    const float common = (4.0F * d * smithVisibility(wo.z, -wi.z, alpha) * std::abs(wiDotH) * woDotH) / denom2;
    const float vndfPdf = d * woDotH * smithG1OverCos(wo.z, alpha);
    // transmitWeight, the same factors the delta branch carries via transmitPhysicalValue: (1-metallic), since a conductor transmits nothing however its transmissionFactor is set, and the entering side's transmissionFactor, without which this refracted at full strength on top of a diffuse substrate already scaled by (1-transmissionFactor).
    // The VNDF strategy refracts about this facet with probability 1 - facetReflectProbability, and never where facetTransmit is 0.
    const float refractProbability =
        lobes.facetTransmit > 0.0F ? 1.0F - facetReflectProbability(params, woDotH, fresnel, lobes) : 0.0F;
    return {params.transmissionTint * (1.0F - fresnel) * lobes.transmitWeight * common,
             refractProbability * vndfPdf * etaR * etaR * std::abs(wiDotH) / denom2};
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
        const float msPdf = msTransmitPdf(-wi.z, lobes.transmitShape);
        return {glm::vec3(0.0F), glm::vec3(0.0F), transmission.f + transmitMultiScatter(params, -wi.z, msPdf, lobes),
                (lobes.specular * transmission.pdf) + (lobes.msTransmit * msPdf)};
    }
    const LobeEval specular = evaluateSpecularLobe(params, wo, wi, alpha, lobes);
    const LobeEval diffuse = evaluateDiffuseLobe(params, wo, wi, lobes);
    float pdf = (lobes.specular * specular.pdf) + (lobes.diffuse * diffuse.pdf) +
                (lobes.msReflect * msReflectPdf(wi.z, params.roughness));
    // Gated so an opaque material skips the row lookup; the term is 0 there either way.
    if (lobes.msReflectTransmissive > 0.0F) {
        pdf += lobes.msReflectTransmissive * msTransmitPdf(wi.z, lobes.reflectShape);
    }
    return {diffuse.f, specular.f, glm::vec3(0.0F), pdf};
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

    // One-sample MIS: whichever strategy drew wi, the throughput divides by the whole mixture's density at it.
    const auto weigh = [&](const glm::vec3& wi, LobeType type) -> std::optional<BsdfSample> {
        const BsdfEval eval = evaluateContinuousLobes(params, wo, wi, alpha, lobes);
        if (eval.pdf <= 1e-8F) {
            return std::nullopt;
        }
        return BsdfSample{glm::vec3(wi.x, wi.y, wi.z * sign), (eval.total() * std::abs(wi.z)) / eval.pdf, type,
                           eval.pdf};
    };

    // VNDF single scatter. On a rough transmissive interface it refracts about the sampled facet as well (Walter 2007), split per facet by facetReflectProbability; lobeU/specular is uniform given this strategy, so it decides the split without another sampler dimension.
    // A facet whose refraction totally internally reflects has Fresnel 1 and so always reflects.
    if (lobeU < lobes.specular) {
        const glm::vec3 nh = sampleGGXVNDF(wo, alpha, sampler.next2D());
        if (lobes.facetTransmit > 0.0F) {
            const float woDotNh = glm::dot(wo, nh);
            const float fDielectric = fresnelDielectric(woDotNh, lobes.etaI, lobes.etaT);
            if (lobeU >= lobes.specular * facetReflectProbability(params, woDotNh, fDielectric, lobes)) {
                glm::vec3 wi;
                if (!refractAbout(wo, nh, lobes.etaI / lobes.etaT, wi) || wi.z >= 0.0F) {
                    return std::nullopt;
                }
                return weigh(wi, LobeType::Transmission);
            }
        }
        const glm::vec3 wi = glm::reflect(-wo, nh);
        if (wi.z <= 0.0F) {
            return std::nullopt;
        }
        return weigh(wi, LobeType::SpecularReflection);
    }

    // The reflection region's sub-ranges are prefix sums of one expression, so they partition it exactly: a strategy with no mass has an empty range and is never reached.
    if (lobeU < lobes.specular + lobes.diffuse + lobes.msReflect + lobes.msReflectTransmissive) {
        const bool sampledDiffuse = lobeU < lobes.specular + lobes.diffuse;
        glm::vec3 wi;
        if (sampledDiffuse) {
            wi = sampleEon(wo, params.diffuseRoughness, sampler.next2D());
        } else if (lobeU < lobes.specular + lobes.diffuse + lobes.msReflect) {
            wi = sampleMsReflect(params.roughness, sampler.next2D());
        } else {
            wi = sampleMsTransmit(lobes.reflectShape, sampler.next2D());
        }
        if (wi.z <= 0.0F) {
            return std::nullopt;
        }
        // The multiple-scattering branches report SpecularReflection: path_tracer.cpp buckets transport by strategy, and repeated bounces on a GGX microsurface are specular however broad their exitant distribution is.
        return weigh(wi, sampledDiffuse ? LobeType::Diffuse : LobeType::SpecularReflection);
    }

    // Top slice of the ladder: the multiple-scattering transmission lobe, drawn from its own tabulated shape over the far hemisphere. It needs a strategy of its own because the refraction VNDF reaches only directions some microfacet can refract into, while this lobe spans the whole hemisphere.
    // msTransmit tested first, not inside: the five probabilities below it sum to 1.0 only to float precision, so with no mass here a top-of-range lobeU must fall through to the delta transmit lobe rather than be rejected.
    // Draws from lobes.transmitShape, the row transmitMultiScatter's value and msTransmitPdf both read, so the shape drawn is the shape evaluated and divided by.
    if (lobes.msTransmit > 0.0F &&
        lobeU >= lobes.specular + lobes.diffuse + lobes.msReflect + lobes.msReflectTransmissive + lobes.transmit) {
        glm::vec3 wi = sampleMsTransmit(lobes.transmitShape, sampler.next2D());
        wi.z = -wi.z;
        return weigh(wi, LobeType::Transmission);
    }

    if (lobes.transmit <= 0.0F) {
        return std::nullopt;
    }

    // Smooth specular transmission (delta lobe): Snell's law, TIR already folded into lobes.transmit, whose (1-F) energy is 0 past the critical angle -- exactly, since both sites now decide it with the same cos2Transmitted predicate rather than two separately-rounded transcriptions.
    const float eta = lobes.etaI / lobes.etaT;
    const float cos2ThetaT = cos2Transmitted(wo.z, eta);
    if (cos2ThetaT < 0.0F) {
        return std::nullopt;
    }
    const float cosThetaT = std::sqrt(cos2ThetaT);
    const glm::vec3 wt(-eta * wo.x, -eta * wo.y, -cosThetaT);
    // Non-symmetric radiance-compression factor for camera-originated (Veach 1997 sec. 5.2, PBRT's SpecularTransmission::Sample_f under TransportMode::Radiance) transport: eta^2 = (etaI/etaT)^2, the squared ratio of the medium the ray is leaving to the medium it's entering. Self-consistent under round trips -- entering (eta=1/ior) times exiting (eta=ior/1) squared multiplies to 1, so a ray that enters and exits the same surface loses no net energy (tools/bsdf_validate.cpp's furnace test).
    const glm::vec3 throughput =
        params.transmissionTint * (lobes.transmitPhysicalValue / lobes.transmit) * (eta * eta);
    // pdf 0: a delta lobe has no density for NEE to double-count against, which is exactly the test path_tracer.cpp's MIS weighting makes.
    return BsdfSample{glm::vec3(wt.x, wt.y, wt.z * sign), throughput, LobeType::Transmission, 0.0F};
}

}  // namespace engine::scene
