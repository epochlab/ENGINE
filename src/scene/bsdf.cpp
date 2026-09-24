#include "pathtracer/scene/bsdf.h"
#include "pathtracer/scene/fresnel_dielectric.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace pathtracer::scene {

namespace {

constexpr float kPi = 3.14159265F;
constexpr float kMinAlpha = 0.02F * 0.02F;  // roughness floor, avoids a degenerate GGX delta lobe

// Perceptual roughness to GGX alpha, in one place: evaluateBsdfSplit, sampleBsdf and fresnelAtMicrofacet must agree,
// or the Fresnel AOV reports a term the lobe never evaluated.
float alphaForRoughness(float roughness) { return std::max(roughness * roughness, kMinAlpha); }

// GGX D in the cancellation-free form (Filament 4.4.2): the textbook denominator cancels catastrophically near the
// normal, which at low roughness is the whole lobe. No denominator floor is needed or wanted -- kPi*d*d >= 8e-14.
// See DERIVATIONS.md "GGX numerical forms" for both measurements.
float distributionGGX(const glm::vec3& nh, float alpha) {
    const float alpha2 = alpha * alpha;
    const float d = (alpha2 * nh.z * nh.z) + (nh.x * nh.x) + (nh.y * nh.y);
    return alpha2 / (kPi * d * d);
}

// cos*sqrt(1 + alpha^2*tan^2), the GGX Smith Lambda's radical scaled by the cosine: 1 + Lambda(c) = (c + radical)/(2c),
// and unlike tan it is finite at c = 0.
float smithRadical(float cosTheta, float alpha) {
    const float alpha2 = alpha * alpha;
    return std::sqrt(alpha2 + ((1.0F - alpha2) * cosTheta * cosTheta));
}

// G2/(4*cosO*cosI) for the height-correlated Smith G2 (Heitz 2014; Filament's V_SmithGGXCorrelated): the cosines
// multiply rather than divide, so it is exact to the silhouette and vanishes only where both cosines do.
float smithVisibility(float cosO, float cosI, float alpha) {
    return 0.5F / ((cosI * smithRadical(cosO, alpha)) + (cosO * smithRadical(cosI, alpha)));
}

// G1(c)/c, the VNDF pdf's projected-area factor, in the same division-free form: 2/alpha at grazing rather than 0/0.
float smithG1OverCos(float cosTheta, float alpha) { return 2.0F / (cosTheta + smithRadical(cosTheta, alpha)); }

// --- Conductor Fresnel: Gulbrandsen 2014, "Artist Friendly Metallic Fresnel", JCGT 3(4). Replaces Schlick on the
// metal path, which is monotone in cos and so cannot express the reflectance dip real metals have. Parameterised by
// reflectivity r and edgetint g, inverted to a complex IOR. See DERIVATIONS.md "Conductor Fresnel".

// Reflectivity is clamped, not asserted: f0 arrives as an unbounded texture product, so both ends are reachable from
// an asset. r=1 would evaluate 0*inf; r=0 inverts to an index-matched interface. The 0.9999 upper clamp is safe only
// because of the factored k^2 below -- the paper's literal form needs 0.99, costing 1% at f0=1.
constexpr float kMinReflectivity = 1e-4F;
constexpr float kMaxReflectivity = 0.9999F;

struct ConductorIor {
    glm::vec3 n;
    glm::vec3 k;
};

// Paper eq 12 for n, eq 2 for k. k^2 is the factored (nMax - n)(n - nLow), not the listing's literal form, which
// returns k^2 = -1.28e6 in float32 at r=0.9999, g=0 where the truth is exactly 0. Reflectivity and edgeTint come
// from measured (lambda, n, k) via tools/metal_fit, never from eyeballing a colour.
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

// Exact unpolarized Fresnel of one conductor channel with complex IOR n + ik (Born & Wolf), in real arithmetic: two
// sqrts, no complex division. Not the paper's Appendix A rs/rp, the large-|eta| approximation, which deviates by up
// to 0.094 around r~0.25 -- where real metals sit. Deliberately clampless; DERIVATIONS.md says why none can help.
float fresnelConductorChannel(float cosTheta, float n, float k) {
    const float c2 = cosTheta * cosTheta;
    const float s2 = 1.0F - c2;
    const float nk2 = (n * n) * (k * k);
    const float t0 = (n * n) - (k * k) - s2;
    const float a2b2 = std::sqrt((t0 * t0) + (4.0F * nk2));
    // a^2 = (a2b2 + t0)/2, through whichever of its two algebraically equal forms is a sum: the direct form subtracts
    // near-equal magnitudes when t0 < 0 and collapses a to zero in float32, pinning F at exactly 1.0 -- which is
    // Schlick's own answer at f0=1, so the furnace rows read byte-identical and the fault looked inert.
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

// Kulla-Conty energy tables, baked offline by tools/albedo_table.cpp (Kulla & Conty 2017). The .inc defines the grid
// resolutions alongside the arrays, so the grid the lookups index is the grid the generator wrote.
// See DERIVATIONS.md "Kulla-Conty energy tables" for what each array holds and why the escape tables use exact Fresnel.
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

// Bilinear lookup, indexed by sqrt(mu) to match the grid the generator wrote: E climbs from 1 at grazing to its
// plateau over mu ~ alpha, a layer a uniform axis would span with under one cell at low roughness.
// checkAlbedoTableInterpolation measures the residual per axis.
AlbedoSplit directionalAlbedo(float mu, float roughness) {
    const float rf = std::clamp(roughness, 0.0F, 1.0F) * (kAlbedoRoughnessRes - 1);
    const float mf = std::sqrt(std::clamp(mu, 0.0F, 1.0F)) * (kAlbedoMuRes - 1);
    const int r0 = std::min(static_cast<int>(rf), kAlbedoRoughnessRes - 2);
    const int m0 = std::min(static_cast<int>(mf), kAlbedoMuRes - 2);
    const float rt = rf - static_cast<float>(r0);
    const float mt = mf - static_cast<float>(m0);
    const int i0 = (r0 * kAlbedoMuRes) + m0;
    const int i1 = ((r0 + 1) * kAlbedoMuRes) + m0;
    return {lerp1(lerp1(kAlbedoA[i0], kAlbedoA[i0 + 1], mt),
                   lerp1(kAlbedoA[i1], kAlbedoA[i1 + 1], mt), rt),
             lerp1(lerp1(kAlbedoB[i0], kAlbedoB[i0 + 1], mt),
                   lerp1(kAlbedoB[i1], kAlbedoB[i1 + 1], mt), rt)};
}

AlbedoSplit averageAlbedo(float roughness) {
    const float rf = std::clamp(roughness, 0.0F, 1.0F) * (kAlbedoRoughnessRes - 1);
    const int r0 = std::min(static_cast<int>(rf), kAlbedoRoughnessRes - 2);
    const float rt = rf - static_cast<float>(r0);
    return {lerp1(kAlbedoAvgA[r0], kAlbedoAvgA[r0 + 1], rt),
             lerp1(kAlbedoAvgB[r0], kAlbedoAvgB[r0 + 1], rt)};
}

// --- Reflected multiple-scattering lobe, from kMsReflectDensity/kMsReflectCdf. The zero-variance density is
// (1-E(mu_i))*cos/(pi*(1-Eavg)), and the table holds exactly that shape, piecewise-linear with exact prefix
// integrals. Cosine sampling costs up to +17.3 relative variance at low roughness. DERIVATIONS.md has the rest.
struct MsReflectRow {
    int base;
    float blend;
};

MsReflectRow msReflectRow(float roughness) {
    const float rf = std::clamp(roughness, 0.0F, 1.0F) * (kAlbedoRoughnessRes - 1);
    const int r0 = std::min(static_cast<int>(rf), kAlbedoRoughnessRes - 2);
    return {r0 * kMsReflectMuRes, rf - static_cast<float>(r0)};
}

float msReflectDensity(const MsReflectRow& row, int index) {
    return lerp1(kMsReflectDensity[row.base + index],
                  kMsReflectDensity[row.base + kMsReflectMuRes + index], row.blend);
}

float msReflectCdf(const MsReflectRow& row, int index) {
    return lerp1(kMsReflectCdf[row.base + index], kMsReflectCdf[row.base + kMsReflectMuRes + index],
                  row.blend);
}

// Solid-angle density: the mu density spread over 2*pi of azimuth, reducing to mu/pi wherever the stored shape is
// 2*mu. Uniform in mu, not sqrt(mu): this reads the sampling shape, whose grid stays uniform so the inversion keeps
// one step width. The generator already applied the albedo warp when it built each node from E.
float msReflectPdf(float mu, float roughness) {
    const MsReflectRow row = msReflectRow(roughness);
    const float mf = std::clamp(mu, 0.0F, 1.0F) * (kMsReflectMuRes - 1);
    const int m0 = std::min(static_cast<int>(mf), kMsReflectMuRes - 2);
    const float mt = mf - static_cast<float>(m0);
    return lerp1(msReflectDensity(row, m0), msReflectDensity(row, m0 + 1), mt) / (2.0F * kPi);
}

// Exact inversion of a tabulated piecewise-linear density over mu on an edge-aligned grid: binary search the prefix
// integrals for the segment, then take the positive root of its quadratic. Written in the form that stays finite as
// a segment flattens and at mu = 0 -- see DERIVATIONS.md "Multiple-scattering lobe sampling".
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
                                                   [&](int i) { return msReflectCdf(row, i); },
                                                   kMsReflectMuRes, u.x);
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

// --- Transmitted multiple-scattering lobe, from kMsTransmitDensity/kMsTransmitCdf: the far-hemisphere twin of
// sampleMsReflect, one axis wider because the escape it is built from is eta-dependent. Value, density and sampler
// read one interpolant (Dupuy & Jakob 2018), so f*cos/pdf is constant for every wi.
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

// Stored unnormalised and divided by its own blended total here rather than normalised per row at bake time:
// integration is linear, so a blend of exact prefix integrals is the exact prefix integral of the blended density.
// It also makes a numerically dead row harmless, contributing near-zero weight rather than amplified noise.
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
    // Last prefix integral is the row's total energy deficit. Zero only if every clamped deficit in all four rows is
    // zero, leaving the lobe no energy to carry, so a zero scale correctly reports a zero density rather than dividing.
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

// Malley's method: a uniform point on the unit disk lifted to the hemisphere, exactly the cosine distribution
// (PBR 4th ed. 13.6.3). External linkage for path_tracer.cpp's AO lane, whose estimator is the mean of visibility
// alone because this pdf cancels the cosine.
glm::vec3 sampleCosineHemisphere(glm::vec2 u) {
    const float r = std::sqrt(u.x);
    const float phi = 2.0F * kPi * u.y;
    return {r * std::cos(phi), r * std::sin(phi), std::sqrt(std::max(0.0F, 1.0F - u.x))};
}

// Cosine-weighted average Fresnel, 2*int_0^1 F(mu)*mu dmu. One 3-node rule serves both interfaces, each over the
// Fresnel its own single scatter evaluates. External linkage for checkAverageFresnel, the only instrument that can
// see an error in either. Fit, error bounds and rejected alternatives: DERIVATIONS.md "Average Fresnel quadrature".
constexpr float kFresnelAvgNodes[3] = {0.105319802F, 0.382154433F, 0.796427281F};
constexpr float kFresnelAvgWeights[3] = {0.038972482F, 0.280518736F, 0.680508783F};

// The dielectric interface, over the same fresnelDielectric the coat and specular lobes evaluate. The conductor
// nodes are carried here unrefitted: worst 5.5e-5 over ior [1.05, 3.0]. Exactly +0 at ior 1, structurally rather
// than by an algebraic accident -- checkIndexMatchedCoat requires that zero at tolerance exactly 0.
float dielectricFresnelAvg(float ior) {
    float sum = 0.0F;
    for (int i = 0; i < 3; ++i) {
        sum += kFresnelAvgWeights[i] * fresnelDielectric(kFresnelAvgNodes[i], 1.0F, ior);
    }
    return sum;
}

// The two reflect-side albedo lookups have external linkage under the same rule: checkAlbedoTableInterpolation is
// the only instrument that can see albedo_table.inc's interpolation error, the second error source beside the
// quadrature residual the generator prints. glm::vec2 rather than AlbedoSplit, which is a shading-side concept.
glm::vec2 directionalAlbedoSplit(float mu, float roughness) {
    const AlbedoSplit split = directionalAlbedo(mu, roughness);
    return {split.a, split.b};
}

glm::vec2 averageAlbedoSplit(float roughness) {
    const AlbedoSplit split = averageAlbedo(roughness);
    return {split.a, split.b};
}

// The grid the two lookups index, described rather than transcribed -- see the header. Both axes are edge-aligned,
// so index 0 and res-1 are exact endpoints and a fractional index lands where the lookups interpolate.
glm::ivec2 albedoGridRes() { return {kAlbedoRoughnessRes, kAlbedoMuRes}; }

float albedoGridRoughness(float index) { return index / static_cast<float>(kAlbedoRoughnessRes - 1); }

// Inverts directionalAlbedo's sqrt(mu) index, so an instrument placing a sample at a fractional index lands where
// that lookup interpolates rather than where a linear axis would put it.
float albedoGridMu(float index) {
    const float t = index / static_cast<float>(kAlbedoMuRes - 1);
    return t * t;
}

// The conductor interface, over the same fresnelConductorChannel its single scatter evaluates. Karis' mean is exact
// for Schlick and so averages a different function once the single scatter is complex-IOR; worse, its error changes
// sign with edgeTint, which f0 alone cannot express.
glm::vec3 conductorFresnelAvg(const glm::vec3& n, const glm::vec3& k) {
    glm::vec3 sum(0.0F);
    for (int i = 0; i < 3; ++i) {
        sum += kFresnelAvgWeights[i] * fresnelConductor(kFresnelAvgNodes[i], n, k);
    }
    return sum;
}

// Fraunhofer d, F and C lines: the three wavelengths the Abbe number is defined at, V_d = (n_d-1)/(n_F-n_C), and the
// only ones (ior, abbe) pins. Physical constants of the definition, not tuning.
constexpr float kLambdaDNm = 587.56F;
constexpr float kLambdaFNm = 486.13F;
constexpr float kLambdaCNm = 656.27F;

// Cauchy's two-term n(lambda) = A + B/lambda^2, (A, B) inverted from the authored (n_d, V_d), as Khronos
// KHR_materials_dispersion specifies. abbe <= 0 is the documented off switch, which also keeps 1/abbe off the hot
// path; n_d = 1 gives B = 0 exactly. See DERIVATIONS.md "Cauchy dispersion".
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

// Kulla-Conty multiple-scattering tint: the share of the (1-E) energy surviving repeated microsurface bounces, each
// attenuated by Favg. Equals 1 for a perfect reflector (Favg=1), so a white conductor conserves exactly.
float multiScatterTint(float fresnelAvg, float albedoAvg) {
    return (fresnelAvg * fresnelAvg * albedoAvg) /
           std::max(1.0F - (fresnelAvg * (1.0F - albedoAvg)), 1e-4F);
}

float schlickScalar(float cosTheta, float f0) {
    return f0 + ((1.0F - f0) * std::pow(std::clamp(1.0F - cosTheta, 0.0F, 1.0F), 5.0F));
}

// etaI/etaT rather than a bare ior: on the exiting side fresnelDielectric returns exactly 1.0 past the critical
// angle, and Schlick cannot express TIR at all. Hardcoding the entering orientation under-reported an exiting ray's
// reflected share by up to the whole TIR cone, which the compensation then handed back as multiple scattering.
float coatFresnelRatio(float cosTheta, float etaI, float etaT, float f0) {
    return fresnelDielectric(cosTheta, etaI, etaT) / std::max(schlickScalar(cosTheta, f0), 1e-6F);
}

// Total directional albedo of the dielectric coat, single scatter plus its own multiple-scattering lobe -- not the
// macro-facet F(mu_o), which differs by 4x at roughness 1, mu 0.4 and cost a measured 10% energy loss. fresnelRatio
// and fresnelAvg reconcile the Schlick-basis table with the exact Fresnel. DERIVATIONS.md "Dielectric coat coupling".
float coatAlbedo(const AlbedoSplit& split, float albedoAvg, float f0, float fresnelRatio,
                  float fresnelAvg) {
    return (split.at(f0) * fresnelRatio) +
           (multiScatterTint(fresnelAvg, albedoAvg) * (1.0F - split.total()));
}

// Below this the GGX transmission lobe is a delta (PBRT's EffectivelySmooth). kMinAlpha (roughness 0.02) sits inside
// this region, so smooth glass keeps its exact noise-free Snell path.
constexpr float kSmoothAlpha = 1e-3F;

// ior == 1 is a delta at every roughness, not a rough interface: the half-vector normalizes the zero vector and every
// guard below becomes a NaN comparison. See DERIVATIONS.md "Smooth-transmission threshold" for the measured rates.
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
    // Energy-compensation state, hoisted so the wo-side table lookups happen once per evaluation, not per lobe call.
    float albedoWo;         // E(mu_o, roughness), Fresnel-free
    float albedoAvg;        // Eavg(roughness)
    float coatF0;           // dielectric f0 implied by ior, for the diffuse coupling
    // The coat's own cosine-mean Fresnel. Separate from fresnelAvg below, the metallic-blended mean the specular lobe
    // needs and wrong for the coat at any metallic > 0.
    float coatFresnelAvg;
    glm::vec3 fresnelAvg;
    // Complex IOR inverted from (f0, edgeTint) once per evaluation rather than once per lobe call.
    // Index-matched (1, 0) when metallic==0, where no consumer reads them: evaluateSpecularLobe gates its conductor
    // Fresnel on the same metallic>0 test, and the two must stay identical -- (1, 0) is 0/0 at cosTheta=0 exactly.
    glm::vec3 conductorN;
    glm::vec3 conductorK;
    // Multiple-scattering state for a transmissive interface, which needs its own deficit: a facet either reflects or
    // refracts, so the escaping fraction is Fresnel-weighted (R_ss + T_ss) and cannot reuse the opaque Fresnel-free
    // (1 - E). Blended by transmissionFactor rather than unified, so an opaque material keeps its measured behaviour.
    float escapeWo;         // R_ss(mu_o) + T_ss(mu_o), the Fresnel-weighted escaping fraction
    // Escape-deficit shape at the reciprocal eta (etaT/etaI); scale 0 where no transmitted multiple scattering exists.
    MsTransmitRow transmitShape;
    MsTransmitRow reflectShape;   // the same at the forward eta (etaI/etaT), for the reflected share whose wi stays in wo's medium
    float transmitShare;    // of the multiple-scattered energy, the fraction leaving refracted
    float etaSq;            // (etaI/etaT)^2, the radiance compression the transmit lobe must carry
    // effectiveTransmission*(1-metallic): how much transmission actually happens. Scales the single-scatter and the
    // multiple-scattering transmit value; the delta branch carries the same factors through transmitPhysicalValue.
    float transmitWeight;
    // Refraction's value per unit (1-F) in the VNDF strategy's per-facet reflect/refract split, transmitWeight;
    // 0 where that strategy only reflects.
    float facetTransmit;
};

// The transmitted share of the multiple-scattering energy, for any wi on the far side, free of the half-vector
// rejections that describe single scattering only. K*pdf/mu integrates to exactly K for any table noise, so no gate
// is needed. mu > 0 strictly -- the only caller is evaluateContinuousLobes' wi.z < 0 branch.
glm::vec3 transmitMultiScatter(const BsdfParams& params, float mu, float msPdf, const LobeProbabilities& lobes) {
    return params.transmissionTint * lobes.transmitWeight * lobes.transmitShare * lobes.etaSq *
           (std::max(1.0F - lobes.escapeWo, 0.0F) * msPdf / mu);
}

// The full reciprocal coupling factor at wi: the wo-side half is precomputed into lobes.diffuseKd, the
// wi-side half is the same (1 - coatAlbedo) evaluated here. The cosine mean is lobes.coatFresnelAvg, the same float
// computeLobeProbabilities already produced from the same params.ior -- bit-identical to recomputing it here.
float diffuseKdAt(const BsdfParams& params, const glm::vec3& wi, const LobeProbabilities& lobes) {
    const AlbedoSplit splitWi = directionalAlbedo(wi.z, params.roughness);
    const float coat = coatAlbedo(splitWi, lobes.albedoAvg, lobes.coatF0,
                                   coatFresnelRatio(wi.z, lobes.etaI, lobes.etaT, lobes.coatF0),
                                   lobes.coatFresnelAvg);
    return std::max(lobes.diffuseKd, 0.0F) * (1.0F - coat);
}

// --- EON rough-diffuse BRDF (Portsmouth, Kutz, Hill 2025, JCGT 14(1)), replacing plain Lambertian as
// evaluateDiffuseLobe's base reflectance, still wrapped by diffuseKdAt's Fresnel-coat coupling. Fujii's FON model
// plus analytic multiple-scattering compensation. Ported from the paper's GLSL; do not hand-derive its constants.

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

// Paper Appendix A: the rho achieving a desired observed albedo, so diffuseColour means what OpenPBR says base_color
// means. One branch-free expression via the stable conjugate-multiplied root, not eq. 30's. Exported for
// checkEonAlbedoInversion and resolved once per hit. See DERIVATIONS.md "EON albedo inversion".
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

// Clipped-LTC direction sample (paper Sec. 4, Listing 3): cosine-weighted sampling of the hemisphere clipped to
// the LTC lobe's positive half-space, the Nusselt-analog projection, restricted to the positive hemisphere by
// construction -- no rejected below-surface samples, unlike naive LTC sampling.
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

// Mixing weight between the CLTC lobe and a uniform-hemisphere lobe (paper Sec. 4, fitted by minimizing the CLTC
// estimator's maximum throughput weight): CLTC alone has a bias/variance spike the uniform term corrects via
// one-sample MIS. Shared by sampleEon and pdfEon so the two agree on which mixture they draw from.
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
    // Strict: pUniform is exactly 0 at r=0 (pow(0,0.1)), where an inclusive test admits u.x==0 -- reachable from the
    // sampler's Cranley-Patterson wrap -- and reshuffles it as 0/0. The NaN direction passes every downstream guard,
    // NaN failing all ordered comparisons, and poisons the pixel for the rest of the progressive render.
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

// Probability the VNDF strategy reflects about a facet on a rough transmissive interface: the facet's reflected value
// over reflected plus refracted (Walter 2007 sec. 5.3; PBRT-v4 DielectricBxDF), so both branches weigh G2/G1 alone.
// The caller gates on facetTransmit > 0, where the denominator is positive.
float facetReflectProbability(const BsdfParams& params, float cosTheta, float fDielectric, const LobeProbabilities& lobes) {
    const float reflect = facetReflectance(params, cosTheta, fDielectric, lobes);
    return reflect / (reflect + ((1.0F - fDielectric) * lobes.facetTransmit));
}

// Single scatter D*G2*F/(4*ndotV*ndotL) plus the Kulla-Conty multiple-scattering lobe, and the VNDF pdf (Heitz 2018
// eq.3) times its reflection Jacobian. The pdf covers single scattering only: the reflected multiple-scattering
// share is (1-E)cos-shaped with its own strategy, lobes.msReflect, whose density joins the same mixture.
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

    // Kulla & Conty 2017. Integrates to (1-E(mu_o)) at Favg=1 -- the energy G2 discarded -- so a white conductor
    // conserves, within the 1% the white furnace bounds. Symmetric in wo/wi, so it preserves reciprocity. The tint
    // blends to 1 as the interface becomes fully transmissive, a lossless dielectric returning all of it.
    const glm::vec3 fms(multiScatterTint(lobes.fresnelAvg.x, lobes.albedoAvg),
                         multiScatterTint(lobes.fresnelAvg.y, lobes.albedoAvg),
                         multiScatterTint(lobes.fresnelAvg.z, lobes.albedoAvg));
    const float albedoWi = directionalAlbedo(wi.z, params.roughness).total();
    const glm::vec3 opaqueMs = fms * ((1.0F - lobes.albedoWo) * (1.0F - albedoWi)) /
                                (kPi * std::max(1.0F - lobes.albedoAvg, 1e-4F));
    // transmitWeight is exactly zero for every opaque material -- the common case -- so skip the escape-shape row
    // rather than build it and mix it away at weight 0. The transmissive reflected share is transmitMultiScatter's
    // near-side twin over the forward-eta row, so the two shares sum to the deficit exactly.
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

// Opaque: specular is the Fresnel reflectance probability times E; the rest is diffuse and msReflect. Transmissive:
// every strategy by its energy at wo, and an exiting ray has no diffuse substrate. transmitPhysicalValue must
// independently carry transmit's transmissionFactor and metallic factors, or they cancel out of the throughput.
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
    // Reciprocal diffuse coupling: the substrate receives what the coat did not reflect, in and out, renormalised by
    // 1/(1-coatAlbedoAvg) so the directional albedo integrates to (1-coatAlbedo(mu_o)). Symmetric in wo/wi, which the
    // bare (1-F(mu_o)) form was not. Not exact -- the white furnace bounds the residual under 1%.
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
    // Each interface's own cosine mean, over the Fresnel its single scatter evaluates: one 3-node rule, over the
    // conductor's complex IOR and over the dielectric's. conductorAvg is 0 off the metal path, where mix returns the
    // dielectric term exactly.
    const glm::vec3 fresnelAvg = glm::mix(glm::vec3(dielectricAvg), conductorAvg, params.metallic);
    // msEnergy is exact: opaqueMs integrates over the hemisphere to fms*(1-E(mu_o)), since int (1-E(mu_i)) cos =
    // pi*(1-Eavg). diffuseEnergy drops the wi-side coat factor -- selection mass need only be proportional to energy.
    const float msReflectEnergy = ((multiScatterTint(fresnelAvg.x, splitAvg.total()) +
                                     multiScatterTint(fresnelAvg.y, splitAvg.total()) +
                                     multiScatterTint(fresnelAvg.z, splitAvg.total())) /
                                    3.0F) *
                                   std::max(1.0F - splitWo.total(), 0.0F);
    const float diffuseEnergy =
        diffuseKd * (params.diffuseRho.x + params.diffuseRho.y + params.diffuseRho.z) / 3.0F;
    // R_ss uses the same Schlick split with exact-Fresnel rescale as the coat; T_ss is the (1-fc) channel scaled by
    // (1-f0). transmitWeight, not transmissionFactor: a metallic=1 material transmits nothing, and inside the medium
    // there is no diffuse substrate, so transmissionFactor gates the entering side only.
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
                            .coatFresnelAvg = dielectricAvg,
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
        // Scaled by E: only the single-scattering part is drawn by VNDF sampling. The multiple-scattering part is
        // (1-E)cos-shaped and drawn by msReflect, so its selection mass must move there too.
        const float specularProb = std::clamp(
            glm::mix(fresnelAtNormal, conductorLuma, params.metallic) * splitWo.total(), 0.05F, 0.95F);
        const float diffuseProb = 1.0F - specularProb;
        // Splits the non-specular mass proportional to the energy each strategy carries; without it the Kulla-Conty
        // lobe borrows the diffuse slot and takes a CLTC shape set by diffuseRoughness. No deficit gate -- the ratio
        // already vanishes with the deficit. All to cosine on underflow, it being positive over the whole hemisphere.
        const float msReflectProb =
            diffuseProb *
            (msReflectEnergy + diffuseEnergy > 1e-6F ? msReflectEnergy / (msReflectEnergy + diffuseEnergy) : 1.0F);
        lobes.specular = specularProb;
        lobes.diffuse = diffuseProb - msReflectProb;
        lobes.msReflect = msReflectProb;
        return lobes;
    }

    // Transmissive: every strategy's mass is proportional to the energy it carries at wo, so f*cos/pdf is as flat as
    // their shapes allow. Flux terms without the eta^2 compression, as PBRT-v4's DielectricBxDF selects R/T, and
    // untinted so directions are independent of transmissionTint. Index-matched interfaces take the exact boundary.
    float reflectSs = 0.0F;
    float transmitSs = 1.0F;
    if (params.ior == 1.0F) {
        lobes.escapeWo = 1.0F;
        lobes.transmitShare = 1.0F;
    } else {
        // R + T, with no transmissionFactor weighting: energy the interface refracts but transmissionFactor withholds
        // from the transmit lobe enters the diffuse substrate instead -- that is what diffuseKd's (1-transmissionFactor)
        // does -- and escapes from there.
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
    // A rough interface's refraction is the VNDF strategy's other branch, chosen per facet, so its energy joins the
    // specular mass; a delta keeps its own.
    const float transmitEnergy = rough ? transmitWeight * transmitSs : transmitPhysicalValue;
    // evaluateContinuousLobes drops the far-side multiple scattering of a delta interface, so it has no energy to select for.
    const float msTransmitEnergy = rough && lobes.transmitShape.scale > 0.0F
                                       ? transmitWeight * lobes.transmitShare * deficit
                                       : 0.0F;
    const float total = specularEnergy + diffuseEnergy + opaqueMsEnergy + msReflectTransmissiveEnergy +
                        transmitEnergy + msTransmitEnergy;
    // Zero only where every lobe's value is zero (a fully metallic interface whose conductor Fresnel is 0), leaving
    // sampleBsdf nothing to draw.
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

// Walter et al. 2007 rough transmission, single scatter only (value eq. 21, half-vector eq. 16, Jacobian eq. 17), in
// PBRT-v3's radiance-transport form. The eta^2 radiance compression (Veach 1997 sec. 5.2) is already folded in, so
// this must not apply it again; the etaR^2 surviving in the pdf but not the value is that asymmetry.
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
    // transmitWeight carries the same factors as the delta branch: (1-metallic), a conductor transmitting nothing
    // whatever its transmissionFactor, and the entering side's transmissionFactor.
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
        // The multiple-scattering term stays inside this gate. A delta interface keeps pdf=0 on the far side, and a
        // non-zero value there would be silently discarded by path_tracer.cpp's bsdfPdf>0 guard: energy lost.
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

glm::vec3 fresnelAtMicrofacet(const BsdfParams& params, const glm::vec3& woLocal, glm::vec2 u) {
    // sampleGGXVNDF's +z-hemisphere precondition, met as evaluateBsdfSplit and sampleBsdf meet it. Only dot(wo, wh) is
    // read, and reflecting the frame leaves it unchanged, so the flip needs no undoing.
    const glm::vec3 wo(woLocal.x, woLocal.y, std::abs(woLocal.z));
    const glm::vec3 wh = sampleGGXVNDF(wo, alphaForRoughness(params.roughness), u);
    // Clamped, not raw: fresnelDielectric swaps etaI/etaT below zero, so a negative dot would report the exiting-side
    // term instead of the entering one fresnelAtViewAngle documents.
    return fresnelAtViewAngle(params, std::max(glm::dot(wo, wh), 0.0F));
}

BsdfEval evaluateBsdfSplit(const BsdfParams& params, const glm::vec3& woLocal,
                            const glm::vec3& wiLocal) {
    const float sign = woLocal.z >= 0.0F ? 1.0F : -1.0F;
    const glm::vec3 wo(woLocal.x, woLocal.y, woLocal.z * sign);
    const glm::vec3 wi(wiLocal.x, wiLocal.y, wiLocal.z * sign);
    const float alpha = alphaForRoughness(params.roughness);
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
    const float alpha = alphaForRoughness(params.roughness);
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

    // VNDF single scatter, refracting about the sampled facet too on a rough transmissive interface (Walter 2007),
    // split per facet by facetReflectProbability; lobeU/specular is uniform given this strategy, so it decides the
    // split without another draw. A facet whose refraction totally internally reflects has Fresnel 1 and reflects.
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

    // The reflection region's sub-ranges are prefix sums of one expression, so they partition it exactly: a strategy
    // with no mass has an empty range and is never reached.
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
        // The multiple-scattering branches report SpecularReflection: path_tracer.cpp buckets by strategy, and
        // repeated bounces on a GGX microsurface are specular however broad their exitant distribution.
        return weigh(wi, sampledDiffuse ? LobeType::Diffuse : LobeType::SpecularReflection);
    }

    // Top slice of the ladder: the multiple-scattering transmission lobe, from its own tabulated shape over the far
    // hemisphere, needed because refraction VNDF reaches only directions some microfacet can refract into. Tested
    // first because the probabilities below sum to 1.0 only to float precision.
    if (lobes.msTransmit > 0.0F &&
        lobeU >= lobes.specular + lobes.diffuse + lobes.msReflect + lobes.msReflectTransmissive + lobes.transmit) {
        glm::vec3 wi = sampleMsTransmit(lobes.transmitShape, sampler.next2D());
        wi.z = -wi.z;
        return weigh(wi, LobeType::Transmission);
    }

    if (lobes.transmit <= 0.0F) {
        return std::nullopt;
    }

    // Smooth specular transmission (delta lobe): Snell, with TIR already folded into lobes.transmit, whose (1-F)
    // energy is exactly 0 past the critical angle since both sites decide it with the same cos2Transmitted predicate.
    const float eta = lobes.etaI / lobes.etaT;
    const float cos2ThetaT = cos2Transmitted(wo.z, eta);
    if (cos2ThetaT < 0.0F) {
        return std::nullopt;
    }
    const float cosThetaT = std::sqrt(cos2ThetaT);
    const glm::vec3 wt(-eta * wo.x, -eta * wo.y, -cosThetaT);
    // Non-symmetric radiance-compression factor for camera-originated transport (Veach 1997 sec. 5.2): eta^2 =
    // (etaI/etaT)^2, the squared ratio of the medium being left to the one being entered.
    const glm::vec3 throughput =
        params.transmissionTint * (lobes.transmitPhysicalValue / lobes.transmit) * (eta * eta);
    // pdf 0: a delta lobe has no density for NEE to double-count against, which is exactly the test path_tracer.cpp's MIS weighting makes.
    return BsdfSample{glm::vec3(wt.x, wt.y, wt.z * sign), throughput, LobeType::Transmission, 0.0F};
}

}  // namespace pathtracer::scene
