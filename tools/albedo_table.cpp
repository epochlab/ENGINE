// Offline generator for src/scene/albedo_table.inc, the Kulla-Conty energy tables bsdf.cpp bakes in ("Revisiting
// Physically Based Shading at Imageworks", SIGGRAPH 2017 Course Notes). Same standalone-CLI convention as the other
// tools, and grouped with bluenoise_mask/gltf_tangent rather than the validate tools: it produces a committed artifact,
// it does not check one, so it stays out of the ctest loop.
// This used to run at every process start (bsdf.cpp's `const AlbedoTable kAlbedo = buildAlbedoTable()`), which sized
// the grid and the quadrature by startup latency rather than by the accuracy the energy tests need. Offline that bound
// is gone, so the reflect side is resolved to the point where E is an instrument rather than a floor -- README section
// 5.1 wants F_avg pinned against a 2.0e-4 signal, and the old table's own error was ~1.5e-3; this reports 1.1e-6 on
// the cosine-weighted means and 3.0e-5 on the directional grid.
// Determinism is why this target is built with neither -march=native nor IPO, unlike every other tool here: the output
// is committed source, so it must reproduce bit-for-bit on any machine, and FMA contraction is free to differ. The bake
// runs in seconds, so there is nothing to buy back.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <atomic>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

namespace {

constexpr double kPi = 3.14159265358979323846;

// Must match bsdf.cpp's roughness floor: the table row for perceptual roughness r stores the albedo of the lobe that
// actually ships at r, which is alpha = max(r*r, kMinAlpha), not of an unclamped alpha the shading never evaluates.
constexpr float kMinAlpha = 0.02F * 0.02F;

// Reflect side. 128 rather than the 32 that fit in a startup budget: the bilinear error of the stored grid is a second
// error source beside the quadrature's, and at 32 it is the same order (~1e-3 near grazing, where E climbs steeply).
constexpr int kAlbedoRes = 128;

// Transmit side, unchanged at 32. It carries a third eta axis, so sharing the reflect side's 128 would take r and t to
// 128*128*16 = 262144 floats each -- a multi-megabyte .inc for an axis nothing on the roadmap is waiting on.
constexpr int kTransmitRes = 32;
constexpr int kEtaRes = 16;
constexpr double kEtaMin = 1.0 / 2.5;  // exiting a 2.5-ior medium; the reciprocal end is entering one
constexpr double kEtaMax = 2.5;

// Stratified midpoint samples per axis on the transmit side, 16*16 = 256 per cell, the count this table has always
// used. Deliberately NOT raised with the move offline, though it is now free: at 128 the escape total at roughness 1
// falls 0.00165 (measured 0.81216 vs 0.81381 at eta index 5), the compensation returns that much more, and
// integrator_validate's white slab goes 1.02428 -> 1.03017, past its 0.03 band. The slab was already 2.4% over, so
// what the coarse table was doing was partly funding that error, not avoiding it -- the defect is in the transmit
// compensation, not here, and it is README section 5.1's rough-transmission item to fix. Overridable so that
// measurement reproduces.
int gTransmitSamples = 16;

double smithLambda(double ndotV, double alpha) {
    const double ndotV2 = std::max(ndotV * ndotV, 1e-8);
    const double tan2 = std::max(0.0, 1.0 - ndotV2) / ndotV2;
    return 0.5 * (-1.0 + std::sqrt(1.0 + (alpha * alpha * tan2)));
}

double smithG2(double ndotV, double ndotL, double alpha) {
    return 1.0 / (1.0 + smithLambda(ndotV, alpha) + smithLambda(ndotL, alpha));
}

// --- Reflect side: exact-domain Gauss-Legendre, not Monte Carlo.
//
// The quantity is the directional albedo of the single-scattering GGX lobe with Fresnel forced to 1, the fraction of
// energy smithG2 lets through, so 1-E is exactly what the multiple-scattering lobe must return. It depends on nothing
// but (mu, alpha): Fresnel, metallic, baseColor and lobe-selection probabilities are all applied by the caller.
//
// Sampling it (VNDF draws, discarding wi.z <= 0) puts a jump discontinuity -- the horizon -- inside the integration
// domain, which caps any quadrature at first order in the sample count no matter how smooth the rest of the integrand
// is. That, not the sample budget, is what held the old table at ~1.5e-3. The fix is to integrate over a domain whose
// boundary IS the horizon, and both halves of that are available in closed form:
//
//   1. Measure. GGX NDF sampling (Walter et al. 2007, "Microfacet Models for Refraction through Rough Surfaces",
//      eq. 35) is tan(theta_h) = alpha*tan(psi) with u = sin^2(psi) uniform, and its density is exactly D(h)*cos(h),
//      so D(h) cos(theta_h) dw_h = du dphi / (2*pi) = sin(psi) cos(psi) dpsi dphi / pi. Changing variable to psi
//      absorbs the peak D would otherwise have -- at alpha = 4e-4 that peak is 4e-4 radians wide and no fixed grid in
//      theta_h could resolve it -- and leaves an integrand that is analytic in psi.
//
//   2. Domain. With wo = (sin tv, 0, cos tv) and h at (theta_h, phi), both cosines collapse to a single harmonic:
//      wo.h = R cos(theta_h - d) and wi.z = R cos(2 theta_h - d), where R = hypot(sin tv cos phi, cos tv) and
//      d = atan2(sin tv cos phi, cos tv). So the horizon clip wi.z > 0 is exactly theta_h < (d + pi/2)/2, one bound
//      per phi, and wo.h > 0 holds throughout it. Nothing is discarded, because nothing invalid is ever evaluated.
//
// The integrand that remains -- (wo.h/cos theta_h) * G2 * sin psi cos psi -- is analytic on the closed interval
// (G2 vanishes linearly at the upper limit, where wi.z does), so Gauss-Legendre converges geometrically and the node
// count below is a measured choice, reported against a doubled rule on every run.
//
// Split by Schlick's form F(c) = f0*(1 - (1-c)^5) + (1-c)^5 so one table serves any f0 (the standard environment-BRDF
// split): Ess(mu, f0) = f0*a + b, and with f0 = 1 that collapses to a + b = E, the Fresnel-free albedo above.
struct Split {
    double a;
    double b;
};

// Gauss-Legendre nodes and weights mapped to [0,1], by Newton iteration on P_n through Bonnet's recurrence (Press et
// al., Numerical Recipes 3rd ed., sec. 4.6.1). Weights sum to 1, so a node array doubles as the [0,1] average.
struct GaussLegendre {
    std::vector<double> node;
    std::vector<double> weight;
};

GaussLegendre gaussLegendre(int n) {
    GaussLegendre quadrature{std::vector<double>(static_cast<std::size_t>(n)),
                              std::vector<double>(static_cast<std::size_t>(n))};
    for (int i = 0; i < n; ++i) {
        double x = std::cos(kPi * (i + 0.75) / (n + 0.5));
        double derivative = 0.0;
        for (int iteration = 0; iteration < 100; ++iteration) {
            double p0 = 1.0;
            double p1 = 0.0;
            for (int k = 0; k < n; ++k) {
                const double p2 = p1;
                p1 = p0;
                p0 = ((((2.0 * k) + 1.0) * x * p1) - (k * p2)) / (k + 1.0);
            }
            derivative = n * ((x * p0) - p1) / ((x * x) - 1.0);
            const double step = p0 / derivative;
            x -= step;
            if (std::abs(step) <= 1e-16) {
                break;
            }
        }
        quadrature.node[static_cast<std::size_t>(i)] = 0.5 * (1.0 - x);
        quadrature.weight[static_cast<std::size_t>(i)] =
            1.0 / ((1.0 - (x * x)) * derivative * derivative);
    }
    return quadrature;
}

Split reflectAlbedo(double mu, double alpha, const GaussLegendre& phiRule, const GaussLegendre& psiRule) {
    const double sinTv = std::sqrt(std::max(0.0, 1.0 - (mu * mu)));
    double a = 0.0;
    double b = 0.0;
    // phi is even about 0, so half the circle is integrated and the result doubled by the scale below. Two panels
    // meeting at pi/2, where cos(phi) changes sign: d(phi) sweeps the full -pi/2..pi/2 of its range within |cos phi| <
    // mu there, a boundary layer that narrows with mu and would otherwise be missed entirely by the grazing rows. As a
    // panel endpoint it is resolved instead, Gauss-Legendre placing its outermost node O(1/n^2) from the edge.
    for (int panel = 0; panel < 2; ++panel) {
        const double phiBase = 0.5 * kPi * panel;
        for (std::size_t p = 0; p < phiRule.node.size(); ++p) {
            const double horizontal = sinTv * std::cos(phiBase + (0.5 * kPi * phiRule.node[p]));
            const double radius = std::sqrt((horizontal * horizontal) + (mu * mu));
            const double delta = std::atan2(horizontal, mu);
            const double psiMax = std::atan(std::tan(0.5 * (delta + (0.5 * kPi))) / alpha);
            for (std::size_t s = 0; s < psiRule.node.size(); ++s) {
                const double psi = psiMax * psiRule.node[s];
                const double thetaH = std::atan(alpha * std::tan(psi));
                const double woDotH = radius * std::cos(thetaH - delta);
                const double wiZ = radius * std::cos((2.0 * thetaH) - delta);
                const double weight = phiRule.weight[p] * psiRule.weight[s] * psiMax *
                                       (woDotH / std::cos(thetaH)) * smithG2(mu, wiZ, alpha) *
                                       std::sin(psi) * std::cos(psi);
                const double fc = std::pow(std::clamp(1.0 - woDotH, 0.0, 1.0), 5.0);
                a += weight * (1.0 - fc);
                b += weight * fc;
            }
        }
    }
    return {a / mu, b / mu};
}

// --- Transmit side: the stratified midpoint VNDF quadrature this table has always used, moved verbatim.
//
// Its energy curve is not the reflect side's: the below-horizon reflections that drive E down are the valid side for
// refraction, so far fewer samples are discarded (measured 0.559 combined vs 0.307 reflect-only at roughness 1.0).
// Unlike the reflect side it genuinely depends on eta (G2 uses the refracted |wi.z|, and TIR gates validity), so the
// Schlick split cannot factor it out. One axis in log(eta) covers entering and exiting, since the two are reciprocals.
// It keeps the sampled scheme rather than the exact-domain rule above because its valid set is the intersection of the
// horizon clip, the TIR cone and wt.z < 0, and only the first of those has a closed-form boundary; the sample count is
// simply raised, which is free offline. Sampling and refraction stay in float glm, the shipped arithmetic.
constexpr float kFloatPi = 3.14159265F;

float smithG1f(float ndotV, float alpha) {
    return static_cast<float>(1.0 / (1.0 + smithLambda(ndotV, alpha)));
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

// Heitz 2018 VNDF sampling. wo.z > 0 required.
glm::vec3 sampleGGXVNDF(const glm::vec3& wo, float alpha, glm::vec2 u) {
    const glm::vec3 vh = glm::normalize(glm::vec3(alpha * wo.x, alpha * wo.y, wo.z));
    const float lensq = (vh.x * vh.x) + (vh.y * vh.y);
    const glm::vec3 t1 = lensq > 0.0F ? glm::vec3(-vh.y, vh.x, 0.0F) * (1.0F / std::sqrt(lensq))
                                       : glm::vec3(1.0F, 0.0F, 0.0F);
    const glm::vec3 t2 = glm::cross(vh, t1);
    const float r = std::sqrt(u.x);
    const float phi = 2.0F * kFloatPi * u.y;
    const float t1p = r * std::cos(phi);
    float t2p = r * std::sin(phi);
    const float s = 0.5F * (1.0F + vh.z);
    t2p = ((1.0F - s) * std::sqrt(std::max(0.0F, 1.0F - (t1p * t1p)))) + (s * t2p);
    const glm::vec3 nh = (t1p * t1) + (t2p * t2) +
                          (std::sqrt(std::max(0.0F, 1.0F - (t1p * t1p) - (t2p * t2p))) * vh);
    return glm::normalize(glm::vec3(alpha * nh.x, alpha * nh.y, std::max(0.0F, nh.z)));
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

// log-spaced so eta and 1/eta are symmetric about index kEtaRes/2.
double etaAtIndex(int index) {
    const double u = static_cast<double>(index) / static_cast<double>(kEtaRes - 1);
    return std::exp(std::log(kEtaMin) + (u * (std::log(kEtaMax) - std::log(kEtaMin))));
}

struct EscapeSums {
    std::array<double, kEtaRes> reflect;
    std::array<double, kEtaRes> transmit;
};

// Escaping fraction of a dielectric interface, split into the reflected and transmitted shares. Both use exact
// dielectric Fresnel rather than the Schlick split above: inside the total-internal-reflection cone exact Fresnel is
// 1.0 while Schlick reads ~0.1, so no rescale of a Schlick-basis number can stand in for it, and the escape budget
// would under-count the reflected share by the whole TIR cone. A facet reflects with probability F and refracts with
// 1-F, so the two shares are Fresnel-weighted complements of one throughput, never independent quantities.
EscapeSums escapeAlbedo(float mu, float alpha) {
    const glm::vec3 wo(std::sqrt(std::max(0.0F, 1.0F - (mu * mu))), 0.0F, mu);
    const float g1 = smithG1f(mu, alpha);
    EscapeSums sums{};
    for (int i = 0; i < gTransmitSamples; ++i) {
        for (int j = 0; j < gTransmitSamples; ++j) {
            const glm::vec2 u((static_cast<float>(i) + 0.5F) / gTransmitSamples,
                               (static_cast<float>(j) + 0.5F) / gTransmitSamples);
            const glm::vec3 nh = sampleGGXVNDF(wo, alpha, u);
            const glm::vec3 wi = glm::reflect(-wo, nh);
            // Same visible normal, reflected and refracted: the VNDF sample is the expensive part and is shared across
            // every eta, so the third axis costs only the refraction.
            const float woDotNh = glm::dot(wo, nh);
            for (int ei = 0; ei < kEtaRes; ++ei) {
                const auto eta = static_cast<float>(etaAtIndex(ei));
                const float fresnel = fresnelDielectric(woDotNh, eta, 1.0F);
                if (wi.z > 0.0F) {
                    sums.reflect[static_cast<std::size_t>(ei)] +=
                        (static_cast<double>(smithG2(mu, wi.z, alpha)) / std::max(g1, 1e-8F)) * fresnel;
                }
                glm::vec3 wt;
                if (refractAbout(wo, nh, eta, wt) && wt.z < 0.0F) {
                    sums.transmit[static_cast<std::size_t>(ei)] +=
                        (static_cast<double>(smithG2(mu, -wt.z, alpha)) / std::max(g1, 1e-8F)) *
                        (1.0F - fresnel);
                }
            }
        }
    }
    const auto cells = static_cast<double>(gTransmitSamples) * gTransmitSamples;
    for (int ei = 0; ei < kEtaRes; ++ei) {
        sums.reflect[static_cast<std::size_t>(ei)] /= cells;
        sums.transmit[static_cast<std::size_t>(ei)] /= cells;
    }
    return sums;
}

// Rows are independent and each writes only its own slice of the table, so the split is a pure speedup with no
// effect on the values -- the whole point of moving the bake offline is that it can afford the node counts.
template <typename Row>
void parallelRows(int rows, Row row) {
    const unsigned workers = std::max(1U, std::thread::hardware_concurrency());
    std::atomic<int> next{0};
    std::vector<std::thread> pool;
    pool.reserve(workers);
    for (unsigned w = 0; w < workers; ++w) {
        pool.emplace_back([&] {
            for (int i = next++; i < rows; i = next++) {
                row(i);
            }
        });
    }
    for (std::thread& worker : pool) {
        worker.join();
    }
}

// --- Table assembly.

struct AlbedoTable {
    std::vector<float> a;  // [roughnessIndex][muIndex], kAlbedoRes^2
    std::vector<float> b;
    std::vector<float> aavg;  // cosine-weighted means, 2*integral(.(mu)*mu dmu)
    std::vector<float> bavg;
    std::vector<float> r;  // [roughnessIndex][muIndex][etaIndex], kTransmitRes^2 * kEtaRes
    std::vector<float> t;
    std::vector<float> ravg;
    std::vector<float> tavg;
    std::vector<float> msDensity;  // [roughnessIndex][muIndex], the reflected multiple-scattering lobe's own shape
    std::vector<float> msCdf;
};

// mu = 0 is a degenerate view direction (wo lies in the surface plane); nudge off it, as the runtime lookup's own
// clamp does. Only the mu = 0 grid column is affected and nothing samples it at full weight.
double gridMu(int index, int resolution) {
    return std::max(static_cast<double>(index) / static_cast<double>(resolution - 1), 1e-3);
}

double gridAlpha(int index, int resolution) {
    const double roughness = static_cast<double>(index) / static_cast<double>(resolution - 1);
    return std::max(roughness * roughness, static_cast<double>(kMinAlpha));
}

// Directional tables at the stored grid, plus their cosine-weighted means. The mean is a Gauss-Legendre integral over
// mu in its own right, not a trapezoid over the stored columns: Eavg is the denominator of the multiple-scattering
// normalisation, so its error enters every compensated shade directly rather than being smoothed by interpolation.
void buildReflect(AlbedoTable& table, int phiNodes, int psiNodes, int muNodes) {
    const GaussLegendre phiRule = gaussLegendre(phiNodes);
    const GaussLegendre psiRule = gaussLegendre(psiNodes);
    const GaussLegendre muRule = gaussLegendre(muNodes);
    table.a.assign(static_cast<std::size_t>(kAlbedoRes) * kAlbedoRes, 0.0F);
    table.b.assign(static_cast<std::size_t>(kAlbedoRes) * kAlbedoRes, 0.0F);
    table.aavg.assign(kAlbedoRes, 0.0F);
    table.bavg.assign(kAlbedoRes, 0.0F);
    // One roughness row per worker. Rows share no accumulator and each writes only its own slice, so the result is
    // identical to the serial order -- the determinism the committed artifact needs survives the threading.
    parallelRows(kAlbedoRes, [&](int ri) {
        const double alpha = gridAlpha(ri, kAlbedoRes);
        for (int mi = 0; mi < kAlbedoRes; ++mi) {
            const Split split = reflectAlbedo(gridMu(mi, kAlbedoRes), alpha, phiRule, psiRule);
            table.a[static_cast<std::size_t>((ri * kAlbedoRes) + mi)] = static_cast<float>(split.a);
            table.b[static_cast<std::size_t>((ri * kAlbedoRes) + mi)] = static_cast<float>(split.b);
        }
        double aMean = 0.0;
        double bMean = 0.0;
        for (std::size_t k = 0; k < muRule.node.size(); ++k) {
            const double mu = muRule.node[k];
            const Split split = reflectAlbedo(mu, alpha, phiRule, psiRule);
            aMean += muRule.weight[k] * 2.0 * split.a * mu;
            bMean += muRule.weight[k] * 2.0 * split.b * mu;
        }
        table.aavg[static_cast<std::size_t>(ri)] = static_cast<float>(aMean);
        table.bavg[static_cast<std::size_t>(ri)] = static_cast<float>(bMean);
    });
}

void buildTransmit(AlbedoTable& table) {
    table.r.assign(static_cast<std::size_t>(kTransmitRes) * kTransmitRes * kEtaRes, 0.0F);
    table.t.assign(static_cast<std::size_t>(kTransmitRes) * kTransmitRes * kEtaRes, 0.0F);
    table.ravg.assign(static_cast<std::size_t>(kTransmitRes) * kEtaRes, 0.0F);
    table.tavg.assign(static_cast<std::size_t>(kTransmitRes) * kEtaRes, 0.0F);
    parallelRows(kTransmitRes, [&](int ri) {
        const auto alpha = static_cast<float>(gridAlpha(ri, kTransmitRes));
        std::array<double, kEtaRes> rWeighted{};
        std::array<double, kEtaRes> tWeighted{};
        for (int mi = 0; mi < kTransmitRes; ++mi) {
            const double mu = gridMu(mi, kTransmitRes);
            const EscapeSums sums = escapeAlbedo(static_cast<float>(mu), alpha);
            // Trapezoid over the mu axis: the grid is edge-aligned, so the two endpoints span half a cell each and the
            // step is 1/(kTransmitRes-1), not 1/kTransmitRes. First order, matching the sampled quadrature it averages.
            const double endpoint = (mi == 0 || mi == kTransmitRes - 1) ? 0.5 : 1.0;
            for (int ei = 0; ei < kEtaRes; ++ei) {
                const auto e = static_cast<std::size_t>(ei);
                const double r = sums.reflect[e];
                const double t = sums.transmit[e];
                table.r[static_cast<std::size_t>((((ri * kTransmitRes) + mi) * kEtaRes) + ei)] =
                    static_cast<float>(r);
                table.t[static_cast<std::size_t>((((ri * kTransmitRes) + mi) * kEtaRes) + ei)] =
                    static_cast<float>(t);
                rWeighted[e] += endpoint * 2.0 * r * mu;
                tWeighted[e] += endpoint * 2.0 * t * mu;
            }
        }
        for (int ei = 0; ei < kEtaRes; ++ei) {
            const auto e = static_cast<std::size_t>(ei);
            table.ravg[static_cast<std::size_t>((ri * kEtaRes) + ei)] =
                static_cast<float>(rWeighted[e] / (kTransmitRes - 1));
            table.tavg[static_cast<std::size_t>((ri * kEtaRes) + ei)] =
                static_cast<float>(tWeighted[e] / (kTransmitRes - 1));
        }
    });
}

// --- Sampling shape for the reflected multiple-scattering lobe (README section 5.1's exact (1-E)cos sampler).
// That lobe's value is fms*(1-E(mu_o))*(1-E(mu_i))/(pi*(1-Eavg)), so its own zero-variance density is
// (1-E(mu_i))*cos / (pi*(1-Eavg)), and cosine sampling pays the ratio (1-E(mu_i))/(1-Eavg) as weight variance.
// Measured off the table above rather than assumed, and the answer inverts the intuition: the ratio is harmless at
// high roughness (relative variance 0.029 at roughness 1, 0.042 at 0.5) and severe at low, where 1-Eavg and 1-E(mu)
// are both small and their quotient is not -- +1.66 at roughness 0.25 and +17.3 at 0.126, with weights reaching 92x.
// Stored as a piecewise-linear density over mu with its exact prefix integrals beside it, NOT as the mu-at-quantile
// inverse CDF the roadmap names. A sampler and a pdf must agree on the density itself; a quantile table defines it
// only implicitly, leaving the pdf to reconstruct it by differencing and biasing the estimator by whatever the two
// then disagree about. With the density stored, bsdf.cpp inverts it exactly (one quadratic per segment) and evaluates
// the same interpolant for its pdf, so the pair is consistent by construction at any resolution and the remaining
// approximation is only how closely the interpolant tracks (1-E)*mu -- variance, never bias.
// Same mu grid as the albedo table, so the shape is built from the E values the shading actually reads, with no
// second interpolation between them.
void buildMultipleScatteringShape(AlbedoTable& table) {
    const double step = 1.0 / (kAlbedoRes - 1);
    table.msDensity.assign(static_cast<std::size_t>(kAlbedoRes) * kAlbedoRes, 0.0F);
    table.msCdf.assign(static_cast<std::size_t>(kAlbedoRes) * kAlbedoRes, 0.0F);
    for (int ri = 0; ri < kAlbedoRes; ++ri) {
        std::vector<double> raw(kAlbedoRes);
        for (int mi = 0; mi < kAlbedoRes; ++mi) {
            const auto index = static_cast<std::size_t>((ri * kAlbedoRes) + mi);
            const double deficit = 1.0 - (table.a[index] + table.b[index]);
            raw[static_cast<std::size_t>(mi)] = deficit * mi * step;
        }
        double norm = 0.0;
        for (int mi = 0; mi + 1 < kAlbedoRes; ++mi) {
            norm += 0.5 * (raw[static_cast<std::size_t>(mi)] + raw[static_cast<std::size_t>(mi) + 1]) * step;
        }
        // 1-E is strictly positive at every roughness the table reaches (measured minimum 1.4e-7, at roughness 0), so this is a bake-time assertion and not a shading-time guard: a non-positive row would mean the albedo table itself is wrong, and silently substituting cosine would hide that.
        if (!(norm > 0.0)) {
            std::cerr << "albedo_table: roughness row " << ri << " has non-positive energy deficit " << norm
                      << " -- the reflect table is wrong, not this shape\n";
            std::exit(EXIT_FAILURE);
        }
        double cdf = 0.0;
        for (int mi = 0; mi < kAlbedoRes; ++mi) {
            const auto index = static_cast<std::size_t>((ri * kAlbedoRes) + mi);
            const double density = raw[static_cast<std::size_t>(mi)] / norm;
            if (mi > 0) {
                cdf += 0.5 * (table.msDensity[index - 1] + density) * step;
            }
            table.msDensity[index] = static_cast<float>(density);
            table.msCdf[index] = static_cast<float>(mi == kAlbedoRes - 1 ? 1.0 : cdf);
        }
    }
}

// Largest disagreement between the shipped reflect rule and one at doubled order, over the stored grid and the means.
// The rule's own error, measured rather than asserted: the integrand is analytic on the domain built for it, so the
// doubled rule is exact to well past float32 and the printed number is the committed table's accuracy.
struct Residual {
    double value;
    const char* channel;
    int roughnessIndex;
    int muIndex;
};

Residual verifyReflect(const AlbedoTable& table, int phiNodes, int psiNodes, int muNodes) {
    AlbedoTable reference;
    buildReflect(reference, phiNodes * 2, psiNodes * 2, muNodes * 2);
    Residual worst{0.0, "a", 0, 0};
    const std::array<std::tuple<const char*, const std::vector<float>*, const std::vector<float>*>, 4>
        channels = {{{"a", &table.a, &reference.a},
                     {"b", &table.b, &reference.b},
                     {"aavg", &table.aavg, &reference.aavg},
                     {"bavg", &table.bavg, &reference.bavg}}};
    for (const auto& [name, shipped, exact] : channels) {
        Residual channelWorst{0.0, name, 0, 0};
        for (std::size_t i = 0; i < shipped->size(); ++i) {
            const double delta = std::abs(static_cast<double>((*shipped)[i]) -
                                           static_cast<double>((*exact)[i]));
            if (delta > channelWorst.value) {
                const int stride = shipped->size() == table.aavg.size() ? 1 : kAlbedoRes;
                channelWorst = {delta, name, static_cast<int>(i) / stride,
                                 stride == 1 ? -1 : static_cast<int>(i) % kAlbedoRes};
            }
        }
        std::cout << "albedo_table: " << name << " residual " << channelWorst.value
                  << " at roughnessIndex " << channelWorst.roughnessIndex << " muIndex "
                  << channelWorst.muIndex << "\n";
        if (channelWorst.value > worst.value) {
            worst = channelWorst;
        }
    }
    return worst;
}

// %.9g is FLT_DECIMAL_DIG digits, which round-trips float32 exactly, so the committed values are the computed ones.
// It drops the point on a whole number though ("1"), and "1F" is not a literal, so one is restored where needed.
void writeArray(std::ofstream& out, const char* name, const std::vector<float>& values) {
    out << "\nconstexpr std::array<float, " << values.size() << "> " << name << " = {{";
    std::array<char, 32> buffer{};
    for (std::size_t i = 0; i < values.size(); ++i) {
        std::snprintf(buffer.data(), buffer.size(), "%.9g", static_cast<double>(values[i]));
        const std::string literal(buffer.data());
        out << (i % 8 == 0 ? "\n    " : " ") << literal
            << (literal.find_first_of(".e") == std::string::npos ? ".0F," : "F,");
    }
    out << "\n}};\n";
}

bool writeInc(const std::string& path, const AlbedoTable& table, double residual) {
    std::ofstream out(path);
    if (!out) {
        std::cerr << "albedo_table: cannot write " << path << "\n";
        return false;
    }
    std::array<char, 32> etaMin{};
    std::array<char, 32> etaMax{};
    std::snprintf(etaMin.data(), etaMin.size(), "%.9g", kEtaMin);
    std::snprintf(etaMax.data(), etaMax.size(), "%.9g", kEtaMax);
    out << "// Generated by tools/albedo_table.cpp -- do not edit. Regenerate with:\n"
           "//   ./build/albedo_table --out src/scene/albedo_table.inc\n"
           "// Kulla-Conty energy tables, indexed by perceptual roughness rather than alpha: E is far better\n"
           "// distributed in sqrt(alpha), and it is what callers already hold. Both grids are edge-aligned, so\n"
           "// roughness 0 and mu 1 are exact table entries and bsdf.cpp's lookups can interpolate on k/(res-1).\n"
           "// Reflect side (a, b, aavg, bavg) is exact-domain Gauss-Legendre, residual "
        << residual << " against a doubled rule.\n"
           "// Transmit side (r, t, ravg, tavg) is stratified midpoint VNDF at "
        << gTransmitSamples << "^2 samples per cell.\n"
           "// kMsReflectDensity/kMsReflectCdf are the reflected multiple-scattering lobe's sampling shape: a\n"
           "// piecewise-linear density over mu, proportional to (1-E(mu))*mu and normalised to 1, with its exact\n"
           "// prefix integrals. bsdf.cpp inverts the first and evaluates it for the matching pdf.\n";
    out << "\nconstexpr int kAlbedoRes = " << kAlbedoRes << ";\n"
        << "constexpr int kTransmitRes = " << kTransmitRes << ";\n"
        << "constexpr int kEtaRes = " << kEtaRes << ";\n"
        << "constexpr float kEtaMin = " << etaMin.data() << "F;\n"
        << "constexpr float kEtaMax = " << etaMax.data() << "F;\n";
    writeArray(out, "kAlbedoA", table.a);
    writeArray(out, "kAlbedoB", table.b);
    writeArray(out, "kAlbedoAvgA", table.aavg);
    writeArray(out, "kAlbedoAvgB", table.bavg);
    writeArray(out, "kEscapeReflect", table.r);
    writeArray(out, "kEscapeTransmit", table.t);
    writeArray(out, "kEscapeAvgReflect", table.ravg);
    writeArray(out, "kEscapeAvgTransmit", table.tavg);
    writeArray(out, "kMsReflectDensity", table.msDensity);
    writeArray(out, "kMsReflectCdf", table.msCdf);
    return out.good();
}

}  // namespace

int main(int argc, char** argv) {
    std::string outPath = "src/scene/albedo_table.inc";
    // Measured against the doubled rule, which is what --verify reports: at 96 the mean channels agree to 1.1e-6 and
    // the directional ones to 3.0e-5, that worst case confined to the mu = 0 column every consumer weights by cos.
    int phiNodes = 96;
    int psiNodes = 96;
    int muNodes = 96;
    for (int i = 1; i < argc; ++i) {
        const bool hasValue = i + 1 < argc;
        if (std::strcmp(argv[i], "--out") == 0 && hasValue) {
            outPath = argv[++i];
        } else if (std::strcmp(argv[i], "--nodes") == 0 && hasValue) {
            phiNodes = psiNodes = muNodes = std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--samples") == 0 && hasValue) {
            gTransmitSamples = std::atoi(argv[++i]);
        } else {
            std::cerr << "albedo_table: unknown or incomplete argument '" << argv[i]
                      << "'\nusage: albedo_table [--out path.inc] [--nodes N] [--samples N]\n";
            return EXIT_FAILURE;
        }
    }
    if (phiNodes < 2 || gTransmitSamples < 2) {
        std::cerr << "albedo_table: --nodes and --samples must be at least 2\n";
        return EXIT_FAILURE;
    }

    AlbedoTable table;
    buildReflect(table, phiNodes, psiNodes, muNodes);
    const Residual residual = verifyReflect(table, phiNodes, psiNodes, muNodes);
    buildTransmit(table);
    buildMultipleScatteringShape(table);
    if (!writeInc(outPath, table, residual.value)) {
        return EXIT_FAILURE;
    }
    std::cout << "albedo_table: wrote " << outPath << " (reflect " << kAlbedoRes << "x" << kAlbedoRes
              << " at " << phiNodes << " nodes, residual " << residual.value << " in " << residual.channel
              << " at roughnessIndex " << residual.roughnessIndex << " muIndex " << residual.muIndex
              << "; transmit " << kTransmitRes
              << "x" << kTransmitRes << "x" << kEtaRes << " at " << gTransmitSamples << "^2 samples)\n";
    return EXIT_SUCCESS;
}
