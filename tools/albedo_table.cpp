// Offline generator for src/scene/albedo_table.inc, the Kulla-Conty energy tables bsdf.cpp bakes in ("Revisiting
// Physically Based Shading at Imageworks", SIGGRAPH 2017 Course Notes). Same standalone-CLI convention as the other
// tools, and grouped with bluenoise_mask/gltf_tangent rather than the validate tools: it produces a committed artifact,
// it does not check one, so it stays out of the ctest loop.
// This used to run at every process start (bsdf.cpp's `const AlbedoTable kAlbedo = buildAlbedoTable()`), which sized
// the grid and the quadrature by startup latency rather than by the accuracy the energy tests need. Offline that bound
// is gone, so the reflect side is resolved to the point where E is an instrument rather than a floor -- bsdf_validate's
// energy tests pin F_avg against a 2.0e-4 signal, and the old table's own error was ~1.5e-3; this reports 1.1e-6 on
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

#include "pathtracer/scene/fresnel_dielectric.h"

namespace {

// The shading path's own dielectric interface, so the table is baked against exactly what reads it.
using pathtracer::scene::cos2Transmitted;
using pathtracer::scene::fresnelDielectric;

constexpr double kPi = 3.14159265358979323846;

// Must match bsdf.cpp's roughness floor: the table row for perceptual roughness r stores the albedo of the lobe that
// actually ships at r, which is alpha = max(r*r, kMinAlpha), not of an unclamped alpha the shading never evaluates.
constexpr float kMinAlpha = 0.02F * 0.02F;

// Reflect side, three resolutions rather than one square grid, each sized by what checkAlbedoTableInterpolation measures on that axis (tools/bsdf_validate.cpp). The bilinear read of the stored grid is a second error source beside the quadrature's, and it is the one that dominates: at 128x128 uniform it was 4.2e-2 in the first mu cell against a 3.0e-5 quadrature residual.
// Roughness 256: that axis' error is a smooth single-signed hump, not a boundary layer, so only resolution touches it. At 128 it measured 1.1e-3, and it enters every shade as a bias rather than as noise a render averages out. Quartering it puts the axis under the quadrature's own residual, which is the stopping criterion -- past that the stored values are the limit and further rows buy nothing.
// mu 256, uniform in sqrt(mu) (reflectMu). The warp is what the grazing layer needs: a uniform mu axis puts that whole layer inside one cell for roughness below ~0.1, where the error saturates at the layer's full amplitude (measured 4.2e-2), and the layer's width is ~alpha in mu and so ~sqrt(alpha) in the warped axis, which is what lets one fixed warp serve every roughness row rather than an alpha-dependent one.
// The doubling is what the warp COSTS, measured rather than assumed. A warp moves cells, it does not add them: cell width in mu becomes 2 sqrt(mu)/(res-1), so at mu = 0.4 the cells are 1.27x a uniform axis' and the error there rose with them. At 128 that turned checkCoatFresnelAvg's worst row from 5.1e-5 into 8.7e-5 -- a regression on the very instrument this table is read by, whose worst rows all sit at mu 0.4. 256 buys the working band back at 0.63x the original uniform width and quarters the grazing layer at the same time.
// An alpha-dependent warp was measured and rejected instead of assumed: Smith G1 as the axis coordinate resolves each row's own layer exactly, but at alpha 0.25 it compresses mu in [0.3, 1] into 12% of the axis, for 5.3e-4 against a uniform axis' 1.2e-5 in exactly the band that matters -- two extra sqrt in a hot lookup to buy a 40x regression where E is not flat.
constexpr int kAlbedoRoughnessRes = 256;
constexpr int kAlbedoMuRes = 256;

// The reflected multiple-scattering lobe's sampling grid, uniform in mu and deliberately its own constant rather than kAlbedoMuRes: bsdf.cpp's piecewise-linear inversion depends on one step width, so this axis cannot carry the albedo table's warp, and a later change to that warp's resolution must not silently resize a sampling density. The transmit side already keeps its msTransmit shape uniform for the same reason.
constexpr int kMsReflectMuRes = 128;

// Transmit side, sized by the energy closure it buys: linear interpolation in mu (the steep grazing rise) and in eta (curvature through the TIR onset) sets the per-vertex error once the quadrature is converged.
// Measured on a white ior-1.5 interface: 32 mu nodes lose 2% at mu 0.02, and 32 eta nodes lose 9e-4 midway between eta nodes against 1e-4 on them; 64 x 64 closes to within 3e-4. Roughness keeps 32 nodes, where node and midpoint already agree to that level.
// Uniform in mu, 64 nodes still left 1e-2 at mu 0.01 at roughness 0.15, where E climbs over mu ~ alpha; the escape tables' mu axis is uniform in sqrt(mu) (escapeMu).
constexpr int kTransmitRoughnessRes = 32;
constexpr int kTransmitMuRes = 64;
constexpr int kEtaRes = 64;
constexpr double kEtaMin = 1.0 / 2.5;  // exiting a 2.5-ior medium; the reciprocal end is entering one
constexpr double kEtaMax = 2.5;

// Gauss-Legendre nodes per panel on the transmit side, in phi and in each psi panel; verifyTransmit reports the table's residual against a doubled rule on every bake.
constexpr int kTransmitNodes = 48;

double smithRadical(double cosTheta, double alpha) {
    const double alpha2 = alpha * alpha;
    return std::sqrt(alpha2 + ((1.0 - alpha2) * cosTheta * cosTheta));
}

// Height-correlated G2 divided by cosO, 2 cosI/(cosI s(cosO) + cosO s(cosI)) (bsdf.cpp's smithVisibility times 4 cosI): no cosine divides, so it holds to cosO = 0, where it is 2/alpha.
// Both sides use it, and the reflect side now MUST. The lambda form it replaced there carries a max(cos^2, 1e-8) clamp, which was harmless while the reflect grid started at mu = 1e-3 (cos^2 = 1e-6, clear of it) and is not once reflectMu's node 1 is mu = 6.2e-5: the clamp would silently substitute a different cosine on the grazing rows with no diagnostic at all. It also supplies the exact grazing limit the warp's node 0 needs, which a form that divides by cosO cannot.
double smithG2OverCosO(double cosO, double cosI, double alpha) {
    return 2.0 * cosI / ((cosI * smithRadical(cosO, alpha)) + (cosO * smithRadical(cosI, alpha)));
}

// --- Reflect side: exact-domain Gauss-Legendre, not Monte Carlo.
//
// The quantity is the directional albedo of the single-scattering GGX lobe with Fresnel forced to 1, the fraction of
// energy G2 lets through, so 1-E is exactly what the multiple-scattering lobe must return. It depends on nothing
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
                                       (woDotH / std::cos(thetaH)) * smithG2OverCosO(mu, wiZ, alpha) *
                                       std::sin(psi) * std::cos(psi);
                const double fc = std::pow(std::clamp(1.0 - woDotH, 0.0, 1.0), 5.0);
                a += weight * (1.0 - fc);
                b += weight * fc;
            }
        }
    }
    // The 1/mu that used to divide out here is inside smithG2OverCosO, which is what lets mu = 0 be a real node.
    // It is never 0/0 there, by two separate arguments. On the first phi panel delta is +pi/2, so wiZ vanishes only at
    // psiMax, which Gauss-Legendre's strictly interior nodes never reach. On the second delta is -pi/2 and psiMax is
    // exactly 0, so every node sits at psi = 0 and the weight carries psiMax and sin(psi) as exact zero factors -- the
    // panel contributes nothing, which is correct: no facet reflects a grazing wo above the horizon on that side.
    return {a, b};
}

// --- Transmit side: Gauss-Legendre in the reflect side's NDF measure, panelled at the interface's own boundaries.
//
// Its energy curve is not the reflect side's: the below-horizon reflections that drive E down are the valid side for refraction (measured 0.559 combined vs 0.307 reflect-only at roughness 1.0).
// It depends on eta (G2 uses the refracted |wt.z|, and TIR gates validity), so the Schlick split cannot factor it out; one axis in log(eta) covers entering and exiting, since the two are reciprocals.
// Same measure as reflectAlbedo, D(h)cos(theta_h) dw = sin(psi)cos(psi) dpsi dphi / pi with tan(theta_h) = alpha*tan(psi), which flattens the peak and resolves the GGX slope tail; a stratified VNDF midpoint rule lumped that tail into its last stratum and converged first order (3.6e-3 at roughness 0.19, mu 1, eta 1.56).
// Same closed forms too: wo.h = R cos(theta_h - d) and wi.z = R cos(2 theta_h - d), so per phi the visibility bound, the reflection horizon and the TIR onset wo.h = sqrt(1 - 1/eta^2) are all exact theta_h breakpoints, the TIR circle's tangency R = sqrt(1 - 1/eta^2) is an exact phi breakpoint, and each panel between them is integrated on its own.
// That matters most at TIR, where 1-F has a square-root singularity no fixed rule resolves (3.9e-3 residual at a doubled rule without the split); the one boundary left unaligned, wt.z = 0, is a kink where G2 vanishes linearly.
// VNDF weight G1*(wo.h)*D/mu divided by G1 per escaping path leaves (wo.h)/(mu*cos(theta_h)) * G2 in this measure; Fresnel and the TIR predicate are the shipped fresnel_dielectric.h, so the table is baked against the interface it is shaded against.

// log-spaced so eta and 1/eta are symmetric about index kEtaRes/2.
double etaAtIndex(int index) {
    const double u = static_cast<double>(index) / static_cast<double>(kEtaRes - 1);
    return std::exp(std::log(kEtaMin) + (u * (std::log(kEtaMax) - std::log(kEtaMin))));
}

struct EscapeSums {
    std::array<double, kEtaRes> reflect;
    std::array<double, kEtaRes> transmit;
};

// Escaping fraction of a dielectric interface, split into the reflected and transmitted shares, with exact dielectric Fresnel: inside the TIR cone Fresnel is 1.0 where Schlick reads ~0.1, so no Schlick-basis rescale can stand in for it.
// A facet reflects with probability F and refracts with 1-F, so the two shares are Fresnel-weighted complements of one throughput, never independent quantities.
EscapeSums escapeAlbedo(double mu, double alpha, const GaussLegendre& rule) {
    const double sinTv = std::sqrt(std::max(0.0, 1.0 - (mu * mu)));
    const glm::dvec3 wo(sinTv, 0.0, mu);
    EscapeSums sums{};
    for (int ei = 0; ei < kEtaRes; ++ei) {
        const auto e = static_cast<std::size_t>(ei);
        const auto eta = static_cast<float>(etaAtIndex(ei));
        // wo.h below which a facet totally internally reflects; zero when entering, where there is no cone.
        const double criticalCos = eta > 1.0F ? std::sqrt(1.0 - (1.0 / (static_cast<double>(eta) * eta))) : 0.0;
        // phi is even about 0, so half the circle is integrated and doubled. Split at pi/2 as reflectAlbedo is, for the grazing boundary layer, and where the TIR circle turns tangent (R = criticalCos), where the inner integral has a square-root kink in phi.
        std::array<double, 5> phiBreaks{0.0, 0.5 * kPi, kPi, 0.0, 0.0};
        int phiCount = 3;
        if (criticalCos > mu && sinTv > 0.0) {
            const double tangent = std::acos(std::sqrt((criticalCos * criticalCos) - (mu * mu)) / sinTv);
            phiBreaks[3] = tangent;
            phiBreaks[4] = kPi - tangent;
            phiCount = 5;
        }
        std::sort(phiBreaks.begin(), phiBreaks.begin() + phiCount);
        for (int phiPanel = 0; phiPanel + 1 < phiCount; ++phiPanel) {
            const double phiLo = phiBreaks[static_cast<std::size_t>(phiPanel)];
            const double phiHi = phiBreaks[static_cast<std::size_t>(phiPanel) + 1];
            for (std::size_t p = 0; p < rule.node.size(); ++p) {
                const double phi = phiLo + ((phiHi - phiLo) * rule.node[p]);
                const double horizontal = sinTv * std::cos(phi);
                const double radius = std::hypot(horizontal, mu);
                const double delta = std::atan2(horizontal, mu);
                const double visible = std::min(0.5 * kPi, delta + (0.5 * kPi));
                std::array<double, 5> breaks{0.0, visible, std::min(visible, 0.5 * (delta + (0.5 * kPi))), 0.0, 0.0};
                int count = 3;
                if (criticalCos > 0.0 && criticalCos < radius) {
                    const double half = std::acos(criticalCos / radius);
                    for (const double at : {delta - half, delta + half}) {
                        if (at > 0.0 && at < visible) {
                            breaks[static_cast<std::size_t>(count++)] = at;
                        }
                    }
                }
                std::sort(breaks.begin(), breaks.begin() + count);
                for (int panel = 0; panel + 1 < count; ++panel) {
                    const double psiLo = std::atan(std::tan(breaks[static_cast<std::size_t>(panel)]) / alpha);
                    const double psiHi = std::atan(std::tan(breaks[static_cast<std::size_t>(panel) + 1]) / alpha);
                    for (std::size_t q = 0; q < rule.node.size(); ++q) {
                        const double psi = psiLo + ((psiHi - psiLo) * rule.node[q]);
                        const double thetaH = std::atan(alpha * std::tan(psi));
                        const glm::dvec3 h(std::sin(thetaH) * std::cos(phi), std::sin(thetaH) * std::sin(phi), std::cos(thetaH));
                        const double woDotH = glm::dot(wo, h);
                        // phi and psi panel widths, measure sin(psi)cos(psi)/pi doubled for the half circle; the escape's 1/mu is in smithG2OverCosO.
                        const double weight = rule.weight[p] * rule.weight[q] * (phiHi - phiLo) * (psiHi - psiLo) *
                                              (2.0 * std::sin(psi) * std::cos(psi) / kPi) * (woDotH / h.z);
                        const double fresnel = fresnelDielectric(static_cast<float>(woDotH), eta, 1.0F);
                        const double wiZ = (2.0 * woDotH * h.z) - mu;
                        if (wiZ > 0.0) {
                            sums.reflect[e] += weight * fresnel * smithG2OverCosO(mu, wiZ, alpha);
                        }
                        const double cos2T = cos2Transmitted(static_cast<float>(woDotH), eta);
                        if (cos2T >= 0.0) {
                            const double wtZ = (((eta * woDotH) - std::sqrt(cos2T)) * h.z) - (eta * mu);
                            if (wtZ < 0.0) {
                                sums.transmit[e] += weight * (1.0 - fresnel) * smithG2OverCosO(mu, -wtZ, alpha);
                            }
                        }
                    }
                }
            }
        }
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
    std::vector<float> a;  // [roughnessIndex][muIndex], kAlbedoRoughnessRes * kAlbedoMuRes
    std::vector<float> b;
    std::vector<float> aavg;  // cosine-weighted means, 2*integral(.(mu)*mu dmu)
    std::vector<float> bavg;
    std::vector<float> r;  // [roughnessIndex][muIndex][etaIndex], kTransmitRoughnessRes * kTransmitMuRes * kEtaRes
    std::vector<float> t;
    std::vector<float> ravg;
    std::vector<float> tavg;
    std::vector<float> msDensity;  // [roughnessIndex][muIndex], the reflected multiple-scattering lobe's own shape
    std::vector<float> msCdf;
    std::vector<float> msTransmitDensity;  // [roughnessIndex][muIndex][etaIndex], the transmitted twin, unnormalised
    std::vector<float> msTransmitCdf;
};

// The reflect table's mu axis, uniform in sqrt(mu) as the escape tables' escapeMu already is, and for the same reason one axis down: E climbs from its grazing limit over mu ~ alpha, a layer a uniform grid spans with well under one cell at low roughness. bsdf.cpp's directionalAlbedo indexes it by sqrt(mu) to match.
// Node 0 is mu = 0 itself, not a nudge off it: E(0, alpha) = 1 exactly for every alpha, and smithG2OverCosO holds to that limit, so the column that used to be the table's worst is now its sharpest. buildReflect asserts the identity on every row.
double reflectMu(int index) {
    const double t = static_cast<double>(index) / static_cast<double>(kAlbedoMuRes - 1);
    return t * t;
}

// The escape tables' mu axis, uniform in sqrt(mu): E climbs from its grazing limit over mu ~ alpha (G1 ~ 2mu/alpha below it), which a uniform mu grid spans with under two cells at roughness 0.15, so nodes crowd toward grazing as sqrt spacing puts them. bsdf.cpp's escapeAlbedo indexes by sqrt(mu) to match.
// Node 0 is mu = 0 itself, the grazing limit smithG2OverCosO holds to.
double escapeMu(int index) {
    const double t = static_cast<double>(index) / static_cast<double>(kTransmitMuRes - 1);
    return t * t;
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
    table.a.assign(static_cast<std::size_t>(kAlbedoRoughnessRes) * kAlbedoMuRes, 0.0F);
    table.b.assign(static_cast<std::size_t>(kAlbedoRoughnessRes) * kAlbedoMuRes, 0.0F);
    table.aavg.assign(kAlbedoRoughnessRes, 0.0F);
    table.bavg.assign(kAlbedoRoughnessRes, 0.0F);
    // One roughness row per worker. Rows share no accumulator and each writes only its own slice, so the result is
    // identical to the serial order -- the determinism the committed artifact needs survives the threading.
    parallelRows(kAlbedoRoughnessRes, [&](int ri) {
        const double alpha = gridAlpha(ri, kAlbedoRoughnessRes);
        for (int mi = 0; mi < kAlbedoMuRes; ++mi) {
            const Split split = reflectAlbedo(reflectMu(mi), alpha, phiRule, psiRule);
            table.a[static_cast<std::size_t>((ri * kAlbedoMuRes) + mi)] = static_cast<float>(split.a);
            table.b[static_cast<std::size_t>((ri * kAlbedoMuRes) + mi)] = static_cast<float>(split.b);
        }
        // E(0, alpha) = 1 exactly, for every alpha: at mu = 0 the integrand collapses to 2 cos(phi) sin^2(psi), whose
        // normalised integral over the domain is 1 -- a grazing surface loses no energy to masking. An analytic
        // identity on the axis' new endpoint, so it costs nothing and is sharp. A bake-time abort rather than a
        // shading-time guard: a row that misses it means the quadrature is wrong at the limit the warp exists to reach.
        const double grazing = static_cast<double>(table.a[static_cast<std::size_t>(ri * kAlbedoMuRes)]) +
                                static_cast<double>(table.b[static_cast<std::size_t>(ri * kAlbedoMuRes)]);
        if (!(std::abs(grazing - 1.0) < 1e-6)) {
            std::cerr << "albedo_table: roughness row " << ri << " has E(mu=0) = " << grazing
                      << ", not 1 -- the reflect quadrature does not reach its own grazing limit\n";
            std::exit(EXIT_FAILURE);
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

void buildTransmit(AlbedoTable& table, int nodes) {
    const GaussLegendre rule = gaussLegendre(nodes);
    const auto cells = static_cast<std::size_t>(kTransmitRoughnessRes) * kTransmitMuRes * kEtaRes;
    table.r.assign(cells, 0.0F);
    table.t.assign(cells, 0.0F);
    table.ravg.assign(static_cast<std::size_t>(kTransmitRoughnessRes) * kEtaRes, 0.0F);
    table.tavg.assign(static_cast<std::size_t>(kTransmitRoughnessRes) * kEtaRes, 0.0F);
    parallelRows(kTransmitRoughnessRes, [&](int ri) {
        const double alpha = gridAlpha(ri, kTransmitRoughnessRes);
        std::array<double, kEtaRes> rWeighted{};
        std::array<double, kEtaRes> tWeighted{};
        EscapeSums previous{};
        for (int mi = 0; mi < kTransmitMuRes; ++mi) {
            const double mu = escapeMu(mi);
            const EscapeSums sums = escapeAlbedo(mu, alpha, rule);
            // Trapezoid of 2*E*mu over the node spacing, which sqrt spacing makes non-uniform. First order.
            const double previousMu = mi > 0 ? escapeMu(mi - 1) : 0.0;
            const double width = mu - previousMu;
            for (int ei = 0; ei < kEtaRes; ++ei) {
                const auto e = static_cast<std::size_t>(ei);
                const double r = sums.reflect[e];
                const double t = sums.transmit[e];
                table.r[static_cast<std::size_t>((((ri * kTransmitMuRes) + mi) * kEtaRes) + ei)] =
                    static_cast<float>(r);
                table.t[static_cast<std::size_t>((((ri * kTransmitMuRes) + mi) * kEtaRes) + ei)] =
                    static_cast<float>(t);
                rWeighted[e] += width * ((r * mu) + (previous.reflect[e] * previousMu));
                tWeighted[e] += width * ((t * mu) + (previous.transmit[e] * previousMu));
            }
            previous = sums;
        }
        for (int ei = 0; ei < kEtaRes; ++ei) {
            const auto e = static_cast<std::size_t>(ei);
            table.ravg[static_cast<std::size_t>((ri * kEtaRes) + ei)] = static_cast<float>(rWeighted[e]);
            table.tavg[static_cast<std::size_t>((ri * kEtaRes) + ei)] = static_cast<float>(tWeighted[e]);
        }
    });
}

// Reflect-side E at a uniform-mu density node, read through the sqrt(mu) axis exactly as bsdf.cpp's directionalAlbedo
// reads it at that roughness row, float arithmetic included. The escape side's escapeAtUniformMu is the same operation
// one axis wider; they stay separate because their strides, resolutions and channel pairs all differ.
float reflectAtUniformMu(const AlbedoTable& table, int ri, int mi) {
    const float mf = std::sqrt(static_cast<float>(mi) / static_cast<float>(kMsReflectMuRes - 1)) * (kAlbedoMuRes - 1);
    const int m0 = std::min(static_cast<int>(mf), kAlbedoMuRes - 2);
    const float mt = mf - static_cast<float>(m0);
    const auto at = [&](int m) {
        const auto index = static_cast<std::size_t>((ri * kAlbedoMuRes) + m);
        return table.a[index] + table.b[index];
    };
    return at(m0) + (mt * (at(m0 + 1) - at(m0)));
}

// --- Sampling shape for the reflected multiple-scattering lobe: the exact (1-E)cos sampler.
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
// Uniform in mu, unlike the albedo table's sqrt(mu) axis, so bsdf.cpp's piecewise-linear inversion keeps one step
// width; each node reads E through that warp exactly as the shading does, so the shape is still built from the values
// the shading actually sees. The transmit twin below is the same arrangement one axis wider.
void buildMultipleScatteringShape(AlbedoTable& table) {
    const double step = 1.0 / (kMsReflectMuRes - 1);
    table.msDensity.assign(static_cast<std::size_t>(kAlbedoRoughnessRes) * kMsReflectMuRes, 0.0F);
    table.msCdf.assign(static_cast<std::size_t>(kAlbedoRoughnessRes) * kMsReflectMuRes, 0.0F);
    for (int ri = 0; ri < kAlbedoRoughnessRes; ++ri) {
        std::vector<double> raw(kMsReflectMuRes);
        for (int mi = 0; mi < kMsReflectMuRes; ++mi) {
            const double deficit = 1.0 - static_cast<double>(reflectAtUniformMu(table, ri, mi));
            raw[static_cast<std::size_t>(mi)] = deficit * mi * step;
        }
        double norm = 0.0;
        for (int mi = 0; mi + 1 < kMsReflectMuRes; ++mi) {
            norm += 0.5 * (raw[static_cast<std::size_t>(mi)] + raw[static_cast<std::size_t>(mi) + 1]) * step;
        }
        // 1-E is strictly positive at every roughness the table reaches (measured minimum 1.4e-7, at roughness 0), so this is a bake-time assertion and not a shading-time guard: a non-positive row would mean the albedo table itself is wrong, and silently substituting cosine would hide that.
        if (!(norm > 0.0)) {
            std::cerr << "albedo_table: roughness row " << ri << " has non-positive energy deficit " << norm
                      << " -- the reflect table is wrong, not this shape\n";
            std::exit(EXIT_FAILURE);
        }
        double cdf = 0.0;
        for (int mi = 0; mi < kMsReflectMuRes; ++mi) {
            const auto index = static_cast<std::size_t>((ri * kMsReflectMuRes) + mi);
            const double density = raw[static_cast<std::size_t>(mi)] / norm;
            if (mi > 0) {
                cdf += 0.5 * (table.msDensity[index - 1] + density) * step;
            }
            table.msDensity[index] = static_cast<float>(density);
            table.msCdf[index] = static_cast<float>(mi == kMsReflectMuRes - 1 ? 1.0 : cdf);
        }
    }
}

// Escape total at a uniform-mu density node, read through the sqrt(mu) axis exactly as bsdf.cpp's escapeAlbedo reads it at that roughness and eta node, float arithmetic included.
float escapeAtUniformMu(const AlbedoTable& table, int ri, int mi, int ei) {
    const float mf = std::sqrt(static_cast<float>(mi) / static_cast<float>(kTransmitMuRes - 1)) * (kTransmitMuRes - 1);
    const int m0 = std::min(static_cast<int>(mf), kTransmitMuRes - 2);
    const float mt = mf - static_cast<float>(m0);
    const auto at = [&](int m) {
        const auto index = static_cast<std::size_t>((((ri * kTransmitMuRes) + m) * kEtaRes) + ei);
        return table.r[index] + table.t[index];
    };
    return at(m0) + (mt * (at(m0 + 1) - at(m0)));
}

// Escape-deficit shape for the transmissive multiple-scattering lobes, the far-hemisphere twin of the shape above, one axis wider because the escape it is built from is eta-dependent and the Schlick split cannot factor that out.
// bsdf.cpp reads it as both value and density: each transmissive share is its energy times this normalised (1-Escape(mu_i))*cos density divided by cos, so the density is the zero-variance one and the share integrates to its energy exactly -- cosine sampling paid relative variance 25 at roughness 0.13.
// Unlike the reflect shape this is stored UNNORMALISED: bsdf.cpp blends four rows over (roughness, eta) and divides by the blended total, which reproduces the raw-deficit interpolation escapeAlbedo itself performs, where a blend of per-row-normalised shapes would not commute with it.
// Uniform in mu, unlike the escape tables' sqrt(mu) axis, so bsdf.cpp's piecewise-linear inversion keeps one step width; each node reads the escape through that axis as the shading does.
// Unnormalised storage is also what removes the degenerate row: a row whose deficit is numerically zero carries near-zero weight into the blend rather than a unit-mass shape of amplified noise, so this needs neither the reflect side's bake-time abort nor a substituted fallback.
void buildTransmitMultipleScatteringShape(AlbedoTable& table) {
    const double step = 1.0 / (kTransmitMuRes - 1);
    const auto size = static_cast<std::size_t>(kTransmitRoughnessRes) * kTransmitMuRes * kEtaRes;
    table.msTransmitDensity.assign(size, 0.0F);
    table.msTransmitCdf.assign(size, 0.0F);
    for (int ri = 0; ri < kTransmitRoughnessRes; ++ri) {
        for (int ei = 0; ei < kEtaRes; ++ei) {
            double cdf = 0.0;
            for (int mi = 0; mi < kTransmitMuRes; ++mi) {
                const auto index = static_cast<std::size_t>((((ri * kTransmitMuRes) + mi) * kEtaRes) + ei);
                // Clamped: the quadrature can land a few 1e-8 past unity, and a negative segment would break the CDF monotonicity the exact inversion depends on.
                // bsdf.cpp derives each share's value from this same density, so value and pdf share one support by construction.
                const double deficit = std::max(1.0 - static_cast<double>(escapeAtUniformMu(table, ri, mi, ei)), 0.0);
                const auto density = static_cast<float>(deficit * mi * step);
                if (mi > 0) {
                    // Trapezoid over the float density as emitted, not the double behind it, so the stored pair is exactly consistent at the precision bsdf.cpp reads them back at.
                    cdf += 0.5 * (table.msTransmitDensity[index - kEtaRes] + density) * step;
                }
                table.msTransmitDensity[index] = density;
                table.msTransmitCdf[index] = static_cast<float>(cdf);
            }
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
                const int stride = shipped->size() == table.aavg.size() ? 1 : kAlbedoMuRes;
                channelWorst = {delta, name, static_cast<int>(i) / stride,
                                 stride == 1 ? -1 : static_cast<int>(i) % kAlbedoMuRes};
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

// Largest disagreement between the shipped transmit tables and a rebake at twice the nodes per axis: the quadrature's own error, measured on every bake as verifyReflect measures the reflect side's.
double verifyTransmit(const AlbedoTable& table, int nodes) {
    AlbedoTable reference;
    buildTransmit(reference, nodes * 2);
    double worst = 0.0;
    const std::array<std::pair<const char*, std::pair<const std::vector<float>*, const std::vector<float>*>>, 4>
        channels = {{{"r", {&table.r, &reference.r}},
                     {"t", {&table.t, &reference.t}},
                     {"ravg", {&table.ravg, &reference.ravg}},
                     {"tavg", {&table.tavg, &reference.tavg}}}};
    for (const auto& [name, pair] : channels) {
        double channelWorst = 0.0;
        std::size_t at = 0;
        for (std::size_t i = 0; i < pair.first->size(); ++i) {
            const double delta = std::abs(static_cast<double>((*pair.first)[i]) - static_cast<double>((*pair.second)[i]));
            if (delta > channelWorst) {
                channelWorst = delta;
                at = i;
            }
        }
        // Directional channels are [roughness][mu][eta]; the means drop the mu axis.
        const bool directional = pair.first->size() == table.r.size();
        const std::size_t muStride = directional ? kTransmitMuRes : 1;
        std::cout << "albedo_table: " << name << " residual " << channelWorst << " against " << nodes * 2
                  << " nodes at roughnessIndex " << at / (muStride * kEtaRes) << " muIndex "
                  << (directional ? static_cast<long>((at / kEtaRes) % kTransmitMuRes) : -1L) << " etaIndex "
                  << at % kEtaRes << "\n";
        worst = std::max(worst, channelWorst);
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

bool writeInc(const std::string& path, const AlbedoTable& table, double residual, int transmitNodes,
              double transmitResidual) {
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
           "// distributed in sqrt(alpha), and it is what callers already hold. Every grid is edge-aligned, so\n"
           "// roughness 0 and mu 1 are exact table entries and bsdf.cpp's lookups can interpolate on k/(res-1).\n"
           "// BOTH mu axes are uniform in sqrt(mu), mu = (k/(res-1))^2, so nodes crowd where E climbs from its\n"
           "// grazing limit over mu ~ alpha; bsdf.cpp indexes each of them by sqrt(mu). The reflect side's node\n"
           "// 0 is mu = 0 itself, where E = 1 exactly for every alpha and the bake asserts that identity. The\n"
           "// two multiple-scattering shapes below stay uniform in mu, where their piecewise-linear inversion\n"
           "// has one step width.\n"
           "// Reflect side (a, b, aavg, bavg) is exact-domain Gauss-Legendre, residual "
        << residual << " against a doubled rule.\n"
           "// Transmit side (r, t, ravg, tavg) is Gauss-Legendre in the NDF measure at "
        << transmitNodes << " nodes per panel, residual " << transmitResidual << " against a doubled rule.\n"
           "// kMsReflectDensity/kMsReflectCdf are the reflected multiple-scattering lobe's sampling shape: a\n"
           "// piecewise-linear density over mu, proportional to (1-E(mu))*mu and normalised to 1, with its exact\n"
           "// prefix integrals. bsdf.cpp inverts the first and evaluates it for the matching pdf.\n"
           "// kMsTransmitDensity/kMsTransmitCdf are the same shape for the transmitted twin, carrying the escape\n"
           "// table's eta axis and stored UNNORMALISED: bsdf.cpp blends four rows over (roughness, eta) and\n"
           "// divides by the blended total, so the sampled shape is the raw-deficit interpolation escapeAlbedo\n"
           "// performs and a numerically zero row cannot contribute a unit-mass shape of amplified noise.\n";
    out << "\nconstexpr int kAlbedoRoughnessRes = " << kAlbedoRoughnessRes << ";\n"
        << "constexpr int kAlbedoMuRes = " << kAlbedoMuRes << ";\n"
        << "constexpr int kMsReflectMuRes = " << kMsReflectMuRes << ";\n"
        << "constexpr int kTransmitRoughnessRes = " << kTransmitRoughnessRes << ";\n"
        << "constexpr int kTransmitMuRes = " << kTransmitMuRes << ";\n"
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
    writeArray(out, "kMsTransmitDensity", table.msTransmitDensity);
    writeArray(out, "kMsTransmitCdf", table.msTransmitCdf);
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
    int transmitNodes = kTransmitNodes;
    for (int i = 1; i < argc; ++i) {
        const bool hasValue = i + 1 < argc;
        if (std::strcmp(argv[i], "--out") == 0 && hasValue) {
            outPath = argv[++i];
        } else if (std::strcmp(argv[i], "--nodes") == 0 && hasValue) {
            phiNodes = psiNodes = muNodes = std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--transmit-nodes") == 0 && hasValue) {
            transmitNodes = std::atoi(argv[++i]);
        } else {
            std::cerr << "albedo_table: unknown or incomplete argument '" << argv[i]
                      << "'\nusage: albedo_table [--out path.inc] [--nodes N] [--transmit-nodes N]\n";
            return EXIT_FAILURE;
        }
    }
    if (phiNodes < 2 || transmitNodes < 2) {
        std::cerr << "albedo_table: --nodes and --transmit-nodes must be at least 2\n";
        return EXIT_FAILURE;
    }

    AlbedoTable table;
    buildReflect(table, phiNodes, psiNodes, muNodes);
    const Residual residual = verifyReflect(table, phiNodes, psiNodes, muNodes);
    buildTransmit(table, transmitNodes);
    const double transmitResidual = verifyTransmit(table, transmitNodes);
    buildMultipleScatteringShape(table);
    buildTransmitMultipleScatteringShape(table);
    if (!writeInc(outPath, table, residual.value, transmitNodes, transmitResidual)) {
        return EXIT_FAILURE;
    }
    std::cout << "albedo_table: wrote " << outPath << " (reflect " << kAlbedoRoughnessRes << "x" << kAlbedoMuRes
              << " at " << phiNodes << " nodes, residual " << residual.value << " in " << residual.channel
              << " at roughnessIndex " << residual.roughnessIndex << " muIndex " << residual.muIndex
              << "; transmit " << kTransmitRoughnessRes << "x" << kTransmitMuRes << "x" << kEtaRes << " at "
              << transmitNodes << " nodes, residual " << transmitResidual << ")\n";
    return EXIT_SUCCESS;
}
