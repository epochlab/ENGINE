#pragma once

#include <algorithm>
#include <cmath>
#include <complex>

// Double-precision conductor Fresnel references shared by tools: bsdf_validate's oracles against bsdf.cpp, and metal_fit's spectral fit. Never include src/scene/bsdf.* here (fixtures.h's oracle-independence rule).
namespace tools::reference {

// Gulbrandsen 2014 eq 12 and eq 2 exactly as the paper's Appendix A prints them -- the LITERAL k^2, not
// the factored form bsdf.cpp ships -- followed by the textbook complex-arithmetic Fresnel, all in double.
// Deliberately the other implementation of both departures: bsdf.cpp uses (nMax-n)(n-nLow) for k^2 and the
// real-arithmetic unpolarized form, so one comparison against this reference tests both at once. Double is
// what makes the literal k^2 usable here; it is what fails in float32 near r=1, which is why bsdf.cpp
// factors it.
inline std::complex<double> referenceConductorIor(double r, double g) {
    r = std::clamp(r, 1e-4, 0.9999);   // matches bsdf.cpp's kMinReflectivity/kMaxReflectivity
    g = std::clamp(g, 0.0, 1.0);
    const double sqrtR = std::sqrt(r);
    const double nMin = (1.0 - r) / (1.0 + r);
    const double nMax = (1.0 + sqrtR) / (1.0 - sqrtR);
    const double n = (nMin * g) + ((1.0 - g) * nMax);
    const double k2 = ((((n + 1.0) * (n + 1.0)) * r) - ((n - 1.0) * (n - 1.0))) / (1.0 - r);
    return {n, std::sqrt(std::max(0.0, k2))};
}

inline double referenceConductorFresnelAt(const std::complex<double>& eta, double cosTheta) {
    const double c = std::clamp(cosTheta, 0.0, 1.0);
    const std::complex<double> cosThetaT = std::sqrt(1.0 - ((1.0 - (c * c)) / (eta * eta)));
    const std::complex<double> rParallel = ((eta * c) - cosThetaT) / ((eta * c) + cosThetaT);
    const std::complex<double> rPerpendicular = (c - (eta * cosThetaT)) / (c + (eta * cosThetaT));
    return 0.5 * (std::norm(rParallel) + std::norm(rPerpendicular));
}

inline double referenceConductorFresnel(double r, double g, double cosTheta) {
    return referenceConductorFresnelAt(referenceConductorIor(r, g), cosTheta);
}

// Cosine-weighted average Fresnel, 2*int_0^1 F(mu)*mu dmu, by composite Simpson. The integrand is analytic on [0,1] for every (n, k) the Gulbrandsen domain reaches, so the O(h^4) error at this width is ~1e-13 -- ten orders under the tolerances it is used to police, and doubling the panel count moves no printed digit.
// The mu=0 endpoint contributes nothing (the mu weight kills it) whatever F does there, which is what keeps the rule insensitive to the grazing limit both Fresnels approach differently.
template <typename Fresnel>
double cosineAverageFresnel(Fresnel fresnel) {
    constexpr int kPanels = 4000;   // even, for Simpson
    const double h = 1.0 / kPanels;
    double sum = fresnel(1.0);
    for (int i = 1; i < kPanels; ++i) {
        const double mu = i * h;
        sum += (i % 2 == 1 ? 4.0 : 2.0) * fresnel(mu) * mu;
    }
    return 2.0 * (h / 3.0) * sum;
}

}  // namespace tools::reference
