#pragma once

#include <algorithm>
#include <cmath>
#include <complex>

// Double-precision conductor Fresnel references shared by tools. Never include src/scene/bsdf.* here: the oracle-independence rule.
namespace tools::reference {

// Gulbrandsen 2014 eq 12 and 2 as printed, the LITERAL k^2 rather than bsdf.cpp's factored form, then textbook complex Fresnel in double.
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

// Cosine-weighted average Fresnel by composite Simpson: the integrand is analytic on [0,1], so the O(h^4) error here is ~1e-13.
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
