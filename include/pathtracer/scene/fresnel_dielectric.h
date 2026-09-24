#pragma once

#include <algorithm>
#include <cmath>
#include <utility>

// The dielectric interface predicate, shared by the shading path (bsdf.cpp) and the offline albedo-table generator
// so the table is baked against exactly the interface it is later shaded against. Header-only and dependency-free
// because the generator cannot link bsdf.cpp, which includes the table the generator produces.
namespace pathtracer::scene {

// Snell in cos^2 form: cos^2(thetaT) = (1 - r^2) + r^2 cos^2(thetaI), r = etaI/etaT; negative means TIR. Never forms
// 1 - cos^2(thetaI), which rounds to exactly 1.0F below cos 2^-12 and falsely reports TIR (PBRT-v4 FrDielectric,
// Walter 2007 eq. 40). At r == 1 it collapses to cos^2(thetaI), so an index-matched interface cannot TIR at all.
[[nodiscard]] inline float cos2Transmitted(float cosThetaI, float etaRatio) {
    const float r2 = etaRatio * etaRatio;
    return (1.0F - r2) + (r2 * cosThetaI * cosThetaI);
}

// Exact unpolarized dielectric Fresnel reflectance (PBRT's FrDielectric); 1.0 from the critical angle inward, where
// cosThetaT is 0 and both polarisations are already exactly 1.
[[nodiscard]] inline float fresnelDielectric(float cosThetaI, float etaI, float etaT) {
    cosThetaI = std::clamp(cosThetaI, -1.0F, 1.0F);
    if (cosThetaI < 0.0F) {
        std::swap(etaI, etaT);
        cosThetaI = -cosThetaI;
    }
    const float cos2ThetaT = cos2Transmitted(cosThetaI, etaI / etaT);
    if (cos2ThetaT < 0.0F) {
        return 1.0F;
    }
    const float cosThetaT = std::sqrt(cos2ThetaT);
    const float rParallel =
        ((etaT * cosThetaI) - (etaI * cosThetaT)) / ((etaT * cosThetaI) + (etaI * cosThetaT));
    const float rPerpendicular =
        ((etaI * cosThetaI) - (etaT * cosThetaT)) / ((etaI * cosThetaI) + (etaT * cosThetaT));
    return ((rParallel * rParallel) + (rPerpendicular * rPerpendicular)) * 0.5F;
}

}  // namespace pathtracer::scene
