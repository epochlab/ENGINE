#pragma once

#include <algorithm>
#include <cmath>
#include <utility>

// The dielectric interface predicate, shared by the shading path (bsdf.cpp) and the offline albedo-table generator
// (tools/albedo_table.cpp) so the table is baked against exactly the interface it is later shaded against.
// Header-only and dependency-free on purpose: the generator cannot link bsdf.cpp, because bsdf.cpp includes the table
// the generator produces. Two verbatim copies used to satisfy that constraint, and a fix applied to one and not the
// other would silently desynchronise the baked table from the shading code -- a class of defect no validator could
// see, since each copy is self-consistent. One definition removes it by construction rather than detecting it.
namespace engine::scene {

// Snell in cos^2 form: cos^2(thetaT) = (1 - r^2) + r^2 cos^2(thetaI), r = etaI/etaT. Negative means total internal
// reflection.
// Algebraically 1 - r^2 sin^2(thetaI) (PBRT-v4 FrDielectric/Refract, Walter 2007 eq. 40), but never forms
// 1 - cos^2(thetaI), which rounds to exactly 1.0F below cos 2^-12 and falsely reports TIR.
// At r == 1 it collapses to cos^2(thetaI), so an index-matched interface cannot total-internally-reflect by
// construction rather than by a branch; at r < 1 the constant term is positive, so entering a denser medium cannot
// either.
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

}  // namespace engine::scene
