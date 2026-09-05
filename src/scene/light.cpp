#include "engine/scene/light.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

#include <glm/gtc/constants.hpp>

namespace engine::scene {

namespace {

// atan2(|a x b|, a.b) -- PBRT v4's AngleBetween: robust near 0 and pi, unlike acos(dot(a,b)) which
// loses precision exactly where buildSphericalRectangle's internal angles (gamma_i) are smallest.
float angleBetween(const glm::vec3& a, const glm::vec3& b) {
    return std::atan2(glm::length(glm::cross(a, b)), glm::dot(a, b));
}

}  // namespace

std::optional<SphericalRectangle> buildSphericalRectangle(const QuadLight& quad,
                                                            const glm::vec3& referencePoint) {
    const float exl = glm::length(quad.edge0);
    const float eyl = glm::length(quad.edge1);
    if (!(exl > 0.0F) || !(eyl > 0.0F)) {
        return std::nullopt;
    }
    const glm::vec3 x = quad.edge0 / exl;
    const glm::vec3 y = quad.edge1 / eyl;
    glm::vec3 z = glm::cross(x, y);
    const float zLen = glm::length(z);
    if (!(zLen > 0.0F)) {
        return std::nullopt;  // edge0 parallel to edge1 -- degenerate quad
    }
    z /= zLen;

    // Rectangle-corner-to-referencePoint vector, in world space; the local frame's origin is
    // referencePoint itself (Ureña et al.'s "o"), matching Sample()'s reconstruction below.
    const glm::vec3 d = quad.origin - referencePoint;
    float z0 = glm::dot(d, z);
    glm::vec3 zFrame = z;
    if (z0 > 0.0F) {
        // Keep z0 negative regardless of which way the quad's own normal happens to point -- purely a
        // numerical-robustness convention of the parametrization, unrelated to one-/two-sidedness of
        // emission (that is LightSet::quadRadianceToward's own, separate front-face test).
        zFrame = -z;
        z0 = -z0;
    }
    if (!(z0 < 0.0F)) {
        return std::nullopt;  // referencePoint lies in the rectangle's own plane -- zero measure
    }

    const float x0 = glm::dot(d, x);
    const float y0 = glm::dot(d, y);
    const float x1 = x0 + exl;
    const float y1 = y0 + eyl;

    // Vectors from referencePoint to the four corners, in the local (x, y, zFrame) frame.
    const glm::vec3 v00(x0, y0, z0);
    const glm::vec3 v01(x0, y1, z0);
    const glm::vec3 v10(x1, y0, z0);
    const glm::vec3 v11(x1, y1, z0);
    const glm::vec3 n0 = glm::normalize(glm::cross(v00, v10));
    const glm::vec3 n1 = glm::normalize(glm::cross(v10, v11));
    const glm::vec3 n2 = glm::normalize(glm::cross(v11, v01));
    const glm::vec3 n3 = glm::normalize(glm::cross(v01, v00));

    // Internal angles of the spherical quadrilateral at each of its four vertices (gamma_i, paper notation).
    const float g0 = angleBetween(-n0, n1);
    const float g1 = angleBetween(-n1, n2);
    const float g2 = angleBetween(-n2, n3);
    const float g3 = angleBetween(-n3, n0);

    const float b0 = n0.z;
    const float b1 = n2.z;
    const float k = (2.0F * glm::pi<float>()) - g2 - g3;
    // Girard's theorem: a spherical polygon's solid angle is its interior-angle sum minus (N-2)*pi.
    const float solidAngle = g0 + g1 - k;

    if (!(solidAngle > 0.0F) || !std::isfinite(solidAngle)) {
        return std::nullopt;
    }
    return SphericalRectangle{referencePoint, x, y, zFrame, z0, x0, x1, y0, y1, b0, b1, k, solidAngle};
}

glm::vec3 SphericalRectangle::sample(glm::vec2 u) const {
    // Ureña/Fajardo/King 2013's closed-form inversion of the constant-solid-angle-density CDF.
    // Variable names (au/fu/cu/xu/hv/yv) match the paper's own derivation, not renamed for this codebase.
    const float au = (u.x * solidAngle) + k;
    const float fu = ((std::cos(au) * b0) - b1) / std::sin(au);
    float cu = std::copysign(1.0F / std::sqrt((fu * fu) + (b0 * b0)), fu);
    cu = glm::clamp(cu, -1.0F, 1.0F);

    const float xu = glm::clamp(-(cu * z0) / std::sqrt(std::max(1.0F - (cu * cu), 0.0F)), x0, x1);
    const float d = std::sqrt((xu * xu) + (z0 * z0));
    const float h0 = y0 / std::sqrt((d * d) + (y0 * y0));
    const float h1 = y1 / std::sqrt((d * d) + (y1 * y1));
    const float hv = h0 + (u.y * (h1 - h0));
    const float hv2 = hv * hv;
    const float yv = hv2 < (1.0F - 1e-6F) ? (hv * d) / std::sqrt(1.0F - hv2) : y1;

    return referencePoint + (xu * x) + (yv * y) + (z0 * z);
}

LightSet::LightSet(const EnvironmentMap* environment, float envRotationRadians, float envExposure,
                    const std::vector<QuadLight>& quads)
    : environment_(environment),
      envRotationRadians_(envRotationRadians),
      envExposure_(envExposure),
      quads_(quads) {}

int LightSet::count() const {
    return (environment_ != nullptr ? 1 : 0) + static_cast<int>(quads_.size());
}

glm::vec3 LightSet::environmentRadiance(const glm::vec3& direction, bool nearest) const {
    if (environment_ == nullptr) {
        return glm::vec3(0.0F);
    }
    return (nearest ? environment_->sampleDirectionNearest(direction, envRotationRadians_)
                     : environment_->sampleDirection(direction, envRotationRadians_)) *
           envExposure_;
}

float LightSet::pdfEnvironment(const glm::vec3& dir) const {
    if (environment_ == nullptr) {
        return 0.0F;
    }
    return environment_->pdf(dir, envRotationRadians_) / static_cast<float>(count());
}

glm::vec3 LightSet::quadRadianceToward(int quadIndex, const glm::vec3& direction) const {
    const QuadLight& quad = quads_[static_cast<std::size_t>(quadIndex)];
    const glm::vec3 normal = glm::normalize(glm::cross(quad.edge0, quad.edge1));
    // Front face: a ray travelling toward the light (direction) opposes its outward normal.
    if (glm::dot(direction, normal) < 0.0F || quad.twoSided) {
        return quad.radiance;
    }
    return glm::vec3(0.0F);
}

float LightSet::pdfQuad(int quadIndex, const glm::vec3& p) const {
    const std::optional<SphericalRectangle> rect =
        buildSphericalRectangle(quads_[static_cast<std::size_t>(quadIndex)], p);
    if (!rect.has_value()) {
        return 0.0F;
    }
    return 1.0F / (rect->solidAngle * static_cast<float>(count()));
}

std::optional<LightSample> LightSet::sample(const glm::vec3& p, Sampler& sampler) const {
    const int n = count();
    if (n == 0) {
        return std::nullopt;
    }
    const bool envPresent = environment_ != nullptr;
    int index = 0;
    if (n > 1) {
        const float u = sampler.next1D();
        index = std::min(n - 1, static_cast<int>(u * static_cast<float>(n)));
    }
    const float selectionPdf = 1.0F / static_cast<float>(n);

    if (envPresent && index == 0) {
        const EnvironmentMap::EnvSample envSample =
            environment_->importanceSampleDirection(sampler.next2D(), envRotationRadians_);
        return LightSample{envSample.direction,
                            environment_->sampleDirectionNearest(envSample.direction, envRotationRadians_) *
                                envExposure_,
                            envSample.pdf * selectionPdf, std::numeric_limits<float>::max()};
    }

    const int quadIndex = index - (envPresent ? 1 : 0);
    const QuadLight& quad = quads_[static_cast<std::size_t>(quadIndex)];
    const std::optional<SphericalRectangle> rect = buildSphericalRectangle(quad, p);
    if (!rect.has_value()) {
        return std::nullopt;
    }
    const glm::vec3 pointOnQuad = rect->sample(sampler.next2D());
    const glm::vec3 toLight = pointOnQuad - p;
    const float distance = glm::length(toLight);
    if (!(distance > 0.0F)) {
        return std::nullopt;
    }
    const glm::vec3 direction = toLight / distance;
    return LightSample{direction, quadRadianceToward(quadIndex, direction),
                        selectionPdf / rect->solidAngle, distance};
}

}  // namespace engine::scene
