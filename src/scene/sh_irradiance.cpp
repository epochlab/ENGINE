#include "engine/scene/sh_irradiance.h"

#include <cmath>
#include <cstddef>

namespace engine::scene {

namespace {

constexpr float kPi = 3.14159265F;

// Real SH basis functions, order 0-2 (9 total), evaluated at unit
// direction n. Constants from Ramamoorthi & Hanrahan 2001 / Robin
// Green's "Spherical Harmonic Lighting: The Gritty Details".
std::array<float, 9> shBasis9(const glm::vec3& n) {
    return {
        0.282095F,
        0.488603F * n.y,
        0.488603F * n.z,
        0.488603F * n.x,
        1.092548F * n.x * n.y,
        1.092548F * n.y * n.z,
        0.315392F * ((3.0F * n.z * n.z) - 1.0F),
        1.092548F * n.x * n.z,
        0.546274F * ((n.x * n.x) - (n.y * n.y)),
    };
}

// Per-band clamped-cosine transfer-function coefficients (band 0, 1, 2)
// -- the same A_l constants the paper convolves the radiance signal
// with to turn it into irradiance. Indexed by SH coefficient, not band,
// so it can be applied in the same loop as shBasis9's output.
constexpr std::array<float, 9> kCosineLobeA = {
    kPi,
    2.0F * kPi / 3.0F,
    2.0F * kPi / 3.0F,
    2.0F * kPi / 3.0F,
    kPi / 4.0F,
    kPi / 4.0F,
    kPi / 4.0F,
    kPi / 4.0F,
    kPi / 4.0F,
};

}  // namespace

std::array<glm::vec3, 9> projectIrradianceSH9(const engine::gfx::HdrImage& equirect) {
    std::array<glm::vec3, 9> coeffs{};

    const float dTheta = kPi / static_cast<float>(equirect.height);
    const float dPhi = 2.0F * kPi / static_cast<float>(equirect.width);

    for (int py = 0; py < equirect.height; ++py) {
        const float v = (static_cast<float>(py) + 0.5F) / static_cast<float>(equirect.height);
        const float theta = v * kPi;
        const float sinTheta = std::sin(theta);
        const float cosTheta = std::cos(theta);
        // Equirect quadrature weight: solid angle per texel, sin(theta)
        // accounting for the pole convergence of the lat/long grid.
        const float solidAngle = sinTheta * dTheta * dPhi;
        if (solidAngle <= 0.0F) {
            continue;  // exact poles (py at a boundary): zero-measure, skip
        }

        for (int px = 0; px < equirect.width; ++px) {
            const float u = (static_cast<float>(px) + 0.5F) / static_cast<float>(equirect.width);
            const float phi = (u - 0.5F) * 2.0F * kPi;
            const glm::vec3 dir(sinTheta * std::sin(phi), cosTheta, sinTheta * std::cos(phi));

            const std::size_t idx = (static_cast<std::size_t>(py) *
                                          static_cast<std::size_t>(equirect.width) +
                                      static_cast<std::size_t>(px)) *
                                     4;
            const glm::vec3 radiance(equirect.rgba[idx + 0], equirect.rgba[idx + 1],
                                      equirect.rgba[idx + 2]);

            const std::array<float, 9> basis = shBasis9(dir);
            for (int i = 0; i < 9; ++i) {
                coeffs[i] += radiance * (basis[i] * solidAngle);
            }
        }
    }

    // Fold the cosine-lobe convolution in now, so evaluateIrradianceSH9
    // is a plain dot product with no separate convolution step.
    for (int i = 0; i < 9; ++i) {
        coeffs[i] *= kCosineLobeA[i];
    }
    return coeffs;
}

glm::vec3 evaluateIrradianceSH9(const glm::vec3& n, const std::array<glm::vec3, 9>& coeffs) {
    const std::array<float, 9> basis = shBasis9(n);
    glm::vec3 result(0.0F);
    for (int i = 0; i < 9; ++i) {
        result += coeffs[i] * basis[i];
    }
    // A strongly peaked source (e.g. a visible sun disk) can produce a
    // small negative lobe after cosine convolution -- clamp rather than
    // pursue full de-ringing (Sloan windowing), out of scope for Phase 4.
    return glm::max(result, glm::vec3(0.0F));
}

}  // namespace engine::scene
