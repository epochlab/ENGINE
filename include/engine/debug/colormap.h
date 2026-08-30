#pragma once

#include <algorithm>
#include <array>

#include <glm/glm.hpp>

namespace engine::debug {

// Turbo (Mikhailov/Google 2019), a perceptually-ordered blue->green->yellow->red palette that (unlike
// jet) has no readback ambiguity between its ends and no perceptual flat spot in the middle. 11 stops
// at t=0,0.1,...,1.0, linearly interpolated -- a small fixed table rather than the full published
// 256-entry LUT or a polynomial fit, since a debug AOV heatmap doesn't need colorimetric precision,
// only a stable, recognisable gradient. t is clamped to [0,1]; out-of-range input saturates to an end colour.
[[nodiscard]] inline glm::vec3 turbo(float t) {
    static constexpr std::array<glm::vec3, 11> kStops = {{
        {0.18995F, 0.07176F, 0.23217F},
        {0.25107F, 0.25237F, 0.63374F},
        {0.15840F, 0.50361F, 0.85780F},
        {0.09267F, 0.72401F, 0.72858F},
        {0.19556F, 0.85885F, 0.41465F},
        {0.63000F, 0.98000F, 0.24000F},
        {0.83636F, 0.79313F, 0.10476F},
        {0.97717F, 0.55712F, 0.14380F},
        {0.94066F, 0.31843F, 0.09600F},
        {0.72596F, 0.12061F, 0.05673F},
        {0.47960F, 0.01583F, 0.01055F},
    }};
    const float clamped = std::clamp(t, 0.0F, 1.0F) * static_cast<float>(kStops.size() - 1);
    const auto lo = static_cast<std::size_t>(clamped);
    const std::size_t hi = std::min(lo + 1, kStops.size() - 1);
    const float frac = clamped - static_cast<float>(lo);
    return glm::mix(kStops[lo], kStops[hi], frac);
}

}  // namespace engine::debug
