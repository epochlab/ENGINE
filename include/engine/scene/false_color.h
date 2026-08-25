#pragma once

#include <cmath>

#include <glm/glm.hpp>

namespace engine::scene {

// Deterministic per-index false color for the Object/Material ID debug AOV (golden-ratio fractional
// hash -- cheap, well-spread across ids). Shared by the rasterizer (main.cpp, keyed by mesh instance
// loop index) and the path tracer (ShadingTriangle::instanceIndex at the primary hit) so the same id
// always renders as the same color in both renderers' ObjectID AOV.
[[nodiscard]] inline glm::vec3 falseColorForId(int id) {
    const auto f = static_cast<float>(id);
    return {std::fmod(f * 0.6180339887F, 1.0F), std::fmod((f * 0.3247179572F) + 0.5F, 1.0F),
            std::fmod((f * 0.1231234F) + 0.25F, 1.0F)};
}

}  // namespace engine::scene
