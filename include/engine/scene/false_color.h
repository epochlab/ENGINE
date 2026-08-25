#pragma once

#include <cmath>

#include <glm/glm.hpp>

namespace engine::scene {

// Deterministic per-index false color for the ObjectID debug AOV (golden-ratio fractional hash --
// cheap, well-spread across ids). Keyed by ShadingTriangle::instanceIndex at the primary hit.
[[nodiscard]] inline glm::vec3 falseColorForId(int id) {
    const auto f = static_cast<float>(id);
    return {std::fmod(f * 0.6180339887F, 1.0F), std::fmod((f * 0.3247179572F) + 0.5F, 1.0F),
            std::fmod((f * 0.1231234F) + 0.25F, 1.0F)};
}

}  // namespace engine::scene
