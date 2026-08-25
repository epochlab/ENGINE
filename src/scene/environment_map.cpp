#include "engine/scene/environment_map.h"

#include <cmath>

#include <glm/gtc/constants.hpp>

namespace engine::scene {

namespace {

// Matches sky.frag's rotateAboutY.
glm::vec3 rotateAboutY(const glm::vec3& v, float angleRadians) {
    const float c = std::cos(angleRadians);
    const float s = std::sin(angleRadians);
    return {(v.x * c) + (v.z * s), v.y, (-v.x * s) + (v.z * c)};
}

}  // namespace

glm::vec3 EnvironmentMap::sampleDirection(const glm::vec3& direction,
                                           float envRotationRadians) const {
    const glm::vec3 rotated = rotateAboutY(direction, -envRotationRadians);
    const float theta = std::acos(glm::clamp(rotated.y, -1.0F, 1.0F));
    const float phi = std::atan2(rotated.x, rotated.z);
    const glm::vec2 uv((phi / (2.0F * glm::pi<float>())) + 0.5F, theta / glm::pi<float>());
    return glm::vec3(engine::gfx::sampleBilinear(image_, uv));
}

}  // namespace engine::scene
