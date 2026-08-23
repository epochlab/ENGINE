#pragma once

#include <glm/glm.hpp>

namespace engine::scene {

// True if the world-space AABB [boundsMin, boundsMax] has any overlap
// with the view frustum described by viewProjection (Gribb/Hartmann
// plane extraction) -- false only when the box is provably entirely
// outside at least one frustum plane.
bool frustumIntersectsAabb(const glm::mat4& viewProjection, const glm::vec3& boundsMin,
                            const glm::vec3& boundsMax);

}  // namespace engine::scene
