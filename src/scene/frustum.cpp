#include "engine/scene/frustum.h"

#include <array>
#include <cmath>

namespace engine::scene {

namespace {

// GLM stores matrices column-major (m[col][row]); this reads row i of the mathematical matrix used as clip = m * vec4(pos, 1).
glm::vec4 matrixRow(const glm::mat4& m, int row) {
    return {m[0][row], m[1][row], m[2][row], m[3][row]};
}

// Plane as (A, B, C, D) with Ax+By+Cz+D >= 0 meaning "inside". Normalized so the sign test below is a true signed distance, not just a sign.
glm::vec4 normalizePlane(const glm::vec4& plane) {
    const float length = std::sqrt(plane.x * plane.x + plane.y * plane.y + plane.z * plane.z);
    return plane / length;
}

}  // namespace

bool frustumIntersectsAabb(const glm::mat4& viewProjection, const glm::vec3& boundsMin,
                            const glm::vec3& boundsMax) {
    const glm::vec4 row0 = matrixRow(viewProjection, 0);
    const glm::vec4 row1 = matrixRow(viewProjection, 1);
    const glm::vec4 row2 = matrixRow(viewProjection, 2);
    const glm::vec4 row3 = matrixRow(viewProjection, 3);

    // Left, right, bottom, top, near, far -- Gribb/Hartmann extraction.
    const std::array<glm::vec4, 6> planes = {
        normalizePlane(row3 + row0), normalizePlane(row3 - row0),
        normalizePlane(row3 + row1), normalizePlane(row3 - row1),
        normalizePlane(row3 + row2), normalizePlane(row3 - row2),
    };

    for (const glm::vec4& plane : planes) {
        // Positive vertex: the AABB corner furthest along the plane normal. If even that corner is on the negative side, the whole box is outside this plane.
        const glm::vec3 positiveVertex{
            plane.x >= 0.0F ? boundsMax.x : boundsMin.x,
            plane.y >= 0.0F ? boundsMax.y : boundsMin.y,
            plane.z >= 0.0F ? boundsMax.z : boundsMin.z,
        };
        if (plane.x * positiveVertex.x + plane.y * positiveVertex.y +
                plane.z * positiveVertex.z + plane.w <
            0.0F) {
            return false;
        }
    }
    return true;
}

}  // namespace engine::scene
