#include "engine/scene/shading_scene.h"

namespace engine::scene {

ShadingVertex interpolateShading(const ShadingTriangle& tri, float u, float v) {
    const float w = 1.0F - u - v;
    ShadingVertex result{};
    result.position = (w * tri.v0.position) + (u * tri.v1.position) + (v * tri.v2.position);
    result.normal = glm::normalize((w * tri.v0.normal) + (u * tri.v1.normal) + (v * tri.v2.normal));
    result.uv = (w * tri.v0.uv) + (u * tri.v1.uv) + (v * tri.v2.uv);
    const glm::vec3 tangentXyz =
        glm::normalize((w * glm::vec3(tri.v0.tangent)) + (u * glm::vec3(tri.v1.tangent)) +
                        (v * glm::vec3(tri.v2.tangent)));
    result.tangent = glm::vec4(tangentXyz, tri.v0.tangent.w);  // handedness: +-1 constant, glTF requires it consistent per-triangle
    result.colour = (w * tri.v0.colour) + (u * tri.v1.colour) + (v * tri.v2.colour);
    return result;
}

glm::vec3 shadowTerminatorOffset(const ShadingTriangle& tri, float u, float v) {
    const float w = 1.0F - u - v;
    const glm::vec3 flatPosition =
        (w * tri.v0.position) + (u * tri.v1.position) + (v * tri.v2.position);

    const auto pulledOntoTangentPlane = [&](const glm::vec3& vertexPos,
                                              const glm::vec3& vertexNormal) {
        const glm::vec3 toHit = flatPosition - vertexPos;
        const float d = glm::dot(toHit, vertexNormal);
        return vertexPos + (d < 0.0F ? toHit - (d * vertexNormal) : toHit);
    };

    const glm::vec3 p0 = pulledOntoTangentPlane(tri.v0.position, tri.v0.normal);
    const glm::vec3 p1 = pulledOntoTangentPlane(tri.v1.position, tri.v1.normal);
    const glm::vec3 p2 = pulledOntoTangentPlane(tri.v2.position, tri.v2.normal);
    return (w * p0) + (u * p1) + (v * p2);
}

}  // namespace engine::scene
