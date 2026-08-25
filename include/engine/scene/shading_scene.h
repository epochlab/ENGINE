#pragma once

#include <glm/glm.hpp>

namespace engine::scene {

// World-space, baked from the glTF loader's per-vertex data -- the path tracer's only source of per-vertex shading data.
struct ShadingVertex {
    glm::vec3 position;
    glm::vec3 normal;
    glm::vec2 uv;
    glm::vec4 tangent;  // xyz = tangent, w = bitangent handedness (glTF convention)
};

// Indexed identically to Bvh's Triangle list -- Bvh::Hit::triangleIndex resolves directly into this.
struct ShadingTriangle {
    ShadingVertex v0;
    ShadingVertex v1;
    ShadingVertex v2;
    int instanceIndex;  // indexes LoadedModel::instances, owner of this triangle's Material
};

// Barycentric interpolation (u/v = Bvh::Hit's Moller-Trumbore convention, w=1-u-v on v0); normal/tangent renormalized after blending.
[[nodiscard]] ShadingVertex interpolateShading(const ShadingTriangle& tri, float u, float v);

// Chiang/Li/Burley 2019 shadow-terminator fix: per-vertex tangent-plane projection, barycentric-blended, for use as a secondary ray origin.
[[nodiscard]] glm::vec3 shadowTerminatorOffset(const ShadingTriangle& tri, float u, float v);

}  // namespace engine::scene
