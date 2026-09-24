#pragma once

#include <vector>

#include <glm/glm.hpp>

namespace pathtracer::scene {

struct AabbBounds {
    glm::vec3 min;
    glm::vec3 max;
};

// World-space, baked from the glTF loader's per-vertex data -- the path tracer's only source of per-vertex shading data.
struct ShadingVertex {
    glm::vec3 position;
    glm::vec3 normal;
    glm::vec2 uv;
    glm::vec4 tangent;              // xyz = tangent, w = bitangent handedness (glTF convention)
    glm::vec3 colour = glm::vec3(1.0F);  // COLOR_0, multiplies baseColor; white = no vertex tint
};

// Indexed identically to the world-space Triangle list (ray_types.h) -- Hit::triangleIndex resolves directly into this.
struct ShadingTriangle {
    ShadingVertex v0;
    ShadingVertex v1;
    ShadingVertex v2;
    int instanceIndex;  // indexes LoadedModel::instances, owner of this triangle's Material
};

// Barycentric interpolation (u/v = Hit's Moller-Trumbore convention, w=1-u-v on v0); normal/tangent renormalized after blending.
[[nodiscard]] ShadingVertex interpolateShading(const ShadingTriangle& tri, float u, float v);

// Chiang/Li/Burley 2019 shadow-terminator fix: per-vertex tangent-plane projection, barycentric-blended, for use as a secondary ray origin.
// normalSide: which side the secondary ray leaves on. The projection moves the hit along the vertex normals, so a ray going the other way (refraction entering a dielectric, TIR inside one) needs it mirrored or the origin lands past the interface it just crossed.
[[nodiscard]] glm::vec3 shadowTerminatorOffset(const ShadingTriangle& tri, float u, float v,
                                               bool normalSide);

// True for the inverted box computeInstanceBounds leaves on an instance that contributed no triangles -- the identity of a min/max reduction over nothing, not a sentinel value.
[[nodiscard]] bool isEmpty(const AabbBounds& box);

// World-space AABB per instance, indexed by ShadingTriangle::instanceIndex (instanceCount entries). Positions are already world-space, so this is one linear pass with no transform work -- computed once at load, never per frame. Feeds the Wireframe AOV's per-object box edges (rasterizer.h).
[[nodiscard]] std::vector<AabbBounds> computeInstanceBounds(const std::vector<ShadingTriangle>& triangles,
                                                             int instanceCount);

}  // namespace pathtracer::scene
