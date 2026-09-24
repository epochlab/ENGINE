#pragma once

#include <optional>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "pathtracer/gfx/scalar_type.h"
#include "pathtracer/scene/light.h"
#include "pathtracer/scene/material.h"
#include "pathtracer/scene/ray_types.h"
#include "pathtracer/scene/shading_scene.h"

namespace pathtracer::scene {

// One glTF primitive: its material and its baked world-space transform, accumulated from the node hierarchy.
// Geometry itself lives only in LoadedModel's world triangle soup.
struct MeshInstance {
    Material material;
    glm::mat4 transform;
    std::string name;  // owning glTF node's name, empty if the node has none; keys SceneConfig::materialOverrides
};

struct LoadedModel {
    std::vector<MeshInstance> instances;
    // Every triangle across every instance, pre-transformed to world space -- accumulated once here, at the one
    // point in loading where each primitive's vertices and its transform are both in hand.
    std::vector<Triangle> worldTriangles;
    // Per-vertex normal/uv/tangent, parallel-indexed with worldTriangles -- Hit::triangleIndex resolves directly into this.
    std::vector<ShadingTriangle> shadingTriangles;
};

// Parses path via cgltf, resolving each material's textures through loadImageTexture at textureType: this project's
// assets ship linear EXR maps, not glTF's usual 8-bit PNGs. rootTransform seeds the node-hierarchy walk in place of
// identity, and a non-empty textureDir replaces path's own directory. nullopt on any parse or load failure.
std::optional<LoadedModel> loadGltf(const std::string& path, pathtracer::gfx::ScalarType textureType,
                                     const glm::mat4& rootTransform = glm::mat4(1.0F),
                                     const std::string& textureDir = "");

// Appends each light's own emitting geometry (2 triangles, 1 MeshInstance with the neutral fallback material) to
// `model`, in the world space its worldTriangles already use, and one entry per light to `instanceLightIndex`, which
// must already be sized model.instances.size() with every entry -1.
void appendQuadLights(LoadedModel& model, const std::vector<QuadLight>& lights,
                       std::vector<int>& instanceLightIndex);

}  // namespace pathtracer::scene
