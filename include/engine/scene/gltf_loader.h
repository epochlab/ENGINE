#pragma once

#include <optional>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "engine/scene/bvh.h"
#include "engine/scene/material.h"
#include "engine/scene/shading_scene.h"

namespace engine::scene {

// One glTF primitive: its material and its baked world-space transform (accumulated from the node hierarchy). Geometry itself lives only in LoadedModel's worldTriangles/shadingTriangles -- instanceIndex resolves a Bvh::Hit back to this instance's material.
struct MeshInstance {
    Material material;
    glm::mat4 transform;
};

struct LoadedModel {
    std::vector<MeshInstance> instances;
    // Every triangle across every instance, pre-transformed to world space -- accumulated once here, at the one point during loading where each primitive's vertices/indices and baked world transform are all in scope together. Feeds Bvh::build (bvh.h) in main.cpp.
    std::vector<Triangle> worldTriangles;
    // Per-vertex normal/uv/tangent, parallel-indexed with worldTriangles -- Bvh::Hit::triangleIndex resolves directly into this.
    std::vector<ShadingTriangle> shadingTriangles;
};

// Parses path via cgltf, resolving each material's textures (relative to path's directory) via loadExr -- this project's glTF assets ship linear EXR maps, not glTF's usual PNG/JPEG. Returns nullopt and logs to stderr on any failure (bad file, missing accessor, missing texture, non-triangle primitive): fails clearly rather than substituting a placeholder for something this loader doesn't yet support.
std::optional<LoadedModel> loadGltf(const std::string& path);

}  // namespace engine::scene
