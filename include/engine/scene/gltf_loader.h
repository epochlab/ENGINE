#pragma once

#include <optional>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "engine/gfx/mesh.h"
#include "engine/scene/bvh.h"
#include "engine/scene/material.h"

namespace engine::scene {

// One glTF primitive: its geometry, its material, and its baked
// world-space transform (accumulated from the node hierarchy).
struct MeshInstance {
    engine::gfx::Mesh mesh;
    Material material;
    glm::mat4 transform;
};

struct LoadedModel {
    std::vector<MeshInstance> instances;
    // Every triangle across every instance, pre-transformed to world
    // space -- Mesh itself is GPU-resident only (no CPU vertex
    // readback), so this is accumulated once here, at the one point
    // during loading where each primitive's CPU vertices/indices and
    // baked world transform are all in scope together. Feeds
    // Bvh::build (bvh.h) in main.cpp; not used for rendering this phase.
    std::vector<Triangle> worldTriangles;
};

// Parses path via cgltf, resolving each material's textures (relative
// to path's directory) through Texture::createFromExr -- this project's
// glTF assets ship linear EXR maps, not glTF's usual PNG/JPEG. Returns
// nullopt and logs to stderr on any failure (bad file, missing
// accessor, missing texture, non-triangle primitive): fails clearly
// rather than substituting a placeholder for something this loader
// doesn't yet support.
std::optional<LoadedModel> loadGltf(const std::string& path);

}  // namespace engine::scene
