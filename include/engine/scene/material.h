#pragma once

#include <glm/glm.hpp>

#include "engine/gfx/texture.h"

namespace engine::scene {

// Metallic-roughness material, extended with Bump/Specular/AO -- the
// raw texture set this project's assets ship, beyond glTF's own
// pbrMetallicRoughness slots (see gltf_loader.cpp for how the latter
// two are read out of the material's `extras`).
struct Material {
    glm::vec4 baseColorFactor;
    float metallicFactor;
    float roughnessFactor;
    engine::gfx::Texture baseColorTexture;
    engine::gfx::Texture normalTexture;
    engine::gfx::Texture roughnessTexture;
    engine::gfx::Texture bumpTexture;
    engine::gfx::Texture specularTexture;
    engine::gfx::Texture aoTexture;
};

}  // namespace engine::scene
