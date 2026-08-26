#pragma once

#include <glm/glm.hpp>

#include "engine/gfx/hdr_image.h"

namespace engine::scene {

// Metallic-roughness material, extended with Specular/AO -- the raw texture set this project's assets ship, beyond glTF's own pbrMetallicRoughness slots (see gltf_loader.cpp for how they're read out of the material's `extras`). ior/transmissionFactor: glTF KHR_materials_ior/KHR_materials_transmission. Textures are CPU-resident (engine::gfx::HdrImage), sampled per-ray by the path tracer.
struct Material {
    glm::vec4 baseColorFactor;
    float metallicFactor;
    float roughnessFactor;
    engine::gfx::HdrImage baseColorTexture;
    engine::gfx::HdrImage normalTexture;
    engine::gfx::HdrImage bumpTexture;
    engine::gfx::HdrImage roughnessTexture;
    engine::gfx::HdrImage specularTexture;
    engine::gfx::HdrImage aoTexture;
    float ior = 1.5F;
    float transmissionFactor = 0.0F;
};

}  // namespace engine::scene
