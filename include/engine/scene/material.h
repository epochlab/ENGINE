#pragma once

#include <glm/glm.hpp>

#include "engine/gfx/hdr_image.h"
#include "engine/gfx/texture.h"

namespace engine::scene {

// Texture retained on both GPU (rasterizer) and CPU (path tracer's per-sample UV lookups).
struct MaterialTexture {
    engine::gfx::Texture gpu;
    engine::gfx::HdrImage cpu;
};

// Metallic-roughness material, extended with Bump/Specular/AO -- the raw texture set this project's assets ship, beyond glTF's own pbrMetallicRoughness slots (see gltf_loader.cpp for how the latter two are read out of the material's `extras`). ior/transmissionFactor: glTF KHR_materials_ior/KHR_materials_transmission.
struct Material {
    glm::vec4 baseColorFactor;
    float metallicFactor;
    float roughnessFactor;
    MaterialTexture baseColorTexture;
    MaterialTexture normalTexture;
    MaterialTexture roughnessTexture;
    MaterialTexture bumpTexture;
    MaterialTexture specularTexture;
    MaterialTexture aoTexture;
    float ior = 1.5F;
    float transmissionFactor = 0.0F;
};

}  // namespace engine::scene
