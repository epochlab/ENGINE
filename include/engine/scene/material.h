#pragma once

#include "engine/gfx/hdr_image.h"

namespace engine::scene {

// Metallic-roughness material, extended with Specular/AO -- the raw texture set this project's assets ship, beyond glTF's own pbrMetallicRoughness slots (see gltf_loader.cpp for how they're read out of the material's `extras`). Scalar/colour factors live on SceneConfig::material (scene_config.h), not here -- see resolveBsdfParams (path_tracer.cpp). Textures are CPU-resident (engine::gfx::HdrImage), sampled per-ray by the path tracer.
struct Material {
    engine::gfx::HdrImage baseColorTexture;
    engine::gfx::HdrImage normalTexture;
    engine::gfx::HdrImage bumpTexture;
    engine::gfx::HdrImage roughnessTexture;
    engine::gfx::HdrImage specularTexture;
    engine::gfx::HdrImage aoTexture;
};

}  // namespace engine::scene
