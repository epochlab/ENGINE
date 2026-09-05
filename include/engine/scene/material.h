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

// Neutral, non-file default: every slot is a 1x1 texture whose constant value is the identity for
// that slot -- see gbuffer_shading.cpp for how each is consumed. 1x1 avoids a div-by-zero in
// buildShadingFrame's bump texel-size calc; the constant bump value makes its finite-difference
// exactly zero. Shared by gltf_loader.cpp (a glTF material with no texture authored for some slot)
// and gltf_loader.cpp's appendQuadLights (a quad light's own injected instance, whose Material is never read by the path
// tracer -- an emitter hit returns before resolveBsdfParams -- but IS read by the CPU rasterizer's
// G-buffer AOVs, which walk every triangle unconditionally).
[[nodiscard]] Material makeDefaultMaterial();

}  // namespace engine::scene
