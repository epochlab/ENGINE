#pragma once

#include "engine/gfx/hdr_image.h"

namespace engine::scene {

// Metallic-roughness material, extended with Specular -- the raw texture set this project's assets ship, beyond glTF's own pbrMetallicRoughness slots (see gltf_loader.cpp for how they're read out of the material's `extras`). Scalar/colour factors live on SceneConfig::material (scene_config.h), not here -- see resolveBsdfParams (path_tracer.cpp). Textures are CPU-resident, sampled per-ray by the path tracer.
// Each slot is stored at the channel count its consumer reads (gbuffer_shading.cpp) and no more: three for the colour and direction maps, one for the scalar maps, none for alpha. glTF's occlusion_texture has no slot at all -- AO is ray-traced per sample now (path_tracer.h's PathTraceResult::ao), which a baked map cannot express, so loading one cost a full texture's memory for a value nothing read.
struct Material {
    engine::gfx::ImageTexture<3> baseColorTexture;
    engine::gfx::ImageTexture<3> normalTexture;
    engine::gfx::ImageTexture<1> bumpTexture;
    engine::gfx::ImageTexture<1> roughnessTexture;
    engine::gfx::ImageTexture<3> specularTexture;
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
