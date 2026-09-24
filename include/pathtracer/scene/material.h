#pragma once

#include "pathtracer/gfx/hdr_image.h"

namespace pathtracer::scene {

// Metallic-roughness material extended with Specular and AO -- the raw texture set this project's assets ship,
// beyond glTF's own pbrMetallicRoughness slots.
struct Material {
    pathtracer::gfx::ImageTexture baseColorTexture;
    pathtracer::gfx::ImageTexture normalTexture;
    pathtracer::gfx::ImageTexture bumpTexture;
    pathtracer::gfx::ImageTexture roughnessTexture;
    pathtracer::gfx::ImageTexture specularTexture;
    // Loaded from glTF's occlusion_texture but read by nothing: AO is ray-traced per sample now
    // (PathTraceResult::ao), which a baked texture cannot match.
    pathtracer::gfx::ImageTexture aoTexture;
};

// Neutral, non-file default: every slot is a 1x1 texture holding the identity for that slot. 1x1 avoids a
// div-by-zero in buildShadingFrame's bump texel-size calculation, and the constant bump makes its finite difference
// exactly zero. Used for an unauthored glTF slot and for a quad light's injected instance.
[[nodiscard]] Material makeDefaultMaterial();

}  // namespace pathtracer::scene
