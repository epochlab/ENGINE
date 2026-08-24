#pragma once

#include "engine/gfx/cubemap_texture.h"

namespace engine::gfx {

class Texture;
class ShaderProgram;

struct PrefilteredEnvironment {
    CubemapTexture specular;
    int specularMipCount;
};

// One-time (startup) preprocessing of an equirectangular HDR environment
// map into a prefiltered specular cubemap mip chain (Karis 2013, "Real
// Shading in Unreal Engine 4" split-sum): mip 0 is a direct equirect
// resample (roughness 0), each subsequent mip is GGX-importance-sample
// prefiltered at increasing roughness. Fragment-shader render-to-texture
// passes only -- GL 4.1 has no compute shaders. Diffuse IBL is handled
// separately (SH-9 projection, see sh_irradiance.h) and needs no GPU
// preprocessing at all.
[[nodiscard]] PrefilteredEnvironment buildPrefilteredEnvironment(
    const Texture& equirect, const ShaderProgram& equirectToCubemapShader,
    const ShaderProgram& prefilterShader);

}  // namespace engine::gfx
