#pragma once

#include <cstddef>

#include "engine/gfx/scalar_type.h"

namespace engine::gfx {

// Owns one GL_TEXTURE_2D object, move-only.
class Texture {
public:
    ~Texture();

    Texture(const Texture&) = delete;
    Texture& operator=(const Texture&) = delete;
    Texture(Texture&& other) noexcept;
    Texture& operator=(Texture&& other) noexcept;

    // Creates the texture and uploads width*height RGBA texels (row-major, no padding) whose components are ALREADY of `format`'s type -- Half for Float16, float for Float32 -- so the client pointer and the GL storage agree and the driver performs no conversion of its own. binary16 is a linear EXR's native precision but overflows to Inf above 65504, binary32 stores the source exactly. Clamped to edge, not repeated: this is a single fixed-scale image (the path tracer's display texture), not a tiled texture. No mip chain: the only consumer is a 1:1 fullscreen blit that samples LOD 0 exclusively, so generating one cost a full extra pass over the image per upload and was never read.
    static Texture create(int width, int height, const void* texels, ScalarType format);

    // Replaces the texel contents, from a buffer in format_'s component type exactly as create() takes. Storage is reallocated only when the dimensions actually change; a same-size update is a glTexSubImage2D into the existing storage, with no allocation and nothing to re-parameterize. The distinction earns its keep because a resize is now routine rather than window-only -- the render resolution switches between the interactive and settled scales (profile_config.h) -- while the common case is the same size frame after frame as the path tracer converges.
    void upload(int width, int height, const void* texels);

    void bind(unsigned int unit) const;

    // Raw GL texture id, for callers that need it directly (e.g. PostProcessPass::draw's unsigned-int parameter) rather than through bind()'s implicit active-unit state.
    [[nodiscard]] unsigned int id() const { return id_; }

private:
    Texture(unsigned int id, ScalarType format);

    unsigned int id_ = 0;
    ScalarType format_ = ScalarType::Float16;  // RGBA component type every (re)allocation in upload uses
    int width_ = 0;   // current storage dimensions, so upload can tell a resize from an in-place update
    int height_ = 0;
    std::size_t byteSize_ = 0;  // reported to engine::debug's GPU memory tracker
};

}  // namespace engine::gfx
