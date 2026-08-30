#pragma once

#include <cstddef>

namespace engine::gfx {

// Owns one GL_TEXTURE_2D object, move-only.
class Texture {
public:
    ~Texture();

    Texture(const Texture&) = delete;
    Texture& operator=(const Texture&) = delete;
    Texture(Texture&& other) noexcept;
    Texture& operator=(Texture&& other) noexcept;

    // Creates the texture and uploads width*height RGBA float texels (row-major, no padding) as GL_RGBA16F -- a linear EXR's native precision without GL_RGBA32F's extra bandwidth. Clamped to edge, not repeated: this is a single fixed-scale image (the path tracer's display texture), not a tiled texture. No mip chain: the only consumer is a 1:1 fullscreen blit that samples LOD 0 exclusively, so generating one cost a full extra pass over the image per upload and was never read.
    static Texture createFromFloatPixels(int width, int height, const float* rgba);

    // Replaces the texel contents. Storage is reallocated only when the dimensions actually change; a same-size update is a glTexSubImage2D into the existing storage, with no allocation and nothing to re-parameterize. The distinction earns its keep because a resize is now routine rather than window-only -- the render resolution switches between the interactive and settled scales (profile_config.h) -- while the common case is the same size frame after frame as the path tracer converges.
    void upload(int width, int height, const float* rgba);

    void bind(unsigned int unit) const;

    // Raw GL texture id, for callers that need it directly (e.g. PostProcessPass::draw's unsigned-int parameter) rather than through bind()'s implicit active-unit state.
    [[nodiscard]] unsigned int id() const { return id_; }

private:
    explicit Texture(unsigned int id);

    unsigned int id_ = 0;
    int width_ = 0;   // current storage dimensions, so upload can tell a resize from an in-place update
    int height_ = 0;
    std::size_t byteSize_ = 0;  // reported to engine::debug's GPU memory tracker
};

}  // namespace engine::gfx
