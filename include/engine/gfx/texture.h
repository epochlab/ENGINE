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

    // Uploads width*height RGBA float texels (row-major, no padding) as GL_RGBA16F -- a linear EXR's native precision without GL_RGBA32F's extra bandwidth. Clamped to edge, not repeated: this is a single fixed-scale image (the path tracer's display texture), not a tiled texture.
    static Texture createFromFloatPixels(int width, int height, const float* rgba);

    void bind(unsigned int unit) const;

    // Raw GL texture id, for callers that need it directly (e.g. PostProcessPass::draw's unsigned-int parameter) rather than through bind()'s implicit active-unit state.
    [[nodiscard]] unsigned int id() const { return id_; }

private:
    Texture(unsigned int id, std::size_t byteSize);

    unsigned int id_ = 0;
    std::size_t byteSize_ = 0;  // reported to engine::debug's GPU memory tracker
};

}  // namespace engine::gfx
