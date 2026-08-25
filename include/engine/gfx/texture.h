#pragma once

#include <cstddef>
#include <optional>
#include <string>

namespace engine::gfx {

// Owns one GL_TEXTURE_2D object, move-only.
class Texture {
public:
    ~Texture();

    Texture(const Texture&) = delete;
    Texture& operator=(const Texture&) = delete;
    Texture(Texture&& other) noexcept;
    Texture& operator=(Texture&& other) noexcept;

    // Uploads width*height RGBA float texels (row-major, no padding) as GL_RGBA16F — matches HdrFramebuffer's storage format and a linear EXR's native precision without GL_RGBA32F's extra bandwidth. wrapMode is a raw GLenum (GL_REPEAT, GL_CLAMP_TO_EDGE, ...) — no default, so every call site states its own intent explicitly: tiled material textures want repeat, a single fixed-scale image wants clamp (repeat would blend its opposite edges at the u/v seam under bilinear filtering).
    static Texture createFromFloatPixels(int width, int height, const float* rgba,
                                          unsigned int wrapMode);

    void bind(unsigned int unit) const;

private:
    Texture(unsigned int id, std::size_t byteSize);

    unsigned int id_ = 0;
    std::size_t byteSize_ = 0;  // reported to engine::debug's GPU memory tracker
};

}  // namespace engine::gfx
