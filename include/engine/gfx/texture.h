#pragma once

namespace engine::gfx {

// Owns one GL_TEXTURE_2D object, move-only.
class Texture {
public:
    ~Texture();

    Texture(const Texture&) = delete;
    Texture& operator=(const Texture&) = delete;
    Texture(Texture&& other) noexcept;
    Texture& operator=(Texture&& other) noexcept;

    // Uploads width*height RGBA float texels (row-major, no padding) as
    // GL_RGBA16F — matches HdrFramebuffer's storage format and a linear
    // EXR's native precision without GL_RGBA32F's extra bandwidth.
    static Texture createFromFloatPixels(int width, int height, const float* rgba);

    // In-memory checkerboard (alternating grey values, deliberately not
    // 0/1 so the unencoded Stage D checkpoint still shows visible
    // structure rather than a binary black/white image) for exercising
    // the render path ahead of Stage E's real EXR loader. Delegates to
    // createFromFloatPixels — no separate upload path to maintain.
    static Texture createPlaceholderCheckerboard(int size);

    void bind(unsigned int unit) const;

private:
    explicit Texture(unsigned int id);

    unsigned int id_ = 0;
};

}  // namespace engine::gfx
