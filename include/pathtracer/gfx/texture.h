#pragma once

#include <cstddef>

#include "pathtracer/gfx/scalar_type.h"

namespace pathtracer::gfx {

// Owns one GL_TEXTURE_2D object, move-only.
class Texture {
public:
    ~Texture();

    Texture(const Texture&) = delete;
    Texture& operator=(const Texture&) = delete;
    Texture(Texture&& other) noexcept;
    Texture& operator=(Texture&& other) noexcept;

    // Uploads width*height RGBA float texels, row-major and unpadded. binary16 overflows above 65504, so scene data may need Float32.
    static Texture createFromFloatPixels(int width, int height, const float* rgba, ScalarType format);

    // Replaces the texel contents. Storage is reallocated only on a dimension change; a same-size update is a glTexSubImage2D.
    void upload(int width, int height, const float* rgba);

    void bind(unsigned int unit) const;

    // Raw GL texture id, for callers needing it directly (PostProcessPass::draw) rather than through bind().
    [[nodiscard]] unsigned int id() const { return id_; }

private:
    Texture(unsigned int id, ScalarType format);

    unsigned int id_ = 0;
    ScalarType format_ = ScalarType::Float16;  // RGBA component type every (re)allocation in upload uses
    int width_ = 0;   // current storage dimensions, so upload can tell a resize from an in-place update
    int height_ = 0;
    std::size_t byteSize_ = 0;  // reported to pathtracer::debug's GPU memory tracker
};

}  // namespace pathtracer::gfx
