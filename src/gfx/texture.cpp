#include "engine/gfx/texture.h"

#include <cstddef>
#include <utility>
#include <vector>

#include <GL/glew.h>

#include "engine/gfx/gl_debug.h"

namespace engine::gfx {

Texture::Texture(unsigned int id) : id_(id) {}

Texture::~Texture() {
    if (id_ != 0) {
        glDeleteTextures(1, &id_);
    }
}

Texture::Texture(Texture&& other) noexcept : id_(std::exchange(other.id_, 0)) {}

Texture& Texture::operator=(Texture&& other) noexcept {
    if (this != &other) {
        if (id_ != 0) {
            glDeleteTextures(1, &id_);
        }
        id_ = std::exchange(other.id_, 0);
    }
    return *this;
}

Texture Texture::createFromFloatPixels(int width, int height, const float* rgba) {
    unsigned int id = 0;
    GL_CALL(glGenTextures(1, &id));
    GL_CALL(glBindTexture(GL_TEXTURE_2D, id));

    // GL_UNPACK_ALIGNMENT untouched: RGBA float rows are always a
    // multiple of the default 4-byte alignment.
    GL_CALL(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, width, height, 0, GL_RGBA, GL_FLOAT, rgba));

    // Clamp + linear: this is a display-test texture, not a tiled
    // material — no wrap artefacts, no mipmaps at a single fixed scale.
    GL_CALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
    GL_CALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));
    GL_CALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR));
    GL_CALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));

    GL_CALL(glBindTexture(GL_TEXTURE_2D, 0));
    return Texture(id);
}

Texture Texture::createPlaceholderCheckerboard(int size) {
    std::vector<float> pixels(static_cast<std::size_t>(size) * static_cast<std::size_t>(size) *
                               4);
    constexpr int kSquarePixels = 8;
    for (int y = 0; y < size; ++y) {
        for (int x = 0; x < size; ++x) {
            const bool isLight = ((x / kSquarePixels) + (y / kSquarePixels)) % 2 == 0;
            const float value = isLight ? 0.5F : 0.04F;
            const std::size_t idx =
                (static_cast<std::size_t>(y) * static_cast<std::size_t>(size) +
                 static_cast<std::size_t>(x)) *
                4;
            pixels[idx + 0] = value;
            pixels[idx + 1] = value;
            pixels[idx + 2] = value;
            pixels[idx + 3] = 1.0F;
        }
    }
    return createFromFloatPixels(size, size, pixels.data());
}

// Not wrapped in GL_CALL: runs every frame.
void Texture::bind(unsigned int unit) const {
    glActiveTexture(GL_TEXTURE0 + unit);
    glBindTexture(GL_TEXTURE_2D, id_);
}

}  // namespace engine::gfx
