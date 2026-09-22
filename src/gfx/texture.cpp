#include "engine/gfx/texture.h"

#include <cstddef>
#include <utility>

#include <GL/glew.h>

#include "engine/debug/memory_tracker.h"
#include "engine/gfx/gl_debug.h"

namespace engine::gfx {

namespace {

// Sized RGBA float internal format for each component type.
GLint glInternalFormat(ScalarType format) {
    switch (format) {
        case ScalarType::Float16:
            return GL_RGBA16F;
        case ScalarType::Float32:
            return GL_RGBA32F;
    }
    return 0;
}

// Client-side component type of the pixels handed to glTex(Sub)Image2D. Matched to the internal format above so the driver copies rather than converts: GL_HALF_FLOAT is IEEE 754 binary16 (GL 4.1 core, table 8.2), the same layout scalar_type.h asserts for Half.
GLenum glComponentType(ScalarType format) {
    switch (format) {
        case ScalarType::Float16:
            return GL_HALF_FLOAT;
        case ScalarType::Float32:
            return GL_FLOAT;
    }
    return 0;
}

}  // namespace

Texture::Texture(unsigned int id, ScalarType format) : id_(id), format_(format) {}

Texture::~Texture() {
    if (id_ != 0) {
        engine::debug::trackGpuFree(byteSize_);
        glDeleteTextures(1, &id_);
    }
}

Texture::Texture(Texture&& other) noexcept
    : id_(std::exchange(other.id_, 0)),
      format_(other.format_),
      width_(std::exchange(other.width_, 0)),
      height_(std::exchange(other.height_, 0)),
      byteSize_(std::exchange(other.byteSize_, 0)) {}

Texture& Texture::operator=(Texture&& other) noexcept {
    if (this != &other) {
        if (id_ != 0) {
            engine::debug::trackGpuFree(byteSize_);
            glDeleteTextures(1, &id_);
        }
        id_ = std::exchange(other.id_, 0);
        format_ = other.format_;
        width_ = std::exchange(other.width_, 0);
        height_ = std::exchange(other.height_, 0);
        byteSize_ = std::exchange(other.byteSize_, 0);
    }
    return *this;
}

Texture Texture::create(int width, int height, const void* texels, ScalarType format) {
    unsigned int id = 0;
    GL_CALL(glGenTextures(1, &id));
    GL_CALL(glBindTexture(GL_TEXTURE_2D, id));

    GL_CALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
    GL_CALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));
    // GL_LINEAR rather than GL_LINEAR_MIPMAP_LINEAR, since there is no longer a mip chain to select from.
    GL_CALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR));
    GL_CALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));

    Texture texture(id, format);
    texture.upload(width, height, texels);
    GL_CALL(glBindTexture(GL_TEXTURE_2D, 0));
    return texture;
}

// Not wrapped in GL_CALL on the in-place path: runs every frame the displayed image changes, and glGetError is a driver sync point -- same convention as bind() below. The resize path is rare enough to check.
void Texture::upload(int width, int height, const void* texels) {
    // GL_UNPACK_ALIGNMENT untouched: an RGBA row is 4 or 8 bytes per texel, both multiples of the default 4-byte alignment.
    if (width == width_ && height == height_) {
        glBindTexture(GL_TEXTURE_2D, id_);
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_RGBA, glComponentType(format_), texels);
        return;
    }
    GL_CALL(glBindTexture(GL_TEXTURE_2D, id_));
    GL_CALL(glTexImage2D(GL_TEXTURE_2D, 0, glInternalFormat(format_), width, height, 0, GL_RGBA,
                          glComponentType(format_), texels));
    width_ = width;
    height_ = height;
    engine::debug::trackGpuFree(byteSize_);
    // No mip chain, so no ~1/3 addition -- the HUD's GPU memory readout reports what is actually allocated.
    byteSize_ = static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 4 * scalarBytes(format_);
    engine::debug::trackGpuAlloc(byteSize_);
}

// Not wrapped in GL_CALL: runs every frame.
void Texture::bind(unsigned int unit) const {
    glActiveTexture(GL_TEXTURE0 + unit);
    glBindTexture(GL_TEXTURE_2D, id_);
}

}  // namespace engine::gfx
