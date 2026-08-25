#include "engine/gfx/texture.h"

#include <cstddef>
#include <utility>

#include <GL/glew.h>

#include "engine/debug/memory_tracker.h"
#include "engine/gfx/gl_debug.h"
#include "engine/gfx/hdr_image.h"

namespace engine::gfx {

Texture::Texture(unsigned int id, std::size_t byteSize) : id_(id), byteSize_(byteSize) {
    engine::debug::trackGpuAlloc(byteSize_);
}

Texture::~Texture() {
    if (id_ != 0) {
        engine::debug::trackGpuFree(byteSize_);
        glDeleteTextures(1, &id_);
    }
}

Texture::Texture(Texture&& other) noexcept
    : id_(std::exchange(other.id_, 0)), byteSize_(std::exchange(other.byteSize_, 0)) {}

Texture& Texture::operator=(Texture&& other) noexcept {
    if (this != &other) {
        if (id_ != 0) {
            engine::debug::trackGpuFree(byteSize_);
            glDeleteTextures(1, &id_);
        }
        id_ = std::exchange(other.id_, 0);
        byteSize_ = std::exchange(other.byteSize_, 0);
    }
    return *this;
}

Texture Texture::createFromFloatPixels(int width, int height, const float* rgba,
                                        unsigned int wrapMode) {
    unsigned int id = 0;
    GL_CALL(glGenTextures(1, &id));
    GL_CALL(glBindTexture(GL_TEXTURE_2D, id));

    // GL_UNPACK_ALIGNMENT untouched: RGBA float rows are always a multiple of the default 4-byte alignment.
    GL_CALL(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, width, height, 0, GL_RGBA, GL_FLOAT, rgba));
    GL_CALL(glGenerateMipmap(GL_TEXTURE_2D));

    GL_CALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, static_cast<GLint>(wrapMode)));
    GL_CALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, static_cast<GLint>(wrapMode)));
    GL_CALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR));
    GL_CALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));

    GL_CALL(glBindTexture(GL_TEXTURE_2D, 0));
    // RGBA16F = 4 channels * 2 bytes/channel; mip chain adds ~1/3 more.
    const std::size_t baseSize =
        static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 8;
    const std::size_t byteSize = baseSize + baseSize / 3;
    return Texture(id, byteSize);
}

std::optional<Texture> Texture::createFromExr(const std::string& path, unsigned int wrapMode) {
    const std::optional<HdrImage> image = loadExr(path);
    if (!image.has_value()) {
        return std::nullopt;
    }
    return createFromFloatPixels(image->width, image->height, image->rgba.data(), wrapMode);
}

// Not wrapped in GL_CALL: runs every frame.
void Texture::bind(unsigned int unit) const {
    glActiveTexture(GL_TEXTURE0 + unit);
    glBindTexture(GL_TEXTURE_2D, id_);
}

}  // namespace engine::gfx
