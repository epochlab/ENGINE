#include "engine/gfx/cubemap_texture.h"

#include <algorithm>
#include <utility>

#include <GL/glew.h>

#include "engine/debug/memory_tracker.h"
#include "engine/gfx/gl_debug.h"

namespace engine::gfx {

CubemapTexture::CubemapTexture(int baseFaceSize, int mipCount)
    : baseFaceSize_(baseFaceSize), mipCount_(mipCount) {
    GL_CALL(glGenTextures(1, &id_));
    GL_CALL(glBindTexture(GL_TEXTURE_CUBE_MAP, id_));

    std::size_t totalBytes = 0;
    for (int level = 0; level < mipCount_; ++level) {
        const int size = faceSizeAtMip(level);
        for (int face = 0; face < 6; ++face) {
            GL_CALL(glTexImage2D(static_cast<unsigned int>(GL_TEXTURE_CUBE_MAP_POSITIVE_X + face),
                                  level, GL_RGBA16F, size, size, 0, GL_RGBA, GL_FLOAT, nullptr));
        }
        // RGBA16F = 4 channels * 2 bytes/channel, 6 faces per level.
        totalBytes += static_cast<std::size_t>(size) * static_cast<std::size_t>(size) * 8 * 6;
    }

    GL_CALL(glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER,
                             mipCount_ > 1 ? GL_LINEAR_MIPMAP_LINEAR : GL_LINEAR));
    GL_CALL(glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
    GL_CALL(glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
    GL_CALL(glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));
    GL_CALL(glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE));
    GL_CALL(glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_BASE_LEVEL, 0));
    GL_CALL(glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAX_LEVEL, mipCount_ - 1));
    GL_CALL(glBindTexture(GL_TEXTURE_CUBE_MAP, 0));

    byteSize_ = totalBytes;
    engine::debug::trackGpuAlloc(byteSize_);
}

CubemapTexture::~CubemapTexture() {
    if (id_ != 0) {
        engine::debug::trackGpuFree(byteSize_);
        glDeleteTextures(1, &id_);
    }
}

CubemapTexture::CubemapTexture(CubemapTexture&& other) noexcept
    : id_(std::exchange(other.id_, 0)),
      baseFaceSize_(std::exchange(other.baseFaceSize_, 0)),
      mipCount_(std::exchange(other.mipCount_, 0)),
      byteSize_(std::exchange(other.byteSize_, 0)) {}

CubemapTexture& CubemapTexture::operator=(CubemapTexture&& other) noexcept {
    if (this != &other) {
        if (id_ != 0) {
            engine::debug::trackGpuFree(byteSize_);
            glDeleteTextures(1, &id_);
        }
        id_ = std::exchange(other.id_, 0);
        baseFaceSize_ = std::exchange(other.baseFaceSize_, 0);
        mipCount_ = std::exchange(other.mipCount_, 0);
        byteSize_ = std::exchange(other.byteSize_, 0);
    }
    return *this;
}

int CubemapTexture::faceSizeAtMip(int mip) const {
    return std::max(1, baseFaceSize_ >> mip);
}

void CubemapTexture::attachFaceForWrite(unsigned int fbo, int face, int mip) const {
    GL_CALL(glBindFramebuffer(GL_FRAMEBUFFER, fbo));
    GL_CALL(glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                                    static_cast<unsigned int>(GL_TEXTURE_CUBE_MAP_POSITIVE_X + face),
                                    id_, mip));
    const int size = faceSizeAtMip(mip);
    GL_CALL(glViewport(0, 0, size, size));
}

// Not wrapped in GL_CALL: mirrors Texture::bind, called every frame once pbr.frag samples the prefiltered specular cubemap.
void CubemapTexture::bind(unsigned int unit) const {
    glActiveTexture(GL_TEXTURE0 + unit);
    glBindTexture(GL_TEXTURE_CUBE_MAP, id_);
}

}  // namespace engine::gfx
