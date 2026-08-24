#pragma once

#include <cstddef>

namespace engine::gfx {

// Owns one GL_TEXTURE_CUBE_MAP object with a full mip chain preallocated
// (all 6 faces x mipCount levels, empty at construction), move-only —
// same ownership pattern as Texture. GL 4.1 has no glTexStorage2D for
// cubemaps, so each level is allocated explicitly via glTexImage2D
// rather than a single storage call.
class CubemapTexture {
public:
    // baseFaceSize is the width/height (square) of mip 0; mipCount
    // levels are allocated, each half the previous (minimum 1x1).
    CubemapTexture(int baseFaceSize, int mipCount);
    ~CubemapTexture();

    CubemapTexture(const CubemapTexture&) = delete;
    CubemapTexture& operator=(const CubemapTexture&) = delete;
    CubemapTexture(CubemapTexture&& other) noexcept;
    CubemapTexture& operator=(CubemapTexture&& other) noexcept;

    [[nodiscard]] unsigned int id() const { return id_; }
    [[nodiscard]] int mipCount() const { return mipCount_; }
    [[nodiscard]] int faceSizeAtMip(int mip) const;

    // Attaches the given face (0-5, GL_TEXTURE_CUBE_MAP_POSITIVE_X order:
    // +X,-X,+Y,-Y,+Z,-Z) at mip as fbo's GL_COLOR_ATTACHMENT0, binds fbo,
    // and sets the viewport to that mip's face size. Caller issues the
    // draw immediately after.
    void attachFaceForWrite(unsigned int fbo, int face, int mip) const;

    void bind(unsigned int unit) const;

private:
    unsigned int id_ = 0;
    int baseFaceSize_ = 0;
    int mipCount_ = 0;
    std::size_t byteSize_ = 0;  // reported to engine::debug's GPU memory tracker
};

}  // namespace engine::gfx
