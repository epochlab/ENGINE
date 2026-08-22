#include "engine/gfx/texture.h"

#include <cstddef>
#include <exception>
#include <iostream>
#include <utility>
#include <vector>

#include <GL/glew.h>

#include <OpenEXR/ImfArray.h>
#include <OpenEXR/ImfRgbaFile.h>

#include "engine/debug/memory_tracker.h"
#include "engine/gfx/gl_debug.h"

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

Texture Texture::createFromFloatPixels(int width, int height, const float* rgba) {
    unsigned int id = 0;
    GL_CALL(glGenTextures(1, &id));
    GL_CALL(glBindTexture(GL_TEXTURE_2D, id));

    // GL_UNPACK_ALIGNMENT untouched: RGBA float rows are always a
    // multiple of the default 4-byte alignment.
    GL_CALL(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, width, height, 0, GL_RGBA, GL_FLOAT, rgba));
    GL_CALL(glGenerateMipmap(GL_TEXTURE_2D));

    // Repeat + trilinear: general-purpose material texture (tiled UVs,
    // viewed at arbitrary scale), not a single-scale display-test image.
    GL_CALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT));
    GL_CALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT));
    GL_CALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR));
    GL_CALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));

    GL_CALL(glBindTexture(GL_TEXTURE_2D, 0));
    // RGBA16F = 4 channels * 2 bytes/channel; mip chain adds ~1/3 more.
    const std::size_t baseSize =
        static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 8;
    const std::size_t byteSize = baseSize + baseSize / 3;
    return Texture(id, byteSize);
}

std::optional<Texture> Texture::createFromExr(const std::string& path) {
    try {
        Imf::RgbaInputFile file(path.c_str());
        const auto& dw = file.dataWindow();
        if (dw.isEmpty()) {
            std::cerr << "Texture::createFromExr: empty data window in " << path << '\n';
            return std::nullopt;
        }

        const int width = dw.max.x - dw.min.x + 1;
        const int height = dw.max.y - dw.min.y + 1;

        // Official RgbaInputFile read idiom (OpenEXR's own
        // rgbaInterfaceExamples.cpp): the base-pointer offset by dw.min
        // handles a non-zero data-window origin, though none of this
        // project's EXR files currently have one.
        Imf::Array2D<Imf::Rgba> pixels;
        pixels.resizeErase(height, width);
        file.setFrameBuffer(&pixels[0][0] - dw.min.x - dw.min.y * width, 1, width);
        file.readPixels(dw.min.y, dw.max.y);

        // One-time load, not a hot path: a plain float conversion loop is
        // enough — no half-accepting overload added to
        // createFromFloatPixels for a call site used exactly once.
        //
        // No row flip: EXR scanline 0 is the top of the image, and
        // glTexImage2D's row 0 of the uploaded buffer becomes texture
        // v=0 — which is exactly glTF's UV convention (v=0 at the top).
        // A flip is needed only against a non-glTF, bottom-origin UV
        // convention (Mesh::createQuad()'s hand-authored quad used one;
        // it's gone once that quad is deleted).
        std::vector<float> floatPixels(static_cast<std::size_t>(width) *
                                        static_cast<std::size_t>(height) * 4);
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                const Imf::Rgba& texel = pixels[y][x];
                const std::size_t idx =
                    (static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
                     static_cast<std::size_t>(x)) *
                    4;
                floatPixels[idx + 0] = static_cast<float>(texel.r);
                floatPixels[idx + 1] = static_cast<float>(texel.g);
                floatPixels[idx + 2] = static_cast<float>(texel.b);
                // 1.0 for a source file missing alpha — RgbaInputFile's
                // own documented default fill for a missing A channel.
                // G/B default to 0 (A still defaults to 1.0 above) for
                // a single-channel (R-only) source, e.g. this project's
                // Roughness/Bump/AO maps.
                floatPixels[idx + 3] = static_cast<float>(texel.a);
            }
        }

        return createFromFloatPixels(width, height, floatPixels.data());
    } catch (const std::exception& e) {
        std::cerr << "Texture::createFromExr: failed to load " << path << ": " << e.what()
                   << '\n';
        return std::nullopt;
    }
}

// Not wrapped in GL_CALL: runs every frame.
void Texture::bind(unsigned int unit) const {
    glActiveTexture(GL_TEXTURE0 + unit);
    glBindTexture(GL_TEXTURE_2D, id_);
}

}  // namespace engine::gfx
