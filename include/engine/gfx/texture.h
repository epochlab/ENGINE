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

    // Uploads width*height RGBA float texels (row-major, no padding) as
    // GL_RGBA16F — matches HdrFramebuffer's storage format and a linear
    // EXR's native precision without GL_RGBA32F's extra bandwidth.
    static Texture createFromFloatPixels(int width, int height, const float* rgba);

    // Loads a scanline EXR via OpenEXR's RgbaInputFile, converts half to
    // float, and uploads through createFromFloatPixels. OpenEXR's C++ API
    // throws Iex-derived (std::exception-derived) exceptions on I/O
    // failure; failures are caught here and translated to nullopt,
    // mirroring ShaderProgram::loadFromFiles's precedent for recoverable
    // bad external input. A source file missing an alpha channel reads
    // back as 1.0 — RgbaInputFile's own documented default fill, not
    // something this loader adds.
    static std::optional<Texture> createFromExr(const std::string& path);

    void bind(unsigned int unit) const;

private:
    Texture(unsigned int id, std::size_t byteSize);

    unsigned int id_ = 0;
    std::size_t byteSize_ = 0;  // reported to engine::debug's GPU memory tracker
};

}  // namespace engine::gfx
