#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include <glm/glm.hpp>

#include "engine/gfx/scalar_type.h"

namespace engine::gfx {

// CPU-side decode of a linear scanline EXR, shared by Texture's GPU upload path and any CPU-side consumer needing the same pixel data without a GPU round-trip (SH irradiance projection, path tracer material/environment lookups). Row 0 is the top, matching EXR/glTF's v=0-at-top convention.
struct HdrImage {
    int width = 0;
    int height = 0;
    std::vector<float> rgba;  // row-major, 4 floats/texel, linear light
};

// A scene input image (environment HDRI, material texture) stored at a chosen ScalarType: RGBA interleaved, row-major, row 0 = top. The component type is a runtime tag fixed per image, so each sample dispatches once and the branch is perfectly predicted (PBRT-v4 Image's PixelFormat design).
struct ImageTexture {
    int width = 0;
    int height = 0;
    std::variant<std::vector<float>, std::vector<Half>> texels;

    // Texel (x, y), widened to float; x in [0, width), y in [0, height).
    [[nodiscard]] glm::vec4 texel(int x, int y) const;
};

// Loads path's R/G/B/A straight into type's storage (OpenEXR HALF or FLOAT slices, no float intermediate). Same failure contract as loadExr; at Float16 a source at or above binary16's overflow threshold (the tie point one half-ulp above kHalfMax) becomes Inf and is rejected by that same all-finite check, naming the type; the half-ulp band below it rounds to kHalfMax like any other value.
[[nodiscard]] std::optional<ImageTexture> loadImageTexture(const std::string& path, ScalarType type);

// Loads path via OpenEXR's InputFile, reading R/G/B/A directly as float -- no half round-trip, so a legitimate source value above half's 65504 ceiling survives instead of becoming inf. Every returned image is guaranteed all-finite: a non-finite texel (already inf/NaN in the source, or otherwise malformed) is rejected at load rather than propagated, since callers like EnvironmentMap build importance-sampling CDFs from these values with no further validation. OpenEXR's C++ API throws Iex-derived (std::exception-derived) exceptions on I/O failure; failures are caught here and translated to nullopt, mirroring ShaderProgram::loadFromFiles's precedent for recoverable bad external input. A source file missing an alpha channel reads back as 1.0.
[[nodiscard]] std::optional<HdrImage> loadExr(const std::string& path);

// Writes image as a linear scanline EXR, the inverse of loadExr and written with the same full-float channels, so a round trip through the pair is lossless. Exists for measurement rather than for asset output: comparing two renders through the 8-bit display-transformed PNG render_beauty writes cannot measure convergence, because the display transform compresses highlights and clamps everything above display range -- exactly the bright, high-variance regions a sampling change moves most, which read as zero difference once clamped. Returns false and reports on I/O failure, mirroring loadExr's nullopt.
[[nodiscard]] bool writeExr(const std::string& path, const HdrImage& image);

// Bilinear sample at uv, wrapping both axes (GL_REPEAT equivalent), filtered in float after widening each texel.
[[nodiscard]] glm::vec4 sampleBilinear(const ImageTexture& image, glm::vec2 uv);

}  // namespace engine::gfx
