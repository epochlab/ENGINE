#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include <glm/glm.hpp>

#include "pathtracer/gfx/scalar_type.h"

namespace pathtracer::gfx {

// CPU-side decode of a linear scanline EXR, shared by Texture's GPU upload path and any CPU consumer wanting the same
// pixels without a GPU round trip.
struct HdrImage {
    int width = 0;
    int height = 0;
    std::vector<float> rgba;  // row-major, 4 floats/texel, linear light
};

// A scene input image (environment HDRI, material texture) at a chosen ScalarType: RGBA interleaved, row-major,
// row 0 at the top. The component type is a runtime tag fixed at load.
struct ImageTexture {
    int width = 0;
    int height = 0;
    std::variant<std::vector<float>, std::vector<Half>> texels;

    // Texel (x, y), widened to float; x in [0, width), y in [0, height).
    [[nodiscard]] glm::vec4 texel(int x, int y) const;
};

// Loads R/G/B/A straight into type's storage (OpenEXR HALF or FLOAT slices, no float intermediate). Same failure and
// all-finite contract as loadExr; at Float16 a source at or above binary16's overflow threshold (the tie point one
// half-ulp above kHalfMax) becomes Inf and is rejected by that check, while the band below it rounds to kHalfMax.
[[nodiscard]] std::optional<ImageTexture> loadImageTexture(const std::string& path, ScalarType type);

// Loads via OpenEXR's InputFile, reading R/G/B/A as float -- no half round trip, so a source value above half's 65504
// ceiling survives. nullopt on I/O failure (Iex exceptions caught here); a file with no alpha channel reads back 1.0.
// Every returned image is all-finite: a non-finite texel is rejected at load, since EnvironmentMap's CDFs never check.
[[nodiscard]] std::optional<HdrImage> loadExr(const std::string& path);

// Writes a linear scanline EXR, the inverse of loadExr and with the same full-float channels, so a round trip is
// lossless. Exists for measurement: an EXR is what a comparison reads, not a display encode.
[[nodiscard]] bool writeExr(const std::string& path, const HdrImage& image);

// Bilinear sample at uv, wrapping both axes (GL_REPEAT equivalent), filtered in float after widening each texel.
[[nodiscard]] glm::vec4 sampleBilinear(const ImageTexture& image, glm::vec2 uv);

}  // namespace pathtracer::gfx
