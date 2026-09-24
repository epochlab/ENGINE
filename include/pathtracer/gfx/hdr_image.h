#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include <glm/glm.hpp>

#include "pathtracer/gfx/scalar_type.h"

namespace pathtracer::gfx {

// CPU-side decode of a linear scanline EXR, shared by Texture's GPU upload and any CPU consumer wanting the same pixels.
struct HdrImage {
    int width = 0;
    int height = 0;
    std::vector<float> rgba;  // row-major, 4 floats/texel, linear light
};

// A scene input image (environment HDRI, material texture) at a chosen ScalarType: RGBA interleaved, row-major, row 0 top.
struct ImageTexture {
    int width = 0;
    int height = 0;
    std::variant<std::vector<float>, std::vector<Half>> texels;

    // Texel (x, y), widened to float; x in [0, width), y in [0, height).
    [[nodiscard]] glm::vec4 texel(int x, int y) const;
};

// loadExr's contract decoded straight into type's storage; at Float16 an over-range source becomes Inf and is rejected there.
[[nodiscard]] std::optional<ImageTexture> loadImageTexture(const std::string& path, ScalarType type);

// Linear EXR read as float; nullopt on I/O failure, absent alpha reads 1.0, all texels finite -- EnvironmentMap's CDFs never check.
[[nodiscard]] std::optional<HdrImage> loadExr(const std::string& path);

// Writes a linear scanline EXR, loadExr's inverse with the same full-float channels, so a round trip is lossless.
[[nodiscard]] bool writeExr(const std::string& path, const HdrImage& image);

// Bilinear sample at uv, wrapping both axes (GL_REPEAT equivalent), filtered in float after widening each texel.
[[nodiscard]] glm::vec4 sampleBilinear(const ImageTexture& image, glm::vec2 uv);

}  // namespace pathtracer::gfx
