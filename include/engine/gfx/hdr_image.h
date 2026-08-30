#pragma once

#include <optional>
#include <string>
#include <vector>

#include <glm/glm.hpp>

namespace engine::gfx {

// CPU-side decode of a linear scanline EXR, shared by Texture's GPU upload path and any CPU-side consumer needing the same pixel data without a GPU round-trip (SH irradiance projection, path tracer material/environment lookups). Row 0 is the top, matching EXR/glTF's v=0-at-top convention.
struct HdrImage {
    int width = 0;
    int height = 0;
    std::vector<float> rgba;  // row-major, 4 floats/texel, linear light
};

// Loads path via OpenEXR's InputFile, reading R/G/B/A directly as float -- no half round-trip, so a legitimate source value above half's 65504 ceiling survives instead of becoming inf. Every returned image is guaranteed all-finite: a non-finite texel (already inf/NaN in the source, or otherwise malformed) is rejected at load rather than propagated, since callers like EnvironmentMap build importance-sampling CDFs from these values with no further validation. OpenEXR's C++ API throws Iex-derived (std::exception-derived) exceptions on I/O failure; failures are caught here and translated to nullopt, mirroring ShaderProgram::loadFromFiles's precedent for recoverable bad external input. A source file missing an alpha channel reads back as 1.0.
[[nodiscard]] std::optional<HdrImage> loadExr(const std::string& path);

// Bilinear sample at uv, wrapping both axes (GL_REPEAT equivalent).
[[nodiscard]] glm::vec4 sampleBilinear(const HdrImage& image, glm::vec2 uv);

}  // namespace engine::gfx
