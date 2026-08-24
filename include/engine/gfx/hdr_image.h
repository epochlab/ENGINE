#pragma once

#include <optional>
#include <string>
#include <vector>

namespace engine::gfx {

// CPU-side decode of a linear scanline EXR, shared by Texture's GPU
// upload path (createFromExr) and any CPU-side consumer that needs the
// same pixel data without a GPU round-trip (e.g. SH irradiance
// projection over an environment map). Row 0 is the top of the image,
// matching EXR's own scanline order and glTF's v=0-at-top UV convention
// -- no row flip.
struct HdrImage {
    int width = 0;
    int height = 0;
    std::vector<float> rgba;  // row-major, 4 floats/texel, linear light
};

// Loads path via OpenEXR's RgbaInputFile, converting half to float.
// OpenEXR's C++ API throws Iex-derived (std::exception-derived)
// exceptions on I/O failure; failures are caught here and translated to
// nullopt, mirroring ShaderProgram::loadFromFiles's precedent for
// recoverable bad external input. A source file missing an alpha
// channel reads back as 1.0 -- RgbaInputFile's own documented default
// fill, not something this loader adds.
[[nodiscard]] std::optional<HdrImage> loadExr(const std::string& path);

}  // namespace engine::gfx
