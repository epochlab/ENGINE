#include "engine/gfx/hdr_image.h"

#include <exception>
#include <iostream>

#include <OpenEXR/ImfArray.h>
#include <OpenEXR/ImfRgbaFile.h>

namespace engine::gfx {

std::optional<HdrImage> loadExr(const std::string& path) {
    try {
        Imf::RgbaInputFile file(path.c_str());
        const auto& dw = file.dataWindow();
        if (dw.isEmpty()) {
            std::cerr << "loadExr: empty data window in " << path << '\n';
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

        HdrImage image;
        image.width = width;
        image.height = height;
        image.rgba.resize(static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 4);
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                const Imf::Rgba& texel = pixels[y][x];
                const std::size_t idx =
                    (static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
                     static_cast<std::size_t>(x)) *
                    4;
                image.rgba[idx + 0] = static_cast<float>(texel.r);
                image.rgba[idx + 1] = static_cast<float>(texel.g);
                image.rgba[idx + 2] = static_cast<float>(texel.b);
                image.rgba[idx + 3] = static_cast<float>(texel.a);
            }
        }
        return image;
    } catch (const std::exception& e) {
        std::cerr << "loadExr: failed to load " << path << ": " << e.what() << '\n';
        return std::nullopt;
    }
}

}  // namespace engine::gfx
