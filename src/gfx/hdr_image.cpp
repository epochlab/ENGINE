#include "engine/gfx/hdr_image.h"

#include <cmath>
#include <exception>
#include <iostream>

#include <OpenEXR/ImfArray.h>
#include <OpenEXR/ImfRgbaFile.h>

namespace engine::gfx {

namespace {

// Wraps a float pixel coordinate into [0, size) the way GL_REPEAT wraps a texture coordinate.
int wrapPixel(int coord, int size) {
    const int wrapped = coord % size;
    return wrapped < 0 ? wrapped + size : wrapped;
}

}  // namespace

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

        // Official RgbaInputFile read idiom (OpenEXR's own rgbaInterfaceExamples.cpp): the base-pointer offset by dw.min handles a non-zero data-window origin, though none of this project's EXR files currently have one.
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

glm::vec4 sampleBilinear(const HdrImage& image, glm::vec2 uv) {
    // Texel-center convention, matching GL_LINEAR.
    const float fx = (uv.x * static_cast<float>(image.width)) - 0.5F;
    const float fy = (uv.y * static_cast<float>(image.height)) - 0.5F;
    const int x0 = static_cast<int>(std::floor(fx));
    const int y0 = static_cast<int>(std::floor(fy));
    const float tx = fx - static_cast<float>(x0);
    const float ty = fy - static_cast<float>(y0);

    const auto texel = [&](int x, int y) {
        const int wx = wrapPixel(x, image.width);
        const int wy = wrapPixel(y, image.height);
        const std::size_t idx = (static_cast<std::size_t>(wy) * static_cast<std::size_t>(image.width) +
                                  static_cast<std::size_t>(wx)) *
                                 4;
        return glm::vec4(image.rgba[idx + 0], image.rgba[idx + 1], image.rgba[idx + 2],
                          image.rgba[idx + 3]);
    };

    const glm::vec4 top = glm::mix(texel(x0, y0), texel(x0 + 1, y0), tx);
    const glm::vec4 bottom = glm::mix(texel(x0, y0 + 1), texel(x0 + 1, y0 + 1), tx);
    return glm::mix(top, bottom, ty);
}

}  // namespace engine::gfx
