#include "engine/gfx/hdr_image.h"

#include <array>
#include <cmath>
#include <exception>
#include <iostream>

#include <OpenEXR/ImfChannelList.h>
#include <OpenEXR/ImfChromaticities.h>
#include <OpenEXR/ImfFrameBuffer.h>
#include <OpenEXR/ImfInputFile.h>
#include <OpenEXR/ImfStandardAttributes.h>

namespace engine::gfx {

namespace {

// Wraps a float pixel coordinate into [0, size) the way GL_REPEAT wraps a texture coordinate.
int wrapPixel(int coord, int size) {
    const int wrapped = coord % size;
    return wrapped < 0 ? wrapped + size : wrapped;
}

// The engine assumes every linear EXR is Rec.709-primaried (ocio_display_transform.cpp's kSceneColorSpace) but never checked -- a linear ACEScg or P3 asset would read back with systematically wrong saturation/hue and nothing would catch it. Imf::Chromaticities' default constructor is itself Rec.709 primaries, so this is a direct comparison against that default rather than a separate hardcoded constant.
bool chromaticitiesMismatchRec709(const Imf::Chromaticities& c) {
    constexpr float kTolerance = 1e-3F;
    const Imf::Chromaticities rec709;
    const auto differs = [](const Imath::V2f& a, const Imath::V2f& b) {
        return std::abs(a.x - b.x) > kTolerance || std::abs(a.y - b.y) > kTolerance;
    };
    return differs(c.red, rec709.red) || differs(c.green, rec709.green) ||
           differs(c.blue, rec709.blue) || differs(c.white, rec709.white);
}

}  // namespace

std::optional<HdrImage> loadExr(const std::string& path) {
    try {
        Imf::InputFile file(path.c_str());
        const Imath::Box2i& dw = file.header().dataWindow();
        if (dw.isEmpty()) {
            std::cerr << "loadExr: empty data window in " << path << '\n';
            return std::nullopt;
        }

        // Primaries, not transfer: "linear" says nothing about which gamut the numbers are linear IN. Absence is left as the documented Rec.709 assumption; a present-but-different attribute is a real defect in the source asset (systematically wrong saturation/hue) but the image data itself is still usable, so this warns rather than rejecting the load the way the non-finite check below does.
        if (Imf::hasChromaticities(file.header()) &&
            chromaticitiesMismatchRec709(Imf::chromaticities(file.header()))) {
            std::cerr << "loadExr: " << path
                      << " declares non-Rec.709 chromaticities -- colours will be systematically wrong "
                         "under this engine's Rec.709 assumption\n";
        }

        const int width = dw.max.x - dw.min.x + 1;
        const int height = dw.max.y - dw.min.y + 1;

        HdrImage image;
        image.width = width;
        image.height = height;
        image.rgba.assign(static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 4, 0.0F);
        // Missing-alpha source defaults to 1.0, matching RgbaInputFile's documented fill this loader previously relied on.
        for (std::size_t i = 3; i < image.rgba.size(); i += 4) {
            image.rgba[i] = 1.0F;
        }

        // Interleaved RGBA float buffer read directly, replacing RgbaInputFile's half decode -- half saturates at 65504 and silently manufactures infinities from finite source values (see EnvironmentMap's importance-sampling CDFs, which this feeds). base offset by dw.min handles a non-zero data-window origin, same idiom RgbaInputFile used internally.
        char* base = reinterpret_cast<char*>(image.rgba.data()) -
                     ((static_cast<std::size_t>(dw.min.x) + (static_cast<std::size_t>(dw.min.y) * width)) *
                      4 * sizeof(float));
        const std::size_t xStride = 4 * sizeof(float);
        const std::size_t yStride = xStride * static_cast<std::size_t>(width);
        Imf::FrameBuffer frameBuffer;
        const std::array<std::pair<const char*, int>, 4> planes{
            {{"R", 0}, {"G", 1}, {"B", 2}, {"A", 3}}};
        for (const auto& [name, offset] : planes) {
            if (file.header().channels().findChannel(name) != nullptr) {
                frameBuffer.insert(name, Imf::Slice(Imf::FLOAT, base + (offset * sizeof(float)),
                                                     xStride, yStride));
            }
        }
        file.setFrameBuffer(frameBuffer);
        file.readPixels(dw.min.y, dw.max.y);

        for (const float texel : image.rgba) {
            if (!std::isfinite(texel)) {
                std::cerr << "loadExr: non-finite texel in " << path
                          << " -- rejecting rather than propagating garbage into importance sampling\n";
                return std::nullopt;
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
