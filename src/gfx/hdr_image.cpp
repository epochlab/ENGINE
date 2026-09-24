#include "pathtracer/gfx/hdr_image.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <iostream>
#include <type_traits>
#include <utility>

#include <OpenEXR/ImfChannelList.h>
#include <OpenEXR/ImfChromaticities.h>
#include <OpenEXR/ImfFrameBuffer.h>
#include <OpenEXR/ImfInputFile.h>
#include <OpenEXR/ImfOutputFile.h>
#include <OpenEXR/ImfStandardAttributes.h>

namespace pathtracer::gfx {

namespace {

// Wraps a float pixel coordinate into [0, size) the way GL_REPEAT wraps a texture coordinate.
int wrapPixel(int coord, int size) {
    const int wrapped = coord % size;
    return wrapped < 0 ? wrapped + size : wrapped;
}

// The engine assumes every linear EXR is Rec.709-primaried and never checked: an ACEScg or P3 asset reads back wrong, silently.
bool chromaticitiesMismatchRec709(const Imf::Chromaticities& c) {
    constexpr float kTolerance = 1e-3F;
    const Imf::Chromaticities rec709;
    const auto differs = [](const Imath::V2f& a, const Imath::V2f& b) {
        return std::abs(a.x - b.x) > kTolerance || std::abs(a.y - b.y) > kTolerance;
    };
    return differs(c.red, rec709.red) || differs(c.green, rec709.green) ||
           differs(c.blue, rec709.blue) || differs(c.white, rec709.white);
}

template <typename T>
inline constexpr Imf::PixelType kExrPixelType = Imf::FLOAT;
template <>
inline constexpr Imf::PixelType kExrPixelType<Half> = Imf::HALF;

template <typename T>
inline constexpr ScalarType kScalarType = ScalarType::Float32;
template <>
inline constexpr ScalarType kScalarType<Half> = ScalarType::Float16;

template <typename T>
struct ExrPixels {
    int width;
    int height;
    std::vector<T> rgba;
};

std::size_t texelIndex(int x, int y, int width) {
    return ((static_cast<std::size_t>(y) * static_cast<std::size_t>(width)) + static_cast<std::size_t>(x)) * 4;
}

// Calls f through a two-way branch on the variant's fixed index, so f inlines; std::visit would dispatch indirectly per sample.
template <typename F>
glm::vec4 withTexels(const ImageTexture& image, F&& f) {
    if (const auto* half = std::get_if<std::vector<Half>>(&image.texels)) {
        return f(*half);
    }
    return f(*std::get_if<std::vector<float>>(&image.texels));
}

template <typename T>
glm::vec4 widenTexel(const std::vector<T>& rgba, std::size_t idx) {
    return {static_cast<float>(rgba[idx + 0]), static_cast<float>(rgba[idx + 1]), static_cast<float>(rgba[idx + 2]),
            static_cast<float>(rgba[idx + 3])};
}

// Reads path's R/G/B/A as interleaved T, missing alpha reading 1. nullopt on I/O failure or a non-finite texel: callers never recheck.
template <typename T>
std::optional<ExrPixels<T>> readExrRgba(const std::string& path) {
    try {
        Imf::InputFile file(path.c_str());
        const Imath::Box2i& dw = file.header().dataWindow();
        if (dw.isEmpty()) {
            std::cerr << "readExrRgba: empty data window in " << path << '\n';
            return std::nullopt;
        }

        // Primaries, not transfer: absence is the documented Rec.709 assumption, while a present-but-different attribute is a real defect.
        if (Imf::hasChromaticities(file.header()) &&
            chromaticitiesMismatchRec709(Imf::chromaticities(file.header()))) {
            std::cerr << "readExrRgba: " << path
                      << " declares non-Rec.709 chromaticities -- colours will be systematically wrong "
                         "under this engine's Rec.709 assumption\n";
        }

        const int width = dw.max.x - dw.min.x + 1;
        const int height = dw.max.y - dw.min.y + 1;

        ExrPixels<T> image{width, height, {}};
        image.rgba.assign(static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 4, T(0));
        // Missing-alpha source defaults to 1.0, matching RgbaInputFile's documented fill this loader previously relied on.
        for (std::size_t i = 3; i < image.rgba.size(); i += 4) {
            image.rgba[i] = T(1);
        }

        // Interleaved RGBA straight into T: a Float16 read turns an over-range source into Inf, caught by the check below.
        char* base = reinterpret_cast<char*>(image.rgba.data()) -
                     ((static_cast<std::size_t>(dw.min.x) + (static_cast<std::size_t>(dw.min.y) * width)) *
                      4 * sizeof(T));
        const std::size_t xStride = 4 * sizeof(T);
        const std::size_t yStride = xStride * static_cast<std::size_t>(width);
        Imf::FrameBuffer frameBuffer;
        const std::array<std::pair<const char*, int>, 4> planes{
            {{"R", 0}, {"G", 1}, {"B", 2}, {"A", 3}}};
        for (const auto& [name, offset] : planes) {
            if (file.header().channels().findChannel(name) != nullptr) {
                frameBuffer.insert(name, Imf::Slice(kExrPixelType<T>, base + (offset * sizeof(T)),
                                                     xStride, yStride));
            }
        }
        file.setFrameBuffer(frameBuffer);
        file.readPixels(dw.min.y, dw.max.y);

        for (const T texel : image.rgba) {
            if (!std::isfinite(static_cast<float>(texel))) {
                std::cerr << "readExrRgba: non-finite texel in " << path << " read as " << scalarTypeName(kScalarType<T>)
                          << " (source Inf/NaN";
                if constexpr (std::is_same_v<T, Half>) {
                    std::cerr << ", or a finite value above binary16's max " << kHalfMax;
                }
                std::cerr << ") -- rejecting rather than propagating garbage into importance sampling\n";
                return std::nullopt;
            }
        }
        return image;
    } catch (const std::exception& e) {
        std::cerr << "readExrRgba: failed to load " << path << ": " << e.what() << '\n';
        return std::nullopt;
    }
}

}  // namespace

std::optional<HdrImage> loadExr(const std::string& path) {
    std::optional<ExrPixels<float>> pixels = readExrRgba<float>(path);
    if (!pixels) {
        return std::nullopt;
    }
    return HdrImage{pixels->width, pixels->height, std::move(pixels->rgba)};
}

std::optional<ImageTexture> loadImageTexture(const std::string& path, ScalarType type) {
    const auto load = [&path]<typename T>() -> std::optional<ImageTexture> {
        std::optional<ExrPixels<T>> pixels = readExrRgba<T>(path);
        if (!pixels) {
            return std::nullopt;
        }
        return ImageTexture{pixels->width, pixels->height, std::move(pixels->rgba)};
    };
    switch (type) {
        case ScalarType::Float16:
            return load.template operator()<Half>();
        case ScalarType::Float32:
            return load.template operator()<float>();
    }
    // Not dead: a scoped enum holds any value of its underlying type, so falling off a covered switch is still undefined behaviour.
    return std::nullopt;
}

glm::vec4 ImageTexture::texel(int x, int y) const {
    const std::size_t idx = texelIndex(x, y, width);
    return withTexels(*this, [idx](const auto& rgba) { return widenTexel(rgba, idx); });
}

bool writeExr(const std::string& path, const HdrImage& image) {
    try {
        Imf::Header header(image.width, image.height);
        for (const char* channel : {"R", "G", "B", "A"}) {
            header.channels().insert(channel, Imf::Channel(Imf::FLOAT));
        }

        Imf::FrameBuffer frameBuffer;
        // const_cast because OpenEXR's OutputFile takes a mutable base pointer though it only reads through it.

        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast) -- required by the OpenEXR API, see above.
        auto* base = const_cast<float*>(image.rgba.data());
        const std::size_t xStride = sizeof(float) * 4;
        const std::size_t yStride = xStride * static_cast<std::size_t>(image.width);
        const std::array<const char*, 4> names = {"R", "G", "B", "A"};
        for (std::size_t c = 0; c < names.size(); ++c) {
            frameBuffer.insert(names[c],
                                Imf::Slice(Imf::FLOAT, reinterpret_cast<char*>(base + c), xStride, yStride));
        }

        Imf::OutputFile file(path.c_str(), header);
        file.setFrameBuffer(frameBuffer);
        file.writePixels(image.height);
        return true;
    } catch (const std::exception& e) {
        std::cerr << "writeExr: failed to write " << path << ": " << e.what() << '\n';
        return false;
    }
}

glm::vec4 sampleBilinear(const ImageTexture& image, glm::vec2 uv, WrapMode wrap) {
    // Texel-center convention, matching GL_LINEAR.
    const float fx = (uv.x * static_cast<float>(image.width)) - 0.5F;
    const float fy = (uv.y * static_cast<float>(image.height)) - 0.5F;
    const int x0 = static_cast<int>(std::floor(fx));
    const int y0 = static_cast<int>(std::floor(fy));
    const float tx = fx - static_cast<float>(x0);
    const float ty = fy - static_cast<float>(y0);
    const int wx0 = wrapPixel(x0, image.width);
    const int wx1 = wrapPixel(x0 + 1, image.width);
    // Clamping v holds the pole row instead of fetching the opposite pole, which wrapping v does at both ends of an equirect map.
    const auto resolveV = [&](int y) {
        return wrap == WrapMode::ClampV ? std::clamp(y, 0, image.height - 1) : wrapPixel(y, image.height);
    };
    const int wy0 = resolveV(y0);
    const int wy1 = resolveV(y0 + 1);

    return withTexels(image, [&](const auto& rgba) {
        const auto texel = [&](int x, int y) { return widenTexel(rgba, texelIndex(x, y, image.width)); };
        const glm::vec4 top = glm::mix(texel(wx0, wy0), texel(wx1, wy0), tx);
        const glm::vec4 bottom = glm::mix(texel(wx0, wy1), texel(wx1, wy1), tx);
        return glm::mix(top, bottom, ty);
    });
}

}  // namespace pathtracer::gfx
