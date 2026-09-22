#include "engine/gfx/hdr_image.h"

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

template <typename T>
inline constexpr Imf::PixelType kExrPixelType = Imf::FLOAT;
template <>
inline constexpr Imf::PixelType kExrPixelType<Half> = Imf::HALF;

template <typename T>
inline constexpr ScalarType kScalarType = ScalarType::Float32;
template <>
inline constexpr ScalarType kScalarType<Half> = ScalarType::Float16;

// EXR channel names in storage order. The first N are what an N-channel image reads; a 4th is only ever reached by loadExr's full-RGBA path.
inline constexpr std::array<const char*, 4> kChannelNames{"R", "G", "B", "A"};

template <typename T>
struct ExrPixels {
    int width;
    int height;
    std::vector<T> data;
};

std::size_t texelIndex(int x, int y, int width, int channels) {
    return ((static_cast<std::size_t>(y) * static_cast<std::size_t>(width)) + static_cast<std::size_t>(x)) *
           static_cast<std::size_t>(channels);
}

// Calls f with the stored texel vector through a two-way branch on the variant's fixed index, so f inlines; std::visit dispatches through libc++'s function-pointer table (__fmatrix), an indirect call per sample.
template <int N, typename F>
glm::vec<N, float, glm::defaultp> withTexels(const ImageTexture<N>& image, F&& f) {
    if (const auto* half = std::get_if<std::vector<Half>>(&image.texels)) {
        return f(*half);
    }
    return f(*std::get_if<std::vector<float>>(&image.texels));
}

template <int N, typename T>
glm::vec<N, float, glm::defaultp> widenTexel(const std::vector<T>& data, std::size_t idx) {
    glm::vec<N, float, glm::defaultp> out;
    for (int c = 0; c < N; ++c) {
        out[c] = static_cast<float>(data[idx + static_cast<std::size_t>(c)]);
    }
    return out;
}

// Reads the first N of path's R/G/B/A as interleaved T. nullopt, reported, on I/O failure or a non-finite texel -- callers like EnvironmentMap build importance-sampling CDFs from these values with no further validation.
// A channel the source omits reads back as 0, except loadExr's alpha, which reads 1 -- the fill RgbaInputFile documented, and the one this loader previously relied on.
template <typename T, int N>
std::optional<ExrPixels<T>> readExrChannels(const std::string& path) {
    try {
        Imf::InputFile file(path.c_str());
        const Imath::Box2i& dw = file.header().dataWindow();
        if (dw.isEmpty()) {
            std::cerr << "readExrChannels: empty data window in " << path << '\n';
            return std::nullopt;
        }

        // Primaries, not transfer: "linear" says nothing about which gamut the numbers are linear IN. Absence is left as the documented Rec.709 assumption; a present-but-different attribute is a real defect in the source asset (systematically wrong saturation/hue) but the image data itself is still usable, so this warns rather than rejecting the load the way the non-finite check below does.
        if (Imf::hasChromaticities(file.header()) &&
            chromaticitiesMismatchRec709(Imf::chromaticities(file.header()))) {
            std::cerr << "readExrChannels: " << path
                      << " declares non-Rec.709 chromaticities -- colours will be systematically wrong "
                         "under this engine's Rec.709 assumption\n";
        }

        const int width = dw.max.x - dw.min.x + 1;
        const int height = dw.max.y - dw.min.y + 1;

        ExrPixels<T> image{width, height, {}};
        image.data.assign(static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * N, T(0));
        if constexpr (N == 4) {
            for (std::size_t i = 3; i < image.data.size(); i += 4) {
                image.data[i] = T(1);
            }
        }

        // Interleaved read straight into T: OpenEXR converts each source channel to kExrPixelType<T>, so a Float16 read turns a source at or above binary16's overflow threshold into Inf, caught by the finiteness check below rather than reaching EnvironmentMap's CDFs. base offset by dw.min handles a non-zero data-window origin, same idiom RgbaInputFile used internally.
        char* base = reinterpret_cast<char*>(image.data.data()) -
                     ((static_cast<std::size_t>(dw.min.x) + (static_cast<std::size_t>(dw.min.y) * width)) *
                      N * sizeof(T));
        const std::size_t xStride = N * sizeof(T);
        const std::size_t yStride = xStride * static_cast<std::size_t>(width);
        Imf::FrameBuffer frameBuffer;
        for (int c = 0; c < N; ++c) {
            if (file.header().channels().findChannel(kChannelNames[c]) != nullptr) {
                frameBuffer.insert(kChannelNames[c], Imf::Slice(kExrPixelType<T>, base + (c * sizeof(T)),
                                                                 xStride, yStride));
            }
        }
        file.setFrameBuffer(frameBuffer);
        file.readPixels(dw.min.y, dw.max.y);

        for (const T texel : image.data) {
            if (!std::isfinite(static_cast<float>(texel))) {
                std::cerr << "readExrChannels: non-finite texel in " << path << " read as " << scalarTypeName(kScalarType<T>)
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
        std::cerr << "readExrChannels: failed to load " << path << ": " << e.what() << '\n';
        return std::nullopt;
    }
}

}  // namespace

std::optional<HdrImage> loadExr(const std::string& path) {
    std::optional<ExrPixels<float>> pixels = readExrChannels<float, 4>(path);
    if (!pixels) {
        return std::nullopt;
    }
    return HdrImage{pixels->width, pixels->height, std::move(pixels->data)};
}

template <int N>
std::optional<ImageTexture<N>> loadImageTexture(const std::string& path, ScalarType type) {
    const auto load = [&path]<typename T>() -> std::optional<ImageTexture<N>> {
        std::optional<ExrPixels<T>> pixels = readExrChannels<T, N>(path);
        if (!pixels) {
            return std::nullopt;
        }
        return ImageTexture<N>{pixels->width, pixels->height, std::move(pixels->data)};
    };
    switch (type) {
        case ScalarType::Float16:
            return load.template operator()<Half>();
        case ScalarType::Float32:
            return load.template operator()<float>();
    }
    return std::nullopt;
}

template <int N>
glm::vec<N, float, glm::defaultp> ImageTexture<N>::texel(int x, int y) const {
    const std::size_t idx = texelIndex(x, y, width, N);
    return withTexels(*this, [idx](const auto& data) { return widenTexel<N>(data, idx); });
}

bool writeExr(const std::string& path, const HdrImage& image) {
    try {
        Imf::Header header(image.width, image.height);
        for (const char* channel : kChannelNames) {
            header.channels().insert(channel, Imf::Channel(Imf::FLOAT));
        }

        Imf::FrameBuffer frameBuffer;
        // const_cast because OpenEXR's OutputFile API takes a mutable base pointer even though it only reads through it
        // on write; the buffer itself is never modified here.
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast) -- required by the OpenEXR API, see above.
        auto* base = const_cast<float*>(image.rgba.data());
        const std::size_t xStride = sizeof(float) * 4;
        const std::size_t yStride = xStride * static_cast<std::size_t>(image.width);
        for (std::size_t c = 0; c < kChannelNames.size(); ++c) {
            frameBuffer.insert(kChannelNames[c],
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

template <int N>
glm::vec<N, float, glm::defaultp> sampleBilinear(const ImageTexture<N>& image, glm::vec2 uv) {
    // Texel-center convention, matching GL_LINEAR.
    const float fx = (uv.x * static_cast<float>(image.width)) - 0.5F;
    const float fy = (uv.y * static_cast<float>(image.height)) - 0.5F;
    const int x0 = static_cast<int>(std::floor(fx));
    const int y0 = static_cast<int>(std::floor(fy));
    const float tx = fx - static_cast<float>(x0);
    const float ty = fy - static_cast<float>(y0);
    const int wx0 = wrapPixel(x0, image.width);
    const int wx1 = wrapPixel(x0 + 1, image.width);
    const int wy0 = wrapPixel(y0, image.height);
    const int wy1 = wrapPixel(y0 + 1, image.height);

    return withTexels(image, [&](const auto& data) {
        const auto texel = [&](int x, int y) { return widenTexel<N>(data, texelIndex(x, y, image.width, N)); };
        const auto top = glm::mix(texel(wx0, wy0), texel(wx1, wy0), tx);
        const auto bottom = glm::mix(texel(wx0, wy1), texel(wx1, wy1), tx);
        return glm::mix(top, bottom, ty);
    });
}

// The two channel counts the engine's material slots and environment map resolve to (material.h, environment_map.h): scalar maps (roughness, bump) and colour/direction maps (base colour, normal, specular, HDRI). Explicit so the definitions above stay out of the header.
template struct ImageTexture<1>;
template struct ImageTexture<3>;
template std::optional<ImageTexture<1>> loadImageTexture<1>(const std::string&, ScalarType);
template std::optional<ImageTexture<3>> loadImageTexture<3>(const std::string&, ScalarType);
template glm::vec<1, float, glm::defaultp> sampleBilinear<1>(const ImageTexture<1>&, glm::vec2);
template glm::vec<3, float, glm::defaultp> sampleBilinear<3>(const ImageTexture<3>&, glm::vec2);

}  // namespace engine::gfx
