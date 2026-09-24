#include "pathtracer/debug/aov_filters.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include <glm/gtc/constants.hpp>

namespace pathtracer::debug {

namespace {

using pathtracer::gfx::HdrImage;
using pathtracer::scene::ThreadPool;

// Single-channel Rec.709 luminance, the shared input to Sobel and Gabor. Materialised once rather than recomputed per tap: Sobel reads 8 neighbours per pixel and Gabor 25, so the shader's per-tap dot product is up to 25x redundant work that one intermediate plane removes. The shader cannot do this -- a fragment has nowhere to put it -- which is why this is not simply a transcription.
[[nodiscard]] std::vector<float> luminancePlane(const HdrImage& beauty, ThreadPool& threadPool) {
    std::vector<float> plane(static_cast<std::size_t>(beauty.width) * static_cast<std::size_t>(beauty.height));
    threadPool.parallelFor(beauty.height, [&](int y) {
        const std::size_t row = static_cast<std::size_t>(y) * static_cast<std::size_t>(beauty.width);
        for (int x = 0; x < beauty.width; ++x) {
            const std::size_t texel = (row + static_cast<std::size_t>(x)) * 4;
            plane[row + static_cast<std::size_t>(x)] =
                (beauty.rgba[texel] * kRec709LuminanceWeights.r) +
                (beauty.rgba[texel + 1] * kRec709LuminanceWeights.g) +
                (beauty.rgba[texel + 2] * kRec709LuminanceWeights.b);
        }
    });
    return plane;
}

// Neighbour fetch with edge clamping, matching the display texture's GL_CLAMP_TO_EDGE.
[[nodiscard]] float tap(const std::vector<float>& plane, int width, int height, int x, int y) {
    const int cx = std::clamp(x, 0, width - 1);
    const int cy = std::clamp(y, 0, height - 1);
    return plane[(static_cast<std::size_t>(cy) * static_cast<std::size_t>(width)) + static_cast<std::size_t>(cx)];
}

[[nodiscard]] HdrImage makeBroadcastImage(int width, int height) {
    return HdrImage{width, height,
                    std::vector<float>(static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 4, 0.0F)};
}

// Writes one scalar to RGB with alpha 1, the broadcast convention every other AOV uses (gbuffer_shading.h's writeTexel), so a single-channel AOV still goes straight through HdrImage's fixed 4-floats/texel layout.
void writeScalar(HdrImage& out, std::size_t pixel, float value) {
    const std::size_t texel = pixel * 4;
    out.rgba[texel] = value;
    out.rgba[texel + 1] = value;
    out.rgba[texel + 2] = value;
    out.rgba[texel + 3] = 1.0F;
}

}  // namespace

std::array<float, kGaborKernelSize> buildGaborKernel() {
    constexpr float kSigma = 1.4F;
    constexpr float kLambda = 4.0F;
    constexpr float kGamma = 0.5F;
    constexpr std::array<float, kGaborOrientations> kOrientationsDeg = {0.0F, 45.0F, 90.0F, 135.0F};

    std::array<float, kGaborKernelSize> kernel{};
    for (int o = 0; o < kGaborOrientations; ++o) {
        const float theta = glm::radians(kOrientationsDeg[static_cast<std::size_t>(o)]);
        int tapIndex = 0;
        for (int dy = -kGaborRadius; dy <= kGaborRadius; ++dy) {
            for (int dx = -kGaborRadius; dx <= kGaborRadius; ++dx) {
                const auto x = static_cast<float>(dx);
                const auto y = static_cast<float>(dy);
                const float xp = (x * std::cos(theta)) + (y * std::sin(theta));
                const float yp = (-x * std::sin(theta)) + (y * std::cos(theta));
                const float envelope = std::exp(
                    -((xp * xp) + (kGamma * kGamma * yp * yp)) / (2.0F * kSigma * kSigma));
                // Odd/quadrature carrier (sin, not cos) -- edge-sensitive, not bar/ridge-sensitive.
                const float carrier = std::sin(2.0F * glm::pi<float>() * xp / kLambda);
                kernel[(static_cast<std::size_t>(o) * kGaborTaps) + static_cast<std::size_t>(tapIndex)] =
                    envelope * carrier;
                ++tapIndex;
            }
        }
    }
    return kernel;
}

HdrImage luminanceAov(const HdrImage& beauty, ThreadPool& threadPool) {
    const std::vector<float> plane = luminancePlane(beauty, threadPool);
    HdrImage out = makeBroadcastImage(beauty.width, beauty.height);
    threadPool.parallelFor(beauty.height, [&](int y) {
        const std::size_t row = static_cast<std::size_t>(y) * static_cast<std::size_t>(beauty.width);
        for (int x = 0; x < beauty.width; ++x) {
            writeScalar(out, row + static_cast<std::size_t>(x), plane[row + static_cast<std::size_t>(x)]);
        }
    });
    return out;
}

HdrImage sobelAov(const HdrImage& beauty, ThreadPool& threadPool) {
    const std::vector<float> plane = luminancePlane(beauty, threadPool);
    HdrImage out = makeBroadcastImage(beauty.width, beauty.height);
    const int width = beauty.width;
    const int height = beauty.height;
    threadPool.parallelFor(height, [&](int y) {
        for (int x = 0; x < width; ++x) {
            const float tl = tap(plane, width, height, x - 1, y - 1);
            const float t = tap(plane, width, height, x, y - 1);
            const float tr = tap(plane, width, height, x + 1, y - 1);
            const float l = tap(plane, width, height, x - 1, y);
            const float r = tap(plane, width, height, x + 1, y);
            const float bl = tap(plane, width, height, x - 1, y + 1);
            const float b = tap(plane, width, height, x, y + 1);
            const float br = tap(plane, width, height, x + 1, y + 1);
            const float gx = (-tl - (2.0F * l) - bl) + tr + (2.0F * r) + br;
            const float gy = (-bl - (2.0F * b) - br) + tl + (2.0F * t) + tr;
            writeScalar(out, (static_cast<std::size_t>(y) * static_cast<std::size_t>(width)) + static_cast<std::size_t>(x),
                        std::sqrt((gx * gx) + (gy * gy)));
        }
    });
    return out;
}

HdrImage gaborAov(const HdrImage& beauty, ThreadPool& threadPool) {
    // Built once per process, not per call: the bank depends on nothing but its own compile-time parameters, and the viewer pays the same 100 transcendentals once at shader setup.
    static const std::array<float, kGaborKernelSize> kernel = buildGaborKernel();

    const std::vector<float> plane = luminancePlane(beauty, threadPool);
    HdrImage out = makeBroadcastImage(beauty.width, beauty.height);
    const int width = beauty.width;
    const int height = beauty.height;
    threadPool.parallelFor(height, [&](int y) {
        for (int x = 0; x < width; ++x) {
            std::array<float, kGaborOrientations> response{};
            int tapIndex = 0;
            // Each neighbourhood texel is fetched once and reused across all four orientations, as the shader does.
            for (int dy = -kGaborRadius; dy <= kGaborRadius; ++dy) {
                for (int dx = -kGaborRadius; dx <= kGaborRadius; ++dx) {
                    const float lum = tap(plane, width, height, x + dx, y + dy);
                    for (int o = 0; o < kGaborOrientations; ++o) {
                        response[static_cast<std::size_t>(o)] +=
                            kernel[(static_cast<std::size_t>(o) * kGaborTaps) + static_cast<std::size_t>(tapIndex)] * lum;
                    }
                    ++tapIndex;
                }
            }
            float magnitude = 0.0F;
            for (const float value : response) {
                magnitude = std::max(magnitude, std::fabs(value));
            }
            writeScalar(out, (static_cast<std::size_t>(y) * static_cast<std::size_t>(width)) + static_cast<std::size_t>(x),
                        magnitude);
        }
    });
    return out;
}

HdrImage hsvAov(const HdrImage& beauty, ThreadPool& threadPool) {
    HdrImage out = makeBroadcastImage(beauty.width, beauty.height);
    threadPool.parallelFor(beauty.height, [&](int y) {
        const std::size_t row = static_cast<std::size_t>(y) * static_cast<std::size_t>(beauty.width);
        for (int x = 0; x < beauty.width; ++x) {
            const std::size_t texel = (row + static_cast<std::size_t>(x)) * 4;
            const float r = beauty.rgba[texel];
            const float g = beauty.rgba[texel + 1];
            const float b = beauty.rgba[texel + 2];
            const float value = std::max({r, g, b});
            const float chroma = value - std::min({r, g, b});
            // Exact degenerate branches rather than the shader's 1e-10 denominator guard: hue is undefined on the achromatic axis and saturation on black, and the convention is 0 for both (Smith 1978). An epsilon only approximates that, and biases every near-grey pixel.
            const float saturation = value > 0.0F ? chroma / value : 0.0F;
            float hue = 0.0F;
            if (chroma > 0.0F) {
                if (value == r) {
                    hue = (g - b) / chroma;
                } else if (value == g) {
                    hue = 2.0F + ((b - r) / chroma);
                } else {
                    hue = 4.0F + ((r - g) / chroma);
                }
                // Sextant index to a [0,1) turn, wrapping the negative sixth the red sector produces.
                hue /= 6.0F;
                if (hue < 0.0F) {
                    hue += 1.0F;
                }
            }
            out.rgba[texel] = hue;
            out.rgba[texel + 1] = saturation;
            out.rgba[texel + 2] = value;
            out.rgba[texel + 3] = 1.0F;
        }
    });
    return out;
}

}  // namespace pathtracer::debug
