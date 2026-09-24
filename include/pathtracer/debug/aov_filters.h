#pragma once

#include <array>
#include <cstddef>

#include <glm/glm.hpp>

#include "pathtracer/gfx/hdr_image.h"
#include "pathtracer/scene/thread_pool.h"

namespace pathtracer::debug {

// CPU implementations of the four AOVs that read Beauty rather than the path tracer or rasterizer: Luminance, Sobel,
// Gabor and HSV. Taps clamp to the edge, matching the display texture's GL_CLAMP_TO_EDGE. Row order differs from GL
// (HdrImage row 0 is the top), flipping Sobel's gy and swapping Gabor's 45/135 -- neither observable in a magnitude.

// Rec.709 luminance weights (ITU-R BT.709-6), the same triple edge_filter.frag's sampleLuminance dots against.
inline constexpr glm::vec3 kRec709LuminanceWeights{0.2126F, 0.7152F, 0.0722F};

inline constexpr int kGaborOrientations = 4;
inline constexpr int kGaborRadius = 2;  // 5x5 support
inline constexpr int kGaborTaps = (2 * kGaborRadius + 1) * (2 * kGaborRadius + 1);
// Extent of the flattened orientation-major bank. size_t because it is only ever an array bound.
inline constexpr std::size_t kGaborKernelSize = std::size_t{kGaborOrientations} * std::size_t{kGaborTaps};

// 4 orientations (0/45/90/135 deg) x 25 taps of an odd-carrier Gabor filter (Gabor 1946; Daugman 1985): a Gaussian
// envelope times a sine carrier, orientation-major so the index is o * 25 + tap. main.cpp uploads this exact array to
// edge_filter.frag, so the CPU and GPU banks are the same weights by construction.
[[nodiscard]] std::array<float, kGaborKernelSize> buildGaborKernel();

// Rec.709 luminance, broadcast to RGB. Single centre tap, no neighbourhood.
[[nodiscard]] pathtracer::gfx::HdrImage luminanceAov(const pathtracer::gfx::HdrImage& beauty,
                                                  pathtracer::scene::ThreadPool& threadPool);

// Gradient magnitude of Luminance under the fixed 3x3 Sobel operator (Sobel & Feldman 1968), broadcast to RGB.
[[nodiscard]] pathtracer::gfx::HdrImage sobelAov(const pathtracer::gfx::HdrImage& beauty,
                                              pathtracer::scene::ThreadPool& threadPool);

// Maximum absolute response of the 4-orientation Gabor bank over Luminance, broadcast to RGB.
[[nodiscard]] pathtracer::gfx::HdrImage gaborAov(const pathtracer::gfx::HdrImage& beauty,
                                              pathtracer::scene::ThreadPool& threadPool);

// Hue, saturation and value in the RGB channels; hue and saturation normalised to [0,1], value left scene-referred
// linear so it is not clamped at display white.
[[nodiscard]] pathtracer::gfx::HdrImage hsvAov(const pathtracer::gfx::HdrImage& beauty,
                                            pathtracer::scene::ThreadPool& threadPool);

}  // namespace pathtracer::debug
