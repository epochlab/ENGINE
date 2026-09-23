#pragma once

#include <array>

#include <glm/glm.hpp>

#include "engine/gfx/hdr_image.h"
#include "engine/scene/thread_pool.h"

namespace engine::debug {

// CPU implementations of the four AOVs that read Beauty rather than the path tracer or the rasterizer: Luminance, Sobel, Gabor and HSV (aov.h's AovSource::BeautyFilter).
// These are the DATA the viewer's assets/shaders/edge_filter.frag and hsv_display.frag display. They deliberately omit four things those shaders do, all of which are display state rather than properties of the AOV: the uExposure multiply (a viewer slider), uChannelView isolation and uInvert (debug toggles), and the final clamp to [0,1] (display range). A consumer training on these needs the unclamped scene-referred quantity, and a clamp at 1.0 would discard every highlight the path tracer worked to resolve.
// Neighbourhood taps clamp to the image edge, matching the GL_CLAMP_TO_EDGE the display texture is sampled with (texture.cpp), so the two paths agree at the border and not only in the interior.
// Row order differs between the two -- HdrImage row 0 is the top, a GL texture's v=0 is the bottom -- which flips the sign of Sobel's gy and swaps Gabor's 45/135 orientations. Neither is observable: Sobel returns a gradient magnitude, and Gabor takes the maximum over an orientation set that the flip maps onto itself.

// Rec.709 luminance weights (ITU-R BT.709-6), the same triple edge_filter.frag's sampleLuminance dots against.
inline constexpr glm::vec3 kRec709LuminanceWeights{0.2126F, 0.7152F, 0.0722F};

inline constexpr int kGaborOrientations = 4;
inline constexpr int kGaborRadius = 2;  // 5x5 support
inline constexpr int kGaborTaps = (2 * kGaborRadius + 1) * (2 * kGaborRadius + 1);

// 4 orientations (0/45/90/135 degrees) x 25 taps of an odd-carrier Gabor filter (Gabor 1946; Daugman 1985): a Gaussian envelope times a sine carrier, laid out orientation-major so index o * 25 + tap.
// Shared with the viewer rather than duplicated: main.cpp uploads this exact array to edge_filter.frag's uGaborKernel uniform, so the CPU and GPU banks are the same weights by construction, the way gbuffer_shading.h already keeps the path tracer and rasterizer in agreement.
[[nodiscard]] std::array<float, kGaborOrientations * kGaborTaps> buildGaborKernel();

// Rec.709 luminance, broadcast to RGB. Single centre tap, no neighbourhood.
[[nodiscard]] engine::gfx::HdrImage luminanceAov(const engine::gfx::HdrImage& beauty,
                                                  engine::scene::ThreadPool& threadPool);

// Gradient magnitude of Luminance under the fixed 3x3 Sobel operator (Sobel & Feldman 1968), broadcast to RGB.
[[nodiscard]] engine::gfx::HdrImage sobelAov(const engine::gfx::HdrImage& beauty,
                                              engine::scene::ThreadPool& threadPool);

// Maximum absolute response of the 4-orientation Gabor bank over Luminance, broadcast to RGB.
[[nodiscard]] engine::gfx::HdrImage gaborAov(const engine::gfx::HdrImage& beauty,
                                              engine::scene::ThreadPool& threadPool);

// Hue/saturation/value in the RGB channels, hue and saturation normalised to [0,1], value left in scene-referred linear so it is not clamped at display white.
[[nodiscard]] engine::gfx::HdrImage hsvAov(const engine::gfx::HdrImage& beauty,
                                            engine::scene::ThreadPool& threadPool);

}  // namespace engine::debug
