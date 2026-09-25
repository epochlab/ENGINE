#pragma once

#include <array>
#include <vector>

namespace pathtracer::debug {

// Dyadic partition of the frequency axis, parameter-free. Eight octaves reach 1/256 of Nyquist, below any fundamental measured here.
inline constexpr int kSpectrumBands = 8;

// Radially averaged power per octave; index 0 is the highest octave.
[[nodiscard]] std::array<double, kSpectrumBands> octaveBandPower(const std::vector<double>& field, int width,
                                                                 int height);

// Band shares expected of a white-noise field of these dimensions: the analytic null. Never sampled -- a measured one correlates.
[[nodiscard]] std::array<double, kSpectrumBands> whiteNoiseBandShare(int width, int height);

}  // namespace pathtracer::debug
