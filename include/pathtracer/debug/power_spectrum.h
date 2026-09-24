#pragma once

#include <array>
#include <vector>

namespace pathtracer::debug {

// Octaves are the dyadic partition of the frequency axis: parameter-free, so "power at low frequency" is read off a
// fixed band rather than off a cutoff chosen once the answer is known. Eight of them reach 1/256 of Nyquist, below the
// fundamental of any field measured here; anything lower folds into the last band rather than being dropped.
inline constexpr int kSpectrumBands = 8;

// Radially averaged power spectrum of a scalar field, as total power per octave of spatial frequency. Index 0 is the
// highest octave (Nyquist/2 to Nyquist), index kSpectrumBands-1 the lowest. The mean is removed first and a separable
// DFT is used, not an FFT. See DERIVATIONS.md "Radially averaged power spectrum".
[[nodiscard]] std::array<double, kSpectrumBands> octaveBandPower(const std::vector<double>& field, int width,
                                                                 int height);

// The band distribution a white-noise field of these dimensions is expected to have, as shares summing to 1: the
// analytic null any measured spectrum is blue or red relative to, bands differing too widely in width for a raw
// share to mean anything. Never sampled -- a measured null once correlated with the field and inflated it eightfold.
[[nodiscard]] std::array<double, kSpectrumBands> whiteNoiseBandShare(int width, int height);

}  // namespace pathtracer::debug
