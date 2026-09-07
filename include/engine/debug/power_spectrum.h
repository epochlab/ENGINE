#pragma once

#include <array>
#include <vector>

namespace engine::debug {

// Octaves are the dyadic partition of the frequency axis: parameter-free, so "power at low frequency" is read off a
// fixed band rather than off a cutoff chosen once the answer is known. Eight of them reach 1/256 of Nyquist, below the
// fundamental of any field measured here; anything lower folds into the last band rather than being dropped.
inline constexpr int kSpectrumBands = 8;

// Radially averaged power spectrum of a scalar field, as total power per octave of spatial frequency. Index 0 is the
// highest octave (Nyquist/2 to Nyquist), index kSpectrumBands-1 the lowest.
//
// Exists because two separate questions in this repo are questions about how energy distributes over spatial frequency
// rather than how much of it there is: whether the baked dither mask is actually blue noise (sampler_validate) and
// whether a sampling change rearranged a render's error without changing its magnitude (render_beauty --error-spectrum).
// Both are invisible to any single scalar, and neither is answerable by eye.
//
// The mean is removed first: DC is a bias in the field, not an arrangement of it, and it would otherwise swamp the
// lowest band. Frequencies are normalised per axis (cycles/pixel, Nyquist 0.5) so a non-square field bins correctly.
// Separable DFT, rows then columns -- O(n^3) rather than a direct 2D transform's O(n^4), which is milliseconds at the
// 128^2 of a mask and seconds at render resolution. Not an FFT: these run behind a flag or in a validator, never per
// frame, and a radix-2 implementation would constrain the caller's dimensions to powers of two for no gain here.
[[nodiscard]] std::array<double, kSpectrumBands> octaveBandPower(const std::vector<double>& field, int width,
                                                                 int height);

// The band distribution a white-noise field of these dimensions is expected to have, as shares summing to 1. White noise
// has a flat spectrum, so this is just each band's share of the DFT lattice points -- the analytic null, and the
// reference any measured spectrum is blue or red RELATIVE to. Bands are far from equal in width, so a raw share is not
// interpretable without it.
// Analytic rather than measured from a shuffled control on purpose. A sampled null costs a seed, carries its own
// sampling noise, and -- the failure actually hit here -- can be correlated with the field under test: shuffling a
// 16384-element array with mt19937(1) reproduces the exact permutation bluenoise_mask.cpp uses to seed the mask, which
// inflated this null eightfold and silently loosened the gate built on it.
[[nodiscard]] std::array<double, kSpectrumBands> whiteNoiseBandShare(int width, int height);

}  // namespace engine::debug
