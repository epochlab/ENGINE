#pragma once

#include <cstdint>

#include <glm/glm.hpp>

namespace engine::scene {

// Per-pixel-per-sample sampler: Owen-scrambled, shuffled Sobol (Sobol 1967; Joe & Kuo 2008 direction numbers; Burley
// 2020 hash-based scrambling, shuffling and padding), blue-noise dithered across the image (Georgiev & Fajardo 2016).
//
// Every draw is an independently randomized copy of the SAME low Sobol dimensions rather than a slice of one
// high-dimensional sequence -- "padding", Burley 2020 Sec. 5, and what Cycles ships. Each draw gets its own Owen
// scramble and its own Owen-shuffled sample index, so consecutive draws are uncorrelated while each one individually
// keeps the best stratification the sequence offers. That matters because Sobol's 2D projections degrade as the
// dimension pair climbs -- measured here, dimensions (0,1) are a perfect (0,7,2)-net while (62,63) is only a (4,7,2)-net,
// sixteen points per cell instead of one -- so a deep path drawing dimensions 50+ for its last bounces samples them
// worse than white noise would. Padding means a 12-bounce path's last bounce is stratified exactly as well as its first,
// and it removes any dimension budget: there is no depth past which the sampler degrades.
//
// Pixels do NOT get their own randomization. They all draw the one sequence, and what separates them is a toroidal
// shift read from a blue-noise mask tiled over the image -- blue-noise dithered sampling, Georgiev & Fajardo 2016.
// Giving each pixel its own scramble instead (which is what this did before) is the white-noise special case of the
// same construction: it leaves neighbouring pixels' errors independent, which is the arrangement the eye and every
// subsequent filter handle worst. Correlating them moves the error out of the low frequencies without reducing it, so
// this is a perceptual change, not a convergence one -- expect the RMSE curve to sit where it did.
// The shift is a property of (pixel, dither channel), fixed across the accumulation. It must NOT vary with the sample
// index: that would be a fresh random rotation per sample and would destroy the stratification the sequence exists to
// provide, exactly as a per-sample scrambleSeed would. It must vary per channel: one shared shift puts a pixel's whole
// sample vector on the diagonal of the d-torus, and a neighbourhood of pixels then integrates the path integrand along
// a line rather than over the torus, leaving a slowly varying residual that is itself low-frequency error. Measured at
// 199x white noise in the lowest octave before each channel was given its own translation of the mask.
//
// The two trailing constructor arguments have distinct, non-interchangeable roles, and getting them the wrong way round
// silently degrades this to white noise (which is what it was before -- see sampler.cpp):
//   sampleIndex   ADVANCES per accumulated sample. It selects the point along the sequence, and it is what makes a
//                 pixel's accumulated point set stratified rather than N independent points.
//   scrambleSeed  is FIXED across an accumulation and fresh per render/generation. It randomizes the sequence (Owen
//                 1995) so different pixels and different renders decorrelate without disturbing stratification.
// Averaging N independently randomized copies of a SINGLE point is plain Monte Carlo no matter which sequence produced
// the point, so a scrambleSeed that varies per sample -- or a sampleIndex that does not advance -- forfeits the entire
// benefit. sampler_validate's pass-direction check is the gate on exactly that mistake.
// sampleCount is the total this image will accumulate, and it bounds the sequence the sampler draws from (Burley 2020
// Sec. 5.2; Cycles calls it shuffled_index_mask). It is a performance parameter, not a correctness one -- an overstated
// count only costs speed -- but understating it would alias two sample indices onto one sequence point, so the sampler
// floors the bound at the index it is actually given. Pass 0 when the accumulation is unbounded: that selects the full
// 32-bit sequence, which is correct and simply the slowest option.
class Sampler {
public:
    Sampler(int pixelX, int pixelY, int sampleIndex, int sampleCount, std::uint32_t scrambleSeed);
    // Each call consumes one padded dimension set. next2D's pair is jointly stratified; two next1D calls are not, so
    // draw a 2D quantity with next2D rather than two next1D calls.
    [[nodiscard]] float next1D();
    [[nodiscard]] glm::vec2 next2D();

private:
    int pixelX_;
    int pixelY_;
    std::uint32_t sampleIndex_;
    std::uint32_t indexMask_;
    std::uint32_t scrambleSeed_;
    int dimensionSet_ = 0;
};

// The shift Sampler applies at (pixelX, pixelY) on one dither channel, as the [0,1) value it adds. Channels are
// numbered as the dimension sets' hash inputs are: a set consumes 2*set for a 1D draw, and 2*set and 2*set+1 for the two
// halves of a 2D draw. Exposed because it is the only way to undo the shift exactly and recover the underlying
// sequence, which is what lets sampler_validate keep asserting the net properties with no tolerance: both this and a
// drawn sample are multiples of 2^-24 below 1, so the subtraction is exact in float32.
[[nodiscard]] float blueNoiseDither(int pixelX, int pixelY, int ditherChannel);

}  // namespace engine::scene
