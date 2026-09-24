#pragma once

#include <cstdint>

#include <glm/glm.hpp>

namespace pathtracer::scene {

// Owen-scrambled, shuffled Sobol (Sobol 1967; Joe & Kuo 2008; Burley 2020), blue-noise dithered (Georgiev 2016).
// Draws are padded, not slices of one high-dimensional sequence, so a 12th bounce stratifies as well as the first.
// See DERIVATIONS.md "Sobol padding and net quality" -- the three trailing arguments are not interchangeable.
class Sampler {
public:
    // sampleIndex advances per accumulated sample; scrambleSeed is fixed across an accumulation and fresh per render;
    // sampleCount is the total to be accumulated, 0 when unbounded. Swapping the first two degrades this to white noise.
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

// The shift Sampler adds at (pixelX, pixelY) on one dither channel. A dimension set consumes channel 2*set for a 1D
// draw, 2*set and 2*set+1 for a 2D draw. Exposed so sampler_validate can undo the shift and recover the raw sequence:
// both this and a drawn sample are multiples of 2^-24 below 1, so the subtraction is exact in float32.
[[nodiscard]] float blueNoiseDither(int pixelX, int pixelY, int ditherChannel);

}  // namespace pathtracer::scene
