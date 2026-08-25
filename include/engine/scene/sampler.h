#pragma once

#include <cstdint>

#include <glm/glm.hpp>

namespace engine::scene {

// PCG32 (O'Neill 2014). Fallback generator once a path's dimension count exceeds Sampler's Halton budget.
class Pcg32 {
public:
    Pcg32(std::uint64_t seed, std::uint64_t sequence);
    [[nodiscard]] std::uint32_t nextU32();
    [[nodiscard]] float nextFloat();  // [0,1)

private:
    std::uint64_t state_;
    std::uint64_t inc_;
};

// Per-pixel-per-sample sampler: randomized Halton (radical inverse, Cranley-Patterson rotated per pixel) for the first 32 scalar dimensions, PCG32 beyond that -- low variance for primary dimensions (pixel jitter, early bounces), correct-by-construction for unbounded path depth beyond the fixed dimension budget.
class Sampler {
public:
    Sampler(int pixelX, int pixelY, int sampleIndex, std::uint32_t runSeed);
    [[nodiscard]] float next1D();
    [[nodiscard]] glm::vec2 next2D();

private:
    Pcg32 rng_;
    Pcg32 rotationRng_;
    int sampleIndex_;
    int dimension_ = 0;
};

}  // namespace engine::scene
