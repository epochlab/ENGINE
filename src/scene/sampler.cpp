#include "engine/scene/sampler.h"

#include <cmath>

namespace engine::scene {

namespace {

// SplitMix64 finalizer (Vigna) -- decorrelates nearby (pixel, sample) inputs into unrelated seeds.
std::uint64_t hashSeed(int pixelX, int pixelY, int extra, std::uint32_t runSeed) {
    std::uint64_t h = static_cast<std::uint64_t>(static_cast<std::uint32_t>(pixelX)) * 0x9E3779B97F4A7C15ULL;
    h ^= static_cast<std::uint64_t>(static_cast<std::uint32_t>(pixelY)) * 0xC2B2AE3D27D4EB4FULL;
    h ^= static_cast<std::uint64_t>(static_cast<std::uint32_t>(extra)) * 0x165667B19E3779F9ULL;
    h ^= static_cast<std::uint64_t>(runSeed);
    h ^= h >> 30;
    h *= 0xBF58476D1CE4E5B9ULL;
    h ^= h >> 27;
    h *= 0x94D049BB133111EBULL;
    h ^= h >> 31;
    return h;
}

// First 32 primes -- Halton radical-inverse bases, one per dimension.
constexpr int kHaltonDimensionCount = 32;
constexpr int kPrimes[kHaltonDimensionCount] = {2,  3,  5,  7,  11, 13, 17, 19, 23, 29, 31,
                                                 37, 41, 43, 47, 53, 59, 61, 67, 71, 73, 79,
                                                 83, 89, 97, 101, 103, 107, 109, 113, 127, 131};

float radicalInverse(int n, int base) {
    float result = 0.0F;
    float f = 1.0F / static_cast<float>(base);
    while (n > 0) {
        result += f * static_cast<float>(n % base);
        n /= base;
        f /= static_cast<float>(base);
    }
    return result;
}

}  // namespace

Pcg32::Pcg32(std::uint64_t seed, std::uint64_t sequence) : state_(0), inc_((sequence << 1U) | 1U) {
    static_cast<void>(nextU32());
    state_ += seed;
    static_cast<void>(nextU32());
}

std::uint32_t Pcg32::nextU32() {
    const std::uint64_t oldState = state_;
    state_ = (oldState * 6364136223846793005ULL) + inc_;
    const auto xorshifted = static_cast<std::uint32_t>(((oldState >> 18U) ^ oldState) >> 27U);
    const auto rot = static_cast<std::uint32_t>(oldState >> 59U);
    return (xorshifted >> rot) | (xorshifted << ((~rot + 1U) & 31U));
}

float Pcg32::nextFloat() {
    return static_cast<float>(nextU32() >> 8) * 0x1.0p-24F;  // 24 significant bits -> [0,1)
}

Sampler::Sampler(int pixelX, int pixelY, int sampleIndex, std::uint32_t runSeed)
    : rng_(hashSeed(pixelX, pixelY, sampleIndex, runSeed), 1),
      rotationRng_(hashSeed(pixelX, pixelY, 0, runSeed), 2),
      sampleIndex_(sampleIndex) {}

float Sampler::next1D() {
    float value = 0.0F;
    if (dimension_ < kHaltonDimensionCount) {
        value = radicalInverse(sampleIndex_ + 1, kPrimes[dimension_]) + rotationRng_.nextFloat();
        value -= std::floor(value);  // Cranley-Patterson rotation, wrapped into [0,1)
    } else {
        value = rng_.nextFloat();
    }
    ++dimension_;
    return value;
}

glm::vec2 Sampler::next2D() {
    const float x = next1D();
    const float y = next1D();
    return {x, y};
}

}  // namespace engine::scene
