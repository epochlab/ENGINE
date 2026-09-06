#include "engine/scene/sampler.h"

#include <algorithm>
#include <array>
#include <bit>

namespace engine::scene {

namespace {

// SplitMix64 finalizer (Vigna) -- decorrelates nearby (pixel, dimension set) inputs into unrelated seeds.
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

// Only two Sobol dimensions exist here, and that is the whole point of padding (see sampler.h): every draw reuses this
// one perfectly stratified pair under a fresh randomization instead of walking into the sequence's weaker high
// dimensions. Raising this means adding a jointly-stratified next3D/next4D and regenerating the seed table wider.
constexpr int kSobolDimensions = 2;
constexpr int kSobolBits = 32;  // direction vectors are scaled by 2^32, so a 32-bit index spans the sequence's period

constexpr int kMaxDegree = 1;  // largest polynomial degree across the generated rows, reported by generate_sobol_table
struct SobolPolynomial {
    std::uint32_t degree;
    std::uint32_t coefficients;
    std::array<std::uint32_t, kMaxDegree> initial;
};

// Sobol dimension 2 only; dimension 1 needs no seeds and is generated directly below.
constexpr std::array<SobolPolynomial, kSobolDimensions - 1> kSobolPolynomials = {{
#include "sobol_direction_seeds.inc"
}};

// Direction vectors V, scaled by 2^32, derived from the seeds above by the recurrence in Joe & Kuo's own reference
// implementation (sobol.cc, https://web.maths.unsw.edu.au/~fkuo/sobol/), which is ACM Algorithm 659 (Bratley & Fox
// 1988). Published V is 1-indexed with V[i] scaled by 2^(32-i); this table is 0-indexed by bit position, so index b
// holds published V[b+1] and the recurrence's shape carries over unchanged.
// Derived at static init rather than checked in, matching path_tracer.cpp's buildFilterTable(): it keeps the committed
// data to the published seed rows and puts the recurrence in the source where it can be checked against the paper.
using SobolDirections = std::array<std::array<std::uint32_t, kSobolBits>, kSobolDimensions>;

SobolDirections buildSobolDirections() {
    SobolDirections directions{};

    // Sobol dimension 1: every initial direction number is 1, giving the van der Corput sequence in base 2.
    for (int b = 0; b < kSobolBits; ++b) {
        directions[0][static_cast<std::size_t>(b)] = 1U << (kSobolBits - 1 - b);
    }

    for (std::size_t d = 1; d < kSobolDimensions; ++d) {
        const SobolPolynomial& poly = kSobolPolynomials[d - 1];
        const auto degree = static_cast<int>(poly.degree);
        std::array<std::uint32_t, kSobolBits>& v = directions[d];

        for (int b = 0; b < degree; ++b) {
            v[static_cast<std::size_t>(b)] = poly.initial[static_cast<std::size_t>(b)] << (kSobolBits - 1 - b);
        }
        for (int b = degree; b < kSobolBits; ++b) {
            const std::uint32_t previous = v[static_cast<std::size_t>(b - degree)];
            std::uint32_t next = previous ^ (previous >> degree);
            for (int k = 1; k < degree; ++k) {
                const std::uint32_t coefficient = (poly.coefficients >> (degree - 1 - k)) & 1U;
                next ^= coefficient * v[static_cast<std::size_t>(b - k)];
            }
            v[static_cast<std::size_t>(b)] = next;
        }
    }
    return directions;
}

const SobolDirections kSobolDirections = buildSobolDirections();

std::uint32_t reverseBits(std::uint32_t x) {
#if defined(__has_builtin)
#if __has_builtin(__builtin_bitreverse32)
    return __builtin_bitreverse32(x);  // one RBIT instruction on arm64
#endif
#endif
    x = (x >> 16) | (x << 16);
    x = ((x & 0xFF00FF00U) >> 8) | ((x & 0x00FF00FFU) << 8);
    x = ((x & 0xF0F0F0F0U) >> 4) | ((x & 0x0F0F0F0FU) << 4);
    x = ((x & 0xCCCCCCCCU) >> 2) | ((x & 0x33333333U) << 2);
    return ((x & 0xAAAAAAAAU) >> 1) | ((x & 0x55555555U) << 1);
}

// Base-2 Owen scrambling of a bit-reversed integer. Owen (1995) defines the scramble as a random permutation of each
// node of the value's binary digit tree, which preserves the sequence's net properties exactly while decorrelating it;
// Burley (2020, JCGT 9(4)) showed a cheap integer hash reproduces that structure without building the tree, since in
// bit-reversed order a hash's carry propagation touches exactly the ancestors of each digit.
// Constants are Vegdahl's improved Laine-Karras hash as shipped by Cycles (intern/cycles/kernel/sample/util.h), not
// Burley's original: same construction, measurably better tree-permutation quality per
// https://psychopath.io/post/2021_01_30_building_a_better_lk_hash. Transcribed from that source, never hand-derived --
// these are permutation constants, and a mistyped digit degrades the scramble in ways no image inspection would catch.
std::uint32_t reversedBitOwen(std::uint32_t n, std::uint32_t seed) {
    n ^= n * 0x3D20ADEAU;
    n += seed;
    n *= (seed >> 16) | 1U;
    n ^= n * 0x05526C56U;
    n ^= n * 0x53A22864U;
    return n;
}

std::uint32_t nestedUniformScramble(std::uint32_t x, std::uint32_t seed) {
    return reverseBits(reversedBitOwen(reverseBits(x), seed));
}

// Sobol point for one dimension: XOR the direction vectors selected by the set bits of the index (Sobol 1967, in the
// direct random-access form -- not the Gray-code recurrence sobol.cc uses to walk indices in order, which this sampler
// cannot use because it addresses an arbitrary sample index per pixel with no sequential state).
// Dimension 1 is not looped at all. Its direction vectors are V[b] = 1 << (31 - b), so XOR-ing the ones the index's set
// bits select is by definition that index's bit reversal -- an identity, not an approximation, and one RBIT rather than
// a loop iteration per set bit. Cycles carries the same fast path for the same reason. Measured over next2D: 62 -> 48
// ns/draw from this alone, and 14 ns once the index mask below bounds dimension 2's remaining loop.
std::uint32_t sobolPoint(std::uint32_t index, int dimension) {
    if (dimension == 0) {
        return reverseBits(index);
    }
    std::uint32_t x = 0;
    for (std::uint32_t bits = index; bits != 0U; bits &= bits - 1U) {
        x ^= kSobolDirections[static_cast<std::size_t>(dimension)][static_cast<std::size_t>(std::countr_zero(bits))];
    }
    return x;
}

float toUnitFloat(std::uint32_t x) {
    return static_cast<float>(x >> 8) * 0x1.0p-24F;  // 24 significant bits -> [0,1)
}

}  // namespace

// The index mask bounds the sequence to the length actually drawn from. Without it the per-set shuffle spreads a small
// sample index across all 32 bits, and sobolPoint's loop then runs once per set bit -- about 16 iterations rather than
// at most log2(sampleCount). Measured over next2D at 128 samples: 62 ns/draw unmasked, 14 ns masked.
// The floor at sampleIndex + 1 is a correctness guard, not a tuning knob: a caller understating sampleCount would
// otherwise mask two distinct sample indices onto the same sequence point, drawing one point twice and biasing the
// estimate with nothing to signal it. sampleCount <= 0 means an unbounded accumulation, which takes the full 32-bit
// sequence -- Cycles' documented "safe default, least performant" case.
Sampler::Sampler(int pixelX, int pixelY, int sampleIndex, int sampleCount, std::uint32_t scrambleSeed)
    : pixelX_(pixelX),
      pixelY_(pixelY),
      sampleIndex_(static_cast<std::uint32_t>(sampleIndex)),
      indexMask_(sampleCount > 0 ? std::bit_ceil(static_cast<std::uint32_t>(std::max(sampleCount, sampleIndex + 1))) - 1U
                                 : 0xFFFFFFFFU),
      scrambleSeed_(scrambleSeed) {}

float Sampler::next1D() {
    // Two independent seeds per set, taken as the halves of one avalanched hash rather than from separate magic salts:
    // hashSeed's SplitMix64 finalizer already decorrelates its halves, which is the same argument kAoSeedOffset rests on
    // in path_tracer.cpp. Distinct `extra` values are what separate one dimension set from the next.
    const std::uint64_t setHash = hashSeed(pixelX_, pixelY_, 2 * dimensionSet_, scrambleSeed_);
    ++dimensionSet_;
    // Shuffling the index per set (Burley 2020 Sec. 5.2) is what decorrelates sets from one another: without it every
    // set would visit the sequence in the same order and the sets would share one point ordering. Owen-scrambling the
    // index permutes its digit tree, so a power-of-two prefix stays a stratified point set rather than becoming an
    // arbitrary subset -- sampler_validate's net checks are what hold this claim to account.
    const auto indexSeed = static_cast<std::uint32_t>(setHash);
    const auto scramble = static_cast<std::uint32_t>(setHash >> 32U);
    const std::uint32_t shuffled = nestedUniformScramble(sampleIndex_, indexSeed) & indexMask_;
    return toUnitFloat(nestedUniformScramble(sobolPoint(shuffled, 0), scramble));
}

glm::vec2 Sampler::next2D() {
    // Both coordinates share one shuffled index -- they are the two components of a single point of the 2D sequence, so
    // shuffling them apart would destroy the joint stratification that makes next2D worth using over two next1D calls.
    // Their Owen scrambles differ, which is Owen's own per-dimension independence requirement (1995).
    const std::uint64_t setHash = hashSeed(pixelX_, pixelY_, 2 * dimensionSet_, scrambleSeed_);
    const std::uint64_t scrambleHash = hashSeed(pixelX_, pixelY_, (2 * dimensionSet_) + 1, scrambleSeed_);
    ++dimensionSet_;
    const auto indexSeed = static_cast<std::uint32_t>(setHash);
    const auto scrambleX = static_cast<std::uint32_t>(setHash >> 32U);
    const auto scrambleY = static_cast<std::uint32_t>(scrambleHash);
    const std::uint32_t shuffled = nestedUniformScramble(sampleIndex_, indexSeed) & indexMask_;
    return {toUnitFloat(nestedUniformScramble(sobolPoint(shuffled, 0), scrambleX)),
            toUnitFloat(nestedUniformScramble(sobolPoint(shuffled, 1), scrambleY))};
}

}  // namespace engine::scene
