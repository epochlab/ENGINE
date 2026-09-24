#include "pathtracer/scene/sampler.h"

#include <algorithm>
#include <array>
#include <bit>

namespace pathtracer::scene {

namespace {

// SplitMix64 finalizer (Vigna) -- decorrelates nearby (dimension set, run seed) inputs into unrelated seeds.
// Deliberately NOT a function of the pixel: see the dither mask below. Every pixel draws the same randomized sequence,
// and what separates them is the toroidal shift, not the scramble.
std::uint64_t hashSeed(int extra, std::uint32_t runSeed) {
    std::uint64_t h = static_cast<std::uint64_t>(static_cast<std::uint32_t>(extra)) * 0x165667B19E3779F9ULL;
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

// Direction vectors V, scaled by 2^32, from the recurrence in Joe & Kuo's reference implementation (ACM Algorithm 659,
// Bratley & Fox 1988). Derived at static init rather than checked in, so the committed data stays the published seed
// rows and the recurrence lives where it can be checked. See DERIVATIONS.md "Sobol padding and net quality".
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

// Base-2 Owen scrambling of a bit-reversed integer: Owen (1995) permutes each node of the binary digit tree, and
// Burley (2020) showed a cheap hash reproduces that structure. Constants are Vegdahl's improved Laine-Karras hash as
// shipped by Cycles -- transcribed, never hand-derived, a mistyped digit degrading the scramble invisibly.
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

// Sobol point for one dimension: XOR the direction vectors selected by the set bits of the index (Sobol 1967), in the
// direct random-access form rather than the Gray-code recurrence, which needs sequential state this sampler has not.
// Dimension 1 is a bit reversal by identity, so it skips the loop: 62 -> 48 ns/draw. See DERIVATIONS.md.
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

// Blue-noise dithered sampling (Georgiev & Fajardo 2016): every pixel uses the same point set, toroidally shifted by
// an offset read from a blue-noise mask tiled over the image. This moves the error field's power out of the low
// frequencies without reducing it -- apparent cleanliness, not convergence. DERIVATIONS.md has the measurements.
constexpr int kMaskSize = 128;  // the size Georgiev & Fajardo used for their rendered images
constexpr int kMaskPixels = kMaskSize * kMaskSize;
constexpr std::array<std::uint16_t, kMaskPixels> kBlueNoiseRanks = {{
#include "blue_noise_mask.inc"
}};

// R2, the 2D low-discrepancy sequence of Roberts (2018): point c is frac(c * (1/phi2, 1/phi2^2)), phi2 the plastic
// number. A hash was tried first and is not sufficient -- hashed offsets land close together, measured |r| = 0.14
// between channels 8 and 23. R2 spreads them by construction. See DERIVATIONS.md "The blue-noise shift".
constexpr std::uint32_t kR2AlphaX = 0xC13FA9A9U;
constexpr std::uint32_t kR2AlphaY = 0x91E10DA6U;
// Keeps the top log2(kMaskSize) bits of the fixed-point fraction, so the offset follows kMaskSize rather than a literal.
constexpr int kR2Shift = 32 - std::countr_zero(static_cast<unsigned>(kMaskSize));

// The shift in the sampler's own 24-bit output space, so applying it is one add and one mask and the wraparound is the
// toroidal wrap. Exactly representable, which is what lets sampler_validate invert it at zero tolerance. Each set
// reads the same mask under its own translation, which is load-bearing: see DERIVATIONS.md "The blue-noise shift".
std::uint32_t ditherFixed(int pixelX, int pixelY, int ditherChannel) {
    const auto channel = static_cast<std::uint32_t>(ditherChannel);
    const auto x = (static_cast<std::uint32_t>(pixelX) + ((channel * kR2AlphaX) >> kR2Shift)) & (kMaskSize - 1U);
    const auto y = (static_cast<std::uint32_t>(pixelY) + ((channel * kR2AlphaY) >> kR2Shift)) & (kMaskSize - 1U);
    return (static_cast<std::uint32_t>(kBlueNoiseRanks[(y * kMaskSize) + x]) << 10U) | 512U;
}

// One dimension set's output: 24 significant bits of the scrambled point, shifted by this pixel's dither for that set.
float toShiftedUnitFloat(std::uint32_t x, std::uint32_t dither) {
    return static_cast<float>(((x >> 8) + dither) & 0xFFFFFFU) * 0x1.0p-24F;
}

}  // namespace

float blueNoiseDither(int pixelX, int pixelY, int ditherChannel) {
    return static_cast<float>(ditherFixed(pixelX, pixelY, ditherChannel)) * 0x1.0p-24F;
}

// The index mask bounds the sequence to the length actually drawn from: 62 ns/draw unmasked against 14 masked, at 128
// samples. The floor at sampleIndex + 1 is a correctness guard, not a tuning knob -- understating sampleCount would
// alias two indices onto one sequence point. sampleCount <= 0 means unbounded, taking the full 32-bit sequence.
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
    const int set = dimensionSet_;
    const std::uint64_t setHash = hashSeed(2 * set, scrambleSeed_);
    ++dimensionSet_;
    // Shuffling the index per set (Burley 2020 Sec. 5.2) is what decorrelates sets from one another; without it every set
    // visits the sequence in the same order. Owen-scrambling the index permutes its digit tree, so a power-of-two prefix
    // stays a stratified point set rather than an arbitrary subset -- sampler_validate's net checks hold that to account.
    const auto indexSeed = static_cast<std::uint32_t>(setHash);
    const auto scramble = static_cast<std::uint32_t>(setHash >> 32U);
    const std::uint32_t shuffled = nestedUniformScramble(sampleIndex_, indexSeed) & indexMask_;
    return toShiftedUnitFloat(nestedUniformScramble(sobolPoint(shuffled, 0), scramble),
                               ditherFixed(pixelX_, pixelY_, 2 * set));
}

glm::vec2 Sampler::next2D() {
    // Both coordinates share one shuffled index -- they are the two components of a single point of the 2D sequence, so
    // shuffling them apart would destroy the joint stratification that makes next2D worth using over two next1D calls.
    // Their Owen scrambles differ, which is Owen's own per-dimension independence requirement (1995).
    const int set = dimensionSet_;
    const std::uint64_t setHash = hashSeed(2 * set, scrambleSeed_);
    const std::uint64_t scrambleHash = hashSeed((2 * set) + 1, scrambleSeed_);
    ++dimensionSet_;
    const auto indexSeed = static_cast<std::uint32_t>(setHash);
    const auto scrambleX = static_cast<std::uint32_t>(setHash >> 32U);
    const auto scrambleY = static_cast<std::uint32_t>(scrambleHash);
    const std::uint32_t shuffled = nestedUniformScramble(sampleIndex_, indexSeed) & indexMask_;
    return {toShiftedUnitFloat(nestedUniformScramble(sobolPoint(shuffled, 0), scrambleX),
                               ditherFixed(pixelX_, pixelY_, 2 * set)),
            toShiftedUnitFloat(nestedUniformScramble(sobolPoint(shuffled, 1), scrambleY),
                               ditherFixed(pixelX_, pixelY_, (2 * set) + 1))};
}

}  // namespace pathtracer::scene
