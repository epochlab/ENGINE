// Standalone correctness gate for sampler.cpp's padded, Owen-scrambled Sobol sampler. Distinct from the other
// validators in that it asserts EXACT combinatorial properties rather than Monte Carlo tolerances: a digital
// (t,m,s)-net's defining property is an integer count per elementary interval, and Owen scrambling is a digit-tree
// permutation that preserves it exactly, so most checks here are deterministic with no tolerance to tune.
// Same standalone-CLI convention as the other validate tools: no test framework, non-zero exit on failure.
//
// This suite exists because the sampler it replaces was white noise in the shipped configuration and no test noticed.
// The renderer accumulates one sample per pass, so the pass direction -- Sampler's sampleIndex -- is the only axis along
// which stratification can reduce variance, and every pre-existing validator happened to drive that axis correctly
// (Sampler(0, 0, i, seed), index advancing) while the renderer held it at 0 and varied the seed instead. Constant plus a
// fresh per-pass rotation is a uniform variate, so the low-discrepancy structure never applied to a single shipped
// pixel. checkPassDirectionOccupancy below is the specific regression test for that class of mistake.

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "engine/scene/sampler.h"

using engine::scene::Sampler;

namespace {

constexpr std::uint32_t kSeed = 0x9E3779B9U;
// Every check here measures a 2^kM-sample prefix, so the sampler is told that length -- the same value main() uses for kM.
constexpr int kSampleCount = 128;
constexpr int kPixelX = 37;
constexpr int kPixelY = 41;

int failures = 0;

void report(bool passed, const char* name, const char* detail) {
    std::printf("  %-46s %s%s%s\n", name, passed ? "PASS" : "FAIL", detail[0] != '\0' ? "  " : "", detail);
    if (!passed) {
        ++failures;
    }
}

// Draws the 1D value of dimension set `set`, exactly as the renderer would: a fresh Sampler per sample, index
// advancing, scramble seed fixed.
float draw1D(int sampleIndex, int set) {
    Sampler sampler(kPixelX, kPixelY, sampleIndex, kSampleCount, kSeed);
    float value = 0.0F;
    for (int i = 0; i <= set; ++i) {
        value = sampler.next1D();
    }
    return value;
}

// Draws the 2D value of dimension set `set`, with the preceding sets consumed as 1D draws.
glm::vec2 draw2D(int sampleIndex, int set) {
    Sampler sampler(kPixelX, kPixelY, sampleIndex, kSampleCount, kSeed);
    for (int i = 0; i < set; ++i) {
        static_cast<void>(sampler.next1D());
    }
    return sampler.next2D();
}

std::size_t binOf(float value, int binCount) {
    const auto bin = static_cast<std::size_t>(value * static_cast<float>(binCount));
    return bin >= static_cast<std::size_t>(binCount) ? static_cast<std::size_t>(binCount) - 1 : bin;
}

// (0,m,1)-net: the first 2^m samples of any dimension set put exactly one point in each of the 2^m equal intervals.
// Guaranteed by the direction vectors forming a nonsingular generator matrix and preserved by both the Owen scramble
// and the per-set index shuffle -- so this fails on a mis-derived recurrence, a mistranscribed seed row, or a shuffle
// that moved a power-of-two prefix off its strata.
void checkOneDimensionalNet(int m, int setCount) {
    const int n = 1 << m;
    int worstSet = -1;
    for (int set = 0; set < setCount && worstSet < 0; ++set) {
        std::vector<int> bins(static_cast<std::size_t>(n), 0);
        for (int i = 0; i < n; ++i) {
            ++bins[binOf(draw1D(i, set), n)];
        }
        for (const int count : bins) {
            if (count != 1) {
                worstSet = set;
                break;
            }
        }
    }
    char detail[128];
    if (worstSet < 0) {
        std::snprintf(detail, sizeof(detail), "%d sets, 2^%d samples each", setCount, m);
    } else {
        std::snprintf(detail, sizeof(detail), "set %d is not a (0,m,1)-net", worstSet);
    }
    report(worstSet < 0, "(0,m,1)-net: every 1D set", detail);
}

// Net quality t of a 2D set: the smallest t for which the first 2^m samples form a (t,m,2)-net, i.e. every 2^a x 2^b
// cell with a + b = m - t holds exactly 2^t points. t = 0 is a perfect net; m means no stratification at all.
int measureNetQuality(int m, int set) {
    const int n = 1 << m;
    std::vector<glm::vec2> points(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
        points[static_cast<std::size_t>(i)] = draw2D(i, set);
    }

    for (int t = 0; t <= m; ++t) {
        bool holds = true;
        for (int a = 0; a <= m - t && holds; ++a) {
            const int columns = 1 << a;
            const int rows = 1 << (m - t - a);
            std::vector<int> cells(static_cast<std::size_t>(columns) * static_cast<std::size_t>(rows), 0);
            for (const glm::vec2& p : points) {
                ++cells[(binOf(p.y, rows) * static_cast<std::size_t>(columns)) + binOf(p.x, columns)];
            }
            for (const int count : cells) {
                if (count != (1 << t)) {
                    holds = false;
                    break;
                }
            }
        }
        if (holds) {
            return t;
        }
    }
    return m;
}

// The property padding buys, and the reason this sampler pads at all: EVERY 2D set is a perfect (0,m,2)-net, including
// the sets a deep path reaches. Drawing dimension pairs out of one high-dimensional sequence instead would degrade with
// depth -- Sobol's (62,63) projection is only a (4,m,2)-net, sixteen points per cell -- so a 12-bounce path would
// sample its last bounces worse than white noise. Asserted at depths a path actually reaches, exactly, no tolerance.
void checkEverySetIsPerfectNet(int m, const std::vector<int>& sets) {
    int worstSet = -1;
    int worstT = 0;
    std::string measured;
    for (const int set : sets) {
        const int t = measureNetQuality(m, set);
        measured += "s" + std::to_string(set) + ":t=" + std::to_string(t) + " ";
        if (t != 0 && worstSet < 0) {
            worstSet = set;
            worstT = t;
        }
    }
    char detail[256];
    if (worstSet < 0) {
        std::snprintf(detail, sizeof(detail), "%s", measured.c_str());
    } else {
        std::snprintf(detail, sizeof(detail), "set %d is only a (%d,m,2)-net", worstSet, worstT);
    }
    report(worstSet < 0, "(0,m,2)-net: every 2D set, at any depth", detail);
}

// The regression test for the defect this sampler replaces. Drives it exactly as PathTraceDriver does -- one sample per
// pass, index advancing, seed fixed -- and requires the accumulated points to be stratified. Under the old white-noise
// behaviour every dimension left ~N/e (36.8%) of bins empty; a correct sequence leaves none.
void checkPassDirectionOccupancy(int m, int setCount) {
    const int n = 1 << m;
    int worstSet = -1;
    int worstEmpty = 0;
    for (int set = 0; set < setCount; ++set) {
        std::vector<int> bins(static_cast<std::size_t>(n), 0);
        for (int pass = 0; pass < n; ++pass) {
            ++bins[binOf(draw1D(pass, set), n)];
        }
        int empty = 0;
        for (const int count : bins) {
            empty += count == 0 ? 1 : 0;
        }
        if (empty > worstEmpty) {
            worstEmpty = empty;
            worstSet = set;
        }
    }
    char detail[128];
    if (worstSet < 0) {
        std::snprintf(detail, sizeof(detail), "0 empty bins across %d sets", setCount);
    } else {
        std::snprintf(detail, sizeof(detail), "set %d leaves %d/%d bins empty (%.1f%%)", worstSet, worstEmpty, n,
                      100.0 * worstEmpty / n);
    }
    report(worstSet < 0, "pass-direction occupancy (white-noise guard)", detail);
}

// Sobol's index-0 point is all zeros before scrambling, and the renderer's very first displayed pass is index 0 for
// every pixel -- unscrambled, that would put every pixel's jitter exactly on its pixel corner. Owen scrambling maps it
// to a uniformly distributed point instead, so index 0 must vary across pixels rather than being a constant.
void checkIndexZeroIsScrambled() {
    constexpr int kPixels = 4096;
    constexpr int kBins = 16;
    std::vector<int> bins(kBins, 0);
    for (int i = 0; i < kPixels; ++i) {
        Sampler sampler(i % 64, i / 64, 0, kSampleCount, kSeed);
        ++bins[binOf(sampler.next1D(), kBins)];
    }
    // Chi-square against uniform over 16 bins, 15 degrees of freedom. The 1e-6 critical value is ~54; a correct scramble
    // sits near 15, and the degenerate all-zeros case would pile every sample into bin 0 (chi2 = kPixels * 15).
    double chiSquare = 0.0;
    const double expected = static_cast<double>(kPixels) / kBins;
    for (const int count : bins) {
        const double delta = count - expected;
        chiSquare += delta * delta / expected;
    }
    char detail[128];
    std::snprintf(detail, sizeof(detail), "chi2 = %.1f over 15 dof (degenerate would be %d)", chiSquare, kPixels * 15);
    report(chiSquare < 54.0, "index 0 is scrambled, not degenerate", detail);
}

// Padding's other requirement: consecutive sets must be uncorrelated, or a path's successive decisions would move in
// lockstep. Two sets sharing one shuffled index and scramble would return identical values.
void checkSetsAreDecorrelated() {
    constexpr int kSamples = 128;
    int identical = 0;
    for (int i = 0; i < kSamples; ++i) {
        identical += draw1D(i, 0) == draw1D(i, 1) ? 1 : 0;
    }
    char detail[128];
    std::snprintf(detail, sizeof(detail), "%d/%d collisions between set 0 and set 1", identical, kSamples);
    report(identical == 0, "adjacent dimension sets decorrelate", detail);
}

// The scramble seed must decorrelate neighbouring pixels: two pixels drawing the same set at the same sample index must
// not return the same value, or every pixel would share one noise realization.
void checkPixelsAreDecorrelated() {
    constexpr int kSamples = 128;
    int identical = 0;
    for (int i = 0; i < kSamples; ++i) {
        Sampler a(kPixelX, kPixelY, i, kSampleCount, kSeed);
        Sampler b(kPixelX + 1, kPixelY, i, kSampleCount, kSeed);
        identical += a.next1D() == b.next1D() ? 1 : 0;
    }
    char detail[128];
    std::snprintf(detail, sizeof(detail), "%d/%d collisions between adjacent pixels", identical, kSamples);
    report(identical == 0, "adjacent pixels decorrelate", detail);
}

}  // namespace

int main() {
    constexpr int kM = 7;  // 2^7 = 128 points, matching profile.json's maxSamples accumulation cap
    // A 12-bounce path (integrator_validate's slab case) consumes ~5 sets per bounce plus 2 at the camera, so set 64+
    // is genuinely reached in practice and is where an unpadded sampler would have degraded.
    constexpr int kSetCount = 72;
    std::printf("sampler_validate: padded Owen-scrambled Sobol, 2^%d-sample prefixes\n\n", kM);

    checkOneDimensionalNet(kM, kSetCount);
    checkEverySetIsPerfectNet(kM, {0, 1, 2, 7, 31, 64, 71});
    checkPassDirectionOccupancy(kM, kSetCount);
    checkIndexZeroIsScrambled();
    checkSetsAreDecorrelated();
    checkPixelsAreDecorrelated();

    std::printf("\nsampler_validate: %s\n", failures == 0 ? "all checks passed" : "FAILURES PRESENT");
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
