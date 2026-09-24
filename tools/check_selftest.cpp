// Guards the harness's own invariants -- the negated-pass form and name-derived seeds -- which nothing else would notice regressing.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "check.h"
#include "stats.h"

namespace {

const double kNaN = std::numeric_limits<double>::quiet_NaN();

// A helper Context cannot be constructed outside the runner, so NaN rejection is asserted on the raw comparison the macro performs.
PT_CHECK(nan_fails_every_ordered_comparison, Fast, Exact) {
    ctx.plan(5);
    PT_EXPECT(ctx, !(kNaN <= 1.0), "NaN <= x must be false, so a negated-pass assert rejects it");
    PT_EXPECT(ctx, !(kNaN >= 1.0), "NaN >= x must be false");
    PT_EXPECT(ctx, !(kNaN < 1.0), "NaN < x must be false");
    PT_EXPECT(ctx, !(kNaN > 1.0), "NaN > x must be false");
    PT_EXPECT(ctx, !(kNaN == kNaN), "NaN == NaN must be false");
}

// A band stated as ordered comparisons must reject a NaN value AND a NaN bound: a broken estimator and a broken tolerance derivation.
PT_CHECK(nan_band_rejects_both_sides, Fast, Exact) {
    ctx.plan(2);
    const auto within = [](double v, double lo, double hi) { return v >= lo && v <= hi; };
    PT_EXPECT(ctx, !within(kNaN, 0.0, 1.0), "a NaN measurement must fall outside every band");
    PT_EXPECT(ctx, !within(0.5, kNaN, kNaN), "a NaN band must contain nothing");
}

// Seeds are name-derived, so they must differ between checks and be stable within one.
PT_CHECK(seed_is_name_derived_and_stable, Fast, Exact) {
    ctx.plan(3);
    // Deliberately not `seed() == seed()`, a tautology the compiler may fold that would pass even if the derivation returned a constant.
    PT_EXPECT(ctx, ctx.seed() != 0, "a derived seed of zero means the mixing collapsed");
    PT_EXPECT(ctx, ctx.subSeed("row-a") != ctx.seed(), "a sub-seed must derive away from its master seed");
    PT_EXPECT(ctx, ctx.subSeed("row-a") != ctx.subSeed("row-b"), "distinct labels must give distinct sub-seeds");
}

// The correction must be strictly tighter than the family rate, and tighter as a check asserts more, so more assertions buy no slack.
PT_CHECK(significance_is_family_wise_corrected, Fast, Exact) {
    ctx.plan(2);
    const double perAssertion = ctx.alpha();
    PT_EXPECT(ctx, perAssertion > 0.0 && perAssertion < ::tools::check::kFamilyAlpha,
                  "per-assertion alpha must be positive and tighter than the family rate");
    PT_EXPECT(ctx, std::isfinite(perAssertion), "per-assertion alpha must be finite");
}

// Exact nulls against brute-force enumeration and published values: Wilcoxon n=10 P(T+<=8)=0.0244, Mann-Whitney 8x8 P(U<=13)=0.0249.
PT_CHECK(rank_nulls_match_enumeration, Fast, Exact) {
    ctx.plan(4);
    double worstSigned = 0.0;
    for (int n = 1; n <= 12; ++n) {
        std::vector<double> counted(static_cast<std::size_t>(n * (n + 1) / 2) + 1, 0.0);
        for (std::uint32_t signs = 0; signs < (1U << static_cast<unsigned>(n)); ++signs) {
            int t = 0;
            for (int r = 0; r < n; ++r) {
                t += ((signs >> static_cast<unsigned>(r)) & 1U) != 0 ? r + 1 : 0;
            }
            counted[static_cast<std::size_t>(t)] += 1.0 / static_cast<double>(1U << static_cast<unsigned>(n));
        }
        const std::vector<double> pmf = ::tools::stats::signedRankNull(n);
        for (std::size_t t = 0; t < counted.size(); ++t) {
            worstSigned = std::max(worstSigned, pmf.size() == counted.size() ? std::fabs(pmf[t] - counted[t]) : 1.0);
        }
    }
    PT_EXPECT(ctx, worstSigned <= 1e-15, "signedRankNull disagrees with subset enumeration");

    // Every placement of n y's among m + n ranks, U counting x's below each y.
    const auto enumerateRankSum = [](int m, int n) {
        std::vector<double> counted(static_cast<std::size_t>(m * n) + 1, 0.0);
        double total = 0.0;
        for (std::uint32_t mask = 0; mask < (1U << static_cast<unsigned>(m + n)); ++mask) {
            if (__builtin_popcount(mask) != n) {
                continue;
            }
            int u = 0;
            int xsBelow = 0;
            for (int k = 0; k < m + n; ++k) {
                if (((mask >> static_cast<unsigned>(k)) & 1U) != 0) {
                    u += xsBelow;
                } else {
                    ++xsBelow;
                }
            }
            counted[static_cast<std::size_t>(u)] += 1.0;
            total += 1.0;
        }
        for (double& c : counted) {
            c /= total;
        }
        return counted;
    };
    double worstRankSum = 0.0;
    for (const auto& [m, n] : {std::pair{5, 6}, std::pair{3, 8}, std::pair{7, 2}}) {
        const std::vector<double> counted = enumerateRankSum(m, n);
        const std::vector<double> pmf = ::tools::stats::rankSumNull(m, n);
        for (std::size_t u = 0; u < counted.size(); ++u) {
            worstRankSum = std::max(worstRankSum, pmf.size() == counted.size() ? std::fabs(pmf[u] - counted[u]) : 1.0);
        }
    }
    PT_EXPECT(ctx, worstRankSum <= 1e-15, "rankSumNull disagrees with placement enumeration");

    const std::vector<double> signed10 = ::tools::stats::signedRankNull(10);
    double lowerTail = 0.0;
    for (std::size_t t = 0; t <= 8; ++t) {
        lowerTail += signed10[t];
    }
    PT_EXPECT(ctx, std::fabs(lowerTail - 0.0244) < 5e-5, "Wilcoxon n=10 P(T+ <= 8) is not the tabulated 0.0244");
    const std::vector<double> rankSum88 = ::tools::stats::rankSumNull(8, 8);
    double uTail = 0.0;
    for (std::size_t u = 0; u <= 13; ++u) {
        uTail += rankSum88[u];
    }
    PT_EXPECT(ctx, std::fabs(uTail - 0.0249) < 5e-5, "Mann-Whitney 8x8 P(U <= 13) is not the tabulated 0.0249");
}

// Hand-computable estimates, and the too-small case: at n = 5 the most extreme T+ has probability 1/32 > alpha/2 = 0.025, so none exists.
PT_CHECK(hodges_lehmann_point_estimates, Fast, Exact) {
    ctx.plan(3);
    PT_EXPECT(ctx, ::tools::stats::hodgesLehmannPaired({1.0, 2.0, 3.0}, 0.05).estimate == 2.0, "Walsh-average median of {1,2,3} must be 2");
    PT_EXPECT(ctx, ::tools::stats::hodgesLehmannShift({0.0, 1.0}, {10.0, 12.0}, 0.05).estimate == 10.5, "median of pairwise differences {10,12,9,11} must be 10.5");
    const ::tools::stats::ShiftEstimate tooFew = ::tools::stats::hodgesLehmannPaired({0.1, 0.2, 0.3, 0.4, 0.5}, 0.05);
    PT_EXPECT(ctx, std::isinf(tooFew.lower) && std::isinf(tooFew.upper), "n = 5 cannot support a 95% signed-rank interval");
}

// Empirical coverage under a known shift must match the exact discrete coverage; a one-rank index error moves it far outside the band.
PT_CHECK(hodges_lehmann_coverage_is_exact, Fast, Statistical) {
    ctx.plan(2);
    constexpr int kTrials = 20000;
    constexpr double kShift = 0.3;
    constexpr double kAlpha = 0.05;
    std::mt19937_64 rng(ctx.seed());
    std::normal_distribution<double> noise(0.0, 1.0);

    long long pairedHits = 0;
    double pairedCoverage = 0.0;
    long long shiftHits = 0;
    double shiftCoverage = 0.0;
    for (int trial = 0; trial < kTrials; ++trial) {
        std::vector<double> d(12);
        for (double& v : d) {
            v = kShift + noise(rng);
        }
        const ::tools::stats::ShiftEstimate paired = ::tools::stats::hodgesLehmannPaired(d, kAlpha);
        pairedHits += paired.lower <= kShift && kShift <= paired.upper ? 1 : 0;
        pairedCoverage = paired.coverage;

        std::vector<double> x(7);
        std::vector<double> y(9);
        for (double& v : x) {
            v = noise(rng);
        }
        for (double& v : y) {
            v = kShift + noise(rng);
        }
        const ::tools::stats::ShiftEstimate shift = ::tools::stats::hodgesLehmannShift(x, y, kAlpha);
        shiftHits += shift.lower <= kShift && kShift <= shift.upper ? 1 : 0;
        shiftCoverage = shift.coverage;
    }
    PT_EXPECT(ctx, ::tools::stats::wilsonBand(pairedHits, kTrials, ctx.alpha()).contains(pairedCoverage),
                  "paired interval's empirical coverage disagrees with its exact coverage");
    PT_EXPECT(ctx, ::tools::stats::wilsonBand(shiftHits, kTrials, ctx.alpha()).contains(shiftCoverage),
                  "two-sample interval's empirical coverage disagrees with its exact coverage");
}

}  // namespace

PT_CHECK_MAIN("selftest")
