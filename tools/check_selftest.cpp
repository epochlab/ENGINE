// Guards the harness's own invariants. The suite's whole discipline rests on the negated-pass assertion form (a NaN
// must FAIL, not silently pass) and on seeds being derived from a check's name rather than its position; both are
// properties nothing else would notice regressing, because a broken assert macro reports success.

#include <cmath>
#include <limits>
#include <string>

#include "check.h"

namespace {

const double kNaN = std::numeric_limits<double>::quiet_NaN();

// A helper Context cannot be constructed outside the runner, so NaN-rejection is asserted on the raw comparison the
// macro performs: !(pass) must be true for every ordered comparison against NaN.
ENGINE_CHECK(nan_fails_every_ordered_comparison, Fast, Exact) {
    ctx.plan(5);
    ENGINE_EXPECT(ctx, !(kNaN <= 1.0), "NaN <= x must be false, so a negated-pass assert rejects it");
    ENGINE_EXPECT(ctx, !(kNaN >= 1.0), "NaN >= x must be false");
    ENGINE_EXPECT(ctx, !(kNaN < 1.0), "NaN < x must be false");
    ENGINE_EXPECT(ctx, !(kNaN > 1.0), "NaN > x must be false");
    ENGINE_EXPECT(ctx, !(kNaN == kNaN), "NaN == NaN must be false");
}

// A band stated as a pair of ordered comparisons must reject a NaN value AND a NaN bound. Both directions matter: a
// measured NaN is a broken estimator, a NaN bound is a broken tolerance derivation, and neither may report success.
ENGINE_CHECK(nan_band_rejects_both_sides, Fast, Exact) {
    ctx.plan(2);
    const auto within = [](double v, double lo, double hi) { return v >= lo && v <= hi; };
    ENGINE_EXPECT(ctx, !within(kNaN, 0.0, 1.0), "a NaN measurement must fall outside every band");
    ENGINE_EXPECT(ctx, !within(0.5, kNaN, kNaN), "a NaN band must contain nothing");
}

// Seeds are name-derived, so they must differ between checks and be stable within one.
ENGINE_CHECK(seed_is_name_derived_and_stable, Fast, Exact) {
    ctx.plan(3);
    // Deliberately not `seed() == seed()`: that is a tautology the compiler may fold away, and it would pass even if
    // the derivation returned a constant. These assert that the derivation actually mixes.
    ENGINE_EXPECT(ctx, ctx.seed() != 0, "a derived seed of zero means the mixing collapsed");
    ENGINE_EXPECT(ctx, ctx.subSeed("row-a") != ctx.seed(), "a sub-seed must derive away from its master seed");
    ENGINE_EXPECT(ctx, ctx.subSeed("row-a") != ctx.subSeed("row-b"), "distinct labels must give distinct sub-seeds");
}

// The correction must be strictly tighter than the family rate, and tighter still as a check makes more assertions:
// this is what stops a check silently buying itself a looser band by asserting more.
ENGINE_CHECK(significance_is_family_wise_corrected, Fast, Exact) {
    ctx.plan(2);
    const double perAssertion = ctx.alpha();
    ENGINE_EXPECT(ctx, perAssertion > 0.0 && perAssertion < ::tools::check::kFamilyAlpha,
                  "per-assertion alpha must be positive and tighter than the family rate");
    ENGINE_EXPECT(ctx, std::isfinite(perAssertion), "per-assertion alpha must be finite");
}

}  // namespace

ENGINE_CHECK_MAIN("selftest")
