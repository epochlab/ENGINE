// Correctness gate for the CPU-evaluable parts of the display and statistics path: the OCIO display transform
// (gfx/ocio_cpu_transform.cpp) and the frame-time ring buffer (debug/frame_stats.cpp).
//
// The OCIO half turns a comment into a check. ocio_display_transform.h pins its config by name rather than by
// "-latest", on the stated grounds that this config's "Un-tone-mapped" view is a pure colorimetric pass -- "empirically
// verified (compiled + ran against the installed library)", which is to say verified once, by hand, and never again.
// A brew upgrade that changed the built-in registry under that name would silently alter every rendered PNG and every
// displayed frame. These checks assert the property the pin exists to guarantee.
//
// Not covered, deliberately: debug/histogram.cpp is FBO/PBO-bound with no CPU-reachable binning function, so there is
// nothing to assert without a GL context. A test that only confirmed it constructs would be worse than none, because
// it would report coverage that does not exist.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>

#include "check.h"
#include "engine/debug/frame_stats.h"
#include "engine/gfx/ocio_cpu_transform.h"

namespace {

// Applies the transform to one RGB triple and returns the red channel; the transform is per-channel identical for a
// neutral input, so one channel is the whole story for a grey ramp.
float transformScalar(float value) {
    std::vector<float> rgb{value, value, value};
    engine::gfx::applyOcioDisplayTransform(rgb, 1, 1);
    return rgb[0];
}

// The anchors the pin exists to guarantee. "Un-tone-mapped" is claimed to be a pure colorimetric pass, which fixes
// exactly two points: scene-referred 0 must display as 0, and scene-referred 1 (diffuse white) must display as 1.
// A filmic view would roll 1.0 off to roughly 0.8 and lift 0 off the floor, so these two rows alone separate the
// pinned config from every tone-mapped alternative a registry change could substitute.
ENGINE_CHECK(ocio_view_is_colorimetric_at_its_anchors, Fast, Exact) {
    ctx.plan(2);
    // sRGB's encoding curve is steep near zero but passes exactly through the origin; the tolerance here is float
    // round-trip noise through the processor, not a tone-curve allowance.
    constexpr float kAnchorTolerance = 1e-5F;
    const float atZero = transformScalar(0.0F);
    const float atOne = transformScalar(1.0F);
    char zeroDetail[160];
    std::snprintf(zeroDetail, sizeof(zeroDetail), "scene-referred 0.0 displays as %.9g, must be 0", 
                  static_cast<double>(atZero));
    ENGINE_EXPECT(ctx, std::fabs(atZero) <= kAnchorTolerance, zeroDetail);
    char oneDetail[192];
    std::snprintf(oneDetail, sizeof(oneDetail),
                  "scene-referred 1.0 displays as %.9g, must be 1 -- a filmic view would roll this off",
                  static_cast<double>(atOne));
    ENGINE_EXPECT(ctx, std::fabs(atOne - 1.0F) <= kAnchorTolerance, oneDetail);
}

// A display transform must be order-preserving: if it were not, a brighter scene value could display darker, which is
// a visible inversion rather than a subtle grade. Swept across the whole working range including well above display
// white, where a tone curve would compress but must still not fold back on itself.
ENGINE_CHECK(ocio_transform_is_monotone, Slow, Exact) {
    constexpr int kSteps = 256;
    ctx.plan(1);
    float previous = transformScalar(0.0F);
    int inversions = 0;
    float worstAt = 0.0F;
    for (int i = 1; i <= kSteps; ++i) {
        // Geometric-ish sweep to 16.0: linear steps would spend almost every sample above display white, where the
        // curve is flattest, and almost none in the toe where an inversion is most likely.
        const float x = 16.0F * std::pow(static_cast<float>(i) / static_cast<float>(kSteps), 3.0F);
        const float y = transformScalar(x);
        if (!(y >= previous)) {
            if (inversions == 0) {
                worstAt = x;
            }
            ++inversions;
        }
        previous = y;
    }
    char detail[176];
    std::snprintf(detail, sizeof(detail), "%d inversions over %d steps to 16.0 (first at input %.6g)", inversions,
                  kSteps, static_cast<double>(worstAt));
    ENGINE_EXPECT(ctx, inversions == 0, detail);
}

// The transform must act per channel and identically on each: a channel swap or a matrix applied where a curve was
// intended would leave a neutral grey looking neutral while tinting everything else, so a grey input cannot detect it.
ENGINE_CHECK(ocio_transform_is_channel_independent, Fast, Exact) {
    ctx.plan(3);
    std::vector<float> rgb{0.25F, 0.5F, 0.75F};
    engine::gfx::applyOcioDisplayTransform(rgb, 1, 1);
    const float r = transformScalar(0.25F);
    const float g = transformScalar(0.5F);
    const float b = transformScalar(0.75F);
    constexpr float kTolerance = 1e-6F;
    char detail[192];
    std::snprintf(detail, sizeof(detail), "packed (%.6g, %.6g, %.6g) vs per-channel (%.6g, %.6g, %.6g)",
                  static_cast<double>(rgb[0]), static_cast<double>(rgb[1]), static_cast<double>(rgb[2]),
                  static_cast<double>(r), static_cast<double>(g), static_cast<double>(b));
    ENGINE_EXPECT(ctx, std::fabs(rgb[0] - r) <= kTolerance, detail);
    ENGINE_EXPECT(ctx, std::fabs(rgb[1] - g) <= kTolerance, detail);
    ENGINE_EXPECT(ctx, std::fabs(rgb[2] - b) <= kTolerance, detail);
}

// Exercises the packed-image path at a real stride rather than the 1x1 the rows above use: a wrong stride or channel
// ordering in PackedImageDesc would transform the right values in the wrong places, which a single texel cannot show.
ENGINE_CHECK(ocio_transform_handles_a_real_image, Slow, Exact) {
    constexpr int kWidth = 13;  // deliberately not a multiple of any SIMD width
    constexpr int kHeight = 7;
    ctx.plan(1);
    std::vector<float> image(static_cast<std::size_t>(kWidth) * kHeight * 3);
    for (std::size_t i = 0; i < image.size(); ++i) {
        image[i] = static_cast<float>(i % 17) / 16.0F;
    }
    const std::vector<float> original = image;
    engine::gfx::applyOcioDisplayTransform(image, kWidth, kHeight);

    std::size_t mismatched = 0;
    for (std::size_t i = 0; i < image.size(); ++i) {
        const float expected = transformScalar(original[i]);
        mismatched += std::fabs(image[i] - expected) > 1e-6F ? 1 : 0;
    }
    char detail[192];
    std::snprintf(detail, sizeof(detail), "%zu of %zu texel channels disagree with a per-scalar transform of the same value",
                  mismatched, image.size());
    ENGINE_EXPECT(ctx, mismatched == 0, detail);
}

// --- FrameStats ---------------------------------------------------------------------------------------------------
// Testable exactly only because tick() now takes an injectable clock: reading steady_clock internally made every
// assertion below a race against the machine's own speed.

engine::debug::FrameStats tickedWith(const std::vector<float>& millisecondGaps) {
    engine::debug::FrameStats stats;
    auto now = std::chrono::steady_clock::time_point{};
    stats.tick(now);  // the first tick establishes the baseline and records nothing
    for (const float gap : millisecondGaps) {
        now += std::chrono::microseconds(static_cast<long long>(gap * 1000.0F));
        stats.tick(now);
    }
    return stats;
}

// The first tick has no predecessor, so it must record no interval: counting it would enter a garbage frame time at
// startup, which is exactly when a dashboard is being read.
ENGINE_CHECK(frame_stats_ignores_the_first_tick, Fast, Exact) {
    ctx.plan(2);
    engine::debug::FrameStats stats;
    stats.tick(std::chrono::steady_clock::time_point{});
    ENGINE_EXPECT(ctx, stats.avgMs() == 0.0F, "a single tick must record no interval");
    ENGINE_EXPECT(ctx, stats.fps() == 0.0F, "fps must be zero before any interval has been measured");
}

ENGINE_CHECK(frame_stats_reports_exact_intervals, Fast, Exact) {
    ctx.plan(3);
    const engine::debug::FrameStats stats = tickedWith({10.0F, 20.0F, 30.0F});
    char avgDetail[160];
    std::snprintf(avgDetail, sizeof(avgDetail), "mean of 10/20/30 ms read as %.6g", static_cast<double>(stats.avgMs()));
    ENGINE_EXPECT(ctx, std::fabs(stats.avgMs() - 20.0F) <= 1e-3F, avgDetail);
    ENGINE_EXPECT(ctx, std::fabs(stats.minMs() - 10.0F) <= 1e-3F, "minimum interval must be 10 ms");
    ENGINE_EXPECT(ctx, std::fabs(stats.maxMs() - 30.0F) <= 1e-3F, "maximum interval must be 30 ms");
}

// The ring buffer's wrap is where an off-by-one would silently mix a stale frame time into the current window. Feeding
// more than kHistoryLength intervals, all identical after the first batch, makes any survivor visible in the extremes.
ENGINE_CHECK(frame_stats_ring_buffer_wraps_cleanly, Fast, Exact) {
    ctx.plan(2);
    std::vector<float> gaps;
    gaps.reserve(static_cast<std::size_t>(engine::debug::FrameStats::kHistoryLength) * 2);
    for (int i = 0; i < engine::debug::FrameStats::kHistoryLength; ++i) {
        gaps.push_back(99.0F);  // the stale value that must be fully overwritten
    }
    for (int i = 0; i < engine::debug::FrameStats::kHistoryLength; ++i) {
        gaps.push_back(5.0F);
    }
    const engine::debug::FrameStats stats = tickedWith(gaps);
    char detail[176];
    std::snprintf(detail, sizeof(detail), "after a full wrap the window reads min %.6g max %.6g, both must be 5 ms",
                  static_cast<double>(stats.minMs()), static_cast<double>(stats.maxMs()));
    ENGINE_EXPECT(ctx, std::fabs(stats.maxMs() - 5.0F) <= 1e-3F, detail);
    ENGINE_EXPECT(ctx, std::fabs(stats.minMs() - 5.0F) <= 1e-3F, detail);
}

// p50 and p95 against a known distribution. The header is explicit that 120 samples cannot express a p99, so the
// assertion stops where the data does.
ENGINE_CHECK(frame_stats_percentiles_are_correct, Fast, Exact) {
    ctx.plan(2);
    std::vector<float> gaps;
    gaps.reserve(100);
    for (int i = 1; i <= 100; ++i) {
        gaps.push_back(static_cast<float>(i));  // 1..100 ms, so the k-th percentile is k
    }
    const engine::debug::FrameStats stats = tickedWith(gaps);
    char p50[160];
    std::snprintf(p50, sizeof(p50), "p50 of 1..100 ms read as %.6g", static_cast<double>(stats.percentileMs(0.5F)));
    ENGINE_EXPECT(ctx, std::fabs(stats.percentileMs(0.5F) - 51.0F) <= 1.5F, p50);
    char p95[160];
    std::snprintf(p95, sizeof(p95), "p95 of 1..100 ms read as %.6g", static_cast<double>(stats.percentileMs(0.95F)));
    ENGINE_EXPECT(ctx, std::fabs(stats.percentileMs(0.95F) - 96.0F) <= 1.5F, p95);
}

}  // namespace

ENGINE_CHECK_MAIN("display")
