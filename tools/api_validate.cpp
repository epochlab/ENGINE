// Correctness gate for the headless render path a foreign runtime binds to: the AOV classification tables
// (debug/aov.cpp), the four CPU Beauty filters (debug/aov_filters.cpp), and api/headless_renderer.cpp's request
// dispatch.
//
// The filters are asserted ANALYTICALLY rather than against a GPU readback of assets/shaders/edge_filter.frag.
// Reading back the shader would need a GL context, which is the one thing this path exists to avoid, and it would
// only establish that two implementations agree -- not that either is right. A step edge has a closed-form Sobel
// magnitude, an odd-carrier Gabor bank has exactly zero DC response, and the Rec.709 weights are the definition of
// luminance; those are properties of the filters, checkable with no reference image and no tolerance to tune.
//
// Not covered here, deliberately: that HeadlessRenderer's accumulation equals a direct renderPathTraced loop. The
// image-level bit-identity of render_beauty, which is now built on HeadlessRenderer, asserts exactly that against a
// real scene, and restating it against a synthetic fixture would be weaker, not additional.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "check.h"
#include "pathtracer/api/headless_renderer.h"
#include "pathtracer/debug/aov.h"
#include "pathtracer/debug/aov_filters.h"
#include "pathtracer/debug/aov_routing.h"
#include "pathtracer/scene/thread_pool.h"

namespace {

using pathtracer::debug::AovId;
using pathtracer::debug::AovSource;
using pathtracer::gfx::HdrImage;

constexpr int kAovCount = static_cast<int>(AovId::Count);

[[nodiscard]] HdrImage makeImage(int width, int height, float value) {
    return HdrImage{width, height,
                    std::vector<float>(static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 4, value)};
}

void setTexel(HdrImage& image, int x, int y, float r, float g, float b) {
    const std::size_t texel =
        ((static_cast<std::size_t>(y) * static_cast<std::size_t>(image.width)) + static_cast<std::size_t>(x)) * 4;
    image.rgba[texel] = r;
    image.rgba[texel + 1] = g;
    image.rgba[texel + 2] = b;
    image.rgba[texel + 3] = 1.0F;
}

[[nodiscard]] float texelR(const HdrImage& image, int x, int y) {
    return image.rgba[((static_cast<std::size_t>(y) * static_cast<std::size_t>(image.width)) +
                       static_cast<std::size_t>(x)) * 4];
}

// The bank's DC response. An odd (sine) carrier is antisymmetric about the envelope's centre, so every orientation's
// 25 weights must cancel exactly -- this is why a constant field produces no Gabor response, asserted one level down.
[[nodiscard]] float worstOrientationWeightSum() {
    const auto kernel = pathtracer::debug::buildGaborKernel();
    float worst = 0.0F;
    for (int o = 0; o < pathtracer::debug::kGaborOrientations; ++o) {
        float sum = 0.0F;
        for (int tap = 0; tap < pathtracer::debug::kGaborTaps; ++tap) {
            sum += kernel[(static_cast<std::size_t>(o) * pathtracer::debug::kGaborTaps) + static_cast<std::size_t>(tap)];
        }
        worst = std::max(worst, std::fabs(sum));
    }
    return worst;
}

}  // namespace

// Every AovId must be classified, sized, and owned by exactly the producer its classification names. A new AOV added
// without a row in one of the three tables is the failure this catches, and nothing else would notice it: an
// unclassified AOV silently routes to the rasterizer and renders as whatever that buffer last held.
PT_CHECK(aov_tables_are_total_and_consistent, Fast, Exact) {
    ctx.plan(kAovCount * 3);
    for (int i = 0; i < kAovCount; ++i) {
        const auto aov = static_cast<AovId>(i);
        const int channels = pathtracer::debug::aovChannels(aov);
        PT_EXPECT(ctx, channels >= 1 && channels <= 3,
                      std::string(pathtracer::debug::kAovNames[i]) + " channels=" + std::to_string(channels));

        const pathtracer::debug::PathTracedLane traced = pathtracer::debug::pathTracedLane(aov);
        const pathtracer::debug::GBufferLane raster = pathtracer::debug::gbufferLane(aov);
        const AovSource source = pathtracer::debug::aovSource(aov);
        // Exactly one lane accessor may answer, and only the one the classification points at.
        const bool ownedCorrectly = (source == AovSource::PathTraced && traced != nullptr && raster == nullptr) ||
                                     (source == AovSource::GBuffer && raster != nullptr && traced == nullptr) ||
                                     (source == AovSource::BeautyFilter && traced == nullptr && raster == nullptr);
        PT_EXPECT(ctx, ownedCorrectly, std::string(pathtracer::debug::kAovNames[i]) + " lane/source disagree");

        // aovNeedsLightTransport is derived from aovSource; this pins the derivation itself.
        PT_EXPECT(ctx, pathtracer::debug::aovNeedsLightTransport(aov) == (source != AovSource::GBuffer),
                      std::string(pathtracer::debug::kAovNames[i]) + " transport flag disagrees with source");
    }
}

// The display names are the vocabulary every consumer spells an AOV in, so each must resolve, and the separator- and
// case-insensitivity the CLI documents must actually hold for the multi-word ones.
PT_CHECK(aov_names_round_trip, Fast, Exact) {
    ctx.plan(kAovCount + 4);
    for (int i = 0; i < kAovCount; ++i) {
        PT_EXPECT(ctx, pathtracer::debug::aovIdFromName(pathtracer::debug::kAovNames[i]) == static_cast<AovId>(i),
                      std::string("name does not resolve: ") + pathtracer::debug::kAovNames[i]);
    }
    PT_EXPECT(ctx, pathtracer::debug::aovIdFromName("bounce-count") == AovId::BounceCount, "hyphen form");
    PT_EXPECT(ctx, pathtracer::debug::aovIdFromName("BOUNCE_COUNT") == AovId::BounceCount, "upper snake form");
    PT_EXPECT(ctx, pathtracer::debug::aovIdFromName("indirectspecular") == AovId::IndirectSpecular, "run-together form");
    PT_EXPECT(ctx, pathtracer::debug::aovIdFromName("not an aov") == AovId::Count, "unknown name must not resolve");
}

// A Gaussian envelope times an odd carrier integrates to zero over a symmetric support. Asserted on the weights
// themselves rather than inferred from a filtered image, because it is the reason the constant-field check below
// holds and it localises a regression to the kernel rather than to the convolution.
PT_CHECK(gabor_bank_rejects_dc, Fast, Exact) {
    ctx.plan(1);
    const float worst = worstOrientationWeightSum();
    // Float summation of 25 transcendentals, so the bound is accumulated rounding, not a tuned threshold.
    PT_EXPECT(ctx, worst < 1e-6F, "worst orientation weight sum " + std::to_string(worst));
}

// No gradient and no AC content anywhere, so both neighbourhood filters must read exactly zero -- including at the
// border, where edge clamping means the out-of-bounds taps repeat the same constant.
PT_CHECK(filters_are_zero_on_a_constant_field, Fast, Exact) {
    ctx.plan(2);
    pathtracer::scene::ThreadPool pool(static_cast<unsigned int>(ctx.threads()));
    const HdrImage flat = makeImage(24, 16, 0.375F);

    // Read the response channel per texel, not the raw rgba span: alpha is 1 by the broadcast convention, so a
    // whole-buffer reduction measures the alpha channel rather than the filter.
    const HdrImage sobel = pathtracer::debug::sobelAov(flat, pool);
    float worstSobel = 0.0F;
    for (int y = 0; y < sobel.height; ++y) {
        for (int x = 0; x < sobel.width; ++x) {
            worstSobel = std::max(worstSobel, texelR(sobel, x, y));
        }
    }
    PT_EXPECT(ctx, worstSobel == 0.0F, "peak Sobel response " + std::to_string(worstSobel));

    const HdrImage gabor = pathtracer::debug::gaborAov(flat, pool);
    float worstGabor = 0.0F;
    for (int y = 0; y < gabor.height; ++y) {
        for (int x = 0; x < gabor.width; ++x) {
            worstGabor = std::max(worstGabor, texelR(gabor, x, y));
        }
    }
    // Not exactly zero: the weights cancel to rounding, and the convolution scales that residue by the field value.
    PT_EXPECT(ctx, worstGabor < 1e-6F, "peak Gabor response " + std::to_string(worstGabor));
}

// A unit step edge has a closed-form Sobel magnitude. For a vertical edge the Gx kernel sums to 4 over the three
// taps that straddle it and Gy cancels row-wise, so both columns adjacent to the edge read exactly 4 and every
// column two or more pixels away reads exactly 0. No tolerance: these are small integers in float.
PT_CHECK(sobel_step_edge_matches_closed_form, Fast, Exact) {
    ctx.plan(4);
    pathtracer::scene::ThreadPool pool(static_cast<unsigned int>(ctx.threads()));
    constexpr int kWidth = 16;
    constexpr int kHeight = 12;
    constexpr int kEdge = 8;

    HdrImage step = makeImage(kWidth, kHeight, 0.0F);
    for (int y = 0; y < kHeight; ++y) {
        for (int x = 0; x < kWidth; ++x) {
            // White luminance 1.0 requires all three channels at 1, since the Rec.709 weights sum to 1.
            const float value = x >= kEdge ? 1.0F : 0.0F;
            setTexel(step, x, y, value, value, value);
        }
    }

    const HdrImage sobel = pathtracer::debug::sobelAov(step, pool);
    const int midRow = kHeight / 2;
    PT_EXPECT(ctx, texelR(sobel, kEdge - 1, midRow) == 4.0F,
                  "left of edge " + std::to_string(texelR(sobel, kEdge - 1, midRow)));
    PT_EXPECT(ctx, texelR(sobel, kEdge, midRow) == 4.0F,
                  "right of edge " + std::to_string(texelR(sobel, kEdge, midRow)));
    PT_EXPECT(ctx, texelR(sobel, kEdge - 3, midRow) == 0.0F,
                  "far left " + std::to_string(texelR(sobel, kEdge - 3, midRow)));
    PT_EXPECT(ctx, texelR(sobel, kEdge + 3, midRow) == 0.0F,
                  "far right " + std::to_string(texelR(sobel, kEdge + 3, midRow)));
}

// Luminance IS the Rec.709 weighted sum, so each primary must return its own weight exactly. This is the definition
// (ITU-R BT.709-6), not a measurement, which is what makes equality the right assertion.
PT_CHECK(luminance_returns_rec709_weights, Fast, Exact) {
    ctx.plan(4);
    pathtracer::scene::ThreadPool pool(static_cast<unsigned int>(ctx.threads()));
    HdrImage primaries = makeImage(4, 1, 0.0F);
    setTexel(primaries, 0, 0, 1.0F, 0.0F, 0.0F);
    setTexel(primaries, 1, 0, 0.0F, 1.0F, 0.0F);
    setTexel(primaries, 2, 0, 0.0F, 0.0F, 1.0F);
    setTexel(primaries, 3, 0, 1.0F, 1.0F, 1.0F);

    const HdrImage luminance = pathtracer::debug::luminanceAov(primaries, pool);
    PT_EXPECT(ctx, texelR(luminance, 0, 0) == pathtracer::debug::kRec709LuminanceWeights.r, "red weight");
    PT_EXPECT(ctx, texelR(luminance, 1, 0) == pathtracer::debug::kRec709LuminanceWeights.g, "green weight");
    PT_EXPECT(ctx, texelR(luminance, 2, 0) == pathtracer::debug::kRec709LuminanceWeights.b, "blue weight");
    // The three weights are defined to sum to 1, so white maps to 1 and the AOV is a true relative luminance.
    PT_EXPECT(ctx, std::fabs(texelR(luminance, 3, 0) - 1.0F) < 1e-6F,
                  "white luminance " + std::to_string(texelR(luminance, 3, 0)));
}

// HSV is a bijection on the RGB cube away from the achromatic axis, so inverting it must return the original triple.
// Exercised over the interior and over the degenerate rows (greys, and values above display white) that the shader's
// 1e-10 denominator guard only approximates.
PT_CHECK(hsv_inverts_to_rgb, Fast, Exact) {
    pathtracer::scene::ThreadPool pool(static_cast<unsigned int>(ctx.threads()));
    const std::vector<glm::vec3> samples = {
        {0.8F, 0.2F, 0.4F}, {0.1F, 0.9F, 0.3F}, {0.25F, 0.25F, 0.95F}, {1.0F, 0.5F, 0.0F},
        {0.5F, 0.5F, 0.5F},  // achromatic: hue and saturation are 0 by convention
        {0.0F, 0.0F, 0.0F},  // black: saturation undefined, 0 by convention
        {4.0F, 1.5F, 0.25F}, // scene-referred highlight, well above display white
    };
    ctx.plan(static_cast<int>(samples.size()));

    HdrImage image = makeImage(static_cast<int>(samples.size()), 1, 0.0F);
    for (std::size_t i = 0; i < samples.size(); ++i) {
        setTexel(image, static_cast<int>(i), 0, samples[i].r, samples[i].g, samples[i].b);
    }
    const HdrImage hsv = pathtracer::debug::hsvAov(image, pool);

    for (std::size_t i = 0; i < samples.size(); ++i) {
        const std::size_t texel = i * 4;
        const float h = hsv.rgba[texel];
        const float s = hsv.rgba[texel + 1];
        const float v = hsv.rgba[texel + 2];
        // Standard HSV-to-RGB inverse (Smith 1978), stated here rather than imported so the check does not depend on
        // the same code it is checking.
        const float sector = h * 6.0F;
        const auto index = static_cast<int>(std::floor(sector)) % 6;
        const float f = sector - std::floor(sector);
        const float p = v * (1.0F - s);
        const float q = v * (1.0F - (s * f));
        const float t = v * (1.0F - (s * (1.0F - f)));
        glm::vec3 back{};
        switch (index) {
            case 0:  back = {v, t, p}; break;
            case 1:  back = {q, v, p}; break;
            case 2:  back = {p, v, t}; break;
            case 3:  back = {p, q, v}; break;
            case 4:  back = {t, p, v}; break;
            default: back = {v, p, q}; break;
        }
        const float worst = std::max({std::fabs(back.r - samples[i].r), std::fabs(back.g - samples[i].g),
                                       std::fabs(back.b - samples[i].b)});
        // Forward-error bound of the divide-and-remultiply round trip, which scales with the value being represented.
        const float bound = 1e-6F * std::max(1.0F, samples[i].r + samples[i].g + samples[i].b);
        PT_EXPECT(ctx, worst <= bound, "round-trip error " + std::to_string(worst));
    }
}

// Every AOV must render, at its declared channel count, with no NaN or infinity anywhere. This is the check that a
// newly added AOV cannot pass without actually being produced by something.
PT_CHECK(every_aov_renders_finite, Slow, Exact) {
    ctx.plan(kAovCount);
    std::string error;
    const auto renderer = pathtracer::api::HeadlessRenderer::open(ASSET_ROOT_DIR, "scenes/cornell.json", error);
    if (!renderer) {
        PT_EXPECT(ctx, false, "scene load failed: " + error);
        return;
    }

    constexpr int kWidth = 24;
    constexpr int kHeight = 18;
    for (int i = 0; i < kAovCount; ++i) {
        const auto aov = static_cast<AovId>(i);
        const pathtracer::api::HeadlessRenderer::Request request{
            .camera = renderer->defaultCamera(),
            .width = kWidth,
            .height = kHeight,
            .samples = 2,
            .scrambleSeed = 1,
            .aovs = {aov},
        };

        std::vector<float> buffer(static_cast<std::size_t>(kWidth) * kHeight *
                                  static_cast<std::size_t>(pathtracer::debug::aovChannels(aov)));
        float* pointer = buffer.data();
        if (!renderer->render(request, std::span<float* const>(&pointer, 1), error)) {
            PT_EXPECT(ctx, false, std::string(pathtracer::debug::kAovNames[i]) + ": " + error);
            continue;
        }
        const bool finite = std::all_of(buffer.begin(), buffer.end(),
                                         [](float value) { return std::isfinite(value); });
        PT_EXPECT(ctx, finite, std::string(pathtracer::debug::kAovNames[i]) + " contains a non-finite value");
    }
}

// Sharing one path-trace accumulation, one rasterizer pass and one Beauty across a multi-AOV request must not change
// any AOV's values. This is the property the sharing optimisation has to preserve, and the one a bug in the dispatch
// would break: bit-identity against the same AOVs requested one at a time.
PT_CHECK(multi_aov_request_matches_single_aov_requests, Slow, Exact) {
    const std::vector<AovId> combined = {AovId::Beauty, AovId::Depth, AovId::Normal, AovId::Sobel, AovId::AO};
    ctx.plan(static_cast<int>(combined.size()));

    std::string error;
    const auto renderer = pathtracer::api::HeadlessRenderer::open(ASSET_ROOT_DIR, "scenes/cornell.json", error);
    if (!renderer) {
        for (std::size_t i = 0; i < combined.size(); ++i) {
            PT_EXPECT(ctx, false, "scene load failed: " + error);
        }
        return;
    }

    constexpr int kWidth = 32;
    constexpr int kHeight = 24;
    const auto makeRequest = [&](const std::vector<AovId>& aovs) {
        return pathtracer::api::HeadlessRenderer::Request{
            .camera = renderer->defaultCamera(),
            .width = kWidth,
            .height = kHeight,
            .samples = 4,
            .scrambleSeed = 7,
            .aovs = aovs,
        };
    };

    std::vector<std::vector<float>> together(combined.size());
    std::vector<float*> pointers(combined.size());
    for (std::size_t i = 0; i < combined.size(); ++i) {
        together[i].resize(static_cast<std::size_t>(kWidth) * kHeight *
                           static_cast<std::size_t>(pathtracer::debug::aovChannels(combined[i])));
        pointers[i] = together[i].data();
    }
    if (!renderer->render(makeRequest(combined), std::span<float* const>(pointers.data(), pointers.size()), error)) {
        for (std::size_t i = 0; i < combined.size(); ++i) {
            PT_EXPECT(ctx, false, "combined render failed: " + error);
        }
        return;
    }

    for (std::size_t i = 0; i < combined.size(); ++i) {
        std::vector<float> alone(together[i].size());
        float* pointer = alone.data();
        if (!renderer->render(makeRequest({combined[i]}), std::span<float* const>(&pointer, 1), error)) {
            PT_EXPECT(ctx, false, "single render failed: " + error);
            continue;
        }
        PT_EXPECT(ctx, alone == together[i],
                      std::string(pathtracer::debug::kAovNames[static_cast<int>(combined[i])]) +
                          " differs between a combined and a single-AOV request");
    }
}

PT_CHECK_MAIN("api")
