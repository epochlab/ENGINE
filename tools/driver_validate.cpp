// Correctness gate for PathTraceDriver (path_trace_driver.h), previously untested. Driven only through its public API
// (requestTrace / accumulatedSamples / latestResult, maxSamples as the stop condition); also pins the request invariant
// that driverLoop's suppressed unchecked-optional-access reports rely on.

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <optional>
#include <thread>
#include <vector>

#include <glm/glm.hpp>

#include "check.h"
#include "fixtures.h"
#include "stats.h"
#include "engine/scene/camera.h"
#include "engine/scene/embree_accel.h"
#include "engine/scene/environment_map.h"
#include "engine/scene/light.h"
#include "engine/scene/path_trace_driver.h"
#include "engine/scene/path_tracer.h"
#include "engine/scene/shading_scene.h"
#include "engine/scene/thread_pool.h"

namespace {

using engine::scene::Camera;
using engine::scene::EmbreeAccel;
using engine::scene::EnvironmentMap;
using engine::scene::MeshInstance;
using engine::scene::PathTraceDriver;
using engine::scene::PathTraceResult;
using engine::scene::PathTraceSettings;
using engine::scene::QuadLight;
using engine::scene::ShadingTriangle;
using engine::scene::ShadingVertex;
using engine::scene::Triangle;

// Small enough that a pass is milliseconds -- this file measures accumulation arithmetic and thread handshakes, not
// transport, so the image only has to be non-trivial, not converged.
constexpr int kImageSize = 8;
constexpr float kQuadExtent = 50.0F;

// A single quad facing the camera, the same shape integrator_validate uses, kept local because this file must build
// its scene at a permanent address that outlives the driver (see PathTraceDriver's constructor contract).
struct TestScene {
    std::vector<Triangle> worldTriangles;
    std::vector<ShadingTriangle> shadingTriangles;
    std::vector<MeshInstance> instances;
    std::vector<int> instanceLightIndex;
    std::vector<QuadLight> quadLights;
    std::vector<PathTraceSettings> perInstanceSettings;
};

void pushQuad(TestScene& scene, float z) {
    const glm::vec3 corners[4] = {{-kQuadExtent, -kQuadExtent, z},
                                   {kQuadExtent, -kQuadExtent, z},
                                   {kQuadExtent, kQuadExtent, z},
                                   {-kQuadExtent, kQuadExtent, z}};
    const glm::vec3 normal(0.0F, 0.0F, 1.0F);
    const std::array<int, 6> order = {0, 1, 2, 0, 2, 3};
    for (int t = 0; t < 2; ++t) {
        Triangle world{};
        ShadingTriangle shading{};
        for (int v = 0; v < 3; ++v) {
            const glm::vec3 p = corners[order[(t * 3) + v]];
            (v == 0 ? world.v0 : v == 1 ? world.v1 : world.v2) = p;
            ShadingVertex vertex{};
            vertex.position = p;
            vertex.normal = normal;
            vertex.tangent = glm::vec4(1.0F, 0.0F, 0.0F, 1.0F);
            vertex.uv = glm::vec2(0.0F);
            (v == 0 ? shading.v0 : v == 1 ? shading.v1 : shading.v2) = vertex;
        }
        shading.instanceIndex = 0;
        scene.worldTriangles.push_back(world);
        scene.shadingTriangles.push_back(shading);
    }
}

PathTraceSettings makeSettings() {
    PathTraceSettings settings{};
    settings.samplesPerPixel = 1;  // one sample per pass: the progressive configuration the driver exists for
    settings.maxBounces = 1;
    settings.russianRouletteStartBounce = 8;
    settings.bumpStrength = 0.0F;
    settings.roughnessMin = 0.0F;
    settings.roughnessMax = 1.0F;
    settings.diffuseColour = glm::vec3(1.0F);
    settings.ior = 1.5F;
    settings.transmissionFactor = 0.0F;
    settings.metallicFactor = 0.0F;
    settings.roughnessFactor = 1.0F;
    return settings;
}

// Everything the driver holds by reference must outlive it and never move, so the whole fixture is heap-allocated once
// and the driver is constructed only after the referenced members are at their final addresses.
struct DriverFixture {
    TestScene scene;
    std::optional<EmbreeAccel> accel;
    EnvironmentMap environment = tools::fixtures::makeUniformEnvironment();
    std::optional<PathTraceDriver> driver;
    engine::scene::ThreadPool pool;

    [[nodiscard]] bool valid() const { return accel.has_value() && driver.has_value(); }
};

std::unique_ptr<DriverFixture> makeFixture() {
    auto fixture = std::make_unique<DriverFixture>();
    pushQuad(fixture->scene, -3.0F);
    fixture->scene.instances.push_back(
        MeshInstance{tools::fixtures::makeMaterial(1.0F, glm::vec3(0.04F)), glm::mat4(1.0F), ""});
    fixture->scene.instanceLightIndex.assign(fixture->scene.instances.size(), -1);
    fixture->scene.perInstanceSettings.assign(fixture->scene.instances.size(), makeSettings());
    fixture->accel = EmbreeAccel::build(fixture->scene.worldTriangles);
    if (!fixture->accel.has_value()) {
        return fixture;
    }
    fixture->driver.emplace(*fixture->accel, fixture->scene.shadingTriangles, fixture->scene.instances,
                             fixture->scene.instanceLightIndex, fixture->environment,
                             fixture->scene.quadLights, fixture->scene.perInstanceSettings);
    return fixture;
}

Camera makeCamera() {
    return Camera(glm::vec3(0.0F, 0.0F, 5.0F), 0.0F, 0.0F, Camera::FilmBack{36.0F, 24.0F}, 50.0F, 0.01F, 1000.0F,
                   2.8F, 1.0F / 125.0F, 100.0F);
}

PathTraceDriver::Request makeRequest(int maxSamples, const Camera& camera) {
    PathTraceDriver::Request request{camera, 0, 0, 0.0F, true, true, 1.0F, makeSettings(), 0};
    request.width = kImageSize;
    request.height = kImageSize;
    request.envRotationRadians = 0.0F;
    request.showSky = true;
    request.envLightEnabled = true;
    request.envExposure = 1.0F;
    request.settings = makeSettings();
    request.maxSamples = maxSamples;
    return request;
}

// Polls until the predicate holds or the deadline passes. The deadline is a HANG DETECTOR, never a performance
// assertion: an async driver may legitimately take any amount of time, so the only honest assertions are "eventually"
// and "monotonically". Generous by an order of magnitude over the slowest configuration here.
template <typename Predicate>
bool waitFor(Predicate predicate) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(60);
    while (std::chrono::steady_clock::now() < deadline) {
        if (predicate()) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false;
}

// Renders the same passes the driver would, synchronously, and forms the batch mean in double. The driver's own
// scramble seed is its generation, which is 1 for the first request of a driver's life.
engine::gfx::HdrImage oracleBatchMean(DriverFixture& fixture, const Camera& camera, int passes,
                                       std::uint32_t scrambleSeed) {
    const engine::scene::LightSet lights(&fixture.environment, 0.0F, 1.0F, fixture.scene.quadLights);
    PathTraceResult pass = engine::scene::makePathTraceResult(kImageSize, kImageSize);
    std::vector<double> sum(static_cast<std::size_t>(kImageSize) * kImageSize * 4, 0.0);
    const std::atomic<std::uint64_t> generation{scrambleSeed};
    engine::debug::PassStats stats;
    for (int p = 0; p < passes; ++p) {
        stats.reset();
        engine::scene::renderPathTraced(camera, *fixture.accel, fixture.scene.shadingTriangles,
                                         fixture.scene.instances, fixture.scene.instanceLightIndex, lights,
                                         kImageSize, kImageSize, /*showSky=*/true, makeSettings(),
                                         fixture.scene.perInstanceSettings, scrambleSeed, /*sampleBase=*/p,
                                         /*sampleCount=*/passes, generation, scrambleSeed, fixture.pool, stats,
                                         pass);
        for (std::size_t i = 0; i < sum.size(); ++i) {
            sum[i] += static_cast<double>(pass.beauty.rgba[i]);
        }
    }
    engine::gfx::HdrImage mean{kImageSize, kImageSize,
                                std::vector<float>(static_cast<std::size_t>(kImageSize) * kImageSize * 4, 0.0F)};
    for (std::size_t i = 0; i < sum.size(); ++i) {
        mean.rgba[i] = static_cast<float>(sum[i] / static_cast<double>(passes));
    }
    return mean;
}

// --- Checks ------------------------------------------------------------------------------------------------------

// The core accumulation property. The driver publishes a RUNNING mean, m_n = m_{n-1} + (x_n - m_{n-1})/n, which is a
// different rounding sequence from the batch mean sum(x)/n -- so the two are close but NOT bit-identical, and the band
// is a deterministic forward-error bound rather than a statistical one (Higham, Accuracy and Stability of Numerical
// Algorithms, 2nd ed. 4.2). There is no probability in this assertion.
// The bound is still extremely tight relative to the failures it must catch: a running sum published without its
// division, a stale previous mean, or a missing entry in the nine-image accumulate loop (whose own comment warns that
// nothing guards it at compile time) all miss by orders of magnitude, not by ulps.
ENGINE_CHECK(running_mean_matches_batch_mean, Slow, Exact) {
    constexpr int kPasses = 8;
    ctx.plan(1);
    std::unique_ptr<DriverFixture> fixture = makeFixture();
    if (!fixture->valid()) {
        ENGINE_EXPECT(ctx, false, "scene/driver construction failed");
        return;
    }
    const Camera camera = makeCamera();
    fixture->driver->requestTrace(makeRequest(kPasses, camera));
    const bool reached = waitFor([&] { return fixture->driver->accumulatedSamples() >= kPasses; });
    if (!reached) {
        ENGINE_EXPECT(ctx, false, "driver never reached its maxSamples cap");
        return;
    }
    const std::shared_ptr<const PathTraceResult> published = fixture->driver->latestResult();
    if (published == nullptr) {
        ENGINE_EXPECT(ctx, false, "driver published nothing after reaching its cap");
        return;
    }
    // Generation 1: the first requestTrace of this driver's life, and the driver uses the generation AS the seed.
    const engine::gfx::HdrImage expected = oracleBatchMean(*fixture, camera, kPasses, 1U);

    double worstError = 0.0;
    double maxTerm = 0.0;
    for (float v : expected.rgba) {
        maxTerm = std::max(maxTerm, static_cast<double>(std::fabs(v)));
    }
    // gamma_n = n*u/(1 - n*u) with u = 2^-24 for float32, applied to the magnitude being accumulated.
    constexpr double kUnitRoundoff = 5.9604644775390625e-08;  // 2^-24
    const double gamma = (kPasses * kUnitRoundoff) / (1.0 - (kPasses * kUnitRoundoff));
    const double bound = gamma * std::max(maxTerm, 1e-6);
    for (std::size_t i = 0; i < expected.rgba.size(); ++i) {
        const double error = std::fabs(static_cast<double>(published->beauty.rgba[i]) -
                                        static_cast<double>(expected.rgba[i]));
        worstError = std::max(worstError, error);
    }
    char detail[224];
    std::snprintf(detail, sizeof(detail),
                  "worst |running - batch| = %.3e over %zu texels, forward-error bound %.3e (gamma_%d, peak %.3f)",
                  worstError, expected.rgba.size(), bound, kPasses, maxTerm);
    ENGINE_EXPECT(ctx, worstError <= bound, detail);
}

// The sampler's two inputs are not interchangeable (sampler.h), and a driver that held sampleBase at 0 would publish
// the mean of N IDENTICAL passes -- the exact defect sampler_validate's pass-direction check was written against,
// here asserted through the driver that would commit it.
// Checked exactly at maxSamples = 1, where the published image must be bit-identical to the single oracle pass: no
// averaging has happened yet, so there is no rounding difference to admit a tolerance.
ENGINE_CHECK(first_pass_is_bit_identical_to_oracle, Slow, Exact) {
    ctx.plan(1);
    std::unique_ptr<DriverFixture> fixture = makeFixture();
    if (!fixture->valid()) {
        ENGINE_EXPECT(ctx, false, "scene/driver construction failed");
        return;
    }
    const Camera camera = makeCamera();
    fixture->driver->requestTrace(makeRequest(1, camera));
    if (!waitFor([&] { return fixture->driver->accumulatedSamples() >= 1; })) {
        ENGINE_EXPECT(ctx, false, "driver never completed its first pass");
        return;
    }
    const std::shared_ptr<const PathTraceResult> published = fixture->driver->latestResult();
    if (published == nullptr) {
        ENGINE_EXPECT(ctx, false, "driver published nothing");
        return;
    }
    const engine::gfx::HdrImage expected = oracleBatchMean(*fixture, camera, 1, 1U);
    std::size_t differing = 0;
    for (std::size_t i = 0; i < expected.rgba.size(); ++i) {
        differing += published->beauty.rgba[i] != expected.rgba[i] ? 1 : 0;
    }
    char detail[192];
    std::snprintf(detail, sizeof(detail), "%zu of %zu floats differ from a direct single-pass render", differing,
                  expected.rgba.size());
    ENGINE_EXPECT(ctx, differing == 0, detail);
}

// The cap is a hard stop, not a target to overshoot: a driver that kept accumulating past it would keep a converged
// image moving under a caller that had asked it to stop, and would burn a core doing it.
ENGINE_CHECK(max_samples_cap_is_respected, Slow, Exact) {
    constexpr int kCap = 4;
    ctx.plan(2);
    std::unique_ptr<DriverFixture> fixture = makeFixture();
    if (!fixture->valid()) {
        ENGINE_EXPECT(ctx, false, "scene/driver construction failed");
        ENGINE_EXPECT(ctx, false, "scene/driver construction failed");
        return;
    }
    fixture->driver->requestTrace(makeRequest(kCap, makeCamera()));
    const bool reached = waitFor([&] { return fixture->driver->accumulatedSamples() >= kCap; });
    ENGINE_EXPECT(ctx, reached, "driver never reached its maxSamples cap");
    // Long enough that an uncapped driver would have run many further passes at this image size.
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    char detail[128];
    std::snprintf(detail, sizeof(detail), "accumulated %d samples against a cap of %d",
                  fixture->driver->accumulatedSamples(), kCap);
    ENGINE_EXPECT(ctx, fixture->driver->accumulatedSamples() == kCap, detail);
}

// A new request must RESTART accumulation rather than merge into it: the class comment's central invariant is that the
// accumulator never mixes samples from two different camera poses. Also pins the invariant the sixteen suppressed
// unchecked-optional-access reports rest on -- that a generation bump is always accompanied by an engaged request --
// since a driver that read an empty request here would not survive the round trip.
ENGINE_CHECK(new_request_restarts_accumulation, Slow, Exact) {
    ctx.plan(2);
    std::unique_ptr<DriverFixture> fixture = makeFixture();
    if (!fixture->valid()) {
        ENGINE_EXPECT(ctx, false, "scene/driver construction failed");
        ENGINE_EXPECT(ctx, false, "scene/driver construction failed");
        return;
    }
    fixture->driver->requestTrace(makeRequest(4, makeCamera()));
    if (!waitFor([&] { return fixture->driver->accumulatedSamples() >= 4; })) {
        ENGINE_EXPECT(ctx, false, "driver never reached the first cap");
        ENGINE_EXPECT(ctx, false, "driver never reached the first cap");
        return;
    }

    // A different camera: a genuinely different image, which must not be averaged into the first.
    const Camera moved(glm::vec3(0.5F, 0.25F, 5.0F), 10.0F, -5.0F, Camera::FilmBack{36.0F, 24.0F}, 50.0F, 0.01F,
                        1000.0F, 2.8F, 1.0F / 125.0F, 100.0F);
    fixture->driver->requestTrace(makeRequest(4, moved));
    // The counter must return to a value at or below the new cap having restarted, not continue climbing past it.
    const bool restarted = waitFor([&] { return fixture->driver->accumulatedSamples() >= 4; });
    ENGINE_EXPECT(ctx, restarted, "driver never reached the second cap after the camera moved");
    char detail[160];
    std::snprintf(detail, sizeof(detail), "accumulated %d after restart, which must equal the new cap of 4",
                  fixture->driver->accumulatedSamples());
    ENGINE_EXPECT(ctx, fixture->driver->accumulatedSamples() == 4, detail);
}

// Suspension parks the driver while a rasterizer-backed AOV is shown. Entering it bumps the generation (cancelling the
// in-flight pass), so resuming restarts accumulation under that new generation; asserted here: halts, then resumes.
ENGINE_CHECK(suspension_halts_and_resumes, Slow, Exact) {
    ctx.plan(2);
    std::unique_ptr<DriverFixture> fixture = makeFixture();
    if (!fixture->valid()) {
        ENGINE_EXPECT(ctx, false, "scene/driver construction failed");
        ENGINE_EXPECT(ctx, false, "scene/driver construction failed");
        return;
    }
    // Uncapped, so only suspension can stop it.
    fixture->driver->requestTrace(makeRequest(0, makeCamera()));
    if (!waitFor([&] { return fixture->driver->accumulatedSamples() >= 2; })) {
        ENGINE_EXPECT(ctx, false, "driver never started accumulating");
        ENGINE_EXPECT(ctx, false, "driver never started accumulating");
        return;
    }
    fixture->driver->setSuspended(true);
    // Let any pass already in flight finish and be discarded before sampling the counter.
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    const int parked = fixture->driver->accumulatedSamples();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    char haltDetail[160];
    std::snprintf(haltDetail, sizeof(haltDetail), "accumulated %d while suspended, was %d when parked",
                  fixture->driver->accumulatedSamples(), parked);
    ENGINE_EXPECT(ctx, fixture->driver->accumulatedSamples() == parked, haltDetail);

    fixture->driver->setSuspended(false);
    ENGINE_EXPECT(ctx, waitFor([&] { return fixture->driver->accumulatedSamples() > parked; }),
                  "driver did not resume accumulating after suspension was lifted");
}

// The buffer pool holds four images for up to three simultaneously-pinned results, an arithmetic nothing tested. A
// caller holding strong references must never have one overwritten underneath it: acquireFreeBuffer is required to
// STALL rather than hand back a buffer someone is still reading, and this is the only external test of that.
ENGINE_CHECK(published_results_are_not_overwritten, Slow, Exact) {
    ctx.plan(1);
    std::unique_ptr<DriverFixture> fixture = makeFixture();
    if (!fixture->valid()) {
        ENGINE_EXPECT(ctx, false, "scene/driver construction failed");
        return;
    }
    fixture->driver->requestTrace(makeRequest(0, makeCamera()));
    if (!waitFor([&] { return fixture->driver->latestResult() != nullptr; })) {
        ENGINE_EXPECT(ctx, false, "driver published nothing");
        return;
    }
    // Pin a published result and snapshot it, then let the driver run many further passes. The running mean changes
    // every pass, so if the pinned buffer were recycled its contents would move.
    const std::shared_ptr<const PathTraceResult> pinned = fixture->driver->latestResult();
    const std::vector<float> snapshot = pinned->beauty.rgba;
    const int startedAt = fixture->driver->accumulatedSamples();
    if (!waitFor([&] { return fixture->driver->accumulatedSamples() >= startedAt + 6; })) {
        ENGINE_EXPECT(ctx, false, "driver stalled while a single result was pinned");
        return;
    }
    std::size_t moved = 0;
    for (std::size_t i = 0; i < snapshot.size(); ++i) {
        moved += pinned->beauty.rgba[i] != snapshot[i] ? 1 : 0;
    }
    char detail[192];
    std::snprintf(detail, sizeof(detail), "%zu of %zu floats in a pinned result changed while %d further passes ran",
                  moved, snapshot.size(), fixture->driver->accumulatedSamples() - startedAt);
    ENGINE_EXPECT(ctx, moved == 0, detail);
}

// The over-range histogram is a parallel reduction with a published serial definition, so it is assertable exactly:
// integer counts, recomputed single-threaded from the very image the driver published alongside them.
ENGINE_CHECK(over_range_stats_match_serial_scan, Slow, Exact) {
    ctx.plan(4);
    std::unique_ptr<DriverFixture> fixture = makeFixture();
    if (!fixture->valid()) {
        for (int i = 0; i < 4; ++i) {
            ENGINE_EXPECT(ctx, false, "scene/driver construction failed");
        }
        return;
    }
    fixture->driver->requestTrace(makeRequest(2, makeCamera()));
    if (!waitFor([&] { return fixture->driver->accumulatedSamples() >= 2; })) {
        for (int i = 0; i < 4; ++i) {
            ENGINE_EXPECT(ctx, false, "driver never reached its cap");
        }
        return;
    }
    const std::shared_ptr<const PathTraceResult> published = fixture->driver->latestResult();
    if (published == nullptr) {
        for (int i = 0; i < 4; ++i) {
            ENGINE_EXPECT(ctx, false, "driver published nothing");
        }
        return;
    }

    const engine::scene::OverRangeStats& stats = published->overRange;
    const std::size_t texels = static_cast<std::size_t>(kImageSize) * kImageSize;

    // aboveBin is a complementary CDF: bin 0 counts every texel and the last bin counts none.
    char zeroDetail[160];
    std::snprintf(zeroDetail, sizeof(zeroDetail), "aboveBin[0] = %u, must equal the texel count %zu",
                  stats.aboveBin[0], texels);
    ENGINE_EXPECT(ctx, static_cast<std::size_t>(stats.aboveBin[0]) == texels, zeroDetail);
    ENGINE_EXPECT(ctx, stats.aboveBin[engine::scene::kOverRangeBinCount] == 0,
                  "the last complementary-CDF entry must be empty by construction");

    bool monotone = true;
    for (std::size_t b = 1; b < stats.aboveBin.size(); ++b) {
        monotone = monotone && stats.aboveBin[b] <= stats.aboveBin[b - 1];
    }
    ENGINE_EXPECT(ctx, monotone, "a complementary CDF must be non-increasing in the bin index");

    float serialPeak = 0.0F;
    for (std::size_t t = 0; t < texels; ++t) {
        const float r = published->beauty.rgba[(t * 4) + 0];
        const float g = published->beauty.rgba[(t * 4) + 1];
        const float b = published->beauty.rgba[(t * 4) + 2];
        serialPeak = std::max(serialPeak, std::max(r, std::max(g, b)));
    }
    char peakDetail[176];
    std::snprintf(peakDetail, sizeof(peakDetail), "published rawPeak %.9g vs single-threaded scan %.9g",
                  static_cast<double>(stats.rawPeak), static_cast<double>(serialPeak));
    ENGINE_EXPECT(ctx, stats.rawPeak == serialPeak, peakDetail);
}

// Thread-count invariance, checked one level DOWN on renderPathTraced rather than through the driver: the driver's
// pool is private and default-sized, with no constructor hook, so the property is not reachable from its API. That is
// the right level anyway. The assertion is bit-identity rather than a band, because tiles are owned outright by one
// worker, the filter halo is re-traced rather than shared, and per-pixel sample order is fixed -- so the accumulation
// order does not depend on the pool size. If this fails it is a finding, not a tolerance problem.
ENGINE_CHECK(render_is_invariant_to_thread_count, Slow, Exact) {
    ctx.plan(1);
    std::unique_ptr<DriverFixture> fixture = makeFixture();
    if (!fixture->valid()) {
        ENGINE_EXPECT(ctx, false, "scene construction failed");
        return;
    }
    const Camera camera = makeCamera();
    const engine::scene::LightSet lights(&fixture->environment, 0.0F, 1.0F, fixture->scene.quadLights);
    const std::atomic<std::uint64_t> generation{1};
    engine::debug::PassStats stats;

    const auto renderWith = [&](unsigned int threads) {
        engine::scene::ThreadPool pool(threads);
        PathTraceResult out = engine::scene::makePathTraceResult(kImageSize, kImageSize);
        stats.reset();
        engine::scene::renderPathTraced(camera, *fixture->accel, fixture->scene.shadingTriangles,
                                         fixture->scene.instances, fixture->scene.instanceLightIndex, lights,
                                         kImageSize, kImageSize, /*showSky=*/true, makeSettings(),
                                         fixture->scene.perInstanceSettings, /*scrambleSeed=*/1U, /*sampleBase=*/0,
                                         /*sampleCount=*/1, generation, 1U, pool, stats, out);
        return out.beauty.rgba;
    };

    const std::vector<float> single = renderWith(1);
    const std::vector<float> many = renderWith(std::max(2U, std::thread::hardware_concurrency()));
    std::size_t differing = 0;
    for (std::size_t i = 0; i < single.size(); ++i) {
        differing += single[i] != many[i] ? 1 : 0;
    }
    char detail[176];
    std::snprintf(detail, sizeof(detail), "%zu of %zu floats differ between a 1-thread and an N-thread render",
                  differing, single.size());
    ENGINE_EXPECT(ctx, differing == 0, detail);
}

}  // namespace

ENGINE_CHECK_MAIN("driver")
