#include "engine/scene/path_trace_driver.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <thread>
#include <utility>
#include <vector>

namespace engine::scene {

namespace {

constexpr std::chrono::milliseconds kIdlePollInterval{5};

// Incremental running mean, out of place: newMean = previousMean + (sample - previousMean)/n, computed in the freshly rendered pass buffer so the published set is only ever READ. That is what lets the publish below be a pointer assignment instead of the whole-image copy it used to hold resultMutex_ for, and what keeps a render thread holding the previous set from seeing a half-updated image. Running mean rather than a running sum: no separate sum buffer, and the result is always already display-ready with no final division. Row-parallel over the same pool the pass just finished with, which is otherwise idle at this moment.
void accumulateMean(PathTraceResult& sample, const PathTraceResult& previousMean, int n,
                     ThreadPool& threadPool) {
    // Index-aligned with `sources` below, and every PathTraceResult image must appear: unlike rasterizer.cpp's aovImages() there is no compile-time guard here, so a missing entry silently publishes that image's last pass instead of the running mean.
    const std::array<engine::gfx::HdrImage*, 9> destinations{
        &sample.beauty,          &sample.bounceHeatmap,    &sample.ao,
        &sample.shadow,          &sample.directDiffuse,    &sample.indirectDiffuse,
        &sample.directSpecular,  &sample.indirectSpecular, &sample.refraction};
    const std::array<const engine::gfx::HdrImage*, 9> sources{
        &previousMean.beauty,          &previousMean.bounceHeatmap,
        &previousMean.ao,              &previousMean.shadow,
        &previousMean.directDiffuse,   &previousMean.indirectDiffuse,
        &previousMean.directSpecular,  &previousMean.indirectSpecular,
        &previousMean.refraction};
    const float invN = 1.0F / static_cast<float>(n);
    const auto rowFloats = static_cast<std::size_t>(sample.beauty.width) * 4;
    threadPool.parallelFor(sample.beauty.height, [&](int y) {
        const std::size_t begin = static_cast<std::size_t>(y) * rowFloats;
        for (std::size_t image = 0; image < destinations.size(); ++image) {
            float* destination = destinations[image]->rgba.data();
            const float* source = sources[image]->rgba.data();
            for (std::size_t i = begin; i < begin + rowFloats; ++i) {
                destination[i] = source[i] + ((destination[i] - source[i]) * invN);
            }
        }
    });
}

// Reduces the published mean's beauty to OverRangeStats on the driver's own pool -- the same walk the render thread used to do serially, on the thread that just wrote these texels and still holds them in cache. Row-chunked with per-chunk private accumulators folded serially afterwards, following buildSubTriangles (rasterizer.cpp): a histogram bin is far too contended for one atomic increment per texel.
// One chunk per worker rather than buildSubTriangles' four: every texel costs the same, so there is no imbalance for finer granularity to absorb, and each extra chunk is another kOverRangeBinCount-entry array to zero and fold.
// A separate pass over beauty rather than a fold into accumulateMean's inner loop, which would save the re-read: accumulateMean is skipped on the first pass of every generation (`if (n > 1)` below), which is exactly the interactive case, so fusing would need a conditional duplicate of this code on that path.
void reduceOverRange(PathTraceResult& pass, std::vector<OverRangeHistogram>& histograms,
                      std::vector<float>& peaks, ThreadPool& threadPool) {
    const engine::gfx::HdrImage& beauty = pass.beauty;
    const int chunkCount =
        std::max(1, std::min(beauty.height, static_cast<int>(threadPool.threadCount())));
    const int chunkRows = (beauty.height + chunkCount - 1) / chunkCount;
    const auto rowFloats = static_cast<std::size_t>(beauty.width) * 4;
    histograms.assign(static_cast<std::size_t>(chunkCount), OverRangeHistogram{});
    peaks.assign(static_cast<std::size_t>(chunkCount), 0.0F);

    threadPool.parallelFor(chunkCount, [&](int chunk) {
        OverRangeHistogram& bins = histograms[static_cast<std::size_t>(chunk)];
        const std::size_t begin = static_cast<std::size_t>(chunk * chunkRows) * rowFloats;
        const std::size_t end =
            static_cast<std::size_t>(std::min((chunk + 1) * chunkRows, beauty.height)) * rowFloats;
        const float* rgba = beauty.rgba.data();
        // Peak kept in a register and stored once at the end: `peaks` is the only array adjacent chunks could false-share, since each chunk's bin writes stay inside its own histogram.
        float peak = 0.0F;
        for (std::size_t i = begin; i < end; i += 4) {
            const float maxChannel = std::max({rgba[i], rgba[i + 1], rgba[i + 2]});
            ++bins[static_cast<std::size_t>(overRangeBin(maxChannel))];
            peak = std::max(peak, maxChannel);
        }
        peaks[static_cast<std::size_t>(chunk)] = peak;
    });

    OverRangeStats& out = pass.overRange;
    out.rawPeak = *std::max_element(peaks.begin(), peaks.end());
    // Summed chunk-major, then turned into a suffix sum in place: both walks are sequential over one array, where folding straight into the complementary CDF would stride across every chunk's histogram per bin.
    std::fill(out.aboveBin.begin(), out.aboveBin.end(), 0U);
    for (const OverRangeHistogram& bins : histograms) {
        for (std::size_t bin = 0; bin < bins.size(); ++bin) {
            out.aboveBin[bin] += bins[bin];
        }
    }
    for (std::size_t bin = kOverRangeBinCount; bin-- > 0;) {
        out.aboveBin[bin] += out.aboveBin[bin + 1];
    }
}

double millisecondsSince(std::chrono::steady_clock::time_point start) {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

}  // namespace

PathTraceDriver::PathTraceDriver(const EmbreeAccel& accel,
                                  const std::vector<ShadingTriangle>& shadingTriangles,
                                  const std::vector<MeshInstance>& instances,
                                  const std::vector<int>& instanceLightIndex,
                                  const EnvironmentMap& environmentMap,
                                  const std::vector<QuadLight>& quadLights,
                                  const std::vector<PathTraceSettings>& perInstanceSettings)
    : accel_(accel),
      shadingTriangles_(shadingTriangles),
      instances_(instances),
      instanceLightIndex_(instanceLightIndex),
      environmentMap_(environmentMap),
      quadLights_(quadLights),
      perInstanceSettings_(perInstanceSettings),
      thread_([this](std::stop_token stopToken) { driverLoop(std::move(stopToken)); }) {}

PathTraceDriver::~PathTraceDriver() = default;  // jthread requests stop + joins automatically

void PathTraceDriver::requestTrace(const Request& request) {
    const std::lock_guard<std::mutex> lock(requestMutex_);
    pendingRequest_.emplace(request);
    generation_.fetch_add(1, std::memory_order_relaxed);
}

void PathTraceDriver::setSuspended(bool suspended) {
    // Only the false -> true edge bumps: cancelling once is enough, and bumping on every frame that stays suspended would keep the driver churning through generations it is not running anyway.
    const bool wasSuspended = suspended_.exchange(suspended, std::memory_order_relaxed);
    if (suspended && !wasSuspended) {
        generation_.fetch_add(1, std::memory_order_relaxed);
    }
}

std::shared_ptr<const PathTraceResult> PathTraceDriver::latestResult() const {
    const std::lock_guard<std::mutex> lock(resultMutex_);
    return result_;
}

// Driver-thread-only. Hands back the first pool slot nothing else still holds, reallocating it only if the requested dimensions changed, so a steady-state pass allocates nothing at all. use_count() == 1 means the pool's own reference is the last one: the render thread has released both the frame-local snapshot and the display texture's owner ref. Returns nullptr in the rare window where all four are still pinned, which the caller waits out rather than allocating a set it would immediately throw away.
std::shared_ptr<PathTraceResult> PathTraceDriver::acquireFreeBuffer(int width, int height) {
    for (std::shared_ptr<PathTraceResult>& slot : bufferPool_) {
        if (slot != nullptr && slot.use_count() > 1) {
            continue;
        }
        if (slot == nullptr || slot->beauty.width != width || slot->beauty.height != height) {
            slot = std::make_shared<PathTraceResult>(makePathTraceResult(width, height));
        }
        return slot;
    }
    return nullptr;
}

// Runs until destruction (jthread's stop token), picking up the latest requested state whenever its generation changes and otherwise repeatedly re-tracing the same request, accumulating each pass into a running mean that converges over time. A pass superseded mid-flight (renderPathTraced's own generation check, polled once per row) is discarded whole, never partially merged.
engine::debug::PassRecord PathTraceDriver::lastPassRecord() const {
    const std::lock_guard<std::mutex> lock(statsMutex_);
    return lastPass_;
}

// Driver-thread-only. passStats_ is read here rather than inside driverLoop so both the completed and the cancelled call site share one definition of what a PassRecord contains.
void PathTraceDriver::publishPassRecord(std::uint64_t generation, int passIndex, int width,
                                         int height, double traceMs, double accumulateMs,
                                         double overRangeMs, double publishMs, double passMs,
                                         bool cancelled) {
    const engine::debug::PassRecord record{generation,
                                            passIndex,
                                            width,
                                            height,
                                            traceMs,
                                            accumulateMs,
                                            overRangeMs,
                                            publishMs,
                                            passMs,
                                            passStats_.rays(),
                                            passStats_.tilesCompleted(),
                                            passStats_.tilesCancelled(),
                                            cancelled};
    const std::lock_guard<std::mutex> lock(statsMutex_);
    lastPass_ = record;
}

void PathTraceDriver::driverLoop(std::stop_token stopToken) {
    std::shared_ptr<PathTraceResult> currentMean;  // the pool slot holding the last published mean of the active generation -- read as the previous mean by the next pass, never written again
    std::optional<Request> activeRequest;
    std::uint64_t activeGeneration = 0;  // 0 == no request handled yet; requestTrace's first bump makes generation_ 1

    while (!stopToken.stop_requested()) {
        const std::uint64_t requestedGeneration = generation_.load(std::memory_order_relaxed);
        if (requestedGeneration == 0 || suspended_.load(std::memory_order_relaxed)) {
            std::this_thread::sleep_for(kIdlePollInterval);
            continue;
        }

        if (requestedGeneration != activeGeneration) {
            {
                const std::lock_guard<std::mutex> lock(requestMutex_);
                activeRequest = pendingRequest_;  // guaranteed engaged: requestedGeneration != 0 implies at least one requestTrace() call has completed
            }
            activeGeneration = requestedGeneration;
            accumulatedSamples_.store(0, std::memory_order_relaxed);
        }

        if (activeRequest->width <= 0 || activeRequest->height <= 0) {
            std::this_thread::sleep_for(kIdlePollInterval);
            continue;
        }

        if (activeRequest->maxSamples > 0 &&
            accumulatedSamples_.load(std::memory_order_relaxed) >= activeRequest->maxSamples) {
            std::this_thread::sleep_for(kIdlePollInterval);
            continue;
        }

        const std::shared_ptr<PathTraceResult> pass =
            acquireFreeBuffer(activeRequest->width, activeRequest->height);
        if (pass == nullptr) {
            std::this_thread::sleep_for(kIdlePollInterval);
            continue;  // every buffer still referenced by the render thread -- retry rather than allocate
        }

        // Two distinct roles, deliberately not one value (see sampler.h). sampleBase is how many samples this image has
        // already accumulated, so the sampler continues its Sobol sequence where the previous pass stopped instead of
        // re-drawing the same point under a new randomization; passIndex is the same count 1-based, for display only.
        const int sampleBase = accumulatedSamples_.load(std::memory_order_relaxed);
        const int passIndex = sampleBase + 1;
        const auto passStart = std::chrono::steady_clock::now();
        passStats_.reset();
        // Built fresh each pass from this request's env state -- cheap (holds references/scalars, no
        // copies) -- rather than stored, so the HUD's environment-light toggle takes effect on the
        // very next pass with no separate invalidation path.
        const LightSet lights(activeRequest->envLightEnabled ? &environmentMap_ : nullptr,
                               activeRequest->envRotationRadians, activeRequest->envExposure,
                               quadLights_);
        const auto traceStart = std::chrono::steady_clock::now();
        renderPathTraced(activeRequest->camera, accel_, shadingTriangles_, instances_,
                          instanceLightIndex_, lights, activeRequest->width, activeRequest->height,
                          activeRequest->showSky, activeRequest->settings, perInstanceSettings_,
                          // The generation IS the scramble seed: it is fixed for every pass of one accumulation and
                          // changes exactly when the image restarts (camera move, setting change), which is precisely
                          // the lifetime a randomized-QMC scramble must have. Passed raw rather than pre-hashed --
                          // Sampler's own SplitMix64 avalanches it, so a small monotonic counter is sufficient input.
                          static_cast<std::uint32_t>(activeGeneration), sampleBase, activeRequest->maxSamples,
                          generation_, activeGeneration,
                          threadPool_, passStats_, *pass);
        const double traceMs = millisecondsSince(traceStart);

        if (generation_.load(std::memory_order_relaxed) != activeGeneration) {
            // Published before the discard, not skipped: a camera drag cancels passes continuously, and rays traced for a discarded pass were still paid for. Reporting only completed passes would show an idle tracer during exactly the interaction that loads it hardest.
            publishPassRecord(activeGeneration, passIndex, pass->beauty.width, pass->beauty.height,
                               traceMs, 0.0, 0.0, 0.0, millisecondsSince(passStart),
                               /*cancelled=*/true);
            continue;  // superseded mid-pass -- discard, next iteration picks up the new request
        }

        const int n = accumulatedSamples_.fetch_add(1, std::memory_order_relaxed) + 1;
        // n == 1 leaves the pass exactly as rendered: the running mean of one sample is that sample, and it is also the only case with no previous mean of this generation to read.
        const auto accumulateStart = std::chrono::steady_clock::now();
        if (n > 1) {
            accumulateMean(*pass, *currentMean, n, threadPool_);
        }
        const double accumulateMs = millisecondsSince(accumulateStart);

        // After the mean, before the publish: the statistics must describe the image that is about to go on screen, and the render thread must never see a result whose two are stale.
        const auto overRangeStart = std::chrono::steady_clock::now();
        reduceOverRange(*pass, overRangeHistograms_, overRangePeaks_, threadPool_);
        const double overRangeMs = millisecondsSince(overRangeStart);
        currentMean = pass;

        const auto publishStart = std::chrono::steady_clock::now();
        {
            const std::lock_guard<std::mutex> lock(resultMutex_);
            result_ = currentMean;
        }
        const double publishMs = millisecondsSince(publishStart);

        publishPassRecord(activeGeneration, n, pass->beauty.width, pass->beauty.height, traceMs,
                           accumulateMs, overRangeMs, publishMs, millisecondsSince(passStart),
                           /*cancelled=*/false);
    }
}

}  // namespace engine::scene
