#include "engine/scene/path_trace_driver.h"

#include <array>
#include <chrono>
#include <cstddef>
#include <thread>
#include <utility>

namespace engine::scene {

namespace {

constexpr std::chrono::milliseconds kIdlePollInterval{5};

// Incremental running mean, out of place: newMean = previousMean + (sample - previousMean)/n, computed in the freshly rendered pass buffer so the published set is only ever READ. That is what lets the publish below be a pointer assignment instead of the whole-image copy it used to hold resultMutex_ for, and what keeps a render thread holding the previous set from seeing a half-updated image. Running mean rather than a running sum: no separate sum buffer, and the result is always already display-ready with no final division. Row-parallel over the same pool the pass just finished with, which is otherwise idle at this moment.
void accumulateMean(PathTraceResult& sample, const PathTraceResult& previousMean, int n,
                     ThreadPool& threadPool) {
    const std::array<engine::gfx::HdrImage*, 8> destinations{
        &sample.beauty,          &sample.bounceHeatmap,    &sample.shadow,
        &sample.directDiffuse,   &sample.indirectDiffuse,  &sample.directSpecular,
        &sample.indirectSpecular, &sample.refraction};
    const std::array<const engine::gfx::HdrImage*, 8> sources{
        &previousMean.beauty,          &previousMean.bounceHeatmap,
        &previousMean.shadow,          &previousMean.directDiffuse,
        &previousMean.indirectDiffuse, &previousMean.directSpecular,
        &previousMean.indirectSpecular, &previousMean.refraction};
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

}  // namespace

PathTraceDriver::PathTraceDriver(const EmbreeAccel& accel,
                                  const std::vector<ShadingTriangle>& shadingTriangles,
                                  const std::vector<MeshInstance>& instances,
                                  const EnvironmentMap& environmentMap)
    : accel_(accel),
      shadingTriangles_(shadingTriangles),
      instances_(instances),
      environmentMap_(environmentMap),
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

        const int passIndex = accumulatedSamples_.load(std::memory_order_relaxed) + 1;
        const auto passStart = std::chrono::steady_clock::now();
        renderPathTraced(activeRequest->camera, accel_, shadingTriangles_, instances_,
                          environmentMap_, activeRequest->width, activeRequest->height,
                          activeRequest->envRotationRadians, activeRequest->showSky,
                          activeRequest->envExposure, activeRequest->settings,
                          static_cast<std::uint32_t>(passIndex), generation_, activeGeneration,
                          threadPool_, *pass);

        if (generation_.load(std::memory_order_relaxed) != activeGeneration) {
            continue;  // superseded mid-pass -- discard, next iteration picks up the new request
        }

        const int n = accumulatedSamples_.fetch_add(1, std::memory_order_relaxed) + 1;
        // n == 1 leaves the pass exactly as rendered: the running mean of one sample is that sample, and it is also the only case with no previous mean of this generation to read.
        if (n > 1) {
            accumulateMean(*pass, *currentMean, n, threadPool_);
        }
        currentMean = pass;

        lastPassSeconds_.store(
            std::chrono::duration<double>(std::chrono::steady_clock::now() - passStart).count(),
            std::memory_order_relaxed);

        {
            const std::lock_guard<std::mutex> lock(resultMutex_);
            result_ = currentMean;
        }
    }
}

}  // namespace engine::scene
