#include "engine/scene/path_trace_driver.h"

#include <chrono>
#include <cstddef>
#include <thread>
#include <utility>

namespace engine::scene {

namespace {

constexpr std::chrono::milliseconds kIdlePollInterval{5};

// Incremental running mean, one float at a time: mean += (sample - mean) / n. Avoids a separate running-sum buffer (half the accumulator's memory footprint vs. sum-then-divide) and means runningMean is always already display-ready -- no final division step needed at publish time.
void accumulateInPlace(engine::gfx::HdrImage& runningMean, const engine::gfx::HdrImage& newSample,
                        int n) {
    const float invN = 1.0F / static_cast<float>(n);
    for (std::size_t i = 0; i < runningMean.rgba.size(); ++i) {
        runningMean.rgba[i] += (newSample.rgba[i] - runningMean.rgba[i]) * invN;
    }
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

PathTraceSnapshot PathTraceDriver::latestResult() const {
    const std::lock_guard<std::mutex> lock(resultMutex_);
    return PathTraceSnapshot{gbuffer_, dynamic_};
}

// Runs until destruction (jthread's stop token), picking up the latest requested state whenever its generation changes and otherwise repeatedly re-tracing the same request, accumulating each pass into a running mean that converges over time. A pass superseded mid-flight (renderPathTraced's own generation check, polled once per row) is discarded whole, never partially merged.
void PathTraceDriver::driverLoop(std::stop_token stopToken) {
    PathTraceDynamic accumulator{};
    std::optional<Request> activeRequest;
    std::uint64_t activeGeneration = 0;  // 0 == no request handled yet; requestTrace's first bump makes generation_ 1

    while (!stopToken.stop_requested()) {
        const std::uint64_t requestedGeneration = generation_.load(std::memory_order_relaxed);
        if (requestedGeneration == 0) {
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

        const int passIndex = accumulatedSamples_.load(std::memory_order_relaxed) + 1;
        const auto passStart = std::chrono::steady_clock::now();
        PathTraceResult pass = renderPathTraced(
            activeRequest->camera, accel_, shadingTriangles_, instances_, environmentMap_,
            activeRequest->width, activeRequest->height, activeRequest->envRotationRadians,
            activeRequest->showSky, activeRequest->envExposure, activeRequest->settings,
            static_cast<std::uint32_t>(passIndex), generation_, activeGeneration, threadPool_);

        if (generation_.load(std::memory_order_relaxed) != activeGeneration) {
            continue;  // superseded mid-pass -- discard, next iteration picks up the new request
        }

        const int n = accumulatedSamples_.fetch_add(1, std::memory_order_relaxed) + 1;
        std::shared_ptr<const PathTraceGBuffer> newGBuffer;  // only set on n==1, published below
        if (n == 1) {
            // First pass of this generation: the G-buffer fields are primary-hit-only quantities that don't benefit from averaging across passes (see PathTraceGBuffer's doc comment) -- take them as-is, split out, and publish once; they're never rebuilt again this generation.
            newGBuffer = std::make_shared<const PathTraceGBuffer>(PathTraceGBuffer{
                std::move(pass.iorAov), std::move(pass.depth), std::move(pass.worldPos),
                std::move(pass.uv), std::move(pass.normal), std::move(pass.geomNormal),
                std::move(pass.albedo), std::move(pass.metallic), std::move(pass.roughness),
                std::move(pass.tangent), std::move(pass.objectId), std::move(pass.alpha),
                std::move(pass.fresnel), std::move(pass.ao)});
            accumulator = PathTraceDynamic{
                std::move(pass.beauty),         std::move(pass.bounceHeatmap),
                std::move(pass.shadow),
                std::move(pass.directDiffuse),  std::move(pass.indirectDiffuse),
                std::move(pass.directSpecular), std::move(pass.indirectSpecular),
                std::move(pass.refraction)};
        } else {
            accumulateInPlace(accumulator.beauty, pass.beauty, n);
            accumulateInPlace(accumulator.bounceHeatmap, pass.bounceHeatmap, n);
            accumulateInPlace(accumulator.shadow, pass.shadow, n);
            accumulateInPlace(accumulator.directDiffuse, pass.directDiffuse, n);
            accumulateInPlace(accumulator.indirectDiffuse, pass.indirectDiffuse, n);
            accumulateInPlace(accumulator.directSpecular, pass.directSpecular, n);
            accumulateInPlace(accumulator.indirectSpecular, pass.indirectSpecular, n);
            accumulateInPlace(accumulator.refraction, pass.refraction, n);
        }

        lastPassSeconds_.store(
            std::chrono::duration<double>(std::chrono::steady_clock::now() - passStart).count(),
            std::memory_order_relaxed);

        {
            const std::lock_guard<std::mutex> lock(resultMutex_);
            if (newGBuffer) {
                gbuffer_ = std::move(newGBuffer);
            }
            dynamic_ = std::make_shared<const PathTraceDynamic>(accumulator);
        }
    }
}

}  // namespace engine::scene
