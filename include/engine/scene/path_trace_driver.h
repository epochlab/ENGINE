#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

#include "engine/debug/render_stats.h"
#include "engine/scene/camera.h"
#include "engine/scene/embree_accel.h"
#include "engine/scene/environment_map.h"
#include "engine/scene/gltf_loader.h"
#include "engine/scene/light.h"
#include "engine/scene/path_tracer.h"
#include "engine/scene/thread_pool.h"

namespace engine::scene {

// Drives renderPathTraced() continuously on a dedicated background thread, off the render/UI thread, running short passes and accumulating them into a running-mean PathTraceResult that converges over time while the requested state (camera/scene) stays unchanged. The render thread only ever calls requestTrace() (a mutex-guarded struct copy + an atomic bump) and latestResult() (one mutex-guarded shared_ptr copy) -- it never blocks on a trace. A request superseded before the driver picks it up is simply never run; a request superseded while a pass is in flight is discarded (renderPathTraced's generation/requestedGeneration cancellation) rather than merged, so the accumulator never mixes samples from two different camera poses.
class PathTraceDriver {
public:
    struct Request {
        Camera camera;
        int width = 0;
        int height = 0;
        float envRotationRadians = 0.0F;
        bool showSky = true;
        // Whether the environment is in the light set at all (NEE, MIS, miss radiance) -- distinct
        // from showSky, which only gates the camera ray's own miss. See LightSet (light.h).
        bool envLightEnabled = true;
        float envExposure = 1.0F;
        PathTraceSettings settings;  // samplesPerPixel is "samples per pass", see path_tracer.h
        int maxSamples = 0;  // accumulated-pass cap; 0 = unbounded
    };

    // accel/shadingTriangles/instances/instanceLightIndex/environmentMap/quadLights/perInstanceSettings must already be at their final, permanent address and must outlive the driver (EmbreeAccel has no refit/update API -- the scene is static) -- held by reference, not copied per-request. Do not construct this as part of the same aggregate-initialization expression that also constructs those referenced objects (e.g. AppResources's designated-initializer list): a bare identifier there would still name the pre-move local, and AppResources's return-type conversion to std::optional<AppResources> move-constructs once more regardless -- either way a reference captured that early would dangle. Construct this only after the referenced object is confirmed at its final address (main.cpp emplaces AppResources::pathTraceDriver, itself a std::optional, right after initializeApp() returns).
    PathTraceDriver(const EmbreeAccel& accel, const std::vector<ShadingTriangle>& shadingTriangles,
                     const std::vector<MeshInstance>& instances,
                     const std::vector<int>& instanceLightIndex,
                     const EnvironmentMap& environmentMap, const std::vector<QuadLight>& quadLights,
                     const std::vector<PathTraceSettings>& perInstanceSettings);
    ~PathTraceDriver();

    PathTraceDriver(const PathTraceDriver&) = delete;
    PathTraceDriver& operator=(const PathTraceDriver&) = delete;
    PathTraceDriver(PathTraceDriver&&) = delete;
    PathTraceDriver& operator=(PathTraceDriver&&) = delete;

    // Render-thread-only. Bumps the generation counter and replaces the pending request -- does not queue.
    void requestTrace(const Request& request);

    // Render-thread-only, call at most once per rendered frame. Null until the first pass of the app's life completes. Cheap (one mutex-guarded shared_ptr copy) -- safe to call every frame, and the strong ref keeps that pass's images alive for as long as the caller holds it, however many newer passes the driver publishes meanwhile.
    [[nodiscard]] std::shared_ptr<const PathTraceResult> latestResult() const;

    // Render-thread-only. Parks the driver while nothing can read its output -- a rasterizer-backed AOV is selected, so no pass this thread completes will ever be displayed. Entering suspension also bumps generation_, which cancels the pass already in flight (renderPathTraced polls it per tile) instead of letting it run to completion for no reader.
    // A bare generation_ bump would NOT do this on its own: driverLoop reacts to a changed generation by re-reading pendingRequest_ and restarting accumulation, so bumping alone cancels the pass and then immediately re-runs the same one. The flag is what makes the loop idle rather than restart.
    void setSuspended(bool suspended);

    // How many passes have been accumulated into the currently-published result's generation -- HUD convergence readout.
    [[nodiscard]] int accumulatedSamples() const { return accumulatedSamples_.load(std::memory_order_relaxed); }

    // Render-thread-only. The most recent pass's phase timings and ray counts, including one that was cancelled mid-flight -- generation == 0 until the first pass of the app's life finishes. One POD copy under statsMutex_, cheap enough to call per frame, though the dashboard only asks at its own refresh rate.
    [[nodiscard]] engine::debug::PassRecord lastPassRecord() const;

private:
    void driverLoop(std::stop_token stopToken);
    // width/height come from the rendered buffer, not the Request that asked for it: the same numbers, and it describes the image that actually exists.
    void publishPassRecord(std::uint64_t generation, int passIndex, int width, int height,
                            double traceMs, double accumulateMs, double overRangeMs,
                            double publishMs, double passMs, bool cancelled);
    std::shared_ptr<PathTraceResult> acquireFreeBuffer(int width, int height);

    const EmbreeAccel& accel_;
    const std::vector<ShadingTriangle>& shadingTriangles_;
    const std::vector<MeshInstance>& instances_;
    const std::vector<int>& instanceLightIndex_;
    const EnvironmentMap& environmentMap_;
    const std::vector<QuadLight>& quadLights_;
    const std::vector<PathTraceSettings>& perInstanceSettings_;

    std::mutex requestMutex_;
    // Camera has no default constructor, so this can't be a plain Request -- std::nullopt until the first requestTrace() call, which is fine: driverLoop never reads it before generation_ (whose first bump happens in the same requestTrace call, under the same lock) has gone non-zero.
    std::optional<Request> pendingRequest_;

    // Bumped by requestTrace, polled lock-free by both the driver's dispatch loop and (via renderPathTraced's cancellation parameter) every in-flight pass's tile workers.
    std::atomic<std::uint64_t> generation_{0};
    // Set by setSuspended; polled by driverLoop, which idles instead of dispatching while it is true.
    std::atomic<bool> suspended_{false};
    std::atomic<int> accumulatedSamples_{0};

    // Ray/tile counters for the pass in flight, reset before each dispatch and read after it returns -- driver-thread-owned, reused for the driver's life, so a pass allocates nothing. Declared before thread_, like threadPool_, so it exists before driverLoop starts.
    engine::debug::PassStats passStats_;

    // Republished on every finished pass, cancelled ones included, guarded against the render thread's lastPassRecord() read. Separate from resultMutex_ so a dashboard read never contends with the image publish.
    mutable std::mutex statsMutex_;
    engine::debug::PassRecord lastPass_;

    // Republished on every completed pass, guarded by resultMutex_ against latestResult()'s render-thread read.
    mutable std::mutex resultMutex_;
    std::shared_ptr<const PathTraceResult> result_;

    // Driver-thread-only rotation of buffer sets, allocated on first use and reused for the process's life -- renderPathTraced writes into one of these instead of allocating 8 fresh images per pass. Four, because up to three can be pinned at once: the mean the driver just published, the frame-local snapshot the render thread holds for the duration of a frame, and the older result app.pathTraceDisplayedOwner still holds because the display texture was built from it. The fourth is the one being written.
    std::array<std::shared_ptr<PathTraceResult>, 4> bufferPool_;

    // Per-chunk private accumulators for reduceOverRange, driver-thread-owned and reused across passes -- the same convention as passStats_ and the buffer pool, and the reason a steady-state pass allocates nothing. 128 KiB per worker, which stays a per-core working set because no chunk ever touches another's.
    std::vector<OverRangeHistogram> overRangeHistograms_;
    std::vector<float> overRangePeaks_;

    // Persistent parallel dispatch for renderPathTraced and its accumulate step, reused across every pass -- see ThreadPool's own doc comment. Declared before thread_ so it's fully constructed (and its workers parked and ready) before driverLoop starts, and outlives every renderPathTraced call driverLoop makes (destroyed only after thread_ has stopped and joined).
    ThreadPool threadPool_;

    // Declared last: constructed last (starts driverLoop only once every member above exists), destroyed first (std::jthread's destructor requests a stop and joins before any member above -- or, per the constructor's own precondition, the objects accel_/shadingTriangles_/instances_/environmentMap_/perInstanceSettings_ reference -- could be invalidated by outer teardown).
    std::jthread thread_;
};

}  // namespace engine::scene
