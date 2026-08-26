#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

#include "engine/scene/camera.h"
#include "engine/scene/embree_accel.h"
#include "engine/scene/environment_map.h"
#include "engine/scene/gltf_loader.h"
#include "engine/scene/path_tracer.h"
#include "engine/scene/row_thread_pool.h"

namespace engine::scene {

// One consistent published pair from PathTraceDriver: gbuffer is set once (on pass 1 of the current generation) and never rebuilt again; dynamic is republished on every pass as it keeps re-averaging. Both null until the first pass of the app's life completes.
struct PathTraceSnapshot {
    std::shared_ptr<const PathTraceGBuffer> gbuffer;
    std::shared_ptr<const PathTraceDynamic> dynamic;
};

// Drives renderPathTraced() continuously on a dedicated background thread, off the render/UI thread, running short passes and accumulating them into a running-mean PathTraceResult that converges over time while the requested state (camera/scene) stays unchanged. The render thread only ever calls requestTrace() (a mutex-guarded struct copy + an atomic bump) and latestResult() (two mutex-guarded shared_ptr copies) -- it never blocks on a trace. A request superseded before the driver picks it up is simply never run; a request superseded while a pass is in flight is discarded (renderPathTraced's generation/requestedGeneration cancellation) rather than merged, so the accumulator never mixes samples from two different camera poses.
class PathTraceDriver {
public:
    struct Request {
        Camera camera;
        int width = 0;
        int height = 0;
        float envRotationRadians = 0.0F;
        bool showSky = true;
        float envExposure = 1.0F;
        PathTraceSettings settings;  // samplesPerPixel is "samples per pass", see path_tracer.h
        int maxSamples = 0;  // accumulated-pass cap; 0 = unbounded
    };

    // accel/shadingTriangles/instances/environmentMap must already be at their final, permanent address and must outlive the driver (EmbreeAccel has no refit/update API -- the scene is static) -- held by reference, not copied per-request. Do not construct this as part of the same aggregate-initialization expression that also constructs those referenced objects (e.g. AppResources's designated-initializer list): a bare identifier there would still name the pre-move local, and AppResources's return-type conversion to std::optional<AppResources> move-constructs once more regardless -- either way a reference captured that early would dangle. Construct this only after the referenced object is confirmed at its final address (main.cpp emplaces AppResources::pathTraceDriver, itself a std::optional, right after initializeApp() returns).
    PathTraceDriver(const EmbreeAccel& accel, const std::vector<ShadingTriangle>& shadingTriangles,
                     const std::vector<MeshInstance>& instances,
                     const EnvironmentMap& environmentMap);
    ~PathTraceDriver();

    PathTraceDriver(const PathTraceDriver&) = delete;
    PathTraceDriver& operator=(const PathTraceDriver&) = delete;
    PathTraceDriver(PathTraceDriver&&) = delete;
    PathTraceDriver& operator=(PathTraceDriver&&) = delete;

    // Render-thread-only. Bumps the generation counter and replaces the pending request -- does not queue.
    void requestTrace(const Request& request);

    // Render-thread-only, call at most once per rendered frame. Both fields null until the first pass of the app's life completes. Cheap (two mutex-guarded shared_ptr copies under one lock, so the pair is always consistent) -- safe to call every frame.
    [[nodiscard]] PathTraceSnapshot latestResult() const;

    // How many passes have been accumulated into the currently-published result's generation -- HUD convergence readout.
    [[nodiscard]] int accumulatedSamples() const { return accumulatedSamples_.load(std::memory_order_relaxed); }

    [[nodiscard]] double lastPassSeconds() const { return lastPassSeconds_.load(std::memory_order_relaxed); }

private:
    void driverLoop(std::stop_token stopToken);

    const EmbreeAccel& accel_;
    const std::vector<ShadingTriangle>& shadingTriangles_;
    const std::vector<MeshInstance>& instances_;
    const EnvironmentMap& environmentMap_;

    std::mutex requestMutex_;
    // Camera has no default constructor, so this can't be a plain Request -- std::nullopt until the first requestTrace() call, which is fine: driverLoop never reads it before generation_ (whose first bump happens in the same requestTrace call, under the same lock) has gone non-zero.
    std::optional<Request> pendingRequest_;

    // Bumped by requestTrace, polled lock-free by both the driver's dispatch loop and (via renderPathTraced's cancellation parameter) every in-flight pass's row workers.
    std::atomic<std::uint64_t> generation_{0};
    std::atomic<int> accumulatedSamples_{0};
    std::atomic<double> lastPassSeconds_{0.0};

    // Both published under resultMutex_ together (gbuffer_ only on pass 1, dynamic_ every pass) so latestResult() always hands back a consistent pair -- see PathTraceSnapshot's doc comment for why they're split rather than one shared_ptr<const PathTraceResult>.
    mutable std::mutex resultMutex_;
    std::shared_ptr<const PathTraceGBuffer> gbuffer_;
    std::shared_ptr<const PathTraceDynamic> dynamic_;

    // Persistent row-parallel dispatch for renderPathTraced, reused across every pass -- see RowThreadPool's own doc comment. Declared before thread_ so it's fully constructed (and its workers parked and ready) before driverLoop starts, and outlives every renderPathTraced call driverLoop makes (destroyed only after thread_ has stopped and joined).
    RowThreadPool threadPool_;

    // Declared last: constructed last (starts driverLoop only once every member above exists), destroyed first (std::jthread's destructor requests a stop and joins before any member above -- or, per the constructor's own precondition, the objects accel_/shadingTriangles_/instances_/environmentMap_ reference -- could be invalidated by outer teardown).
    std::jthread thread_;
};

}  // namespace engine::scene
