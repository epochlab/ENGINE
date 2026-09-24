#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

#include "pathtracer/debug/render_stats.h"
#include "pathtracer/scene/camera.h"
#include "pathtracer/scene/embree_accel.h"
#include "pathtracer/scene/environment_map.h"
#include "pathtracer/scene/gltf_loader.h"
#include "pathtracer/scene/light.h"
#include "pathtracer/scene/path_tracer.h"
#include "pathtracer/scene/thread_pool.h"

namespace pathtracer::scene {

// Drives renderPathTraced() on a dedicated background thread, off the render/UI thread, running short passes and
// accumulating them into a running-mean PathTraceResult that converges while the UI stays responsive.
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

    // Every referenced scene object must already be at its final address and outlive the driver: EmbreeAccel has no
    // refit, and LightSet stores a pointer and a reference rather than copies. Never construct this inside the same
    // aggregate initialization that builds those objects -- a reference captured that early dangles after the move.
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

    // Render-thread-only. Bumps the generation and replaces the pending request -- it does not queue. Returns the new
    // generation, which every PassRecord of this request's accumulation carries.
    std::uint64_t requestTrace(const Request& request);

    // Render-thread-only, at most once per rendered frame. Null until the first pass completes. One mutex-guarded
    // shared_ptr copy, so it is safe every frame and the strong reference keeps the image alive while it is read.
    [[nodiscard]] std::shared_ptr<const PathTraceResult> latestResult() const;

    // Render-thread-only. Parks the driver while nothing can read its output -- a rasterizer-backed AOV is selected.
    // Takes effect at the next pass boundary. Non-destructive, and that is the whole contract: generation_ is both the
    // restart signal and the sampler scramble seed, so bumping it here would restart the accumulation, not pause it.
    void setSuspended(bool suspended);

    // Render-thread-only. The most recent pass's phase timings and ray counts, a cancelled one included; generation is
    // 0 until the first pass finishes. One POD copy under the stats mutex.
    [[nodiscard]] pathtracer::debug::PassRecord lastPassRecord() const;

private:
    void driverLoop(std::stop_token stopToken);
    // width/height come from the rendered buffer, not the Request that asked: the same numbers, describing the image
    // that actually exists.
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
    // Camera has no default constructor, so this cannot be a plain Request. nullopt until the first requestTrace(),
    // which driverLoop never reads before generation_ is first bumped.
    std::optional<Request> pendingRequest_;

    // Bumped by requestTrace, polled lock-free by the dispatch loop and by every in-flight pass's tile workers.
    std::atomic<std::uint64_t> generation_{0};
    // Set by setSuspended; polled by driverLoop, which idles instead of dispatching while it is true.
    std::atomic<bool> suspended_{false};

    // Ray/tile counters for the pass in flight, reset before each dispatch and read after: driver-thread-owned and
    // reused for the driver's life, so a pass allocates nothing. Declared before thread_ so it outlives driverLoop.
    pathtracer::debug::PassStats passStats_;

    // Republished on every finished pass, cancelled ones included, guarded against the render thread's read. Separate
    // from resultMutex_ so a dashboard read never contends with the image publish.
    mutable std::mutex statsMutex_;
    pathtracer::debug::PassRecord lastPass_;

    // Republished on every completed pass, guarded by resultMutex_ against latestResult()'s render-thread read.
    mutable std::mutex resultMutex_;
    std::shared_ptr<const PathTraceResult> result_;

    // Driver-thread-only rotation of buffer sets, allocated on first use and reused for the process life, so
    // renderPathTraced writes into one of these rather than allocating 10 fresh images per pass.
    std::array<std::shared_ptr<PathTraceResult>, 4> bufferPool_;

    // Per-chunk private accumulators for reduceOverRange, driver-thread-owned and reused across passes -- the same
    // convention as passStats_ and the buffer pool, and why a steady-state pass allocates nothing.
    std::vector<OverRangeHistogram> overRangeHistograms_;
    std::vector<float> overRangePeaks_;

    // Persistent parallel dispatch for renderPathTraced and its accumulate step, reused across every pass. Declared
    // before thread_ so its workers exist before driverLoop can dispatch to them.
    ThreadPool threadPool_;

    // Declared last so it is constructed last (driverLoop starts only once every member exists) and destroyed first
    // (jthread's destructor requests a stop and joins before any member above is torn down).
    std::jthread thread_;
};

}  // namespace pathtracer::scene
