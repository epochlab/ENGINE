#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>

namespace engine::debug {

// Ray counts by type for one path, tile or pass. Plain integers, never atomic: tracePath increments the tile's own instance directly, and it reaches PassStats once per completed tile -- see PassStats for why a per-ray atomic is not viable.
// MEASURED, not assumed: tracePath takes this by `RayCounts& __restrict`, and that qualifier is load-bearing. Returning the counts by value on TraceResult instead cost a reproducible +4.6% (render_beauty, 32 passes, interleaved A/B on user CPU time) by growing a per-sample sret aggregate by 32 bytes; the reference form measured +0.3%, inside run-to-run noise. __restrict is what makes the reference safe to optimize: RayCounts' uint64 fields and Sampler's uint64 PCG state are the same access type, so TBAA alone cannot prove they do not alias, and without it every increment would force a reload of the sampler's state.
struct RayCounts {
    std::uint64_t primary = 0;  // one per sample: the camera ray at bounce 0
    std::uint64_t bounce = 0;   // BSDF-sampled continuation rays, bounce >= 1
    std::uint64_t ao = 0;       // one bounded occlusion ray per sample
    std::uint64_t shadow = 0;   // NEE occlusion rays, one per light sample

    void add(const RayCounts& other) {
        primary += other.primary;
        bounce += other.bounce;
        ao += other.ao;
        shadow += other.shadow;
    }

    [[nodiscard]] std::uint64_t total() const { return primary + bounce + ao + shadow; }
};

// Concurrent ray/tile accumulator for one renderPathTraced() call, owned by the caller and reused across passes (PathTraceDriver keeps one for its lifetime) -- the same ownership convention as its ThreadPool and PathTraceResult buffer pool, and the reason nothing here allocates after init.
// A fetch_add per ray would cost more than the pass it measures: ~15M increments across 8 cores onto 4 cache lines, each contended RMW needing exclusive line ownership. Workers therefore accumulate into a stack-local RayCounts and fold it in once per tile -- ~1000 relaxed RMWs per pass, immeasurable.
// Every operation is memory_order_relaxed, and that is sufficient rather than merely plausible: these counters carry no other data, and each worker's stores precede its --workersRemaining_ under ThreadPool::mutex_, which parallelFor's doneCv_ wait acquires. That release/acquire pair makes every tile's contribution visible before renderPathTraced returns, which is strictly before the driver reads them.
class PassStats {
public:
    // Driver thread only, before dispatch.
    void reset() {
        primary_.store(0, std::memory_order_relaxed);
        bounce_.store(0, std::memory_order_relaxed);
        ao_.store(0, std::memory_order_relaxed);
        shadow_.store(0, std::memory_order_relaxed);
        tilesCompleted_.store(0, std::memory_order_relaxed);
        tilesCancelled_.store(0, std::memory_order_relaxed);
    }

    // Worker threads, once per completed tile.
    void addTile(const RayCounts& counts) {
        primary_.fetch_add(counts.primary, std::memory_order_relaxed);
        bounce_.fetch_add(counts.bounce, std::memory_order_relaxed);
        ao_.fetch_add(counts.ao, std::memory_order_relaxed);
        shadow_.fetch_add(counts.shadow, std::memory_order_relaxed);
        tilesCompleted_.fetch_add(1, std::memory_order_relaxed);
    }

    // Worker threads, on the stale-generation early-out: the tile contributed no rays but must still be accounted for, or a cancelled pass reads as a short one.
    void addCancelledTile() { tilesCancelled_.fetch_add(1, std::memory_order_relaxed); }

    // Valid only once parallelFor has returned -- see the ordering argument above.
    [[nodiscard]] RayCounts rays() const {
        return {primary_.load(std::memory_order_relaxed), bounce_.load(std::memory_order_relaxed),
                ao_.load(std::memory_order_relaxed), shadow_.load(std::memory_order_relaxed)};
    }

    [[nodiscard]] std::uint64_t tilesCompleted() const { return tilesCompleted_.load(std::memory_order_relaxed); }
    [[nodiscard]] std::uint64_t tilesCancelled() const { return tilesCancelled_.load(std::memory_order_relaxed); }

private:
    std::atomic<std::uint64_t> primary_{0};
    std::atomic<std::uint64_t> bounce_{0};
    std::atomic<std::uint64_t> ao_{0};
    std::atomic<std::uint64_t> shadow_{0};
    std::atomic<std::uint64_t> tilesCompleted_{0};
    std::atomic<std::uint64_t> tilesCancelled_{0};
};

// Immutable snapshot of one completed or cancelled path-trace pass. Plain values, no atomics: it is copied wholesale under PathTraceDriver::statsMutex_, exactly as result_ is under resultMutex_.
struct PassRecord {
    std::uint64_t generation = 0;  // which request this pass belonged to; 0 = no pass has run yet
    int passIndex = 0;             // n, the accumulated sample count this pass produced
    int width = 0;
    int height = 0;
    double traceMs = 0.0;       // renderPathTraced
    double accumulateMs = 0.0;  // accumulateMean
    double publishMs = 0.0;     // resultMutex_ hold
    double passMs = 0.0;        // the whole iteration, including buffer acquisition
    RayCounts rays;
    std::uint64_t tilesCompleted = 0;
    std::uint64_t tilesCancelled = 0;
    bool cancelled = false;  // superseded mid-flight and discarded; its rays were still traced and paid for
};

// Per-frame render-thread CPU stage times in milliseconds. Plain floats with no synchronization because exactly one thread is involved: renderFrame writes them and the dashboard, called from renderFrame, reads them.
// MUST be zeroed at the top of every frame: a stage that did not run this frame (the rasterizer, the texture upload on a cache hit) has to read 0, not the last value it happened to have. Skip that and the rasterizer reports 150 ms forever after one rasterization.
struct FrameStageTimes {
    float pollMs = 0.0F;
    float cameraMs = 0.0F;
    float rasterMs = 0.0F;      // renderRasterGBuffer, only on a trigger change into a rasterizer AOV
    float uploadMs = 0.0F;      // the display texture upload, only when a newly published pass invalidates it
    float presentMs = 0.0F;     // presentFrame, INCLUSIVE of uploadMs -- the blit's own cost is the difference
    float histogramMs = 0.0F;
    float overRangeMs = 0.0F;   // every 4th frame, Histogram::kCaptureIntervalFrames
    float probeMs = 0.0F;       // samplePixelProbe, including its synchronous glReadPixels on the post-filter AOVs
    float hudMs = 0.0F;
    float swapMs = 0.0F;        // swapBuffers: the vsync wait, i.e. slack, not engine cost
};

// RAII steady_clock scope timer writing elapsed milliseconds into a caller-owned float.
// SERIAL CONTEXTS ONLY (render thread, driver thread): the destination is a plain float, so two threads must never share one. NEVER inside a per-pixel, per-sample or per-ray loop -- steady_clock::now() is ~20ns here (mach_absolute_time via the commpage, no syscall), free at ~15 uses per frame and ruinous at 10^7.
class ScopedCpuTimer {
public:
    explicit ScopedCpuTimer(float& outMs) : outMs_(outMs), start_(std::chrono::steady_clock::now()) {}

    ~ScopedCpuTimer() {
        outMs_ = std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - start_).count();
    }

    ScopedCpuTimer(const ScopedCpuTimer&) = delete;
    ScopedCpuTimer& operator=(const ScopedCpuTimer&) = delete;
    ScopedCpuTimer(ScopedCpuTimer&&) = delete;
    ScopedCpuTimer& operator=(ScopedCpuTimer&&) = delete;

private:
    float& outMs_;
    std::chrono::steady_clock::time_point start_;
};

}  // namespace engine::debug
