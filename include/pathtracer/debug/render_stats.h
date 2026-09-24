#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>

namespace pathtracer::debug {

// Ray counts by type for one path, tile or pass. Plain integers, never atomic: tracePath increments the tile's own
// instance and it reaches PassStats once per completed tile. tracePath takes it by `RayCounts& __restrict`, and that
// qualifier is load-bearing -- returning the counts by value cost a reproducible +4.6% (render_beauty, 32 passes).
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

// Concurrent ray/tile accumulator for one renderPathTraced() call, caller-owned and reused across passes. A fetch_add
// per ray would cost more than the pass it measures (~15M contended RMWs onto 4 lines), so workers accumulate into a
// stack-local RayCounts and merge once. Relaxed ordering suffices: the merge precedes --workersRemaining_.
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

    // Worker threads, on the stale-generation early-out: the tile contributed no rays but must still be accounted
    // for, or a cancelled pass reads as a short one.
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

// Immutable snapshot of one completed or cancelled path-trace pass. Plain values, no atomics: it is copied wholesale
// under PathTraceDriver::statsMutex_, exactly as result_ is under resultMutex_.
struct PassRecord {
    std::uint64_t generation = 0;  // which request this pass belonged to; 0 = no pass has run yet
    int passIndex = 0;             // n, the accumulated sample count this pass produced
    int width = 0;
    int height = 0;
    double traceMs = 0.0;       // renderPathTraced
    double accumulateMs = 0.0;  // accumulateMean
    double overRangeMs = 0.0;   // reduceOverRange, the HUD's over-range statistics
    double publishMs = 0.0;     // resultMutex_ hold
    double passMs = 0.0;        // the whole iteration, including buffer acquisition
    RayCounts rays;
    std::uint64_t tilesCompleted = 0;
    std::uint64_t tilesCancelled = 0;
    bool cancelled = false;  // superseded mid-flight and discarded; its rays were still traced and paid for
};

// Per-frame render-thread CPU stage times in milliseconds. Plain floats, unsynchronized because one thread is
// involved: renderFrame writes them and the dashboard, called from renderFrame, reads them. Must be zeroed each
// frame: a stage that did not run (rasterizer, texture upload on a cache hit) has to read 0, not its last value.
struct FrameStageTimes {
    float fenceMs = 0.0F;       // wait for the previous frame's GPU work: non-zero only when the GPU, not the display, bounds the frame
    float paceMs = 0.0F;        // DisplayLink::waitForNextVblank: slack, not engine cost
    float pollMs = 0.0F;
    float cameraMs = 0.0F;
    float rasterMs = 0.0F;      // renderRasterGBuffer, only on a trigger change into a rasterizer AOV
    float uploadMs = 0.0F;      // the display texture upload, only when a newly published pass invalidates it
    bool uploaded = false;      // the upload ran this frame: an explicit event flag, since a timed stage can legitimately read 0
    float presentMs = 0.0F;     // presentFrame, INCLUSIVE of uploadMs -- the blit's own cost is the difference
    float histogramMs = 0.0F;
    float overRangeMs = 0.0F;   // the O(kOverRangeBinCount) read of the driver's reduction, every frame
    float probeMs = 0.0F;       // samplePixelProbe, including its synchronous glReadPixels on the post-filter AOVs
    float hudMs = 0.0F;         // HUD draw + camera write-back + render, INCLUSIVE of hudRenderMs -- the build half is the difference
    float hudRenderMs = 0.0F;   // HudOverlay::render (ImGui::Render + RenderDrawData), unconditional so it is paid with the HUD hidden
    // swapBuffers at swap interval 0: the flushBuffer hand-off to the compositor, no vblank wait. Its tail is
    // WindowServer reply latency (README.md "Frame pacing"), not engine time.
    float swapMs = 0.0F;
};

// RAII steady_clock scope timer writing elapsed milliseconds into a caller-owned float.
// Serial contexts only (render thread, driver thread): the destination is a plain float, so two threads must never
// share one. Never inside a per-pixel, per-sample or per-ray loop -- steady_clock::now() is ~20ns.
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

}  // namespace pathtracer::debug
