#pragma once

namespace engine::debug {

// True if GLEW resolved ARB_timer_query's entry points. Core since GL
// 3.3 (this project's context is 4.1 core, so this should hold) — checked
// at runtime rather than assumed, mirroring khrDebugAvailable's precedent
// (gl_debug.h). macOS's GL-over-Metal translation is known to report
// GL_TIME_ELAPSED at coarser granularity than desktop-native GL; that's an
// accuracy caveat, not an availability one, and doesn't affect this check.
[[nodiscard]] bool gpuTimerQueryAvailable();

// RAII pair of ping-ponged GL_TIME_ELAPSED queries. GL_TIME_ELAPSED (not
// GL_TIMESTAMP) directly measures elapsed time between begin()/end(),
// no second marker + subtraction needed. Double-buffered so
// millisecondsElapsed() never blocks the CPU on the GPU: it only reads a
// slot once that slot has completed a prior begin/end cycle and polls
// GL_QUERY_RESULT_AVAILABLE before touching it, otherwise returns the
// last known value (may be a frame or two stale under GPU queue latency —
// an accepted limitation for a debug HUD, not worth fixing here).
class GpuTimer {
public:
    GpuTimer();
    ~GpuTimer();

    GpuTimer(const GpuTimer&) = delete;
    GpuTimer& operator=(const GpuTimer&) = delete;
    GpuTimer(GpuTimer&& other) noexcept;
    GpuTimer& operator=(GpuTimer&& other) noexcept;

    void begin();
    void end();

    // Call once per frame, after end().
    [[nodiscard]] float millisecondsElapsed();

private:
    unsigned int queries_[2] = {0, 0};
    int writeIndex_ = 0;
    bool slotUsed_[2] = {false, false};
    float lastMs_ = 0.0F;
};

}  // namespace engine::debug
