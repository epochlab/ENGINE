#pragma once

namespace pathtracer::debug {

// True if GLEW resolved ARB_timer_query. Core since GL 3.3 and this context is 4.1, but checked because a driver may not export it.
[[nodiscard]] bool gpuTimerQueryAvailable();

// RAII pair of ping-ponged GL_TIME_ELAPSED queries, not GL_TIMESTAMP: it measures begin()-to-end() with no second marker to subtract.
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

}  // namespace pathtracer::debug
