#pragma once

namespace pathtracer::debug {

// True if GLEW resolved ARB_timer_query's entry points. Core since GL 3.3 and this context is 4.1 core, so it should
// hold; checked at runtime rather than assumed because a driver may still not export it.
[[nodiscard]] bool gpuTimerQueryAvailable();

// RAII pair of ping-ponged GL_TIME_ELAPSED queries. GL_TIME_ELAPSED, not GL_TIMESTAMP: it measures the interval
// between begin() and end() directly, with no second marker to subtract.
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
