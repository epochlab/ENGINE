#pragma once

#include <array>
#include <chrono>

namespace pathtracer::debug {

// Fixed-size ring buffer of recent frame times in ms. 120 entries is a sparkline-width choice (~2s at 60fps), not a
// precision one: the accessors scan the whole ring, which at this size is cheaper than maintaining incremental state.
class FrameStats {
public:
    static constexpr int kHistoryLength = 120;

    // Call exactly once per frame, right after window.pollEvents().
    void tick();

    // The same tick with the clock supplied by the caller. Exists so the ring buffer, its warm-up bound and the
    // percentiles are testable exactly rather than by sleeping: reading steady_clock internally made every assertion
    // about this class a timing race. tick() above forwards to it, so there is one implementation.
    void tick(std::chrono::steady_clock::time_point now);

    [[nodiscard]] float fps() const;
    [[nodiscard]] float avgMs() const;
    [[nodiscard]] float minMs() const;
    [[nodiscard]] float maxMs() const;

    // Percentile of the recorded frame times, fraction in [0,1] (0.5 = median). Copies the filled span onto the
    // stack and nth_elements it, so no allocation. kHistoryLength cannot express a p99 -- the 99th percentile of
    // 120 is the second-largest value, not a tail estimate -- so ask for p95 and read it as "worst few frames".
    [[nodiscard]] float percentileMs(float fraction) const;

    // Raw buffer + write cursor, for ImGui::PlotLines's values_offset.
    [[nodiscard]] const std::array<float, kHistoryLength>& history() const { return history_; }
    [[nodiscard]] int cursor() const { return cursor_; }

private:
    std::array<float, kHistoryLength> history_{};
    int cursor_ = 0;
    int filledCount_ = 0;  // scan bound during warm-up, before the buffer fills once
    std::chrono::steady_clock::time_point lastTick_{};
    bool hasLastTick_ = false;
};

}  // namespace pathtracer::debug
