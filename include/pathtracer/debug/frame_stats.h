#pragma once

#include <array>
#include <chrono>

namespace pathtracer::debug {

// Fixed-size ring of recent frame times in ms. 120 entries is a sparkline width (~2s at 60fps); accessors scan the whole ring.
class FrameStats {
public:
    static constexpr int kHistoryLength = 120;

    // Call exactly once per frame, right after window.pollEvents().
    void tick();

    // The same tick with a caller-supplied clock, so the ring and its percentiles are testable exactly rather than by sleeping.
    void tick(std::chrono::steady_clock::time_point now);

    [[nodiscard]] float fps() const;
    [[nodiscard]] float avgMs() const;
    [[nodiscard]] float minMs() const;
    [[nodiscard]] float maxMs() const;

    // Percentile of recorded frame times, fraction in [0,1]. 120 entries cannot express a p99, so ask p95 and read "worst few frames".
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
