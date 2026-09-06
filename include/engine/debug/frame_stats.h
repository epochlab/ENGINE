#pragma once

#include <array>
#include <chrono>

namespace engine::debug {

// Fixed-size ring buffer of recent frame times (ms). 120 entries is a sparkline-width choice (~2s at 60fps), not a precision one: fps()/avgMs()/minMs()/maxMs() scan the whole buffer each call, cheap at this size, so no running-aggregate bookkeeping is needed.
class FrameStats {
public:
    static constexpr int kHistoryLength = 120;

    // Call exactly once per frame, right after window.pollEvents().
    void tick();

    [[nodiscard]] float fps() const;
    [[nodiscard]] float avgMs() const;
    [[nodiscard]] float minMs() const;
    [[nodiscard]] float maxMs() const;

    // Percentile of the recorded frame times, fraction in [0,1] (0.5 = median). Copies the filled span onto the stack and nth_elements it: no allocation, at most kHistoryLength floats. A mean hides hitches by construction -- one 50ms frame in 120 moves a 60fps average by 0.4fps and is exactly the frame a user feels -- so the dashboard leads with p50/p95 and keeps the mean as a courtesy number.
    // kHistoryLength samples cannot express a p99: the 99th percentile of 120 is the second-largest value, not a tail estimate. Callers should ask for p95 and read maxMs() for the tail.
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

}  // namespace engine::debug
