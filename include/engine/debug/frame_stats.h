#pragma once

#include <array>
#include <chrono>

namespace engine::debug {

// Fixed-size ring buffer of recent frame times (ms). 120 entries is a
// sparkline-width choice (~2s at 60fps), not a precision one — fps()/
// avgMs()/minMs()/maxMs() scan the whole buffer each call, cheap at this
// size, so no running-aggregate bookkeeping is needed.
class FrameStats {
public:
    static constexpr int kHistoryLength = 120;

    // Call exactly once per frame, right after window.pollEvents().
    void tick();

    [[nodiscard]] float fps() const;
    [[nodiscard]] float avgMs() const;
    [[nodiscard]] float minMs() const;
    [[nodiscard]] float maxMs() const;

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
