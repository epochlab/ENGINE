#include "engine/debug/frame_stats.h"

#include <algorithm>

namespace engine::debug {

void FrameStats::tick() {
    const auto now = std::chrono::steady_clock::now();
    if (hasLastTick_) {
        const float ms =
            std::chrono::duration<float, std::milli>(now - lastTick_).count();
        history_[static_cast<std::size_t>(cursor_)] = ms;
        cursor_ = (cursor_ + 1) % kHistoryLength;
        filledCount_ = std::min(filledCount_ + 1, kHistoryLength);
    }
    lastTick_ = now;
    hasLastTick_ = true;
}

float FrameStats::avgMs() const {
    if (filledCount_ == 0) {
        return 0.0F;
    }
    float sum = 0.0F;
    for (int i = 0; i < filledCount_; ++i) {
        sum += history_[static_cast<std::size_t>(i)];
    }
    return sum / static_cast<float>(filledCount_);
}

float FrameStats::fps() const {
    const float avg = avgMs();
    return avg > 0.0F ? 1000.0F / avg : 0.0F;
}

float FrameStats::minMs() const {
    if (filledCount_ == 0) {
        return 0.0F;
    }
    return *std::min_element(history_.begin(), history_.begin() + filledCount_);
}

float FrameStats::maxMs() const {
    if (filledCount_ == 0) {
        return 0.0F;
    }
    return *std::max_element(history_.begin(), history_.begin() + filledCount_);
}

float FrameStats::percentileMs(float fraction) const {
    if (filledCount_ == 0) {
        return 0.0F;
    }
    std::array<float, kHistoryLength> sorted{};
    std::copy(history_.begin(), history_.begin() + filledCount_, sorted.begin());
    // Clamped rather than trusted: fraction is a caller-supplied ratio, and an index of filledCount_ would read one past the copied span.
    const int index = std::clamp(static_cast<int>(fraction * static_cast<float>(filledCount_)), 0,
                                  filledCount_ - 1);
    const auto nth = sorted.begin() + index;
    std::nth_element(sorted.begin(), nth, sorted.begin() + filledCount_);
    return *nth;
}

}  // namespace engine::debug
