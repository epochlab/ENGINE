#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>

#include "pathtracer/debug/render_stats.h"

namespace pathtracer::debug {

class FrameStats;

// Everything one dashboard redraw shows, bundled by renderFrame -- the same convention as HudFrameData
// (hud_overlay.h), keeping update()'s signature from growing as rows are added.
struct DashboardFrame {
    const FrameStats& frameStats;
    const FrameStageTimes& stages;
    const PassRecord& pass;
    // This frame's wall interval, accumulated over the same window as the stage times so the reconciliation row
    // compares like with like. FrameStats' own mean is over a 120-entry ring, a different and longer window.
    float frameMs;
    float presentGpuMs;
    int accumulatedSamples;
    int maxSamples;  // 0 = unbounded
    bool driverSuspended;
    std::size_t ramBytes;
    std::size_t gpuBytes;
    std::size_t bvhBytes;
    std::size_t systemAvailableBytes;
    std::uint64_t systemTotalBytes;
    const char* aovName;
    const char* sceneName;
    int instanceCount;
    int lightCount;
    int triangleCount;
    int windowWidth;
    int windowHeight;
    int renderWidth;
    int renderHeight;
    float renderScale;
    bool interactiveScale;
    double refreshHz;  // the window's display, measured by DisplayLink
    bool vsync;        // profile.json frame cap; the budget stays the refresh period either way
};

// Live terminal dashboard: redraws a fixed-height block in place at kRefreshHz with ANSI cursor-up plus erase-line.
// Render-thread only. Off when stdout is not a TTY, where the escapes would corrupt a log; it then emits one plain
// summary every kNonTtySeconds. The cursor is never hidden: a crash would leave the terminal with no cursor.
class PerfDashboard {
public:
    // 3 Hz: the draw is a blocking write(2) landing after swapBuffers, so it eats the next frame's headroom. Its cost
    // is measured here and shown on the next redraw rather than hidden.
    static constexpr int kRefreshHz = 3;
    static constexpr double kNonTtySeconds = 5.0;
    static constexpr std::size_t kBufferBytes = 8192;
    // Lines the body occupies, used only to reserve the block on the first draw. Every later redraw moves up by what
    // the previous draw actually emitted, so a wrong value here costs one misaligned frame.
    static constexpr int kBodyLines = 30;

    PerfDashboard();

    // Once per frame at the end of renderFrame, after swapBuffers. Rate-limited internally: between redraws this is
    // one steady_clock::now(), a compare and the accumulator adds.
    void update(const DashboardFrame& frame);

private:
    void accumulate(const DashboardFrame& frame);
    void draw(const DashboardFrame& frame);
    // Split only to keep each part inside the one-screen function limit; the split points are the layout's own columns.
    // Each body row carries a render-thread stage on the left and a path-trace stat on the right, so the split is by
    // row group rather than by column. Header, body and footer are each one screenful.
    void drawFrameHeader(const DashboardFrame& frame);
    void drawStageRows(const DashboardFrame& frame);
    void drawRayRows(const DashboardFrame& frame);
    void drawFooter(const DashboardFrame& frame);

    // Per-frame mean over the current window. A helper rather than a local so every row divides by the same n, whichever function emits it.
    [[nodiscard]] float windowMean(float sum) const;
    [[nodiscard]] float cpuTotalMs() const;
    [[nodiscard]] double burstDutyFrames(int stage) const;
    void drawNonTty(const DashboardFrame& frame);
    void append(const char* format, ...) __attribute__((format(printf, 2, 3)));
    void flush();

    // Windowed means of every per-frame stage, so the display is stable rather than flickering at the last frame's
    // cost. Reset after each redraw.
    FrameStageTimes sums_{};
    float cpuTotalSum_ = 0.0F;
    float frameMsSum_ = 0.0F;
    int windowFrames_ = 0;

    // This dashboard's own draw cost, measured around the write(2) and shown on the NEXT redraw -- the only point at
    // which it can be known, and why it is timed here rather than by the caller.
    float lastDrawMs_ = 0.0F;
    // The same cost, held until exactly one frame has charged for it. A redraw lands on ~1 frame in 20, so adding it
    // to every frame would inflate the cpu total by ~20x the dashboard's real share.
    float unbilledDrawMs_ = 0.0F;

    // Bursty stages (rasterizer, texture upload) run on a small fraction of frames. A per-frame mean would report
    // ~1.7ms for a stage that actually stalls the render thread for 150ms and drops a frame, so they are reported
    // over the frames they ran on.
    std::array<float, 2> burstLastMs_{};
    std::array<std::uint64_t, 2> burstFireCount_{};
    std::uint64_t totalFrames_ = 0;

    bool tty_ = false;
    // Lines emitted by the previous draw, which is what the next draw's cursor-up must move by. Derived rather than a
    // constant kept in sync with the layout: a mismatch would make the block walk up the screen.
    std::size_t lastLineCount_ = 0;
    std::chrono::steady_clock::time_point lastDraw_{};
    std::array<char, kBufferBytes> buffer_{};
    std::size_t used_ = 0;
    std::size_t lines_ = 0;
};

}  // namespace pathtracer::debug
