#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>

#include "engine/debug/render_stats.h"

namespace engine::debug {

class FrameStats;

// Everything one dashboard redraw shows, bundled by renderFrame -- same convention, and the same reason, as HudFrameData (hud_overlay.h): it keeps update()'s signature from growing indefinitely as rows are added. References are to render-thread-owned state that outlives the call.
struct DashboardFrame {
    const FrameStats& frameStats;
    const FrameStageTimes& stages;
    const PassRecord& pass;
    // This frame's wall interval. Accumulated over the same window as the stage times, so the reconciliation row compares like with like -- FrameStats' own mean is over its 120-entry ring, a different and longer window, and subtracting the two produced a negative residual.
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
    // '?' toggles the hotkey section on, beneath the block's closing rule. It has to be a section of this block rather than a print of its own: the redraw rewrites every line it owns, so anything printed into that region is erased before the eye can catch it.
    bool showHotkeys;
    int refreshRateHz;
};

// Live terminal dashboard: redraws a fixed-height block in place at kRefreshHz using ANSI cursor-up plus erase-line, so the terminal shows one screenful rather than accumulating scrollback. Render-thread only -- update() is called at the end of renderFrame and reads that thread's own FrameStageTimes/FrameStats with no synchronization.
// Disabled when stdout is not a TTY: in-place redraw is meaningless in a pipe and the escape sequences would corrupt a log file. Disabled, it emits one plain single-line summary every kNonTtySeconds instead, so a piped run still leaves a progress trace.
// Does NOT hide the terminal cursor: a crash or SIGKILL would leave the user's terminal with no cursor and no destructor to restore it, and the cursor sitting on the last line costs nothing.
class PerfDashboard {
public:
    // 3 Hz, because the draw is a blocking write(2) that lands after swapBuffers and therefore eats into the next frame's headroom. Its cost is measured here and shown on the next redraw rather than hidden. Raising this much past 5 starts to distort what it reports.
    static constexpr int kRefreshHz = 3;
    static constexpr double kNonTtySeconds = 5.0;
    static constexpr std::size_t kBufferBytes = 8192;
    // Lines the body occupies, used only to reserve the block on the very first draw. Every later redraw moves up by the count the previous draw actually emitted, so a wrong value here costs one misaligned frame rather than a block that walks up the screen forever.
    static constexpr int kBodyLines = 30;

    PerfDashboard();

    // Call once per frame at the very end of renderFrame, after swapBuffers. Rate-limited internally: on the frames between redraws this is one steady_clock::now(), a compare, and the accumulator adds.
    void update(const DashboardFrame& frame);

private:
    void accumulate(const DashboardFrame& frame);
    void draw(const DashboardFrame& frame);
    // draw() is split only to keep each part inside the one-screen function limit; the split points are the layout's own columns, not arbitrary.
    // Each row of the body carries a render-thread stage on the left and a path-trace stat on the right, so the split is by row group rather than by column. Header/body/footer, each one screenful.
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

    // Windowed means of every per-frame stage, so the display is stable rather than flickering at whatever the last frame happened to cost. Reset after each redraw.
    FrameStageTimes sums_{};
    float cpuTotalSum_ = 0.0F;
    float frameMsSum_ = 0.0F;
    int windowFrames_ = 0;

    // This dashboard's own draw cost, measured around the write(2) and shown on the NEXT redraw -- the only point at which it can be known, and the reason it is timed here rather than by the caller.
    float lastDrawMs_ = 0.0F;
    // The same cost, held until exactly one frame has charged for it. A redraw happens on roughly 1 frame in 20, so adding lastDrawMs_ to every frame's total would inflate cpu total by ~20x the dashboard's real share and push the unaccounted residual negative.
    float unbilledDrawMs_ = 0.0F;

    // Bursty stages (rasterizer, texture upload, over-range scan) run on a small fraction of frames. Their per-frame mean would report ~1.7ms for a stage that actually stalls the render thread for 150ms and drops a frame, so the last ACTUAL cost is kept alongside a duty cycle counted since launch -- cumulative, not windowed, because at 3Hz a window of ~20 frames usually contains zero firings of a 1-in-88 stage.
    std::array<float, 3> burstLastMs_{};
    std::array<std::uint64_t, 3> burstFireCount_{};
    std::uint64_t totalFrames_ = 0;

    bool tty_ = false;
    // Lines emitted by the previous draw, which is what the next draw's cursor-up must move by. Derived rather than a constant the layout has to be kept in sync with: a mismatch would make the block walk up the screen on every redraw.
    std::size_t lastLineCount_ = 0;
    std::chrono::steady_clock::time_point lastDraw_{};
    std::array<char, kBufferBytes> buffer_{};
    std::size_t used_ = 0;
    std::size_t lines_ = 0;
};

}  // namespace engine::debug
