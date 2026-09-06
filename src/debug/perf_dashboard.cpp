#include "engine/debug/perf_dashboard.h"

#include <unistd.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "engine/debug/frame_stats.h"
#include "engine/debug/spec_report.h"

namespace engine::debug {

namespace {

constexpr double kMiB = 1024.0 * 1024.0;
constexpr double kGiB = 1024.0 * 1024.0 * 1024.0;
constexpr double kMillion = 1.0e6;

// Index into PerfDashboard's burst arrays. Bursty means "runs on a small fraction of frames": the rasterizer only on a trigger change into one of its AOVs, the upload only when a newly published pass invalidates the display texture, the over-range scan every 4th frame.
enum BurstStage { kBurstRaster = 0, kBurstUpload = 1, kBurstOverRange = 2 };

// Frames per firing, "1/88", in the column a per-frame stage puts its percentage. "never" for a stage that has not fired since launch -- a real state (the rasterizer, whenever no rasterizer AOV has been selected) and not the same as firing every frame, which is what dividing by a floor of one would have shown.
void formatDuty(std::array<char, 8>& out, double framesPerFiring) {
    if (framesPerFiring > 0.0) {
        std::snprintf(out.data(), out.size(), "1/%.0f", framesPerFiring);
    } else {
        std::snprintf(out.data(), out.size(), "never");
    }
}

using Bar = std::array<char, 64>;
constexpr int kBarCells = 10;
constexpr const char* kBarBlank = "          ";

// Ten cells at 1/8-cell resolution (U+2588 down to U+258F), so a stage worth 1.2% of the frame still leaves a visible mark where a whole-block bar would round it away. Written into a caller-owned buffer: the dashboard allocates nothing while drawing, and nothing about a row's rendering outlives that row.
constexpr std::array<const char*, 9> kBlocks{" ", "▏", "▎", "▍", "▌",
                                              "▋", "▊", "▉", "█"};

void formatBar(Bar& out, double fraction) {
    const int eighths =
        std::clamp(static_cast<int>(std::lround(fraction * kBarCells * 8)), 0, kBarCells * 8);
    std::size_t used = 0;
    const auto put = [&](const char* text) {
        while (*text != '\0' && used + 1 < out.size()) {
            out[used++] = *text++;
        }
    };
    for (int cell = 0; cell < kBarCells; ++cell) {
        put(kBlocks[static_cast<std::size_t>(std::clamp(eighths - cell * 8, 0, 8))]);
    }
    out[used] = '\0';
}

// Display range of the residual bar, not a threshold on anything: a healthy reconciliation sits under 1% of the frame, so drawing it against the same 100% full scale as the stage bars leaves it permanently blank and unable to show the sign it exists to show. The row's own number is the exact value; this only decides how far a given residual deflects. Beyond +-10% the bar simply pins.
constexpr double kResidualFullScale = 0.10;

// Diverging bar for a signed quantity. The reconciliation residual is the one row that can fall either side of zero, and a left-aligned bar draws a small overshoot and a small undershoot identically -- which is the whole thing that row exists to distinguish. Centre is zero, right positive, left negative, and half-scale here equals full scale in formatBar so both are read against the same ruler.
// The negative half is quantised to whole cells: the graded block series fills a cell from its LEFT edge, correct for a bar growing rightwards and wrong for one growing leftwards, and Unicode has no right-anchored series to mirror it with.
void formatSignedBar(Bar& out, double fraction) {
    constexpr int kHalfCells = kBarCells / 2;
    const int eighths =
        std::clamp(static_cast<int>(std::lround(fraction * kHalfCells * 8)), -kHalfCells * 8,
                    kHalfCells * 8);
    std::size_t used = 0;
    const auto put = [&](const char* text) {
        while (*text != '\0' && used + 1 < out.size()) {
            out[used++] = *text++;
        }
    };
    const int wholeLeft = eighths < 0 ? (-eighths + 7) / 8 : 0;
    for (int cell = 0; cell < kHalfCells; ++cell) {
        put(cell >= kHalfCells - wholeLeft ? "█" : " ");
    }
    for (int cell = 0; cell < kHalfCells; ++cell) {
        put(kBlocks[static_cast<std::size_t>(std::clamp(eighths - cell * 8, 0, 8))]);
    }
    out[used] = '\0';
}

float percentOf(float value, float total) { return total > 0.0F ? (value / total) * 100.0F : 0.0F; }

double rate(std::uint64_t count, double milliseconds) {
    return milliseconds > 0.0 ? static_cast<double>(count) / (milliseconds / 1000.0) : 0.0;
}

}  // namespace

// isatty on the descriptor actually written, not on std::cout: a shell redirect changes the former and says nothing about the latter. TERM=dumb is honoured too -- it is the conventional way to say "this terminal has no cursor addressing", and emitting cursor-up into one produces garbage rather than a dashboard.
PerfDashboard::PerfDashboard() : lastDraw_(std::chrono::steady_clock::now()) {
    const char* term = std::getenv("TERM");
    tty_ = isatty(STDOUT_FILENO) == 1 && (term == nullptr || std::strcmp(term, "dumb") != 0);
}

void PerfDashboard::append(const char* format, ...) {
    if (used_ >= buffer_.size()) {
        return;  // full: drop the rest of this frame's text rather than truncate mid-escape-sequence
    }
    std::va_list args;
    va_start(args, format);
    const int written = std::vsnprintf(buffer_.data() + used_, buffer_.size() - used_, format, args);
    va_end(args);
    if (written <= 0) {
        return;
    }
    // vsnprintf returns what it WOULD have written, so it must be clamped before it advances the cursor past the end.
    const std::size_t advance = std::min(static_cast<std::size_t>(written), buffer_.size() - used_ - 1);
    lines_ += static_cast<std::size_t>(
        std::count(buffer_.data() + used_, buffer_.data() + used_ + advance, '\n'));
    used_ += advance;
}

// One write(2) of the whole block, not std::cout: iostream formatting can allocate through locale/num_put, and a partially flushed frame tears visibly. The return value is deliberately ignored -- a failed write to a terminal is not something a renderer can or should act on.
void PerfDashboard::flush() {
    if (used_ > 0) {
        [[maybe_unused]] const ssize_t ignored = ::write(STDOUT_FILENO, buffer_.data(), used_);
    }
    used_ = 0;
}

void PerfDashboard::accumulate(const DashboardFrame& frame) {
    const FrameStageTimes& stages = frame.stages;
    sums_.pollMs += stages.pollMs;
    sums_.cameraMs += stages.cameraMs;
    sums_.presentMs += stages.presentMs;
    sums_.histogramMs += stages.histogramMs;
    sums_.probeMs += stages.probeMs;
    sums_.hudMs += stages.hudMs;
    sums_.swapMs += stages.swapMs;
    sums_.uploadMs += stages.uploadMs;
    sums_.rasterMs += stages.rasterMs;
    sums_.overRangeMs += stages.overRangeMs;
    // Every stage, bursty ones included: this is what must reconcile against the measured frame time, so it cannot exclude the stages that actually cost the most.
    cpuTotalSum_ += stages.pollMs + stages.cameraMs + stages.rasterMs + stages.presentMs +
                     stages.histogramMs + stages.overRangeMs + stages.probeMs + stages.hudMs +
                     stages.swapMs + unbilledDrawMs_;
    unbilledDrawMs_ = 0.0F;  // charged exactly once, to the frame that actually paid it
    frameMsSum_ += frame.frameMs;
    ++windowFrames_;
    ++totalFrames_;

    // A stage that did not run this frame reads exactly 0 (renderFrame zeroes them all), so a non-zero value is a firing, and the value kept is the real cost rather than a mean diluted by the frames it skipped.
    const std::array<float, 3> burst{stages.rasterMs, stages.uploadMs, stages.overRangeMs};
    for (std::size_t i = 0; i < burst.size(); ++i) {
        if (burst[i] > 0.0F) {
            burstLastMs_[i] = burst[i];
            ++burstFireCount_[i];
        }
    }
}

void PerfDashboard::update(const DashboardFrame& frame) {
    accumulate(frame);

    const auto now = std::chrono::steady_clock::now();
    const double sinceDraw = std::chrono::duration<double>(now - lastDraw_).count();
    const double interval = tty_ ? 1.0 / static_cast<double>(kRefreshHz) : kNonTtySeconds;
    if (sinceDraw < interval) {
        return;
    }
    lastDraw_ = now;

    {
        const ScopedCpuTimer drawTimer(lastDrawMs_);
        if (tty_) {
            draw(frame);
        } else {
            drawNonTty(frame);
        }
    }

    unbilledDrawMs_ = lastDrawMs_;

    sums_ = {};
    cpuTotalSum_ = 0.0F;
    frameMsSum_ = 0.0F;
    windowFrames_ = 0;
}

void PerfDashboard::drawNonTty(const DashboardFrame& frame) {
    const auto& pass = frame.pass;
    append("engine perf: frame p50 %.2fms p95 %.2fms | pass #%d %.1fms %.2f Mray/s | rss %.2f GiB\n",
            static_cast<double>(frame.frameStats.percentileMs(0.5F)),
            static_cast<double>(frame.frameStats.percentileMs(0.95F)), pass.passIndex, pass.traceMs,
            rate(pass.rays.total(), pass.traceMs) / kMillion,
            static_cast<double>(frame.ramBytes) / kGiB);
    flush();
}

float PerfDashboard::windowMean(float sum) const {
    return sum / static_cast<float>(std::max(windowFrames_, 1));
}

float PerfDashboard::cpuTotalMs() const { return windowMean(cpuTotalSum_); }

// Frames per firing, since launch. A stage that has never fired returns 0, which the caller renders as "never" rather than dividing by a floor of 1 and reporting it as every frame.
double PerfDashboard::burstDutyFrames(int stage) const {
    const std::uint64_t fires = burstFireCount_[static_cast<std::size_t>(stage)];
    return fires > 0 ? static_cast<double>(totalFrames_) / static_cast<double>(fires) : 0.0;
}

void PerfDashboard::draw(const DashboardFrame& frame) {
    lines_ = 0;
    if (lastLineCount_ == 0) {
        // Reserve the block on the first draw so the cursor-up below always has real lines to move over, instead of walking back over whatever was on screen before.
        for (int i = 0; i < kBodyLines; ++i) {
            append("\n");
        }
        lastLineCount_ = lines_;
        flush();
        lines_ = 0;
    }
    // Up by exactly what the previous draw emitted, then erase each line as it is rewritten. Never \x1b[2J or \x1b[H: those destroy scrollback and pin the block to the screen origin.
    append("\x1b[%zuA", lastLineCount_);

    drawFrameHeader(frame);
    drawStageRows(frame);
    drawRayRows(frame);
    drawFooter(frame);
    // The block grows and shrinks as the hotkey section is toggled. Growth needs nothing -- the next cursor-up uses the count this draw emitted -- but shrinking would leave the rows it shed sitting below, so they are erased here and stay part of the block's height as blank lines.
    while (lines_ < lastLineCount_) {
        append("\x1b[2K\n");
    }

    lastLineCount_ = lines_;
    flush();
}

// Fixed 78-column grid, and it is a grid rather than eyeballed spacing: the left pane is columns 0-42, the divider sits at column 43 on every split row, and the right pane fills 44-77. Rows whose left half carries no numbers (the section headers, the sub-rules, the blank spacer) are padded to the same 43, because one column of drift there is what makes a two-pane table read as two unrelated tables.
void PerfDashboard::drawFrameHeader(const DashboardFrame& frame) {
    const double budgetMs = frame.refreshRateHz > 0 ? 1000.0 / frame.refreshRateHz : 0.0;
    append("\x1b[2K== ENGINE PERF ================================================== %5.1f fps ==\n",
            static_cast<double>(frame.frameStats.fps()));
    // Percentiles lead, mean trails: a mean hides the hitch, and the hitch is what a viewer feels. p95 rather than p99 because 120 samples cannot express a 99th percentile -- see FrameStats::percentileMs.
    append("\x1b[2K frame    p50 %6.2f   p95 %6.2f   max %6.2f   mean %6.2f ms   (%3d frames)\n",
            static_cast<double>(frame.frameStats.percentileMs(0.5F)),
            static_cast<double>(frame.frameStats.percentileMs(0.95F)),
            static_cast<double>(frame.frameStats.maxMs()),
            static_cast<double>(windowMean(frameMsSum_)), FrameStats::kHistoryLength);
    append("\x1b[2K budget   %6.2f ms @ %3d Hz vsync                        headroom %8.2f ms\n",
            budgetMs, frame.refreshRateHz, budgetMs - static_cast<double>(cpuTotalMs()));
    append("\x1b[2K------------------------------------------------------------------------------\n");
}

void PerfDashboard::drawStageRows(const DashboardFrame& frame) {
    const float cpu = cpuTotalMs();
    const auto& pass = frame.pass;
    const double passMs = pass.traceMs + pass.accumulateMs + pass.publishMs;
    const auto phasePct = [&](double ms) { return passMs > 0.0 ? (ms / passMs) * 100.0 : 0.0; };
    const auto mean = [&](float sum) { return static_cast<double>(windowMean(sum)); };
    const auto pct = [&](float sum) { return static_cast<double>(percentOf(windowMean(sum), cpu)); };
    // presentMs is measured inclusive of the upload nested inside it, so the blit's own cost is the difference -- arithmetic, not a second instrument.
    const double blitMs = std::max(mean(sums_.presentMs) - mean(sums_.uploadMs), 0.0);
    // One buffer, rewritten per row: formatBar always overwrites and terminates, and no row's bar is read after its own append.
    Bar bar{};
    // Bursty stages print their last REAL cost and, in place of a percentage, how often they fire: averaging a 150ms stall that happens 1 frame in 88 reports 1.7ms for something that drops a frame every time it runs, and a share-of-this-frame is either ~100% or zero. They get no bar for the same reason.
    std::array<char, 8> duty{};
    const auto barFor = [&](float sum) {
        formatBar(bar, static_cast<double>(percentOf(windowMean(sum), cpu)) / 100.0);
    };

    append("\x1b[2K RENDER THREAD     cpu ms     %%            | PATH TRACE (driver)     ms      %%\n");
    barFor(sums_.pollMs);
    append("\x1b[2K  %-15s%8.3f %5.1f %s |  pass #%-4d gen %-6llu %4dx%-4d \n", "poll",
            mean(sums_.pollMs), pct(sums_.pollMs), bar.data(), pass.passIndex,
            static_cast<unsigned long long>(pass.generation), pass.width, pass.height);
    barFor(sums_.cameraMs);
    append("\x1b[2K  %-15s%8.3f %5.1f %s |  trace           %9.2f %6.1f\n", "camera",
            mean(sums_.cameraMs), pct(sums_.cameraMs), bar.data(), pass.traceMs,
            phasePct(pass.traceMs));
    formatDuty(duty, burstDutyFrames(kBurstRaster));
    append("\x1b[2K  %-15s%8.2f %5s %s |  accumulate      %9.2f %6.1f\n", "raster gbuffer",
            static_cast<double>(burstLastMs_[kBurstRaster]), duty.data(), kBarBlank,
            pass.accumulateMs, phasePct(pass.accumulateMs));
    formatDuty(duty, burstDutyFrames(kBurstUpload));
    append("\x1b[2K  %-15s%8.3f %5s %s |  publish         %9.2f %6.1f\n", "tex upload",
            static_cast<double>(burstLastMs_[kBurstUpload]), duty.data(), kBarBlank, pass.publishMs,
            phasePct(pass.publishMs));
    formatBar(bar, static_cast<double>(percentOf(static_cast<float>(blitMs), cpu)) / 100.0);
    append("\x1b[2K  %-15s%8.3f %5.1f %s |  --------------------------------\n", "present blit",
            blitMs, static_cast<double>(percentOf(static_cast<float>(blitMs), cpu)), bar.data());
    barFor(sums_.histogramMs);
    append("\x1b[2K  %-15s%8.3f %5.1f %s |  tiles     %5llu / %-5llu %-6s\n", "histogram",
            mean(sums_.histogramMs), pct(sums_.histogramMs), bar.data(),
            static_cast<unsigned long long>(pass.tilesCompleted),
            static_cast<unsigned long long>(pass.tilesCompleted + pass.tilesCancelled),
            pass.cancelled ? "CANCEL" : "");
    formatDuty(duty, burstDutyFrames(kBurstOverRange));
    append("\x1b[2K  %-15s%8.3f %5s %s |  samples   %5d / %s\n", "over-range",
            static_cast<double>(burstLastMs_[kBurstOverRange]), duty.data(), kBarBlank,
            frame.accumulatedSamples, frame.maxSamples > 0 ? "capped" : "unbounded");
    barFor(sums_.probeMs);
    append("\x1b[2K  %-15s%8.3f %5.1f %s |  suspended %s\n", "pixel probe", mean(sums_.probeMs),
            pct(sums_.probeMs), bar.data(), frame.driverSuspended ? "yes" : "no");
    barFor(sums_.hudMs);
    append("\x1b[2K  %-15s%8.3f %5.1f %s |\n", "hud", mean(sums_.hudMs), pct(sums_.hudMs),
            bar.data());
}

void PerfDashboard::drawRayRows(const DashboardFrame& frame) {
    const float cpu = cpuTotalMs();
    const float frameMs = windowMean(frameMsSum_);
    const auto& pass = frame.pass;
    const auto& rays = pass.rays;
    const double total = static_cast<double>(rays.total());
    const auto share = [&](std::uint64_t n) { return total > 0.0 ? (static_cast<double>(n) / total) * 100.0 : 0.0; };
    const auto mray = [&](std::uint64_t n) { return rate(n, pass.traceMs) / kMillion; };
    const auto millions = [](std::uint64_t n) { return static_cast<double>(n) / kMillion; };
    Bar bar{};

    append("\x1b[2K  %-15s%8.3f %5s %s | RAYS          count  Mray/s     %%\n", "dashboard",
            static_cast<double>(lastDrawMs_), "", kBarBlank);
    formatBar(bar, static_cast<double>(percentOf(windowMean(sums_.swapMs), cpu)) / 100.0);
    append("\x1b[2K  %-15s%8.3f %5.1f %s |  %-9s%7.3f M %7.2f %5.1f\n", "swap (vsync)",
            static_cast<double>(windowMean(sums_.swapMs)),
            static_cast<double>(percentOf(windowMean(sums_.swapMs), cpu)), bar.data(), "primary",
            millions(rays.primary), mray(rays.primary), share(rays.primary));
    append("\x1b[2K  ---------------------------------------- |  %-9s%7.3f M %7.2f %5.1f\n",
            "bounce", millions(rays.bounce), mray(rays.bounce), share(rays.bounce));
    append("\x1b[2K  %-15s%8.3f %5.1f %s |  %-9s%7.3f M %7.2f %5.1f\n", "cpu total",
            static_cast<double>(cpu), 100.0, kBarBlank, "ao", millions(rays.ao), mray(rays.ao), share(rays.ao));
    append("\x1b[2K  %-15s%8.3f %5s %s |  %-9s%7.3f M %7.2f %5.1f\n", "frame measured",
            static_cast<double>(frameMs), "", kBarBlank, "shadow", millions(rays.shadow),
            mray(rays.shadow), share(rays.shadow));
    // The residual is printed, not left to be subtracted: a growing gap is the signal that a stage exists which nothing is measuring, which is why it gets its own colour rather than sharing the work bar's.
    const double residual = static_cast<double>(percentOf(frameMs - cpu, frameMs)) / 100.0;
    formatSignedBar(bar, residual / kResidualFullScale);
    append("\x1b[2K  %-15s%8.3f %5.1f %s |  --------------------------------\n", "unaccounted",
            static_cast<double>(frameMs - cpu),
            static_cast<double>(percentOf(frameMs - cpu, frameMs)), bar.data());
    append("\x1b[2K  %-15s%8s %5s %s |  %-9s%7.3f M %7.2f %5.1f\n", "", "", "", kBarBlank, "total", millions(rays.total()),
            mray(rays.total()), 100.0);
    // Ratios rather than new counters: one primary ray is traced per sample, so both fall out of the counts already here.
    append("\x1b[2K  GPU  GL_TIME_ELAPSED                     |  rays/sample %5.2f  depth %5.2f  \n",
            rays.primary > 0 ? total / static_cast<double>(rays.primary) : 0.0,
            rays.primary > 0 ? (static_cast<double>(rays.bounce) / static_cast<double>(rays.primary)) + 1.0 : 0.0);
    append("\x1b[2K  %-15s%8.3f %5s %s |\n", "present (blit)",
            static_cast<double>(frame.presentGpuMs), "", kBarBlank);
}

void PerfDashboard::drawFooter(const DashboardFrame& frame) {
    append("\x1b[2K------------------------------------------------------------------------------\n");
    append("\x1b[2K MEMORY  rss %.2f GiB   gl %.1f MiB   bvh %.1f MiB   sys %.2f/%.2f GiB\n",
            static_cast<double>(frame.ramBytes) / kGiB, static_cast<double>(frame.gpuBytes) / kMiB,
            static_cast<double>(frame.bvhBytes) / kMiB,
            static_cast<double>(frame.systemAvailableBytes) / kGiB,
            static_cast<double>(frame.systemTotalBytes) / kGiB);
    append("\x1b[2K SCENE   %s   %d inst   %d light   %d tri   aov %s\n", frame.sceneName,
            frame.instanceCount, frame.lightCount, frame.triangleCount, frame.aovName);
    append("\x1b[2K RENDER  window %dx%d   scale %.2f%s   render %dx%d\n", frame.windowWidth,
            frame.windowHeight, static_cast<double>(frame.renderScale),
            frame.interactiveScale ? " interactive" : "", frame.renderWidth, frame.renderHeight);
    // Bursty stages show their last real cost and how often they fire, never a per-frame mean: averaging a 150ms stall that happens 1 frame in 88 reports 1.7ms for something that drops a frame every time it runs.
    append("\x1b[2K==============================================================================\n");
    if (frame.showHotkeys) {
        for (const char* row : hotkeyRows()) {
            append("\x1b[2K %s\n", row);
        }
    }
}

}  // namespace engine::debug
