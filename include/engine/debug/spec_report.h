#pragma once

#include <cstddef>
#include <span>

namespace engine::debug {

struct GpuInfo;

// Everything the startup spec block reports that isn't queryable from the host itself -- the scene that loaded, the settings it loaded under, and the one-shot costs of loading it. Bundled rather than passed as 25 parameters, same convention as HudFrameData (hud_overlay.h). Pointers are to strings owned by the caller's config, which outlive the printSpec call.
struct EngineSpec {
    const char* scenePath;
    const char* hdriPath;
    const char* aovName;
    int windowWidth;
    int windowHeight;
    float renderScale;
    float interactiveRenderScale;
    int samplesPerPass;
    int maxBounces;
    int russianRouletteStartBounce;
    int maxSamples;  // 0 = unbounded
    float aoMaxDistance;
    unsigned int pathTraceThreads;
    unsigned int rasterThreads;
    int tileSize;
    int instanceCount;
    int lightCount;
    int triangleCount;
    double modelLoadMs;
    double bvhBuildMs;
    std::size_t bvhBytes;
    bool khrDebugAvailable;
    bool gpuTimerAvailable;
};

// One-shot provenance block on stdout, printed once at startup after the GL context exists (queryGpuInfo) and after the scene has loaded, so every number in it is real rather than a default.
// Deliberately NOT the live dashboard: plain text, no ANSI, no redraw, so it survives being piped to a log file. Arnold prints the equivalent header at the top of every render log for the same reason -- a timing number without the machine and build it was measured on is not a measurement.
void printSpec(const EngineSpec& spec, const GpuInfo& gpu);

// The key bindings as rows of text. Data rather than a print call, because '?' under -stats cannot simply print them: the dashboard rewrites every line of its own block on each redraw, so the map has to be drawn as a section of that block, in its erase-line framing. One list, two framings, no restatement.
[[nodiscard]] std::span<const char* const> hotkeyRows();

// The same rows straight to stdout, for the startup block and for '?' when no dashboard is running to host them.
void printHotkeys();

}  // namespace engine::debug
