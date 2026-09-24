#pragma once

#include <cstddef>

namespace pathtracer::debug {

struct GpuInfo;

// Everything the startup spec block reports that is not queryable from the host: the scene that loaded, the settings
// it loaded under, and the one-shot costs of loading it.
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
    double refreshHz;  // the window's display, measured by DisplayLink
};

// One-shot provenance block on stdout, printed once at startup after the GL context exists and the scene has loaded,
// so every number in it is real rather than a default.
// Deliberately not the live dashboard: plain text, no ANSI, no redraw, so it survives being piped to a log file.
void printSpec(const EngineSpec& spec, const GpuInfo& gpu);

}  // namespace pathtracer::debug
