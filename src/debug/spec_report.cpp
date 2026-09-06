#include "engine/debug/spec_report.h"

#include <array>
#include <cstdio>

#include "engine/debug/memory_tracker.h"
#include "engine/debug/system_info.h"

namespace engine::debug {

namespace {

constexpr double kMiB = 1024.0 * 1024.0;
constexpr double kGiB = 1024.0 * 1024.0 * 1024.0;

// std::printf, not std::cout: this block is a fixed-format table, and iostream's width/precision manipulators make that far harder to read than the format string does. The rest of the file's startup logging is being replaced by this, so there is no mixed-style seam left behind.
// Grouped by blank lines rather than by a label column and rule lines. Every line here is self-describing -- "16.00 GiB RAM", "git 3ef6b85+dirty" -- so a HOST/BUILD gutter spent 9 columns per line restating what the values already say, and the group separator is the one piece of structure that isn't redundant. 78 columns, matching perf_dashboard.h's width, so the two blocks line up in the same terminal.
// No CPU brand line: GL_RENDERER reads "Apple M1" on the line above, and on a unified-memory part that IS the CPU brand -- printing machdep.cpu.brand_string as well duplicated the string verbatim. What sysctl uniquely knows is the topology, which is what stays.
void printHost(const HostInfo& host) {
    // Two performance levels are reported separately because they are not interchangeable: the E-cores are several times slower per thread, so a ThreadPool sized to hw.logicalcpu is not 8 equal workers. Printed only when the machine actually has a second level.
    if (!host.perfLevel1Name.empty()) {
        std::printf("%d logical cores: %d %s (L2 %.1f MiB) + %d %s (L2 %.1f MiB)\n",
                     host.logicalCpuTotal, host.perfLevel0LogicalCpu, host.perfLevel0Name.c_str(),
                     static_cast<double>(host.perfLevel0L2Bytes) / kMiB, host.perfLevel1LogicalCpu,
                     host.perfLevel1Name.c_str(), static_cast<double>(host.perfLevel1L2Bytes) / kMiB);
    } else {
        std::printf("%d logical cores (L2 %.1f MiB)\n", host.logicalCpuTotal,
                     static_cast<double>(host.perfLevel0L2Bytes) / kMiB);
    }
    std::printf("%.2f GiB RAM   %d B cache line   macOS %s (build %s)\n",
                 static_cast<double>(totalSystemBytes()) / kGiB, host.cacheLineBytes,
                 host.osProductVersion.c_str(), host.osBuild.c_str());
}

// Compiler on its own line because __VERSION__ is long and unbounded ("Apple LLVM 21.0.0 (clang-2100.1.1.101)"); the flags it was invoked with, and the SHA they were applied to, are the group's payload and stay together on one line -- they are read as a unit when a benchmark number is attributed to a build.
void printBuild(const BuildInfo& build, const LibraryVersions& libs) {
    std::printf("%s\n", build.compiler);
    std::printf("%s  -march=%s  IPO %s  git %s\n", build.buildType, build.march,
                 build.ipo ? "on" : "off", build.gitSha);
    std::printf("embree %s  openexr %s  ocio %s  glew %s  glm %s\n", libs.embree.c_str(),
                 libs.openexr.c_str(), libs.ocio.c_str(), libs.glew.c_str(), libs.glm.c_str());
    // glfw last and alone: glfwGetVersionString appends its compile-time backend list, so this field is ~55 columns where the others are ~12, and anything packed after it wraps.
    std::printf("glfw %s\n", libs.glfw.c_str());
}

}  // namespace

void printSpec(const EngineSpec& spec, const GpuInfo& gpu) {
    // GL_RENDERER leads the block: on a unified-memory part it names the machine as well as the GPU, which is why no separate CPU brand line follows it. Grouped with the host topology rather than set apart, since between them they are the one answer to "what was this measured on".
    std::printf("%s   %s   %d Hz   GL_KHR_debug %s   GL_ARB_timer_query %s\n",
                 gpu.renderer.c_str(), gpu.version.c_str(), gpu.refreshRateHz,
                 spec.khrDebugAvailable ? "yes" : "no", spec.gpuTimerAvailable ? "yes" : "no");
    printHost(queryHostInfo());
    std::printf("\n");
    printBuild(buildInfo(), queryLibraryVersions());
    // Paths stay on their own lines: both are unbounded in length, so packing either alongside a fixed field is what makes the block wrap unpredictably on someone else's machine.
    std::printf("\n%s\n", spec.scenePath);
    std::printf("%d instances (%d light)   %d triangles\n", spec.instanceCount, spec.lightCount,
                 spec.triangleCount);
    std::printf("hdri  %s\n", spec.hdriPath);
    std::printf("load %.1f ms   bvh build %.1f ms   bvh %.1f MiB\n", spec.modelLoadMs,
                 spec.bvhBuildMs, static_cast<double>(spec.bvhBytes) / kMiB);
    std::printf("\nwindow %dx%d   renderScale %.2f (interactive %.2f)   aov %s\n", spec.windowWidth,
                 spec.windowHeight, spec.renderScale, spec.interactiveRenderScale, spec.aovName);
    std::printf("path trace  %u threads  %d px tiles  %d spp/pass  %d bounces  RR@%d\n",
                 spec.pathTraceThreads, spec.tileSize, spec.samplesPerPass, spec.maxBounces,
                 spec.russianRouletteStartBounce);
    if (spec.maxSamples > 0) {
        std::printf("max samples %d   ao range %.2f   raster %u threads\n\n", spec.maxSamples,
                     static_cast<double>(spec.aoMaxDistance), spec.rasterThreads);
    } else {
        std::printf("max samples unbounded   ao range %.2f   raster %u threads\n\n",
                     static_cast<double>(spec.aoMaxDistance), spec.rasterThreads);
    }
    printHotkeys();
    std::fflush(stdout);
}

std::span<const char* const> hotkeyRows() {
    // Two columns, split the way the input handling itself is: continuous camera keys (Window::isKeyDown) on the left, edge-triggered viewer toggles (wireCallbacks' key callback, main.cpp) on the right.
    static constexpr std::array<const char*, 5> kRows{
        "W/A/S/D   move camera           0      reset camera to default",
        "Q/E       move camera down/up   L      cycle viewer LUT (sRGB/Rec709/Raw)",
        "LMB drag  orbit camera          R/G/B  isolate channel of active AOV",
        "H         toggle HUD            I      invert display colour",
        "?         show this map         ESC    quit"};
    return kRows;
}

void printHotkeys() {
    for (const char* row : hotkeyRows()) {
        std::printf("%s\n", row);
    }
    std::printf("\n");
}

}  // namespace engine::debug
