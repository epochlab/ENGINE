#include "pathtracer/debug/spec_report.h"

#include <cstdio>

#include "pathtracer/debug/memory_tracker.h"
#include "pathtracer/debug/system_info.h"

namespace pathtracer::debug {

namespace {

constexpr double kMiB = 1024.0 * 1024.0;
constexpr double kGiB = 1024.0 * 1024.0 * 1024.0;

// std::printf, not std::cout: a fixed-format 78-column table matching the dashboard grid, which iostream manipulators obscure.
void printHost(const HostInfo& host) {
    // Two levels reported separately: E-cores are several times slower, so a pool sized to hw.logicalcpu is not N equal workers.
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

// Compiler alone on its line because __VERSION__ is long and unbounded; the flags and the SHA they applied to stay together.
void printBuild(const BuildInfo& build, const LibraryVersions& libs) {
    std::printf("%s\n", build.compiler);
    std::printf("%s  -march=%s  IPO %s  git %s\n", build.buildType, build.march,
                 build.ipo ? "on" : "off", build.gitSha);
    std::printf("embree %s  openexr %s  ocio %s  glew %s  glm %s\n", libs.embree.c_str(),
                 libs.openexr.c_str(), libs.ocio.c_str(), libs.glew.c_str(), libs.glm.c_str());
    // glfw last and alone: glfwGetVersionString appends its backend list, making this field ~55 columns where the others are ~12.
    std::printf("glfw %s\n", libs.glfw.c_str());
}

}  // namespace

void printSpec(const EngineSpec& spec, const GpuInfo& gpu) {
    // GL_RENDERER leads: on a unified-memory part it names the machine as well as the GPU, which is why no CPU brand line follows.
    std::printf("%s   %s   %.2f Hz   GL_KHR_debug %s   GL_ARB_timer_query %s\n",
                 gpu.renderer.c_str(), gpu.version.c_str(), spec.refreshHz,
                 spec.khrDebugAvailable ? "yes" : "no", spec.gpuTimerAvailable ? "yes" : "no");
    printHost(queryHostInfo());
    printBuild(buildInfo(), queryLibraryVersions());
    // Paths stay on their own lines: both are unbounded, so packing either beside a fixed field is what makes the block wrap.
    std::printf("%s\n", spec.scenePath);
    std::printf("%d instances (%d light)   %d triangles\n", spec.instanceCount, spec.lightCount,
                 spec.triangleCount);
    std::printf("hdri  %s\n", spec.hdriPath);
    std::printf("load %.1f ms   bvh build %.1f ms   bvh %.1f MiB\n", spec.modelLoadMs,
                 spec.bvhBuildMs, static_cast<double>(spec.bvhBytes) / kMiB);
    std::printf("window %dx%d   renderScale %.2f (interactive %.2f)   aov %s\n", spec.windowWidth,
                 spec.windowHeight, spec.renderScale, spec.interactiveRenderScale, spec.aovName);
    std::printf("path trace  %u threads  %d px tiles  %d spp/pass  %d bounces  RR@%d\n",
                 spec.pathTraceThreads, spec.tileSize, spec.samplesPerPass, spec.maxBounces,
                 spec.russianRouletteStartBounce);
    if (spec.maxSamples > 0) {
        std::printf("max samples %d   ao range %.2f   raster %u threads\n", spec.maxSamples,
                     static_cast<double>(spec.aoMaxDistance), spec.rasterThreads);
    } else {
        std::printf("max samples unbounded   ao range %.2f   raster %u threads\n",
                     static_cast<double>(spec.aoMaxDistance), spec.rasterThreads);
    }
    std::fflush(stdout);
}

}  // namespace pathtracer::debug
