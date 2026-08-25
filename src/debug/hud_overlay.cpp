#include "engine/debug/hud_overlay.h"

// GLEW before GLFW — see gl_debug.cpp for why.
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>

#include <glm/glm.hpp>

#include "engine/debug/aov.h"
#include "engine/debug/frame_stats.h"
#include "engine/debug/histogram.h"
#include "engine/debug/system_info.h"
#include "engine/scene/camera.h"

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

namespace engine::debug {

namespace {

constexpr ImVec4 kCyan(0.0F, 0.85F, 0.85F, 1.0F);

constexpr float kHistogramHeight = 72.0F;

// 9-tap triangular smoother (radius 4): every output bin 0-255 is a weighted average of sqrt(min(rawCount, peak)/peak) over neighbors whose index falls in [1, 254] -- bins 0/255 are never read, including as their own center sample, so their displayed height comes entirely from extrapolating nearby interior bins. This is what keeps a background spike at bin 0 (e.g. camera far from a small on-screen subject) from ever reaching the curve or the peak calculation, rather than merely being deprioritized.
void smoothChannel(const std::array<std::uint32_t, 256>& channelBins, std::uint32_t peak,
                    std::array<float, 256>& out) {
    for (int bin = 0; bin < 256; ++bin) {
        float sum = 0.0F;
        float weightSum = 0.0F;
        for (int offset = -4; offset <= 4; ++offset) {
            const int neighbor = bin + offset;
            if (neighbor < 1 || neighbor > 254) {
                continue;
            }
            const int distance = offset < 0 ? -offset : offset;
            const float weight = 5.0F - static_cast<float>(distance);
            const float value = static_cast<float>(
                std::min(channelBins[static_cast<std::size_t>(neighbor)], peak));
            sum += weight * std::sqrt(value / static_cast<float>(peak));
            weightSum += weight;
        }
        out[static_cast<std::size_t>(bin)] = weightSum > 0.0F ? sum / weightSum : 0.0F;
    }
}

bool isGrayscale(const std::array<std::array<std::uint32_t, 256>, 3>& bins) {
    for (int bin = 0; bin < 256; ++bin) {
        if (bins[0][static_cast<std::size_t>(bin)] != bins[1][static_cast<std::size_t>(bin)] ||
            bins[1][static_cast<std::size_t>(bin)] != bins[2][static_cast<std::size_t>(bin)]) {
            return false;
        }
    }
    return true;
}

// One continuous filled+outlined curve from a [0,1]-normalized heights array, rather than 256 independent rectangles -- the technique that actually produces a smooth silhouette instead of a blocky bar chart.
void drawHistogramCurve(ImDrawList* drawList, ImVec2 origin, float histogramWidth,
                         const std::array<float, 256>& vals, ImU32 fillColor, ImU32 lineColor) {
    const float binWidth = histogramWidth / 256.0F;
    std::array<ImVec2, 256> edge{};
    for (int bin = 0; bin < 256; ++bin) {
        edge[static_cast<std::size_t>(bin)] = ImVec2(
            origin.x + ((static_cast<float>(bin) + 0.5F) * binWidth),
            origin.y + (kHistogramHeight * (1.0F - vals[static_cast<std::size_t>(bin)])));
    }
    std::array<ImVec2, 258> poly{};
    poly[0] = ImVec2(origin.x, origin.y + kHistogramHeight);
    for (int bin = 0; bin < 256; ++bin) {
        poly[static_cast<std::size_t>(bin) + 1] = edge[static_cast<std::size_t>(bin)];
    }
    poly[257] = ImVec2(origin.x + histogramWidth, origin.y + kHistogramHeight);

    // Concave fills triangulate internally; per-triangle AA seams show as faint diagonal lines across the fill, so anti-aliasing is switched off for just this call -- the outline stroke below keeps its own.
    const ImDrawListFlags savedFlags = drawList->Flags;
    drawList->Flags &= ~ImDrawListFlags_AntiAliasedFill;
    drawList->AddConcavePolyFilled(poly.data(), 258, fillColor);
    drawList->Flags = savedFlags;
    drawList->AddPolyline(edge.data(), 256, lineColor, 0, 1.0F);
}

// Grayscale AOVs (R==G==B in every bin) collapse to a single curve, using a full-range unsmoothed scale for near-binary content (e.g. alpha/depth masks) so 0/255 clipping spikes stay exactly as captured.
void drawGrayscaleHistogram(ImDrawList* drawList, ImVec2 origin, float histogramWidth,
                             const std::array<std::array<std::uint32_t, 256>, 3>& bins) {
    std::uint64_t interiorTotal = 0;
    for (int bin = 1; bin <= 254; ++bin) {
        interiorTotal += bins[0][static_cast<std::size_t>(bin)];
    }
    // Under ~1% of the 256x144 downsample's pixels fall in the interior -- near-binary content (e.g. an alpha/depth mask): full-range peak, raw sqrt scale, no smoothing, so the 0/255 spikes stay exactly as captured.
    const bool nearBinary = interiorTotal < static_cast<std::uint64_t>(256 * 144 / 100);
    std::array<float, 256> heights{};
    if (nearBinary) {
        std::uint32_t peak = 1;
        for (std::uint32_t count : bins[0]) {
            peak = std::max(peak, count);
        }
        for (int bin = 0; bin < 256; ++bin) {
            heights[static_cast<std::size_t>(bin)] =
                std::sqrt(static_cast<float>(bins[0][static_cast<std::size_t>(bin)]) /
                          static_cast<float>(peak));
        }
    } else {
        std::uint32_t peak = 1;
        for (int bin = 1; bin <= 254; ++bin) {
            peak = std::max(peak, bins[0][static_cast<std::size_t>(bin)]);
        }
        smoothChannel(bins[0], peak, heights);
    }
    drawHistogramCurve(drawList, origin, histogramWidth, heights, IM_COL32(180, 180, 180, 130),
                        IM_COL32(220, 220, 220, 220));
}

// A channel this AOV structurally doesn't use (e.g. blue in a UV AOV) has every pixel at bin 0 -- checked over bins 1-255 (not 0) so a background-dominated but genuinely-used channel isn't mistaken for an unused one.
std::array<bool, 3> activeChannels(const std::array<std::array<std::uint32_t, 256>, 3>& bins) {
    std::array<bool, 3> active{};
    for (int c = 0; c < 3; ++c) {
        bool empty = true;
        for (int bin = 1; bin < 256; ++bin) {
            if (bins[static_cast<std::size_t>(c)][static_cast<std::size_t>(bin)] > 0) {
                empty = false;
                break;
            }
        }
        active[static_cast<std::size_t>(c)] = !empty;
    }
    return active;
}

// One peak shared across every active channel -- not an independent peak per channel -- so a channel with genuinely more signal reads taller than one with less, instead of every channel independently stretching to fill the same height.
std::uint32_t sharedPeak(const std::array<std::array<std::uint32_t, 256>, 3>& bins,
                          const std::array<bool, 3>& active) {
    std::uint32_t peak = 1;
    for (int c = 0; c < 3; ++c) {
        if (!active[static_cast<std::size_t>(c)]) {
            continue;
        }
        for (int bin = 1; bin <= 254; ++bin) {
            peak = std::max(peak, bins[static_cast<std::size_t>(c)][static_cast<std::size_t>(bin)]);
        }
    }
    return peak;
}

// Each channel drawn as one smooth filled+outlined curve layered back-to-front B/G/R, then a "floor" curve (the min across active channels) on top wherever they overlap. All compositing is ImGui's ordinary alpha-over blending -- no additive/custom GL blend state -- so an overlap like R-over-G reads as a muted brown/olive rather than a saturated additive colour.
void drawRgbHistogram(ImDrawList* drawList, ImVec2 origin, float histogramWidth,
                       const std::array<std::array<std::uint32_t, 256>, 3>& bins) {
    const std::array<bool, 3> active = activeChannels(bins);
    const std::uint32_t peak = sharedPeak(bins, active);

    std::array<std::array<float, 256>, 3> heights{};
    for (int c = 0; c < 3; ++c) {
        if (active[static_cast<std::size_t>(c)]) {
            smoothChannel(bins[static_cast<std::size_t>(c)], peak,
                          heights[static_cast<std::size_t>(c)]);
        }
    }

    // B, G, R -- R topmost -- then the shared overlap curve last.
    if (active[2]) {
        drawHistogramCurve(drawList, origin, histogramWidth, heights[2],
                            IM_COL32(40, 80, 200, 120), IM_COL32(80, 140, 255, 220));
    }
    if (active[1]) {
        drawHistogramCurve(drawList, origin, histogramWidth, heights[1],
                            IM_COL32(40, 180, 60, 120), IM_COL32(80, 220, 100, 220));
    }
    if (active[0]) {
        drawHistogramCurve(drawList, origin, histogramWidth, heights[0],
                            IM_COL32(200, 40, 40, 120), IM_COL32(255, 100, 80, 220));
    }

    const int activeCount = (active[0] ? 1 : 0) + (active[1] ? 1 : 0) + (active[2] ? 1 : 0);
    if (activeCount < 2) {
        return;
    }
    std::array<float, 256> overlap{};
    float overlapMax = 0.0F;
    for (int bin = 0; bin < 256; ++bin) {
        float value = 1.0F;
        for (int c = 0; c < 3; ++c) {
            if (active[static_cast<std::size_t>(c)]) {
                value = std::min(value, heights[static_cast<std::size_t>(c)][static_cast<std::size_t>(bin)]);
            }
        }
        overlap[static_cast<std::size_t>(bin)] = value;
        overlapMax = std::max(overlapMax, value);
    }
    if (overlapMax > 0.02F) {
        drawHistogramCurve(drawList, origin, histogramWidth, overlap, IM_COL32(180, 180, 180, 160),
                            IM_COL32(255, 255, 255, 220));
    }
}

// Renders the current AOV's per-channel histogram. Width matches the panel's content width so it lines up with every other section.
void drawHistogramPanel(const std::array<std::array<std::uint32_t, 256>, 3>& bins) {
    ImGui::TextColored(kCyan, "Histogram");

    const float histogramWidth = ImGui::GetContentRegionAvail().x;
    const ImVec2 origin = ImGui::GetCursorScreenPos();
    const ImVec2 boxMax(origin.x + histogramWidth, origin.y + kHistogramHeight);
    ImDrawList* drawList = ImGui::GetWindowDrawList();
    drawList->AddRectFilled(origin, boxMax, IM_COL32(18, 18, 18, 255));

    if (isGrayscale(bins)) {
        drawGrayscaleHistogram(drawList, origin, histogramWidth, bins);
    } else {
        drawRgbHistogram(drawList, origin, histogramWidth, bins);
    }

    drawList->AddRect(origin, boxMax, IM_COL32(60, 60, 60, 180));
    ImGui::Dummy(ImVec2(histogramWidth, kHistogramHeight));
}

// Centre crosshair framing overlay — drawn on the foreground draw list, over the whole viewport, independent of the ##hud panel, so it never contaminates the AOV buffers being debugged.
void drawFramingOverlays(const FramingOverlayState& state, ImVec2 displaySize) {
    if (!state.crosshair) {
        return;
    }
    ImDrawList* drawList = ImGui::GetForegroundDrawList();
    constexpr ImU32 kGuideColor = IM_COL32(255, 255, 255, 140);
    const float cx = displaySize.x * 0.5F;
    const float cy = displaySize.y * 0.5F;
    constexpr float kArmLength = 10.0F;
    drawList->AddLine(ImVec2(cx - kArmLength, cy), ImVec2(cx + kArmLength, cy), kGuideColor);
    drawList->AddLine(ImVec2(cx, cy - kArmLength), ImVec2(cx, cy + kArmLength), kGuideColor);
}

void drawGpuSection(const HudFrameData& frame) {
    ImGui::TextColored(kCyan, "GPU");
    ImGui::Text("%s", frame.gpuInfo.renderer.c_str());
    ImGui::Text("%s", frame.gpuInfo.version.c_str());
    ImGui::Text("%d Hz", frame.gpuInfo.refreshRateHz);
    ImGui::Separator();
}

void drawFrameSection(const HudFrameData& frame) {
    ImGui::TextColored(kCyan, "Frame");
    ImGui::Text("%.0f FPS  avg %.2f ms", frame.frameStats.fps(), frame.frameStats.avgMs());
    ImGui::PlotLines("##frametime", frame.frameStats.history().data(), FrameStats::kHistoryLength,
                      frame.frameStats.cursor(), nullptr, 0.0F, 33.3F,
                      ImVec2(ImGui::GetContentRegionAvail().x, 40.0F));
    ImGui::Text("min %.2f  max %.2f ms", frame.frameStats.minMs(), frame.frameStats.maxMs());
    ImGui::Text("GPU  geom %.2f  post %.2f ms", frame.geomMs, frame.postMs);
    const float fps = frame.frameStats.fps();
    const auto pixelsDrawn = static_cast<long long>(frame.sceneStats.viewportWidth) *
                             frame.sceneStats.viewportHeight;
    ImGui::Text("%.1f Mtri/s  %.1f Mpix/s",
                static_cast<float>(frame.sceneStats.trianglesDrawn) * fps / 1.0e6F,
                static_cast<float>(pixelsDrawn) * fps / 1.0e6F);
    ImGui::Text("Cap  vsync");
    ImGui::Text("LUT  %s", frame.lutName);
    if (frame.pathTraced.hasResult) {
        ImGui::Text("path-traced  %d samples  %.2f s/pass", frame.pathTraced.accumulatedSamples,
                    frame.pathTraced.lastPassSeconds);
    } else {
        ImGui::TextDisabled("path-traced: no render yet");
    }
    ImGui::Separator();
}

void drawMemorySection(const HudFrameData& frame) {
    ImGui::TextColored(kCyan, "Memory");
    ImGui::Text("RAM  %.1f MB", static_cast<double>(frame.ramBytes) / (1024.0 * 1024.0));
    ImGui::Text("GPU alloc  %.1f MB", static_cast<double>(frame.gpuBytes) / (1024.0 * 1024.0));
    ImGui::Text("System  %.1f / %.1f GB free",
                static_cast<double>(frame.systemAvailableBytes) / (1024.0 * 1024.0 * 1024.0),
                static_cast<double>(frame.systemTotalBytes) / (1024.0 * 1024.0 * 1024.0));
    ImGui::Separator();
}

void drawViewportAndSceneSection(const HudFrameData& frame) {
    ImGui::TextColored(kCyan, "Viewport");
    ImGui::Text("%d x %d", frame.sceneStats.viewportWidth, frame.sceneStats.viewportHeight);
    ImGui::Separator();

    ImGui::TextColored(kCyan, "Scene");
    ImGui::Text("Objects  %d", frame.sceneStats.objectCount);
    ImGui::Text("Draw calls  %d / %d  (%d culled)", frame.sceneStats.instancesDrawn,
                frame.sceneStats.objectCount, frame.sceneStats.instancesCulled);
    ImGui::Text("Triangles  %lld", frame.sceneStats.trianglesTotal);
    ImGui::Text("Points  %lld", frame.sceneStats.pointsTotal);
    ImGui::Separator();
}

// Single AOV selector for both renderers -- names/order come from the shared AovId enum
// (engine/debug/aov.h), not a locally duplicated array. The path tracer supplies a result for
// whichever AOVs it has computed; anything else falls back to the rasterizer's pbr.frag branches.
void drawAovSection(int& aov) {
    ImGui::TextColored(kCyan, "AOV");
    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
    ImGui::Combo("##aov", &aov, kAovNames, IM_ARRAYSIZE(kAovNames));
    ImGui::Separator();
}

void drawCameraSection(const HudFrameData& frame) {
    ImGui::TextColored(kCyan, "Camera");
    const glm::vec3 camPos = frame.camera.position();
    ImGui::Text("pos  x %.2f  y %.2f  z %.2f", camPos.x, camPos.y, camPos.z);
    ImGui::Text("rot  x %.1f  y %.1f", frame.cameraPitchDegrees, frame.cameraYawDegrees);
    const engine::scene::Camera::FilmBack filmBack = frame.camera.filmBack();
    ImGui::Text("Filmback  %.1f x %.1f mm", filmBack.widthMm, filmBack.heightMm);
    ImGui::Text("Focal  %.1f mm", frame.camera.focalLengthMm());
    ImGui::Text("Near  %.2f  Far  %.1f", frame.camera.nearClip(), frame.camera.farClip());
    if (frame.cameraOrbiting) {
        ImGui::TextColored(kCyan, "orbiting");
    }
    ImGui::Separator();
}

void drawLensSection(float& focalLengthMm) {
    ImGui::TextColored(kCyan, "Lens");
    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
    ImGui::SliderFloat("##focalLength", &focalLengthMm, 10.0F, 300.0F, "Focal Length  %.0f mm");
    ImGui::Separator();
}

void drawHdriSection(bool& showSky, int& envRotationDegrees) {
    ImGui::TextColored(kCyan, "HDRI");
    // Only visibly affects the Beauty AOV (main.cpp's render loop gates the actual sky draw on aov==0) -- left interactive regardless of the active AOV rather than grayed out, simplest for a checkbox whose effect is just "no-op elsewhere".
    ImGui::Checkbox("Show/Hide Background", &showSky);
    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
    ImGui::SliderInt("##envRotation", &envRotationDegrees, 0, 359, "Y-Axis  %d deg");
    ImGui::Separator();
}

// Active R/G/B channel isolation, top-right corner -- foreground draw list, independent of the ##hud window.
void drawChannelViewCorner(int channelView) {
    if (channelView == 0) {
        return;
    }
    const char* label = channelView == 1 ? "R" : channelView == 2 ? "G" : "B";
    const ImU32 color = channelView == 1   ? IM_COL32(255, 70, 70, 255)
                        : channelView == 2 ? IM_COL32(70, 255, 70, 255)
                                           : IM_COL32(70, 70, 255, 255);
    const ImVec2 displaySize = ImGui::GetIO().DisplaySize;
    const ImVec2 textSize = ImGui::CalcTextSize(label);
    const ImVec2 pos(displaySize.x - textSize.x - 16.0F, 8.0F);
    ImGui::GetForegroundDrawList()->AddText(pos, color, label);
}

}  // namespace

HudOverlay::HudOverlay(GLFWwindow* nativeHandle) {
    ImGui::CreateContext();
    ImGui::GetIO().IniFilename = nullptr;  // no stray imgui.ini for a fixed debug panel

    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 0.0F;
    style.WindowBorderSize = 0.0F;
    style.Colors[ImGuiCol_WindowBg] = ImVec4(0.0F, 0.0F, 0.0F, 0.85F);

    if (!ImGui_ImplGlfw_InitForOpenGL(nativeHandle, true)) {
        std::cerr << "HudOverlay: ImGui_ImplGlfw_InitForOpenGL failed\n";
        std::exit(EXIT_FAILURE);
    }
    // matches this project's GL 4.1 core context
    if (!ImGui_ImplOpenGL3_Init("#version 410")) {
        std::cerr << "HudOverlay: ImGui_ImplOpenGL3_Init failed\n";
        std::exit(EXIT_FAILURE);
    }
}

HudOverlay::~HudOverlay() {
    if (owns_) {
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
    }
}

HudOverlay::HudOverlay(HudOverlay&& other) noexcept : owns_(other.owns_) {
    other.owns_ = false;
}

HudOverlay& HudOverlay::operator=(HudOverlay&& other) noexcept {
    if (this != &other) {
        if (owns_) {
            ImGui_ImplOpenGL3_Shutdown();
            ImGui_ImplGlfw_Shutdown();
            ImGui::DestroyContext();
        }
        owns_ = other.owns_;
        other.owns_ = false;
    }
    return *this;
}

void HudOverlay::beginFrame() const {
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
}

void HudOverlay::draw(const HudFrameData& frame, int& aov, float& focalLengthMm, bool& showSky,
                       int& envRotationDegrees, const FramingOverlayState& framing) const {
    ImGui::SetNextWindowPos(ImVec2(8, 8), ImGuiCond_Always);
    constexpr ImGuiWindowFlags flags =
        ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize |
        ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing |
        ImGuiWindowFlags_NoNav;
    ImGui::Begin("##hud", nullptr, flags);

    drawGpuSection(frame);
    drawFrameSection(frame);
    drawMemorySection(frame);
    drawViewportAndSceneSection(frame);

    if (frame.histogram.hasData()) {
        drawHistogramPanel(frame.histogram.bins());
        ImGui::Separator();
    }

    drawAovSection(aov);
    drawCameraSection(frame);
    drawLensSection(focalLengthMm);
    drawHdriSection(showSky, envRotationDegrees);

    ImGui::End();

    drawChannelViewCorner(frame.channelView);
    drawFramingOverlays(framing, ImGui::GetIO().DisplaySize);
}

void HudOverlay::render() const {
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

bool HudOverlay::wantsCaptureMouse() const {
    return ImGui::GetIO().WantCaptureMouse;
}

}  // namespace engine::debug
