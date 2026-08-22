#include "engine/debug/hud_overlay.h"

// GLEW before GLFW — see gl_debug.cpp for why.
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "engine/debug/frame_stats.h"
#include "engine/debug/system_info.h"

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

namespace engine::debug {

HudOverlay::HudOverlay(GLFWwindow* nativeHandle) {
    ImGui::CreateContext();
    ImGui::GetIO().IniFilename = nullptr;  // no stray imgui.ini for a fixed debug panel

    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 0.0F;
    style.WindowBorderSize = 0.0F;
    style.Colors[ImGuiCol_WindowBg] = ImVec4(0.0F, 0.0F, 0.0F, 0.85F);

    ImGui_ImplGlfw_InitForOpenGL(nativeHandle, true);
    ImGui_ImplOpenGL3_Init("#version 410");  // matches this project's GL 4.1 core context
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

void HudOverlay::draw(const GpuInfo& gpuInfo, const FrameStats& frameStats, float geomMs,
                       float postMs, int trianglesDrawn, long long pixelsDrawn,
                       std::size_t ramBytes, std::size_t gpuBytes) const {
    ImGui::SetNextWindowPos(ImVec2(8, 8), ImGuiCond_Always);
    constexpr ImGuiWindowFlags flags =
        ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize |
        ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing |
        ImGuiWindowFlags_NoNav;
    constexpr ImVec4 kCyan(0.0F, 0.85F, 0.85F, 1.0F);
    ImGui::Begin("##hud", nullptr, flags);

    ImGui::TextColored(kCyan, "GPU");
    ImGui::Text("%s", gpuInfo.renderer.c_str());
    ImGui::Text("%s", gpuInfo.version.c_str());
    ImGui::Separator();

    ImGui::TextColored(kCyan, "Frame");
    ImGui::Text("%.0f FPS  avg %.2f ms", frameStats.fps(), frameStats.avgMs());
    ImGui::PlotLines("##frametime", frameStats.history().data(), FrameStats::kHistoryLength,
                      frameStats.cursor(), nullptr, 0.0F, 33.3F, ImVec2(200, 40));
    ImGui::Text("min %.2f  max %.2f ms", frameStats.minMs(), frameStats.maxMs());
    ImGui::Text("GPU  geom %.2f  post %.2f ms", geomMs, postMs);
    const float fps = frameStats.fps();
    ImGui::Text("%.1f Mtri/s  %.1f Mpix/s", static_cast<float>(trianglesDrawn) * fps / 1.0e6F,
                static_cast<float>(pixelsDrawn) * fps / 1.0e6F);
    ImGui::Text("Cap  none");
    ImGui::Separator();

    ImGui::TextColored(kCyan, "Memory");
    ImGui::Text("RAM  %.1f MB", static_cast<double>(ramBytes) / (1024.0 * 1024.0));
    ImGui::Text("GPU alloc  %.1f MB (meshes + FBO, tracked)",
                static_cast<double>(gpuBytes) / (1024.0 * 1024.0));

    ImGui::End();
}

void HudOverlay::render() const {
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

}  // namespace engine::debug
