#pragma once

#include <cstddef>

struct GLFWwindow;

namespace engine::debug {

struct GpuInfo;
class FrameStats;

// Owns the ImGui context and GLFW/OpenGL3 backends for one window's
// lifetime, move-only like this codebase's other RAII wrappers. Composites
// the debug panel onto the final backbuffer, after the OCIO tonemap pass —
// never into the linear HDR FBO (this project runs with no driver-level
// sRGB framebuffer conversion; display encoding happens only in-shader).
class HudOverlay {
public:
    explicit HudOverlay(GLFWwindow* nativeHandle);
    ~HudOverlay();

    HudOverlay(const HudOverlay&) = delete;
    HudOverlay& operator=(const HudOverlay&) = delete;
    HudOverlay(HudOverlay&& other) noexcept;
    HudOverlay& operator=(HudOverlay&& other) noexcept;

    // Call after window.pollEvents(), before any GL draw calls.
    void beginFrame() const;

    // Builds the panel from already-collected values. Call after
    // beginFrame(), before render(). trianglesDrawn/pixelsDrawn are this
    // frame's counts, used to derive Mtri/s and Mpix/s.
    void draw(const GpuInfo& gpuInfo, const FrameStats& frameStats, float geomMs, float postMs,
              int trianglesDrawn, long long pixelsDrawn, std::size_t ramBytes,
              std::size_t gpuBytes) const;

    // ImGui::Render + backend draw-data submit. Call after the
    // post-process blit, before window.swapBuffers().
    void render() const;

private:
    bool owns_ = true;  // false on a moved-from instance; guards shutdown
};

}  // namespace engine::debug
