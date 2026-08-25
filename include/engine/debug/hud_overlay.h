#pragma once

#include <cstddef>
#include <cstdint>

#include "engine/debug/scene_stats.h"

struct GLFWwindow;

namespace engine::scene {
class Camera;
}

namespace engine::debug {

struct GpuInfo;
class FrameStats;
class Histogram;

// Toggle for the centre-crosshair framing overlay — owned by main.cpp and flipped by its 'K' hotkey (not a HUD checkbox), then read here read-only to decide whether to draw it.
struct FramingOverlayState {
    bool crosshair = true;
};

// Read-only convergence status the AOV section's path-traced readout displays -- built fresh each
// frame by main.cpp from PathTraceDriver's live query methods (accumulatedSamples()/
// lastPassSeconds()), not a snapshot of one completed call: the driver runs continuously in the
// background, so this reflects "as of this frame", not "as of the last Render click".
struct PathTracedStatus {
    bool hasResult = false;
    double lastPassSeconds = 0.0;
    int accumulatedSamples = 0;
};

// Everything HudOverlay::draw needs for one frame, bundled to keep its signature from growing indefinitely as sections are added. aov and FramingOverlayState stay as separate mutable out-parameters on draw() itself since ImGui widgets bind directly to them.
struct HudFrameData {
    const GpuInfo& gpuInfo;
    const FrameStats& frameStats;
    float geomMs;
    float postMs;
    std::size_t ramBytes;
    std::size_t gpuBytes;
    std::size_t systemAvailableBytes;
    std::uint64_t systemTotalBytes;
    int channelView;
    const char* lutName;
    SceneStats sceneStats;
    const engine::scene::Camera& camera;
    float cameraYawDegrees;
    float cameraPitchDegrees;
    bool cameraOrbiting;
    const Histogram& histogram;
    const PathTracedStatus& pathTraced;
};

// Owns the ImGui context and GLFW/OpenGL3 backends for one window's lifetime, move-only like this codebase's other RAII wrappers. Composites the debug panel onto the final backbuffer, after the OCIO tonemap pass — never into the linear HDR FBO (this project runs with no driver-level sRGB framebuffer conversion; display encoding happens only in-shader).
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

    // Builds the panel from already-collected values. Call after beginFrame(), before render(). aov
    // is mutated in place by the single unified AOV combo box (indices/names from the shared AovId
    // enum, engine/debug/aov.h -- the path tracer supplies a result for whichever AOVs it has
    // computed, main.cpp falls back to the rasterizer's pbr.frag branches otherwise); focalLengthMm
    // by the Lens section's slider; showSky by the HDRI section's "Show/Hide Background" checkbox
    // (only takes visible effect for the Beauty AOV -- see main.cpp's render loop); envRotationDegrees
    // ([0,359]) by the HDRI section's "Y-Axis" slider. The crosshair overlay is drawn over the full
    // viewport (foreground draw list), independent of the ##hud panel, gated on framing.crosshair.
    // The path tracer's convergence readout (frame.pathTraced) is shown in the Frame section --
    // there's no manual render trigger, PathTraceDriver retraces automatically on any input change
    // (see main.cpp's requestPathTraceIfTriggerChanged).
    void draw(const HudFrameData& frame, int& aov, float& focalLengthMm, bool& showSky,
              int& envRotationDegrees, const FramingOverlayState& framing) const;

    // ImGui::Render + backend draw-data submit. Call after the post-process blit, before window.swapBuffers().
    void render() const;

    // True while ImGui wants mouse input (e.g. dragging a HUD widget) — callers should not interpret an LMB click as a scene interaction (orbit pivot pick) while this is true.
    [[nodiscard]] bool wantsCaptureMouse() const;

private:
    bool owns_ = true;  // false on a moved-from instance; guards shutdown
};

}  // namespace engine::debug
