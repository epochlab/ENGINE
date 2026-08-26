#pragma once

#include <cstddef>
#include <cstdint>

#include <glm/glm.hpp>

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

// Read-only convergence status the AOV section's path-traced readout displays -- built fresh each frame by main.cpp from PathTraceDriver's live query methods (accumulatedSamples()/lastPassSeconds()), not a snapshot of one completed call: the driver runs continuously in the background, so this reflects "as of this frame", not "as of the last Render click".
struct PathTracedStatus {
    bool hasResult = false;
    double lastPassSeconds = 0.0;
    int accumulatedSamples = 0;
    int maxSamples = 0;  // 0 = unbounded
};

// Pixel under cursor, read back from the composited framebuffer (post-LUT, post-exposure, the literal on-screen value) by main.cpp. valid=false off-viewport.
struct PixelProbeSample {
    bool valid = false;
    glm::vec4 color{0.0F};
};

// Everything HudOverlay::draw needs for one frame, bundled to keep its signature from growing indefinitely as sections are added. aov and FramingOverlayState stay as separate mutable out-parameters on draw() itself since ImGui widgets bind directly to them.
struct HudFrameData {
    const GpuInfo& gpuInfo;
    const FrameStats& frameStats;
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

    // Call after beginFrame(), before render(). aov/focalLengthMm/aperture/shutterSeconds/iso/showSky/envRotationDegrees/envExposureStops are widget-bound out-params (Camera/HDRI sections). envExposureStops is EV; main.cpp's requestPathTrace does exp2(). pixelProbe is read-only, rendered bottom-right. framing.crosshair gates the centre-crosshair foreground overlay.
    void draw(const HudFrameData& frame, int& aov, float& focalLengthMm, float& aperture,
              float& shutterSeconds, float& iso, bool& showSky, int& envRotationDegrees,
              float& envExposureStops, const FramingOverlayState& framing,
              const PixelProbeSample& pixelProbe) const;

    // ImGui::Render + backend draw-data submit. Call after the post-process blit, before window.swapBuffers().
    void render() const;

    // True while ImGui wants mouse input (e.g. dragging a HUD widget) — callers should not interpret an LMB click as a scene interaction (orbit pivot pick) while this is true.
    [[nodiscard]] bool wantsCaptureMouse() const;

private:
    bool owns_ = true;  // false on a moved-from instance; guards shutdown
};

}  // namespace engine::debug
