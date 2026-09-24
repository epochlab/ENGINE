#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <glm/glm.hpp>

#include "pathtracer/debug/scene_stats.h"

struct GLFWwindow;

namespace pathtracer::scene {
class Camera;
}

namespace pathtracer::debug {

struct GpuInfo;
class FrameStats;
class Histogram;

// Toggle for the centre-crosshair framing overlay, owned by main.cpp (defaults on, no runtime toggle), read here.
struct FramingOverlayState {
    bool crosshair = true;
};

// Read-only convergence status for the AOV section's path-traced readout, rebuilt each frame by main.cpp from the
// displayed PathTraceResult's sample count and the driver's state.
struct PathTracedStatus {
    bool hasResult = false;
    double lastPassSeconds = 0.0;
    int accumulatedSamples = 0;
    int maxSamples = 0;  // 0 = unbounded
};

// Pixel under the cursor, sampled by main.cpp's samplePixelProbe. For Beauty and the post-filter AOVs this is the
// composited framebuffer pixel, post-LUT; for the rest it is the raw AOV value.
struct PixelProbeSample {
    bool valid = false;
    glm::vec4 color{0.0F};
};

// Everything HudOverlay::draw needs for one frame, bundled to keep its signature from growing as sections are added.
// aov and FramingOverlayState stay separate mutable arguments because draw writes them back.
struct HudFrameData {
    const GpuInfo& gpuInfo;
    double refreshHz;  // the window's display, measured by DisplayLink
    const FrameStats& frameStats;
    float postMs;
    std::size_t ramBytes;
    std::size_t gpuBytes;
    std::size_t systemAvailableBytes;
    std::uint64_t systemTotalBytes;
    int channelView;
    const char* lutName;
    SceneStats sceneStats;
    const pathtracer::scene::Camera& camera;
    float cameraYawDegrees;
    float cameraPitchDegrees;
    bool cameraOrbiting;
    const Histogram& histogram;
    const PathTracedStatus& pathTraced;
    // Fraction of Beauty's texels that would clip at the display encode, and the peak as a multiple of display range
    // -- see main.cpp's updateOverRangeStats.
    float overRangeFraction;
    float overRangePeakMultiple;
    bool vsync;  // profile.json frame cap, for the Cap readout
};

// Owns the ImGui context and GLFW/OpenGL3 backends for one window's lifetime, move-only like this codebase's other
// RAII wrappers. Composites the debug panel onto the final backbuffer.
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

    // Call after beginFrame(), before render(). Every field the HUD can edit is passed by reference and written back
    // in place, so the caller sees edits without a return value.
    void draw(const HudFrameData& frame, int& aov, float& focalLengthMm, float& aperture,
              float& shutterSeconds, float& iso, int& filmBackPresetIndex,
              const std::vector<const char*>& filmBackPresetNames, bool& showSky,
              bool& envLightEnabled, int& envRotationDegrees, float& envExposureStops,
              float& aberrationStrength, const FramingOverlayState& framing,
              const PixelProbeSample& pixelProbe) const;

    // ImGui::Render + backend draw-data submit. Call after the post-process blit, before window.swapBuffers().
    void render() const;

    // True while ImGui wants the mouse (dragging a HUD widget): callers must not read an LMB click as a scene
    // interaction such as an orbit pivot pick while it holds.
    [[nodiscard]] bool wantsCaptureMouse() const;

private:
    bool owns_ = true;  // false on a moved-from instance; guards shutdown
};

}  // namespace pathtracer::debug
