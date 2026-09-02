#pragma once

#include <optional>
#include <string>

#include <glm/glm.hpp>

#include "engine/gfx/ocio_display_transform.h"
#include "engine/scene/camera.h"

namespace engine::config {

struct WindowConfig {
    int width;
    int height;
};

struct CameraConfig {
    glm::vec3 position;
    float yawDegrees;
    float pitchDegrees;

    engine::scene::Camera::FilmBack filmBack;
    float focalLengthMm;
    float nearClip;
    float farClip;
    float aperture;
    float shutterSeconds;
    float iso;
};

struct ControlsConfig {
    float flySpeedMetersPerSecond;
    float orbitSensitivityDegPerPixel;
};

struct RenderConfig {
    // Fraction of the framebuffer the path tracer and rasterizer actually render at, upscaled to the window by the display blit's GL_LINEAR filter. On a Retina display a 1024x576 window is a 2048x1152 framebuffer, so 1.0 traces 4x the paths the window implies. renderScale applies once the camera settles, interactiveRenderScale while it is moving -- the standard progressive-renderer trade of resolution for latency during interaction. Both in (0,1].
    float renderScale;
    float interactiveRenderScale;
    // Index into engine::debug::AovId / kAovNames (aov.h) (0 = Beauty).
    int defaultAov;
    engine::gfx::OcioDisplayTransform::Lut defaultLut;
};

struct PathTracerConfig {
    int samplesPerPixel;   // path tracer startup default
    int maxBounces;        // path tracer startup default; secondary/indirect bounces beyond the primary hit, 0 = direct lighting only
    int russianRouletteStartBounce;  // 0-based bounce index RR kicks in from
    int maxSamples;  // accumulated-pass cap for PathTraceDriver; 0 = unbounded
};

// Session-wide defaults: engine::scene::DebugCameraController's initial (and reset-to) pose, lens/exposure params, and interactive tuning constants, plus everything else main.cpp needs at startup that isn't specific to one scene/asset (that's SceneConfig, which also owns the HDRI path) -- window size, initial debug-view state, and path-tracer settings. Externalized so these can be edited without recompiling; see assets/config/profile.json for the checked-in defaults. Grouped into window/camera/controls/render/pathTracer sub-objects, matching profile.json's shape.
struct ProfileConfig {
    WindowConfig window;
    CameraConfig camera;
    ControlsConfig controls;
    RenderConfig render;
    PathTracerConfig pathTracer;
};

// Reads and parses path. Returns nullopt and logs to stderr if the file is missing, unreadable, or any required field can't be found/parsed. User-editable input, not an internal invariant: failure is expected and surfaced rather than defaulted around.
[[nodiscard]] std::optional<ProfileConfig> loadProfileConfig(const std::string& path);

}  // namespace engine::config
