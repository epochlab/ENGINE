#pragma once

#include <optional>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "pathtracer/gfx/ocio_display_transform.h"
#include "pathtracer/gfx/scalar_type.h"
#include "pathtracer/scene/camera.h"

namespace pathtracer::config {

struct WindowConfig {
    int width;
    int height;
};

struct CameraConfig {
    glm::vec3 position;
    float yawDegrees;
    float pitchDegrees;

    // Resolved against assets/config/camera.json by name at startup. loadProfileConfig cannot validate it alone,
    // since it does not load that file.
    std::string defaultFilmBackPresetName;
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
    // Fraction of the framebuffer actually rendered, upscaled to the window by the display blit's GL_LINEAR filter.
    float renderScale;
    float interactiveRenderScale;
    // Index into pathtracer::debug::AovId / kAovNames (aov.h) (0 = Beauty).
    int defaultAov;
    pathtracer::gfx::OcioDisplayTransform::Lut defaultLut;
    // true paces each frame to the display's vblank (DisplayLink); false runs uncapped, bounded only by the
    // one-frame-in-flight fence.
    bool vsync;
    // profile.json bit depths: 16 -> Float16, 32 -> Float32. 8-bit UNORM is not offered, as it clamps scene-referred
    // data to [0,1] before exposure.
    pathtracer::gfx::ScalarType displayFormat;  // displayBitDepth: the path-traced display texture's GL storage
    pathtracer::gfx::ScalarType textureType;    // textureBitDepth: environment HDRI and every material texture's CPU storage
};

struct PathTracerConfig {
    int samplesPerPixel;   // path tracer startup default
    int maxBounces;        // path tracer startup default; secondary/indirect bounces beyond the primary hit, 0 = direct lighting only
    int russianRouletteStartBounce;  // 0-based bounce index RR kicks in from
    int maxSamples;  // accumulated-pass cap for PathTraceDriver; 0 = unbounded
    float aoMaxDistance;  // ray-traced AO occlusion range, scene units; occluders beyond it don't darken
    float lookaheadDistance;  // horizon of the Lookahead AOV's ramp, scene units; geometry at or beyond it reads 0
};

// Session-wide defaults: DebugCameraController's initial and reset pose, lens and exposure, interactive tuning
// constants, and everything else main.cpp needs that is not specific to one scene.
struct ProfileConfig {
    WindowConfig window;
    CameraConfig camera;
    ControlsConfig controls;
    RenderConfig render;
    PathTracerConfig pathTracer;
};

// Reads and parses path. Returns nullopt and logs to stderr if the file is missing, unreadable, or a required field
// cannot be parsed. User-editable input, not an internal invariant: failure is expected and surfaced, not asserted.
[[nodiscard]] std::optional<ProfileConfig> loadProfileConfig(const std::string& path);

// Reads the film-back preset catalogue (assets/config/camera.json), a JSON array of {name, widthMm, heightMm}. Same
// failure contract as loadProfileConfig, plus a positivity check on every entry.
[[nodiscard]] std::optional<std::vector<pathtracer::scene::Camera::FilmBackPreset>> loadFilmBackPresets(
    const std::string& path);

}  // namespace pathtracer::config
