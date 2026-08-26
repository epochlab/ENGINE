#pragma once

#include <optional>
#include <string>

#include <glm/glm.hpp>

#include "engine/gfx/ocio_display_transform.h"
#include "engine/scene/camera.h"

namespace engine::config {

// Session-wide defaults: engine::scene::DebugCameraController's initial (and reset-to) pose,
// lens/exposure params, and interactive tuning constants, plus everything else main.cpp needs at
// startup that isn't specific to one scene/asset (that's SceneConfig) -- window size, the HDRI to
// load, initial debug-view state, and path-tracer settings. Externalized so these can be edited
// without recompiling. See assets/config/profile.json for the checked-in defaults.
struct ProfileConfig {
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
    float flySpeedMetersPerSecond;
    float orbitSensitivityDegPerPixel;
    std::string hdriPath;  // environment map, relative to ASSET_ROOT_DIR
    int windowWidth;
    int windowHeight;
    // Index into engine::debug::AovId / kAovNames (aov.h) (0 = Beauty).
    int defaultAov;
    engine::gfx::OcioDisplayTransform::Lut defaultLut;
    int samplesPerPixel;   // path tracer startup default
    int maxBounces;        // path tracer startup default; secondary/indirect bounces beyond the primary hit, 0 = direct lighting only
    int russianRouletteStartBounce;  // 0-based bounce index RR kicks in from
    int maxSamples;  // accumulated-pass cap for PathTraceDriver; 0 = unbounded
};

// Reads and parses path. Returns nullopt and logs to stderr if the file is missing, unreadable, or any required field can't be found/parsed — this is user-editable input, not an internal invariant, so failure is expected to happen and is surfaced rather than defaulted around.
[[nodiscard]] std::optional<ProfileConfig> loadProfileConfig(const std::string& path);

}  // namespace engine::config
