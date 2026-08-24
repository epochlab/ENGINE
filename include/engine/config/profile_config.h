#pragma once

#include <optional>
#include <string>

#include <glm/glm.hpp>

#include "engine/scene/camera.h"

namespace engine::config {

// Everything engine::scene::DebugCameraController needs to construct its
// initial (and reset-to) pose, lens/exposure params, and interactive
// tuning constants — externalized so these can be edited without
// recompiling. See assets/config/profile.json for the checked-in
// defaults.
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
};

// Reads and parses path. Returns nullopt and logs to stderr if the file
// is missing, unreadable, or any required field can't be found/parsed —
// this is user-editable input, not an internal invariant, so failure is
// expected to happen and is surfaced rather than defaulted around.
[[nodiscard]] std::optional<ProfileConfig> loadProfileConfig(const std::string& path);

}  // namespace engine::config
