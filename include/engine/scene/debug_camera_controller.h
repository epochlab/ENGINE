#pragma once

#include <glm/glm.hpp>

#include "engine/scene/camera.h"

namespace engine::platform {
class Window;
}

namespace engine::scene {

// Mutable fly/orbit state that produces a fresh, immutable Camera each frame via snapshot() — Camera itself stays immutable by design (see its header); this is the debug camera its doc comment forward-references. Lens parameters (film back, clip planes, exposure triangle) are loaded once from profile.json and never mutated at runtime; pose (position/yaw/pitch), orbit state, and focal length (editable live via the HUD's Lens slider) do change.
class DebugCameraController {
public:
    // position/yawDegrees/pitchDegrees become both the initial pose and the pose resetToDefault() restores; the remaining lens params are passed through unchanged to every snapshot()'d Camera.
    DebugCameraController(const glm::vec3& position, float yawDegrees, float pitchDegrees,
                           Camera::FilmBack filmBack, float focalLengthMm, float nearClip,
                           float farClip, float aperture, float shutterSeconds, float iso,
                           float flySpeedMetersPerSecond, float orbitSensitivityDegPerPixel);

    // Builds an immutable Camera from the current pose. Call once per frame — this is the only point where a Camera value exists.
    [[nodiscard]] Camera snapshot() const;

    // Polls W/A/S/D/Q/E and moves position_ in the horizontal view plane (WASD) or along world-up (QE), scaled by dtSeconds and the configured fly speed. No-op while orbiting — fly and orbit are mutually exclusive input modes.
    void applyFlyInput(const engine::platform::Window& window, float dtSeconds);

    void beginOrbit(const glm::vec3& pivot);

    // dxPixels/dyPixels are this frame's cursor delta; tumbles position_ around pivot_ (yaw around world up, pitch around the local right vector, gated against the poles). No-op if not currently orbiting.
    void applyOrbitDelta(float dxPixels, float dyPixels);

    void endOrbit();
    [[nodiscard]] bool isOrbiting() const { return orbiting_; }

    void resetToDefault();

    [[nodiscard]] float yawDegrees() const { return yawDegrees_; }
    [[nodiscard]] float pitchDegrees() const { return pitchDegrees_; }
    [[nodiscard]] float focalLengthMm() const { return focalLengthMm_; }

    // Bound directly to the HUD's Lens slider widget, the same "bind a live variable directly to a widget" convention used elsewhere — except here the live variable lives behind this setter rather than as a bare reference, since it's controller-owned state.
    void setFocalLengthMm(float focalLengthMm) { focalLengthMm_ = focalLengthMm; }

private:
    glm::vec3 position_;
    float yawDegrees_;
    float pitchDegrees_;
    float focalLengthMm_;

    const glm::vec3 defaultPosition_;
    const float defaultYawDegrees_;
    const float defaultPitchDegrees_;

    const Camera::FilmBack filmBack_;
    const float nearClip_;
    const float farClip_;
    const float aperture_;
    const float shutterSeconds_;
    const float iso_;

    const float flySpeedMetersPerSecond_;
    const float orbitSensitivityDegPerPixel_;

    bool orbiting_ = false;
    glm::vec3 pivot_{0.0F};
};

}  // namespace engine::scene
