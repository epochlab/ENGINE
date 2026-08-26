#pragma once

#include <cmath>

#include <glm/glm.hpp>

#include "engine/scene/camera.h"

namespace engine::platform {
class Window;
}

namespace engine::scene {

// Mutable fly/orbit state that produces a fresh, immutable Camera each frame via snapshot() — Camera itself stays immutable by design (see its header); this is the debug camera its doc comment forward-references. Film back and clip planes are loaded once from profile.json and never mutated at runtime; pose (position/yaw/pitch), orbit state, focal length, and the exposure triangle (aperture/shutterSeconds/iso, editable live via the HUD's Camera section sliders) do change.
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
    [[nodiscard]] float aperture() const { return aperture_; }
    [[nodiscard]] float shutterSeconds() const { return shutterSeconds_; }
    [[nodiscard]] float iso() const { return iso_; }

    // Bound to the HUD's Camera section sliders via setter, not a bare reference (controller-owned state).
    void setFocalLengthMm(float focalLengthMm) { focalLengthMm_ = focalLengthMm; }
    void setAperture(float aperture) { aperture_ = aperture; }
    void setShutterSeconds(float shutterSeconds) { shutterSeconds_ = shutterSeconds; }
    void setIso(float iso) { iso_ = iso; }

    // EV100 delta vs profile.json defaults. Fed to OcioDisplayTransform::setExposureEv() (display-stage
    // pow(2,ev), not baked into radiance -- scene isn't photometrically calibrated, no retrace needed).
    [[nodiscard]] float relativeExposureEv() const {
        const float defaultEv100 = std::log2((defaultAperture_ * defaultAperture_) /
                                              defaultShutterSeconds_ * (100.0F / defaultIso_));
        const float currentEv100 =
            std::log2((aperture_ * aperture_) / shutterSeconds_ * (100.0F / iso_));
        return defaultEv100 - currentEv100;
    }

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
    float aperture_;
    float shutterSeconds_;
    float iso_;

    const float defaultAperture_;
    const float defaultShutterSeconds_;
    const float defaultIso_;

    const float flySpeedMetersPerSecond_;
    const float orbitSensitivityDegPerPixel_;

    bool orbiting_ = false;
    glm::vec3 pivot_{0.0F};
};

}  // namespace engine::scene
