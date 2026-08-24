#pragma once

#include <glm/glm.hpp>

namespace engine::scene {

// A camera's pose, lens, and exposure, immutable once constructed. No
// input handling lives here — engine::scene::DebugCameraController owns
// pose mutation frame-to-frame (WASD/QE/R/orbit) and builds a fresh
// Camera each frame via its snapshot() method.
//
// Convention: right-handed, +Y up, -Z forward in view space. At yaw=0,
// pitch=0 the camera looks down world -Z with +Y up and +X right — this
// matches GLM's own conventions (glm::lookAt, glm::perspective) and
// glTF's coordinate system (relevant once Phase 2's glTF loader exists).
//
// Orientation is stored as yaw/pitch Euler angles rather than a
// quaternion: nothing on the roadmap needs roll, so the simpler
// representation is sufficient. Known limitation of this representation:
// at pitch = +/-90 degrees the forward vector becomes parallel to world
// up and the right/up basis degenerates; a future mutable debug camera
// should clamp pitch to avoid this rather than Camera guarding against
// it here.
class Camera {
public:
    // Sensor gate size in millimetres (e.g. {36.0F, 24.0F} for 35mm
    // full-frame), paired with focal length to derive vertical FOV.
    struct FilmBack {
        float widthMm;
        float heightMm;
    };

    // yawDegrees/pitchDegrees are authored in degrees (more ergonomic at
    // call sites than radians); converted once here and stored as radians,
    // since every consumer (the Euler-to-forward-vector trig) needs
    // radians.
    // aperture is an f-number (e.g. 2.8 for f/2.8), shutterSeconds is the
    // exposure time (e.g. 1/125), iso is sensor sensitivity — the standard
    // photographic exposure triangle, feeding ev100()/exposure() below.
    Camera(const glm::vec3& position, float yawDegrees, float pitchDegrees, FilmBack filmBack,
           float focalLengthMm, float nearClip, float farClip, float aperture,
           float shutterSeconds, float iso);

    [[nodiscard]] glm::vec3 position() const { return position_; }

    // Unit-length view direction derived from yaw/pitch — the same
    // quantity viewMatrix() computes internally, exposed for callers
    // (fly movement, orbit-pivot fallback) that need it independent of
    // the view matrix.
    [[nodiscard]] glm::vec3 forward() const;
    [[nodiscard]] FilmBack filmBack() const { return filmBack_; }
    [[nodiscard]] float focalLengthMm() const { return focalLengthMm_; }
    [[nodiscard]] float nearClip() const { return nearClip_; }
    [[nodiscard]] float farClip() const { return farClip_; }
    [[nodiscard]] float aperture() const { return aperture_; }
    [[nodiscard]] float shutterSeconds() const { return shutterSeconds_; }
    [[nodiscard]] float iso() const { return iso_; }

    // Vertical FOV derived from focal length + film back height, not set
    // directly — this is what a real lens/sensor combo actually
    // determines.
    [[nodiscard]] float verticalFovRadians() const;

    [[nodiscard]] glm::mat4 viewMatrix() const;
    [[nodiscard]] glm::mat4 projectionMatrix(float aspect) const;

    // Standard photographic exposure value at ISO 100 (log2 scale):
    // log2(aperture^2 / shutterSeconds * (100/iso)).
    [[nodiscard]] float ev100() const;

    // Linear scene-radiance multiplier derived from ev100() (the
    // Filament/Frostbite EV100 calibration convention), ready to apply
    // before Stage F's OCIO display transform.
    [[nodiscard]] float exposure() const;

private:
    glm::vec3 position_;
    float yawRadians_;
    float pitchRadians_;
    FilmBack filmBack_;
    float focalLengthMm_;
    float nearClip_;
    float farClip_;
    float aperture_;
    float shutterSeconds_;
    float iso_;
};

}  // namespace engine::scene
