#pragma once

#include <string>

#include <glm/glm.hpp>

#include "pathtracer/scene/ray_types.h"

namespace pathtracer::scene {

// A camera's pose, lens and exposure, immutable once constructed; DebugCameraController owns mutation and builds a
// fresh one each frame. Right-handed, +Y up, -Z forward, matching GLM and glTF; at yaw=pitch=0 it looks down world -Z.
// Orientation is yaw/pitch Euler, so pitch +/-90 degenerates the basis -- DebugCameraController clamps to +/-89.
class Camera {
public:
    // Sensor gate size in mm ({36.0F, 24.0F} for 35mm full-frame), paired with focal length to derive vertical FOV.
    struct FilmBack {
        float widthMm;
        float heightMm;
    };

    // A named, real-world FilmBack ("ARRI Alexa 65"): the HUD preset dropdown and assets/config/camera.json key off name.
    struct FilmBackPreset {
        std::string name;
        FilmBack filmBack;
    };

    // Authored in degrees, more ergonomic at call sites, converted once here and stored as radians because every
    // consumer is trigonometric.
    Camera(const glm::vec3& position, float yawDegrees, float pitchDegrees, FilmBack filmBack,
           float focalLengthMm, float nearClip, float farClip, float aperture,
           float shutterSeconds, float iso);

    [[nodiscard]] glm::vec3 position() const { return position_; }

    // Orientation in the degrees it was authored in, completing the accessors that return every constructor argument
    // as given. Stored as radians because every consumer of the angles is trigonometric.
    [[nodiscard]] float yawDegrees() const { return glm::degrees(yawRadians_); }
    [[nodiscard]] float pitchDegrees() const { return glm::degrees(pitchRadians_); }

    // Unit-length view direction derived from yaw/pitch.
    [[nodiscard]] glm::vec3 forward() const;
    [[nodiscard]] FilmBack filmBack() const { return filmBack_; }
    [[nodiscard]] float focalLengthMm() const { return focalLengthMm_; }
    [[nodiscard]] float nearClip() const { return nearClip_; }
    [[nodiscard]] float farClip() const { return farClip_; }
    [[nodiscard]] float aperture() const { return aperture_; }
    [[nodiscard]] float shutterSeconds() const { return shutterSeconds_; }
    [[nodiscard]] float iso() const { return iso_; }

    // Vertical FOV from focal length and film-back height, not set directly: what a real lens and sensor determine.
    [[nodiscard]] float verticalFovRadians() const;

    // Orthonormal forward/right/up and view-plane half-extents: everything primaryRay() needs bar the per-pixel ndc
    // weight. Exposed so a screen-space projector can share the basis rather than rebuild it.
    struct ViewBasis {
        glm::vec3 forward;
        glm::vec3 right;
        glm::vec3 up;
        float halfWidth;
        float halfHeight;
    };
    [[nodiscard]] ViewBasis viewBasis(float aspect) const;

    // Pinhole primary ray for a point in normalized device coordinates (ndcX/ndcY in [-1,1], +Y up). tMin/tMax are nearClip()/farClip().
    [[nodiscard]] Ray primaryRay(float ndcX, float ndcY, float aspect) const;

    // Same ray from a basis the caller already built. The aspect-taking overload rebuilds it every call -- two sin,
    // two cos, an atan, a tan, two normalize and two cross -- which is constant across a whole image.
    [[nodiscard]] Ray primaryRay(const ViewBasis& basis, float ndcX, float ndcY) const;

    // Photographic exposure value at ISO 100 (log2): log2(aperture^2 / shutterSeconds * (100/iso)). The static
    // overload is the single definition of the formula, callable without a Camera.
    [[nodiscard]] float ev100() const;
    [[nodiscard]] static float ev100(float aperture, float shutterSeconds, float iso);

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

}  // namespace pathtracer::scene
