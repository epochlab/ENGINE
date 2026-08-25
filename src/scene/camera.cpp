#include "engine/scene/camera.h"

#include <cmath>

#include <glm/gtc/matrix_transform.hpp>

namespace engine::scene {

namespace {

constexpr glm::vec3 kWorldUp{0.0F, 1.0F, 0.0F};

// Standard right-handed Euler-angle forward vector, parameterized so yaw=0/pitch=0 already points down -Z (this codebase's convention) without the usual -90-degree yaw offset used by e.g. LearnOpenGL's camera derivation.
//
// Derivation: rotating the default forward (0,0,-1) about world +Y by yaw using the standard right-handed rotation matrix
//   Ry(yaw) = [ cos(yaw), 0, sin(yaw); 0, 1, 0; -sin(yaw), 0, cos(yaw) ]
// gives (-sin(yaw), 0, -cos(yaw)); pitch then tilts that vector toward +Y (looking up, positive pitch) by scaling the horizontal components by cos(pitch) and introducing sin(pitch) vertically. The result is already unit length (cos^2(yaw)cos^2(pitch) + sin^2(yaw)cos^2(pitch) + sin^2(pitch) == 1); normalize() below is float-error defense only.
glm::vec3 forwardFromEuler(float yawRadians, float pitchRadians) {
    const float cosPitch = std::cos(pitchRadians);
    return glm::normalize(glm::vec3(-std::sin(yawRadians) * cosPitch, std::sin(pitchRadians),
                                     -std::cos(yawRadians) * cosPitch));
}

}  // namespace

Camera::Camera(const glm::vec3& position, float yawDegrees, float pitchDegrees, FilmBack filmBack,
               float focalLengthMm, float nearClip, float farClip, float aperture,
               float shutterSeconds, float iso)
    : position_(position),
      yawRadians_(glm::radians(yawDegrees)),
      pitchRadians_(glm::radians(pitchDegrees)),
      filmBack_(filmBack),
      focalLengthMm_(focalLengthMm),
      nearClip_(nearClip),
      farClip_(farClip),
      aperture_(aperture),
      shutterSeconds_(shutterSeconds),
      iso_(iso) {}

glm::vec3 Camera::forward() const {
    return forwardFromEuler(yawRadians_, pitchRadians_);
}

float Camera::verticalFovRadians() const {
    return 2.0F * std::atan(filmBack_.heightMm / (2.0F * focalLengthMm_));
}

glm::mat4 Camera::viewMatrix() const {
    const glm::vec3 forward = forwardFromEuler(yawRadians_, pitchRadians_);
    const glm::vec3 right = glm::normalize(glm::cross(forward, kWorldUp));
    const glm::vec3 up = glm::cross(right, forward);
    return glm::lookAt(position_, position_ + forward, up);
}

glm::mat4 Camera::projectionMatrix(float aspect) const {
    return glm::perspective(verticalFovRadians(), aspect, nearClip_, farClip_);
}

Ray Camera::primaryRay(float ndcX, float ndcY, float aspect) const {
    const glm::vec3 fwd = forwardFromEuler(yawRadians_, pitchRadians_);
    const glm::vec3 right = glm::normalize(glm::cross(fwd, kWorldUp));
    const glm::vec3 up = glm::cross(right, fwd);
    const float halfHeight = std::tan(verticalFovRadians() * 0.5F);
    const float halfWidth = halfHeight * aspect;
    const glm::vec3 dir =
        glm::normalize(fwd + (ndcX * halfWidth * right) + (ndcY * halfHeight * up));
    return Ray{position_, dir, nearClip_, farClip_};
}

float Camera::ev100() const {
    return std::log2((aperture_ * aperture_) / shutterSeconds_ * (100.0F / iso_));
}

float Camera::exposure() const {
    return 1.0F / (std::pow(2.0F, ev100()) * 1.2F);
}

}  // namespace engine::scene
