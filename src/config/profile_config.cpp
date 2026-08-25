#include "engine/config/profile_config.h"

#include <iostream>

#include "json_scan.h"

namespace engine::config {

std::optional<ProfileConfig> loadProfileConfig(const std::string& path) {
    const std::optional<std::string> text = json::readFile(path);
    if (!text.has_value()) {
        std::cerr << "loadProfileConfig: could not read " << path << '\n';
        return std::nullopt;
    }

    const std::optional<glm::vec3> position = json::findVec3(*text, "position");
    const std::optional<double> yawDegrees = json::findNumber(*text, "yawDegrees");
    const std::optional<double> pitchDegrees = json::findNumber(*text, "pitchDegrees");
    const std::string_view filmBackBody = json::findObjectBody(*text, "filmBack");
    const std::optional<double> filmBackWidthMm = json::findNumber(filmBackBody, "widthMm");
    const std::optional<double> filmBackHeightMm = json::findNumber(filmBackBody, "heightMm");
    const std::optional<double> focalLengthMm = json::findNumber(*text, "focalLengthMm");
    const std::optional<double> nearClip = json::findNumber(*text, "nearClip");
    const std::optional<double> farClip = json::findNumber(*text, "farClip");
    const std::optional<double> aperture = json::findNumber(*text, "aperture");
    const std::optional<double> shutterSeconds = json::findNumber(*text, "shutterSeconds");
    const std::optional<double> iso = json::findNumber(*text, "iso");
    const std::optional<double> flySpeed = json::findNumber(*text, "flySpeedMetersPerSecond");
    const std::optional<double> orbitSensitivity =
        json::findNumber(*text, "orbitSensitivityDegPerPixel");

    if (!position || !yawDegrees || !pitchDegrees || !filmBackWidthMm || !filmBackHeightMm ||
        !focalLengthMm || !nearClip || !farClip || !aperture || !shutterSeconds || !iso ||
        !flySpeed || !orbitSensitivity) {
        std::cerr << "loadProfileConfig: " << path << " is missing one or more required fields\n";
        return std::nullopt;
    }
    // These feed Camera::verticalFovRadians()/ev100() as denominators or bases of a physically meaningful quantity -- a zero/negative value would silently produce inf/NaN there instead of failing at this asset-load boundary.
    if (*filmBackHeightMm <= 0.0 || *focalLengthMm <= 0.0 || *aperture <= 0.0 ||
        *shutterSeconds <= 0.0 || *iso <= 0.0) {
        std::cerr << "loadProfileConfig: " << path
                   << " has a non-positive filmBack/focalLengthMm/aperture/shutterSeconds/iso\n";
        return std::nullopt;
    }

    return ProfileConfig{
        *position,
        static_cast<float>(*yawDegrees),
        static_cast<float>(*pitchDegrees),
        engine::scene::Camera::FilmBack{static_cast<float>(*filmBackWidthMm),
                                         static_cast<float>(*filmBackHeightMm)},
        static_cast<float>(*focalLengthMm),
        static_cast<float>(*nearClip),
        static_cast<float>(*farClip),
        static_cast<float>(*aperture),
        static_cast<float>(*shutterSeconds),
        static_cast<float>(*iso),
        static_cast<float>(*flySpeed),
        static_cast<float>(*orbitSensitivity),
    };
}

}  // namespace engine::config
