#include "engine/config/profile_config.h"

#include <iostream>

#include "json_scan.h"

namespace engine::config {

namespace {

std::optional<engine::gfx::OcioDisplayTransform::Lut> parseLut(const std::string& name) {
    using Lut = engine::gfx::OcioDisplayTransform::Lut;
    if (name == "sRGB") {
        return Lut::SRGB;
    }
    if (name == "Rec709") {
        return Lut::Rec709;
    }
    if (name == "Raw") {
        return Lut::Raw;
    }
    return std::nullopt;
}

}  // namespace

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
    const std::optional<std::string> hdriPath = json::findString(*text, "hdriPath");
    const std::optional<double> windowWidth = json::findNumber(*text, "windowWidth");
    const std::optional<double> windowHeight = json::findNumber(*text, "windowHeight");
    const std::optional<double> defaultAov = json::findNumber(*text, "defaultAOV");
    const std::optional<std::string> defaultLutName = json::findString(*text, "defaultLUT");
    const std::optional<engine::gfx::OcioDisplayTransform::Lut> defaultLut =
        defaultLutName.has_value() ? parseLut(*defaultLutName) : std::nullopt;
    const std::optional<double> samplesPerPixel = json::findNumber(*text, "samplesPerPixel");
    const std::optional<double> maxBounces = json::findNumber(*text, "maxBounces");
    const std::optional<double> russianRouletteStartBounce =
        json::findNumber(*text, "russianRouletteStartBounce");
    const std::optional<double> maxSamples = json::findNumber(*text, "maxSamples");

    if (!position || !yawDegrees || !pitchDegrees || !filmBackWidthMm || !filmBackHeightMm ||
        !focalLengthMm || !nearClip || !farClip || !aperture || !shutterSeconds || !iso ||
        !flySpeed || !orbitSensitivity || !hdriPath.has_value() || !windowWidth.has_value() ||
        !windowHeight.has_value() || !defaultAov.has_value() || !defaultLut.has_value() ||
        !samplesPerPixel.has_value() || !maxBounces.has_value() ||
        !russianRouletteStartBounce.has_value() || !maxSamples.has_value()) {
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
        *hdriPath,
        static_cast<int>(*windowWidth),
        static_cast<int>(*windowHeight),
        static_cast<int>(*defaultAov),
        *defaultLut,
        static_cast<int>(*samplesPerPixel),
        static_cast<int>(*maxBounces),
        static_cast<int>(*russianRouletteStartBounce),
        static_cast<int>(*maxSamples),
    };
}

}  // namespace engine::config
