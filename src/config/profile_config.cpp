#include "engine/config/profile_config.h"

#include <fstream>
#include <iostream>

#include <nlohmann/json.hpp>

#include "json_glm.h"

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
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        std::cerr << "loadProfileConfig: could not read " << path << '\n';
        return std::nullopt;
    }

    try {
        nlohmann::json j;
        file >> j;

        const std::string defaultLutName = j.at("defaultLUT").get<std::string>();
        const std::optional<engine::gfx::OcioDisplayTransform::Lut> defaultLut =
            parseLut(defaultLutName);
        if (!defaultLut.has_value()) {
            std::cerr << "loadProfileConfig: " << path << " has an unrecognised defaultLUT \""
                       << defaultLutName << "\"\n";
            return std::nullopt;
        }

        const int windowWidth = j.at("windowWidth").get<int>();
        const int windowHeight = j.at("windowHeight").get<int>();
        const glm::vec3 position = j.at("position").get<glm::vec3>();
        const float yawDegrees = j.at("yawDegrees").get<float>();
        const float pitchDegrees = j.at("pitchDegrees").get<float>();
        const nlohmann::json& filmBackJson = j.at("filmBack");
        const engine::scene::Camera::FilmBack filmBack{filmBackJson.at("widthMm").get<float>(),
                                                         filmBackJson.at("heightMm").get<float>()};
        const float focalLengthMm = j.at("focalLengthMm").get<float>();
        const float nearClip = j.at("nearClip").get<float>();
        const float farClip = j.at("farClip").get<float>();
        const float aperture = j.at("aperture").get<float>();
        const float shutterSeconds = j.at("shutterSeconds").get<float>();
        const float iso = j.at("iso").get<float>();
        const float flySpeed = j.at("flySpeedMetersPerSecond").get<float>();
        const float orbitSensitivity = j.at("orbitSensitivityDegPerPixel").get<float>();
        const float renderScale = j.at("renderScale").get<float>();
        const float interactiveRenderScale = j.at("interactiveRenderScale").get<float>();
        const int defaultAov = j.at("defaultAOV").get<int>();
        const int samplesPerPixel = j.at("samplesPerPixel").get<int>();
        const int maxBounces = j.at("maxBounces").get<int>();
        const int russianRouletteStartBounce = j.at("russianRouletteStartBounce").get<int>();
        const int maxSamples = j.at("maxSamples").get<int>();

        // These feed Camera::verticalFovRadians()/ev100() as denominators or bases of a physically meaningful quantity -- a zero/negative value would silently produce inf/NaN there instead of failing at this asset-load boundary.
        if (filmBack.heightMm <= 0.0F || focalLengthMm <= 0.0F || aperture <= 0.0F ||
            shutterSeconds <= 0.0F || iso <= 0.0F) {
            std::cerr << "loadProfileConfig: " << path
                       << " has a non-positive filmBack/focalLengthMm/aperture/shutterSeconds/iso\n";
            return std::nullopt;
        }
        // Bounded at (0,1] rather than merely positive: above 1 would render above the framebuffer and hand the display blit a downscale it has no filter for, and at or below 0 the render target collapses.
        if (renderScale <= 0.0F || renderScale > 1.0F || interactiveRenderScale <= 0.0F ||
            interactiveRenderScale > 1.0F) {
            std::cerr << "loadProfileConfig: " << path
                       << " has a renderScale/interactiveRenderScale outside (0,1]\n";
            return std::nullopt;
        }

        return ProfileConfig{
            windowWidth,
            windowHeight,
            position,
            yawDegrees,
            pitchDegrees,
            filmBack,
            focalLengthMm,
            nearClip,
            farClip,
            aperture,
            shutterSeconds,
            iso,
            flySpeed,
            orbitSensitivity,
            renderScale,
            interactiveRenderScale,
            defaultAov,
            *defaultLut,
            samplesPerPixel,
            maxBounces,
            russianRouletteStartBounce,
            maxSamples,
        };
    } catch (const nlohmann::json::exception& e) {
        std::cerr << "loadProfileConfig: " << path << ": " << e.what() << '\n';
        return std::nullopt;
    }
}

}  // namespace engine::config
