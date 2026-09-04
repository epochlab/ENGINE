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

        const nlohmann::json& window = j.at("window");
        const nlohmann::json& camera = j.at("camera");
        const nlohmann::json& controls = j.at("controls");
        const nlohmann::json& render = j.at("render");
        const nlohmann::json& pathTracer = j.at("pathTracer");

        const std::string defaultLutName = render.at("defaultLUT").get<std::string>();
        const std::optional<engine::gfx::OcioDisplayTransform::Lut> defaultLut =
            parseLut(defaultLutName);
        if (!defaultLut.has_value()) {
            std::cerr << "loadProfileConfig: " << path << " has an unrecognised defaultLUT \""
                       << defaultLutName << "\"\n";
            return std::nullopt;
        }

        const int windowWidth = window.at("width").get<int>();
        const int windowHeight = window.at("height").get<int>();
        const glm::vec3 position = camera.at("position").get<glm::vec3>();
        const float yawDegrees = camera.at("yawDegrees").get<float>();
        const float pitchDegrees = camera.at("pitchDegrees").get<float>();
        const std::string defaultFilmBackPresetName = camera.at("filmBackPreset").get<std::string>();
        const float focalLengthMm = camera.at("focalLengthMm").get<float>();
        const float nearClip = camera.at("nearClip").get<float>();
        const float farClip = camera.at("farClip").get<float>();
        const float aperture = camera.at("aperture").get<float>();
        const float shutterSeconds = camera.at("shutterSeconds").get<float>();
        const float iso = camera.at("iso").get<float>();
        const float flySpeed = controls.at("flySpeedMetersPerSecond").get<float>();
        const float orbitSensitivity = controls.at("orbitSensitivityDegPerPixel").get<float>();
        const float renderScale = render.at("renderScale").get<float>();
        const float interactiveRenderScale = render.at("interactiveRenderScale").get<float>();
        const int defaultAov = render.at("defaultAOV").get<int>();
        const int samplesPerPixel = pathTracer.at("samplesPerPixel").get<int>();
        const int maxBounces = pathTracer.at("maxBounces").get<int>();
        const int russianRouletteStartBounce = pathTracer.at("russianRouletteStartBounce").get<int>();
        const int maxSamples = pathTracer.at("maxSamples").get<int>();

        // These feed Camera::verticalFovRadians()/ev100() as denominators or bases of a physically meaningful quantity -- a zero/negative value would silently produce inf/NaN there instead of failing at this asset-load boundary. filmBack itself is validated by loadFilmBackPresets, not here -- this function never loads that file.
        if (focalLengthMm <= 0.0F || aperture <= 0.0F || shutterSeconds <= 0.0F || iso <= 0.0F) {
            std::cerr << "loadProfileConfig: " << path
                       << " has a non-positive focalLengthMm/aperture/shutterSeconds/iso\n";
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
            WindowConfig{
                windowWidth,
                windowHeight,
            },
            CameraConfig{
                position,
                yawDegrees,
                pitchDegrees,
                defaultFilmBackPresetName,
                focalLengthMm,
                nearClip,
                farClip,
                aperture,
                shutterSeconds,
                iso,
            },
            ControlsConfig{
                flySpeed,
                orbitSensitivity,
            },
            RenderConfig{
                renderScale,
                interactiveRenderScale,
                defaultAov,
                *defaultLut,
            },
            PathTracerConfig{
                samplesPerPixel,
                maxBounces,
                russianRouletteStartBounce,
                maxSamples,
            },
        };
    } catch (const nlohmann::json::exception& e) {
        std::cerr << "loadProfileConfig: " << path << ": " << e.what() << '\n';
        return std::nullopt;
    }
}

std::optional<std::vector<engine::scene::Camera::FilmBackPreset>> loadFilmBackPresets(
    const std::string& path) {
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        std::cerr << "loadFilmBackPresets: could not read " << path << '\n';
        return std::nullopt;
    }

    try {
        nlohmann::json j;
        file >> j;

        std::vector<engine::scene::Camera::FilmBackPreset> presets;
        presets.reserve(j.size());
        for (const nlohmann::json& presetJson : j) {
            std::string name = presetJson.at("name").get<std::string>();
            const float widthMm = presetJson.at("widthMm").get<float>();
            const float heightMm = presetJson.at("heightMm").get<float>();
            // widthMm/heightMm are physical sensor dimensions -- feed Camera::verticalFovRadians() and the HUD's aspect-ratio display as denominators, so a non-positive value must fail here rather than surface as inf/NaN later.
            if (widthMm <= 0.0F || heightMm <= 0.0F) {
                std::cerr << "loadFilmBackPresets: " << path << " has a non-positive filmBack for \""
                           << name << "\"\n";
                return std::nullopt;
            }
            presets.push_back({std::move(name), engine::scene::Camera::FilmBack{widthMm, heightMm}});
        }
        return presets;
    } catch (const nlohmann::json::exception& e) {
        std::cerr << "loadFilmBackPresets: " << path << ": " << e.what() << '\n';
        return std::nullopt;
    }
}

}  // namespace engine::config
