#include "engine/config/scene_config.h"

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

std::optional<SceneConfig> loadSceneConfig(const std::string& path) {
    const std::optional<std::string> text = json::readFile(path);
    if (!text.has_value()) {
        std::cerr << "loadSceneConfig: could not read " << path << '\n';
        return std::nullopt;
    }

    const std::optional<std::string> gltfPath = json::findString(*text, "gltfPath");
    const std::optional<std::string> texturePath = json::findString(*text, "texturePath");
    const std::optional<std::string> hdriPath = json::findString(*text, "hdriPath");
    const std::optional<glm::vec3> position = json::findVec3(*text, "position");
    const std::optional<glm::vec3> rotationDegrees = json::findVec3(*text, "rotationDegrees");
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

    if (!gltfPath.has_value() || !texturePath.has_value() || !hdriPath.has_value() ||
        !position.has_value() || !rotationDegrees.has_value() || !windowWidth.has_value() ||
        !windowHeight.has_value() || !defaultAov.has_value() || !defaultLut.has_value() ||
        !samplesPerPixel.has_value() || !maxBounces.has_value() ||
        !russianRouletteStartBounce.has_value() || !maxSamples.has_value()) {
        std::cerr << "loadSceneConfig: " << path << " is missing one or more required fields\n";
        return std::nullopt;
    }

    return SceneConfig{
        *gltfPath,
        *texturePath,
        *hdriPath,
        *position,
        *rotationDegrees,
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
