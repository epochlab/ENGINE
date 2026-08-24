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
    const std::string_view lightBody = json::findObjectBody(*text, "light");
    const std::optional<glm::vec3> lightDirection = json::findVec3(lightBody, "direction");
    const std::optional<glm::vec3> lightColor = json::findVec3(lightBody, "color");
    const std::optional<double> windowWidth = json::findNumber(*text, "windowWidth");
    const std::optional<double> windowHeight = json::findNumber(*text, "windowHeight");
    const std::optional<double> initialAov = json::findNumber(*text, "initialAov");
    const std::optional<std::string> initialLutName = json::findString(*text, "initialLut");
    const std::optional<engine::gfx::OcioDisplayTransform::Lut> initialLut =
        initialLutName.has_value() ? parseLut(*initialLutName) : std::nullopt;

    if (!gltfPath.has_value() || !lightDirection.has_value() || !lightColor.has_value() ||
        !windowWidth.has_value() || !windowHeight.has_value() || !initialAov.has_value() ||
        !initialLut.has_value()) {
        std::cerr << "loadSceneConfig: " << path << " is missing one or more required fields\n";
        return std::nullopt;
    }

    return SceneConfig{
        *gltfPath,
        *lightDirection,
        *lightColor,
        static_cast<int>(*windowWidth),
        static_cast<int>(*windowHeight),
        static_cast<int>(*initialAov),
        *initialLut,
    };
}

}  // namespace engine::config
