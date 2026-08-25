#include "engine/config/scene_config.h"

#include <iostream>
#include <utility>

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

// Directional lights read "direction" (unit vector toward the light, range unused); point lights read "position" and "range". Returns nullopt on any missing/malformed field, matching loadSceneConfig's all-required-fields convention for the rest of the file.
std::optional<Light> parseLight(std::string_view body) {
    const std::optional<std::string> typeName = json::findString(body, "type");
    const std::optional<glm::vec3> color = json::findVec3(body, "color");
    if (!typeName.has_value() || !color.has_value()) {
        return std::nullopt;
    }

    if (*typeName == "directional") {
        const std::optional<glm::vec3> direction = json::findVec3(body, "direction");
        if (!direction.has_value()) {
            return std::nullopt;
        }
        return Light{Light::Type::Directional, *direction, *color, 0.0F};
    }
    if (*typeName == "point") {
        const std::optional<glm::vec3> position = json::findVec3(body, "position");
        const std::optional<double> range = json::findNumber(body, "range");
        if (!position.has_value() || !range.has_value()) {
            return std::nullopt;
        }
        return Light{Light::Type::Point, *position, *color, static_cast<float>(*range)};
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
    const std::vector<std::string_view> lightBodies = json::findObjectArrayBodies(*text, "lights");
    const std::optional<double> windowWidth = json::findNumber(*text, "windowWidth");
    const std::optional<double> windowHeight = json::findNumber(*text, "windowHeight");
    const std::optional<double> initialAov = json::findNumber(*text, "initialAov");
    const std::optional<std::string> initialLutName = json::findString(*text, "initialLut");
    const std::optional<engine::gfx::OcioDisplayTransform::Lut> initialLut =
        initialLutName.has_value() ? parseLut(*initialLutName) : std::nullopt;

    if (!gltfPath.has_value() || !windowWidth.has_value() || !windowHeight.has_value() ||
        !initialAov.has_value() || !initialLut.has_value()) {
        std::cerr << "loadSceneConfig: " << path << " is missing one or more required fields\n";
        return std::nullopt;
    }
    // An empty (or absent) lights[] is a legitimate IBL-only scene, not a load failure -- unlike gltfPath/window size/AOV/LUT above, there's no reasonable "this must be present" invariant for punctual lights.

    std::vector<Light> lights;
    lights.reserve(lightBodies.size());
    for (const std::string_view body : lightBodies) {
        const std::optional<Light> light = parseLight(body);
        if (!light.has_value()) {
            std::cerr << "loadSceneConfig: " << path << " has a malformed lights[] entry\n";
            return std::nullopt;
        }
        lights.push_back(*light);
    }

    return SceneConfig{
        *gltfPath,
        std::move(lights),
        static_cast<int>(*windowWidth),
        static_cast<int>(*windowHeight),
        static_cast<int>(*initialAov),
        *initialLut,
    };
}

}  // namespace engine::config
