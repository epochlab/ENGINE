#include "engine/config/scene_config.h"

#include <iostream>

#include "json_scan.h"

namespace engine::config {

std::optional<SceneConfig> loadSceneConfig(const std::string& path) {
    const std::optional<std::string> text = json::readFile(path);
    if (!text.has_value()) {
        std::cerr << "loadSceneConfig: could not read " << path << '\n';
        return std::nullopt;
    }

    const std::optional<std::string> gltfPath = json::findString(*text, "gltfPath");
    const std::optional<std::string> texturePath = json::findString(*text, "texturePath");
    const std::optional<std::string> materialPath = json::findString(*text, "materialPath");
    const std::optional<glm::vec3> position = json::findVec3(*text, "position");
    const std::optional<glm::vec3> rotationDegrees = json::findVec3(*text, "rotationDegrees");

    if (!gltfPath.has_value() || !texturePath.has_value() || !materialPath.has_value() ||
        !position.has_value() || !rotationDegrees.has_value()) {
        std::cerr << "loadSceneConfig: " << path << " is missing one or more required fields\n";
        return std::nullopt;
    }

    return SceneConfig{
        *gltfPath,
        *texturePath,
        *materialPath,
        *position,
        *rotationDegrees,
    };
}

}  // namespace engine::config
