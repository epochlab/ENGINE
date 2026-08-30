#include "engine/config/scene_config.h"

#include <fstream>
#include <iostream>

#include <nlohmann/json.hpp>

#include "json_glm.h"

namespace engine::config {

std::optional<SceneConfig> loadSceneConfig(const std::string& path) {
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        std::cerr << "loadSceneConfig: could not read " << path << '\n';
        return std::nullopt;
    }

    try {
        nlohmann::json j;
        file >> j;

        return SceneConfig{
            j.at("gltfPath").get<std::string>(),
            j.at("texturePath").get<std::string>(),
            j.at("materialPath").get<std::string>(),
            j.at("position").get<glm::vec3>(),
            j.at("rotationDegrees").get<glm::vec3>(),
            j.at("hdriPath").get<std::string>(),
        };
    } catch (const nlohmann::json::exception& e) {
        std::cerr << "loadSceneConfig: " << path << ": " << e.what() << '\n';
        return std::nullopt;
    }
}

}  // namespace engine::config
