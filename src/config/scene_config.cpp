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

        const nlohmann::json& model = j.at("model");
        const nlohmann::json& environment = j.at("environment");
        const nlohmann::json& material = j.at("material");

        return SceneConfig{
            ModelConfig{
                model.at("gltfPath").get<std::string>(),
                model.at("texturePath").get<std::string>(),
                model.at("position").get<glm::vec3>(),
                model.at("rotation").get<glm::vec3>(),
            },
            EnvironmentConfig{
                environment.at("hdriPath").get<std::string>(),
            },
            MaterialConfig{
                material.at("bumpStrength").get<float>(),
                material.at("roughnessMin").get<float>(),
                material.at("roughnessMax").get<float>(),
                material.at("diffuseColour").get<glm::vec3>(),
                material.at("ior").get<float>(),
                material.at("transmissionFactor").get<float>(),
                material.at("metallicFactor").get<float>(),
                material.at("roughnessFactor").get<float>(),
            },
        };
    } catch (const nlohmann::json::exception& e) {
        std::cerr << "loadSceneConfig: " << path << ": " << e.what() << '\n';
        return std::nullopt;
    }
}

}  // namespace engine::config
