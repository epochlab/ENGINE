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

        std::map<std::string, std::string> materialOverrides;
        if (const auto it = j.find("materialOverrides"); it != j.end()) {
            materialOverrides = it->get<std::map<std::string, std::string>>();
        }

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
            j.at("materialPath").get<std::string>(),
            std::move(materialOverrides),
        };
    } catch (const nlohmann::json::exception& e) {
        std::cerr << "loadSceneConfig: " << path << ": " << e.what() << '\n';
        return std::nullopt;
    }
}

std::optional<MaterialConfig> loadMaterialConfig(const std::string& path) {
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        std::cerr << "loadMaterialConfig: could not read " << path << '\n';
        return std::nullopt;
    }

    try {
        nlohmann::json j;
        file >> j;

        return MaterialConfig{
            j.at("bumpStrength").get<float>(),
            j.at("roughnessMin").get<float>(),
            j.at("roughnessMax").get<float>(),
            j.at("diffuseColour").get<glm::vec3>(),
            j.value("ior", 1.5F),
            j.value("abbe", 0.0F),
            j.value("transmissionFactor", 0.0F),
            j.value("metallicFactor", 0.0F),
            j.at("roughnessFactor").get<float>(),
            j.value("diffuseRoughness", 0.0F),
            j.value("transmissionColor", glm::vec3(1.0F)),
            j.value("transmissionDepth", 1.0F),
            j.value("edgeTint", glm::vec3(1.0F)),
        };
    } catch (const nlohmann::json::exception& e) {
        std::cerr << "loadMaterialConfig: " << path << ": " << e.what() << '\n';
        return std::nullopt;
    }
}

}  // namespace engine::config
