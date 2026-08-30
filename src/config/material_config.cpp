#include "engine/config/material_config.h"

#include <fstream>
#include <iostream>

#include <nlohmann/json.hpp>

#include "json_glm.h"

namespace engine::config {

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
            j.at("ior").get<float>(),
            j.at("transmissionFactor").get<float>(),
            j.at("metallicFactor").get<float>(),
            j.at("roughnessFactor").get<float>(),
        };
    } catch (const nlohmann::json::exception& e) {
        std::cerr << "loadMaterialConfig: " << path << ": " << e.what() << '\n';
        return std::nullopt;
    }
}

}  // namespace engine::config
