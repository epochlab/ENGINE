#include "engine/config/material_config.h"

#include <iostream>

#include "json_scan.h"

namespace engine::config {

std::optional<MaterialConfig> loadMaterialConfig(const std::string& path) {
    const std::optional<std::string> text = json::readFile(path);
    if (!text.has_value()) {
        std::cerr << "loadMaterialConfig: could not read " << path << '\n';
        return std::nullopt;
    }

    const std::optional<double> bumpStrength = json::findNumber(*text, "bumpStrength");
    const std::optional<double> roughnessMin = json::findNumber(*text, "roughnessMin");
    const std::optional<double> roughnessMax = json::findNumber(*text, "roughnessMax");
    const std::optional<glm::vec3> diffuseColour = json::findVec3(*text, "diffuseColour");
    const std::optional<double> ior = json::findNumber(*text, "ior");
    const std::optional<double> transmissionFactor = json::findNumber(*text, "transmissionFactor");
    const std::optional<double> metallicFactor = json::findNumber(*text, "metallicFactor");
    const std::optional<double> roughnessFactor = json::findNumber(*text, "roughnessFactor");

    if (!bumpStrength.has_value() || !roughnessMin.has_value() || !roughnessMax.has_value() ||
        !diffuseColour.has_value() || !ior.has_value() ||
        !transmissionFactor.has_value() || !metallicFactor.has_value() ||
        !roughnessFactor.has_value()) {
        std::cerr << "loadMaterialConfig: " << path << " is missing one or more required fields\n";
        return std::nullopt;
    }

    return MaterialConfig{
        static_cast<float>(*bumpStrength),
        static_cast<float>(*roughnessMin),
        static_cast<float>(*roughnessMax),
        *diffuseColour,
        static_cast<float>(*ior),
        static_cast<float>(*transmissionFactor),
        static_cast<float>(*metallicFactor),
        static_cast<float>(*roughnessFactor),
    };
}

}  // namespace engine::config
