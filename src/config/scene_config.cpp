#include "pathtracer/config/scene_config.h"

#include <cmath>
#include <cstddef>
#include <fstream>
#include <iostream>

#include <nlohmann/json.hpp>

#include "json_glm.h"

namespace pathtracer::config {

namespace {

// Parses and validates the optional "lights" array. Separate so each stays one screen, and every check is an authoring boundary.
std::optional<std::vector<QuadLightConfig>> parseQuadLights(const nlohmann::json& j, const std::string& path) {
    std::vector<QuadLightConfig> lights;
    const auto it = j.find("lights");
    if (it == j.end()) {
        return lights;
    }
    for (const nlohmann::json& light : *it) {
        const std::size_t index = lights.size();
        // Dispatched on rather than ignored: an unrecognised type must not silently load as a quad. A future light type branches here.
        if (const auto type = light.at("type").get<std::string>(); type != "quad") {
            std::cerr << "loadSceneConfig: " << path << ": lights[" << index << "] has unknown type '" << type << "', only 'quad' exists\n";
            return std::nullopt;
        }
        const QuadLightConfig quad{
            light.at("origin").get<glm::vec3>(),
            light.at("edge0").get<glm::vec3>(),
            light.at("edge1").get<glm::vec3>(),
            light.at("color").get<glm::vec3>(),
            light.at("intensity").get<float>(),
            light.value("twoSided", false),
        };
        const float length0 = glm::length(quad.edge0);
        const float length1 = glm::length(quad.edge1);
        // A zero-length edge subtends no solid angle: degenerate geometry in the BVH emitting nothing NEE could sample.
        if (!(length0 > 0.0F) || !(length1 > 0.0F)) {
            std::cerr << "loadSceneConfig: " << path << ": lights[" << index << "] has a zero-length edge0/edge1\n";
            return std::nullopt;
        }
        // The sampler's frame is normalize(edge0)/normalize(edge1); bound is its worst-case error, |cos| 6.1e-3 at 3dp and 9.9e-8 at 8dp.
        constexpr float kMaxEdgeCosine = 1e-4F;
        if (const float cosEdges = glm::dot(quad.edge0 / length0, quad.edge1 / length1); std::fabs(cosEdges) > kMaxEdgeCosine) {
            std::cerr << "loadSceneConfig: " << path << ": lights[" << index << "] has non-perpendicular edge0/edge1 (cos " << cosEdges << ")\n";
            return std::nullopt;
        }
        // Negative radiance is unrepresentable and would propagate through NEE as a permanent bias no downstream clamp removes.
        if (quad.intensity < 0.0F || glm::any(glm::lessThan(quad.color, glm::vec3(0.0F)))) {
            std::cerr << "loadSceneConfig: " << path << ": lights[" << index << "] has a negative intensity/color\n";
            return std::nullopt;
        }
        lights.push_back(quad);
    }
    return lights;
}

}  // namespace

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

        std::optional<std::vector<QuadLightConfig>> lights = parseQuadLights(j, path);
        if (!lights) {
            return std::nullopt;
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
                environment.value("lightEnabled", true),
            },
            j.at("materialPath").get<std::string>(),
            std::move(materialOverrides),
            std::move(*lights),
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
            j.value("transmissionDepth", 0.0F),
            j.value("edgeTint", glm::vec3(1.0F)),
        };
    } catch (const nlohmann::json::exception& e) {
        std::cerr << "loadMaterialConfig: " << path << ": " << e.what() << '\n';
        return std::nullopt;
    }
}

}  // namespace pathtracer::config
