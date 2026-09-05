#include "engine/config/scene_config.h"

#include <cmath>
#include <cstddef>
#include <fstream>
#include <iostream>

#include <nlohmann/json.hpp>

#include "json_glm.h"

namespace engine::config {

namespace {

// Parses and validates the optional "lights" array. Separate from loadSceneConfig so each stays one screen, and because every check here is a scene-authoring boundary: a bad value must fail the load loudly rather than reach the sampler.
std::optional<std::vector<QuadLightConfig>> parseQuadLights(const nlohmann::json& j, const std::string& path) {
    std::vector<QuadLightConfig> lights;
    const auto it = j.find("lights");
    if (it == j.end()) {
        return lights;
    }
    for (const nlohmann::json& light : *it) {
        const std::size_t index = lights.size();
        // Dispatched on rather than ignored: an unrecognised type must not silently load as a quad, and this is where a future disk/sphere light branches.
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
        // A zero-length edge subtends no solid angle: the light would sit in the BVH as degenerate geometry while emitting nothing NEE could ever sample.
        if (!(length0 > 0.0F) || !(length1 > 0.0F)) {
            std::cerr << "loadSceneConfig: " << path << ": lights[" << index << "] has a zero-length edge0/edge1\n";
            return std::nullopt;
        }
        // Urena/Fajardo/King's sampler (light.h) builds its frame from normalize(edge0)/normalize(edge1) and reconstructs sample points as p + xu*x + yv*y + z0*z, which lands on the authored quad only where those edges are perpendicular; a skewed quad is sampled off the geometry the BVH actually holds, so direction, distance and pdf all describe a surface that is not there -- wrong energy, no error, no test failure.
        // Bound is the sampler's own worst-case positional error, not an authoring tolerance: the far corner deviates by |edge1|*|cos|, so 1e-4 admits at most 1e-4 of that edge's own length -- scale-free, and keeping that deviation under kRayEpsilon (path_tracer.cpp, 1e-4 world units) for any light smaller than unit scale, so a sampled point never lands further off the surface than the ray offset that has to clear it. Relaxing to 1e-3 would put it past that epsilon and let the light self-occlude.
        // Measured over 20k random 3D rotations of a 0.3 x 0.2 rectangle, worst |cos| by authored decimal places: 6.1e-3 at 3dp, 5.7e-4 at 4dp, 6.0e-6 at 6dp, 9.9e-8 at 8dp. So an off-axis quad must be authored to 6+ decimals -- what any DCC exporter writes -- and hand-rounding a rotated one to 4dp is rejected by design rather than silently sampled off its own geometry.
        constexpr float kMaxEdgeCosine = 1e-4F;
        if (const float cosEdges = glm::dot(quad.edge0 / length0, quad.edge1 / length1); std::fabs(cosEdges) > kMaxEdgeCosine) {
            std::cerr << "loadSceneConfig: " << path << ": lights[" << index << "] has non-perpendicular edge0/edge1 (cos " << cosEdges << ")\n";
            return std::nullopt;
        }
        // Negative radiance is physically unrepresentable and would propagate through NEE into the accumulator as a permanent negative bias no downstream clamp removes.
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

}  // namespace engine::config
