#include "engine/scene/material_binding.h"

#include <iostream>
#include <utility>

namespace engine::scene {

std::optional<std::vector<PathTraceSettings>> resolvePerInstanceSettings(
    const PathTraceSettings& base, const std::vector<MeshInstance>& instances,
    const std::map<std::string, std::string>& materialOverrides, const std::string& assetRoot) {
    std::map<std::string, engine::config::MaterialConfig> overrideMaterialsByPath;
    for (const auto& [nodeName, path] : materialOverrides) {
        if (overrideMaterialsByPath.contains(path)) {
            continue;
        }
        std::optional<engine::config::MaterialConfig> overrideMaterial =
            engine::config::loadMaterialConfig(assetRoot + "/" + path);
        if (!overrideMaterial) {
            std::cerr << "resolvePerInstanceSettings: materialOverrides entry '" << path
                      << "' failed to load\n";
            return std::nullopt;
        }
        overrideMaterialsByPath.emplace(path, std::move(*overrideMaterial));
    }

    std::vector<PathTraceSettings> perInstanceSettings;
    perInstanceSettings.reserve(instances.size());
    for (const MeshInstance& instance : instances) {
        PathTraceSettings settings = base;
        if (const auto overrideIt = materialOverrides.find(instance.name);
            overrideIt != materialOverrides.end()) {
            const engine::config::MaterialConfig& overrideMaterial =
                overrideMaterialsByPath.at(overrideIt->second);
            settings.bumpStrength = overrideMaterial.bumpStrength;
            settings.roughnessMin = overrideMaterial.roughnessMin;
            settings.roughnessMax = overrideMaterial.roughnessMax;
            settings.diffuseColour = overrideMaterial.diffuseColour;
            settings.ior = overrideMaterial.ior;
            settings.transmissionFactor = overrideMaterial.transmissionFactor;
            settings.metallicFactor = overrideMaterial.metallicFactor;
            settings.roughnessFactor = overrideMaterial.roughnessFactor;
            settings.diffuseRoughness = overrideMaterial.diffuseRoughness;
            settings.transmissionColor = overrideMaterial.transmissionColor;
            settings.transmissionDepth = overrideMaterial.transmissionDepth;
        }
        perInstanceSettings.push_back(settings);
    }
    return perInstanceSettings;
}

}  // namespace engine::scene
