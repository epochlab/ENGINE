#pragma once

#include <optional>
#include <string>

#include <glm/glm.hpp>

namespace engine::config {

// What to load and where to place it. gltfPath/texturePath are relative to ASSET_ROOT_DIR, matching this project's existing asset-path convention.
struct ModelConfig {
    std::string gltfPath;
    std::string texturePath;  // directory (relative to ASSET_ROOT_DIR) glTF image URIs resolve against, overriding the .gltf's own directory -- lets one config swap texture resolution (2K/4K/8K) across every tier without editing each .gltf
    glm::vec3 position;         // model root, composed on top of the glTF's own node transforms
    glm::vec3 rotationDegrees;  // order X,Y,Z, see main.cpp's loadGltf call
};

// What lights the scene.
struct EnvironmentConfig {
    std::string hdriPath;  // environment map, relative to ASSET_ROOT_DIR
};

// Tunable shading constants that shape how a material's textures get turned into BSDF input -- externalized so these can be edited without recompiling. diffuseColour/ior/transmissionFactor/metallicFactor/roughnessFactor are a single global material definition applied to every mesh in the scene, replacing what used to be read per-object off each glTF's material block.
struct MaterialConfig {
    float bumpStrength;   // scales the bump texture's raw per-texel height difference; see path_tracer.cpp's buildShadingFrame
    float roughnessMin;   // floor applied to the roughness texture sample, avoids a near-zero-roughness GGX singularity
    float roughnessMax;   // ceiling applied to the roughness texture sample
    glm::vec3 diffuseColour;      // multiplies baseColorTexture
    float ior;                     // dielectric IOR, non-metal lobes only
    float transmissionFactor;      // KHR_materials_transmission-style factor, 0 = opaque
    float metallicFactor;
    float roughnessFactor;         // multiplies the roughness texture sample, before roughnessMin/Max clamp
};

// The asset to load and how to shade/light it -- everything main.cpp needs that's specific to this scene rather than a session-wide renderer/camera default (those are ProfileConfig). Grouped into model/environment/material sub-objects; see assets/config/scene.json for the checked-in defaults.
struct SceneConfig {
    ModelConfig model;
    EnvironmentConfig environment;
    MaterialConfig material;
};

// Reads and parses path. Returns nullopt and logs to stderr if the file is missing, unreadable, or any required field can't be found/parsed. User-editable input, not an internal invariant: failure is expected and surfaced rather than defaulted around.
[[nodiscard]] std::optional<SceneConfig> loadSceneConfig(const std::string& path);

}  // namespace engine::config
