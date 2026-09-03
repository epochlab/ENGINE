#pragma once

#include <map>
#include <optional>
#include <string>

#include <glm/glm.hpp>

namespace engine::config {

// What to load and where to place it. gltfPath/texturePath are relative to ASSET_ROOT_DIR, matching this project's existing asset-path convention.
struct ModelConfig {
    std::string gltfPath;
    std::string texturePath;  // directory (relative to ASSET_ROOT_DIR) glTF image URIs resolve against, overriding the .gltf's own directory -- lets one config swap texture resolution (2K/4K/8K) across every tier without editing each .gltf
    glm::vec3 position;         // model root, composed on top of the glTF's own node transforms
    glm::vec3 rotation;  // degrees, order X,Y,Z, see main.cpp's loadGltf call
};

// What lights the scene.
struct EnvironmentConfig {
    std::string hdriPath;  // environment map, relative to ASSET_ROOT_DIR
};

// Tunable shading constants that shape how a material's textures get turned into BSDF input -- externalized so these can be edited without recompiling. diffuseColour/ior/transmissionFactor/metallicFactor/roughnessFactor/diffuseRoughness are a single global material definition applied to every mesh in the scene, replacing what used to be read per-object off each glTF's material block. Loaded from its own file (see loadMaterialConfig) rather than inline in scene.json, so a shader library of named material files (assets/materials/*.json) can grow without a scene.json schema change.
struct MaterialConfig {
    float bumpStrength;   // scales the bump texture's raw per-texel height difference; see path_tracer.cpp's buildShadingFrame
    float roughnessMin;   // floor applied to the roughness texture sample, avoids a near-zero-roughness GGX singularity
    float roughnessMax;   // ceiling applied to the roughness texture sample
    glm::vec3 diffuseColour;      // multiplies baseColorTexture
    float ior;                     // dielectric IOR, non-metal lobes only
    float transmissionFactor;      // KHR_materials_transmission-style factor, 0 = opaque
    float metallicFactor;
    float roughnessFactor;         // multiplies the roughness texture sample, before roughnessMin/Max clamp
    float diffuseRoughness;        // EON rough-diffuse parameter r in [0,1]; 0 = Lambertian, see bsdf.cpp's evaluateDiffuseLobe
};

// The asset to load and how to shade/light it -- everything main.cpp needs that's specific to this scene rather than a session-wide renderer/camera default (those are ProfileConfig). Grouped into model/environment sub-objects, plus a path to the scene's material file; see assets/scenes/tree.json for the checked-in defaults.
struct SceneConfig {
    ModelConfig model;
    EnvironmentConfig environment;
    std::string materialPath;  // relative to ASSET_ROOT_DIR, points to a material JSON file (e.g. materials/diffuse.json), matching ModelConfig::gltfPath's convention
    // glTF node name -> material JSON path (relative to ASSET_ROOT_DIR), overriding materialPath for that instance's triangles. Optional key; absent in the scene JSON means an empty map, i.e. every instance uses materialPath. Keyed by MeshInstance::name (gltf_loader.h).
    std::map<std::string, std::string> materialOverrides;
};

// Reads and parses path. Returns nullopt and logs to stderr if the file is missing, unreadable, or any required field can't be found/parsed. User-editable input, not an internal invariant: failure is expected and surfaced rather than defaulted around.
[[nodiscard]] std::optional<SceneConfig> loadSceneConfig(const std::string& path);

// Reads and parses a standalone material file (e.g. assets/materials/diffuse.json). Same failure contract as loadSceneConfig.
[[nodiscard]] std::optional<MaterialConfig> loadMaterialConfig(const std::string& path);

}  // namespace engine::config
