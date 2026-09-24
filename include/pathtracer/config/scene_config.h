#pragma once

#include <map>
#include <optional>
#include <string>
#include <vector>

#include <glm/glm.hpp>

namespace pathtracer::config {

// What to load and where to place it. gltfPath/texturePath are relative to ASSET_ROOT_DIR, as every asset path is.
struct ModelConfig {
    std::string gltfPath;
    // Directory (relative to ASSET_ROOT_DIR) glTF image URIs resolve against, overriding the .gltf's own directory.
    // Lets one config swap texture resolution across every tier without editing the asset.
    std::string texturePath;
    glm::vec3 position;         // model root, composed on top of the glTF's own node transforms
    glm::vec3 rotation;  // degrees, order X,Y,Z, see main.cpp's loadGltf call
};

// What lights the scene, IBL half.
struct EnvironmentConfig {
    std::string hdriPath;  // environment map, relative to ASSET_ROOT_DIR
    // Whether the environment is a light (NEE-sampled, MIS-weighted, contributing to every miss) rather than just the
    // camera-visible background -- see scene::LightSet. Optional, default true, so an existing scene.json is unchanged.
    // The HUD's Environment Light checkbox edits it at runtime; render_beauty's --env-light overrides it headlessly.
    bool lightEnabled = true;
};

// A rectangular area light (Arnold quad_light semantics): origin is one corner, edge0/edge1 span the sides, and
// radiance = color * intensity leaves the face whose outward normal is normalize(cross(edge0, edge1)). edge0 must be
// perpendicular to edge1 and intensity/color non-negative -- loadSceneConfig rejects the scene rather than mis-sample.
struct QuadLightConfig {
    glm::vec3 origin;
    glm::vec3 edge0;
    glm::vec3 edge1;
    glm::vec3 color;
    float intensity;
    bool twoSided = false;
};

// Tunable shading constants turning a material's textures into BSDF input, externalized so they need no recompile.
struct MaterialConfig {
    float bumpStrength;   // scales the bump texture's raw per-texel height difference; see path_tracer.cpp's buildShadingFrame
    float roughnessMin;   // floor applied to the roughness texture sample, avoids a near-zero-roughness GGX singularity
    float roughnessMax;   // ceiling applied to the roughness texture sample
    glm::vec3 diffuseColour;      // multiplies baseColorTexture
    // Dielectric IOR, non-metal lobes only. Optional, default 1.5: nullified by (1-metallic), so a pure conductor
    // (chrome.json) need not declare it.
    float ior = 1.5F;
    // Abbe number V_d = (n_d-1)/(n_F-n_C), pairing with ior to give it a wavelength dependence (transmissionFactor>0).
    // Optional, default 0.0 = no dispersion, matching Arnold and OpenPBR.
    float abbe = 0.0F;
    // KHR_materials_transmission-style factor, 0 = opaque. Optional, default 0.0, a true no-op (transmitProb=0), so a
    // material with no transmission lobe (clay.json, chrome.json) need not declare it.
    float transmissionFactor = 0.0F;
    // Optional, default 0.0: a true no-op, so a non-metal material (e.g. clay.json, glass.json) need not declare it.
    float metallicFactor = 0.0F;
    float roughnessFactor;         // multiplies the roughness texture sample, before roughnessMin/Max clamp
    // EON rough-diffuse parameter r in [0,1]; 0 = Lambertian, see bsdf.cpp evaluateDiffuseLobe. Optional, default 0.0:
    // dead once metallicFactor=1 or transmissionFactor=1, so glass.json/chrome.json need not declare it.
    float diffuseRoughness = 0.0F;
    // The transmission lobe's only tint (transmissionFactor>0); baseColor tints reflection alone (bsdf.h BsdfParams).
    // Optional, default [1,1,1]: -log(1)=0 and a white on-surface tint is the identity, so both regimes no-op.
    glm::vec3 transmissionColor = glm::vec3(1.0F);
    // Distance (world units) at which transmittance reaches transmissionColor by Beer's law -- the interior medium's
    // density, see path_tracer.cpp sigmaAFromTransmission. Optional, default 0.0 = no interior medium.
    float transmissionDepth = 0.0F;
    // Gulbrandsen 2014 edgetint for the conductor lobe (metallicFactor>0). Optional, default [1,1,1]: white is the
    // no-dip edge Schlick always produced, so an existing material file loads unchanged.
    glm::vec3 edgeTint = glm::vec3(1.0F);
};

// The asset to load and how to shade and light it: everything main.cpp needs that is specific to this scene rather
// than a session-wide renderer/camera default, which is ProfileConfig.
struct SceneConfig {
    ModelConfig model;
    EnvironmentConfig environment;
    // Relative to ASSET_ROOT_DIR, a material JSON (materials/clay.json), matching ModelConfig::gltfPath.
    std::string materialPath;
    // glTF node name -> material JSON path (relative to ASSET_ROOT_DIR), overriding materialPath for that instance.
    // Optional; absent means an empty map, so every instance uses materialPath.
    std::map<std::string, std::string> materialOverrides;
    // Rectangular area lights, in the same space as the glTF's own vertices (ModelConfig::position/
    // rotation applies to these too -- see main.cpp's sceneTransform). Optional key; absent means none,
    // matching every scene authored before this field existed.
    std::vector<QuadLightConfig> lights;
};

// Reads and parses path. Returns nullopt and logs to stderr if the file is missing, unreadable, or a required field
// cannot be parsed. User-editable input, not an internal invariant: failure is expected and surfaced, not asserted.
[[nodiscard]] std::optional<SceneConfig> loadSceneConfig(const std::string& path);

// Reads and parses a standalone material file (e.g. assets/materials/clay.json). Same failure contract as loadSceneConfig.
[[nodiscard]] std::optional<MaterialConfig> loadMaterialConfig(const std::string& path);

}  // namespace pathtracer::config
