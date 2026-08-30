#pragma once

#include <optional>
#include <string>

#include <glm/glm.hpp>

namespace engine::config {

// The asset to load and where to place it -- everything main.cpp needs that's specific to this scene rather than a session-wide renderer/camera default (those are ProfileConfig). gltfPath/texturePath/materialPath are relative to ASSET_ROOT_DIR, matching this project's existing asset-path convention; see assets/config/scene.json for the checked-in defaults.
struct SceneConfig {
    std::string gltfPath;
    std::string texturePath;  // directory (relative to ASSET_ROOT_DIR) glTF image URIs resolve against, overriding the .gltf's own directory -- lets one config swap texture resolution (2K/4K/8K) across every tier without editing each .gltf
    std::string materialPath;  // path (relative to ASSET_ROOT_DIR) to the MaterialConfig JSON (material_config.h)
    glm::vec3 position;         // model root, composed on top of the glTF's own node transforms
    glm::vec3 rotationDegrees;  // order X,Y,Z, see main.cpp's loadGltf call
    std::string hdriPath;  // environment map, relative to ASSET_ROOT_DIR
};

// Reads and parses path. Returns nullopt and logs to stderr if the file is missing, unreadable, or any required field can't be found/parsed — this is user-editable input, not an internal invariant, so failure is expected to happen and is surfaced rather than defaulted around.
[[nodiscard]] std::optional<SceneConfig> loadSceneConfig(const std::string& path);

}  // namespace engine::config
