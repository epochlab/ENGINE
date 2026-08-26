#pragma once

#include <optional>
#include <string>

#include <glm/glm.hpp>

#include "engine/gfx/ocio_display_transform.h"

namespace engine::config {

// Everything main.cpp needs at startup that isn't camera/lens state (that's ProfileConfig): the model to load, window size, and initial debug-view state. gltfPath is relative to ASSET_ROOT_DIR, matching this project's existing asset-path convention. See assets/config/scene.json for the checked-in defaults.
struct SceneConfig {
    std::string gltfPath;
    glm::vec3 position;         // model root, composed on top of the glTF's own node transforms
    glm::vec3 rotationDegrees;  // order X,Y,Z, see main.cpp's loadGltf call
    int windowWidth;
    int windowHeight;
    // Index into engine::debug::AovId / kAovNames (aov.h) (0 = Beauty).
    int initialAov;
    engine::gfx::OcioDisplayTransform::Lut initialLut;
    int samplesPerPixel;   // path tracer startup default
    int maxBounces;        // path tracer startup default
    int russianRouletteStartBounce;  // 0-based bounce index RR kicks in from
};

// Reads and parses path. Returns nullopt and logs to stderr if the file is missing, unreadable, or any required field can't be found/parsed — this is user-editable input, not an internal invariant, so failure is expected to happen and is surfaced rather than defaulted around.
[[nodiscard]] std::optional<SceneConfig> loadSceneConfig(const std::string& path);

}  // namespace engine::config
