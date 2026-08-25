#pragma once

#include <optional>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "engine/gfx/ocio_display_transform.h"

namespace engine::config {

// Matches pbr.frag's kMaxLights (its uLightType[16]/uLightPositionOrDir[16]/ uLightColor[16]/uLightRange[16] array uniforms) -- the two can't share a literal across the GLSL/C++ boundary, so both sides name it instead of repeating the bare number.
inline constexpr int kMaxLights = 16;

// One punctual light (Phase 4: direct analytic evaluation, no visibility rays). Directional lights are unit-length, infinite-distance sources (directionOrPosition points toward the light, no falloff); point lights are finite-position sources with windowed inverse-square falloff out to `range` metres (Karis 2013, "Real Shading in Unreal Engine 4" -- avoids both the 1/d^2 singularity at d->0 and an abrupt cutoff at the range boundary).
struct Light {
    enum class Type { Directional, Point };
    Type type;
    glm::vec3 directionOrPosition;
    glm::vec3 color;
    float range;  // metres; unused (but still parsed) for Directional
};

// Everything main.cpp needs at startup that isn't camera/lens state (that's ProfileConfig): the model to load, the punctual light list, window size, and initial debug-view state. gltfPath is relative to ASSET_ROOT_DIR, matching this project's existing asset-path convention. See assets/config/scene.json for the checked-in defaults.
struct SceneConfig {
    std::string gltfPath;
    std::vector<Light> lights;
    int windowWidth;
    int windowHeight;
    // Index into hud_overlay.cpp's kAovNames (0 = Beauty).
    int initialAov;
    engine::gfx::OcioDisplayTransform::Lut initialLut;
    int samplesPerPixel;   // path tracer startup default
    int maxBounces;        // path tracer startup default
    int russianRouletteStartBounce;  // 0-based bounce index RR kicks in from
};

// Reads and parses path. Returns nullopt and logs to stderr if the file is missing, unreadable, or any required field can't be found/parsed — this is user-editable input, not an internal invariant, so failure is expected to happen and is surfaced rather than defaulted around.
[[nodiscard]] std::optional<SceneConfig> loadSceneConfig(const std::string& path);

}  // namespace engine::config
