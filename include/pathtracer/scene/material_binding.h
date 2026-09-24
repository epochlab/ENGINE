#pragma once

#include <map>
#include <optional>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "pathtracer/config/scene_config.h"
#include "pathtracer/scene/gltf_loader.h"
#include "pathtracer/scene/path_tracer.h"

namespace pathtracer::scene {

// Resolves a scene's material overrides into one PathTraceSettings per mesh instance, parallel-indexed with
// `instances`. Each entry copies `base`; a named override substitutes that file's 11 material fields, each distinct
// path loaded once. nullopt if a file fails to load or a key names no instance. Shared with render_beauty.
[[nodiscard]] std::optional<std::vector<PathTraceSettings>> resolvePerInstanceSettings(
    const PathTraceSettings& base, const std::vector<MeshInstance>& instances,
    const std::map<std::string, std::string>& materialOverrides, const std::string& assetRoot);

// Transforms authored quad lights into the model's world space: origin is a point (full sceneTransform), edge0 and
// edge1 are displacements (linear part only). sceneTransform must be rigid or uniformly scaled to stay rectangular,
// which ModelConfig and loadSceneConfig already guarantee. Shared with render_beauty.
[[nodiscard]] std::vector<QuadLight> buildQuadLights(
    const std::vector<pathtracer::config::QuadLightConfig>& lights, const glm::mat4& sceneTransform);

}  // namespace pathtracer::scene
