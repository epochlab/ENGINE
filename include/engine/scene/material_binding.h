#pragma once

#include <map>
#include <optional>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "engine/config/scene_config.h"
#include "engine/scene/gltf_loader.h"
#include "engine/scene/path_tracer.h"

namespace engine::scene {

// Resolves a scene's material overrides into one PathTraceSettings per mesh instance, parallel-indexed with `instances` so ShadingTriangle::instanceIndex resolves straight into the result.
// Every entry starts as a copy of `base` (the scene-wide material file); an instance whose MeshInstance::name has a materialOverrides entry gets that file's 11 material fields substituted. Renderer-only fields (samplesPerPixel/maxBounces/RR) are never overridden -- they are not material properties and no material file carries them.
// Each distinct override path is loaded once, keyed by path rather than by node name, since several nodes may share one material file.
// Returns nullopt if any referenced material file fails to load, or if any override key names no instance, matching the all-or-nothing startup gate the caller applies to the scene/model/environment loads: a scene silently rendering the wrong material is worse than one that refuses to start.
// Shared by the real renderer (main.cpp) and tools/render_beauty.cpp specifically so a headless comparison render cannot drift from what actually ships.
[[nodiscard]] std::optional<std::vector<PathTraceSettings>> resolvePerInstanceSettings(
    const PathTraceSettings& base, const std::vector<MeshInstance>& instances,
    const std::map<std::string, std::string>& materialOverrides, const std::string& assetRoot);

// Transforms authored quad lights into the same world space as the model's own geometry: origin is a point (full sceneTransform, translation included), edge0/edge1 are displacement vectors (linear part only, w=0), and radiance is color * intensity.
// Shared by main.cpp and tools/render_beauty.cpp for the same reason resolvePerInstanceSettings is: a headless comparison render must place lights identically to what ships, and two copies of a transform convention drift.
// sceneTransform must be rigid (or at worst uniformly scaled) for the result to stay rectangular -- ModelConfig exposes only position/rotation, so it is; loadSceneConfig has already rejected any non-perpendicular authored quad.
[[nodiscard]] std::vector<QuadLight> buildQuadLights(
    const std::vector<engine::config::QuadLightConfig>& lights, const glm::mat4& sceneTransform);

}  // namespace engine::scene
