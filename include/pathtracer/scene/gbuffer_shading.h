#pragma once

#include <glm/glm.hpp>

#include "pathtracer/gfx/hdr_image.h"
#include "pathtracer/scene/bsdf.h"
#include "pathtracer/scene/material.h"
#include "pathtracer/scene/path_tracer.h"
#include "pathtracer/scene/shading_scene.h"

namespace pathtracer::scene {

// Primary-hit G-buffer sampling shared by the path tracer (tracePath bounce 0) and the rasterizer, so both resolve
// the same material the same way.

// near: true if p is within thicknessPx of segment [a,b], clamped to its extent rather than the infinite line.
// t: the clamped [0,1] parameter of the closest point.
struct LineProximity {
    bool near;
    float t;
};
[[nodiscard]] LineProximity nearLineSegmentPx(glm::vec2 p, glm::vec2 a, glm::vec2 b, float thicknessPx);

// heroChannel: the RGB channel a dispersive path has committed to, selecting the wavelength the ior is resolved at
// (bsdf.h cauchyIor/kRgbWavelengthsNm). nullopt keeps the authored d-line ior. No default argument, so a third
// caller has to decide rather than inherit silence.
[[nodiscard]] BsdfParams resolveBsdfParams(const Material& material, glm::vec2 uv,
                                            const glm::vec3& vertexColour,
                                            const PathTraceSettings& settings,
                                            std::optional<int> heroChannel);

// Gram-Schmidt re-orthogonalized tangent frame, normal- and bump-mapped.
[[nodiscard]] ShadingFrame buildShadingFrame(const ShadingVertex& shading, const Material& material,
                                              const PathTraceSettings& settings);

[[nodiscard]] glm::vec3 geometricNormalOf(const ShadingTriangle& tri);

// Writes an opaque (alpha=1) RGB texel -- every AOV field broadcasts to RGBA this way, matching HdrImage's fixed
// 4-floats/texel layout so it goes straight to the GPU.
void writeTexel(pathtracer::gfx::HdrImage& image, int x, int y, glm::vec3 rgb);

[[nodiscard]] pathtracer::gfx::HdrImage makeImage(int width, int height);

}  // namespace pathtracer::scene
