#pragma once

#include <glm/glm.hpp>

#include "pathtracer/gfx/hdr_image.h"
#include "pathtracer/scene/bsdf.h"
#include "pathtracer/scene/material.h"
#include "pathtracer/scene/path_tracer.h"
#include "pathtracer/scene/shading_scene.h"

namespace pathtracer::scene {

// Primary-hit G-buffer sampling shared by the path tracer (path_tracer.cpp, tracePath bounce-0) and the rasterizer (rasterizer.cpp) -- both resolve the same material/shading values at a hit point via two different ways of finding that hit (ray intersection vs. scan-conversion); sharing this code guarantees agreement by construction, not convention.

// near: true if p is within thicknessPx of segment [a,b] (clamped to its extent, not the infinite line). t: clamped [0,1] parameter of the closest point, valid when near is true -- lets a caller interpolate a per-endpoint quantity (e.g. depth). Shared by mesh-edge (near only) and box-edge (near + t, for depth) line tests.
struct LineProximity {
    bool near;
    float t;
};
[[nodiscard]] LineProximity nearLineSegmentPx(glm::vec2 p, glm::vec2 a, glm::vec2 b, float thicknessPx);

// heroChannel: the RGB channel a dispersive path has committed to, which selects the wavelength the
// ior is resolved at (bsdf.h's cauchyIor/kRgbWavelengthsNm). nullopt -- the rasterizer's non-spectral
// preview, and every path that has not crossed a dispersive interface -- keeps the authored d-line ior.
// No default argument: the two non-path-tracer callers pass nullopt explicitly, so a third caller has
// to decide rather than inherit silence.
[[nodiscard]] BsdfParams resolveBsdfParams(const Material& material, glm::vec2 uv,
                                            const glm::vec3& vertexColour,
                                            const PathTraceSettings& settings,
                                            std::optional<int> heroChannel);

// Gram-Schmidt re-orthogonalized tangent frame, normal- and bump-mapped.
[[nodiscard]] ShadingFrame buildShadingFrame(const ShadingVertex& shading, const Material& material,
                                              const PathTraceSettings& settings);

[[nodiscard]] glm::vec3 geometricNormalOf(const ShadingTriangle& tri);

// Writes an opaque (alpha=1) RGB texel -- every AOV field broadcasts to RGBA this way, matching HdrImage's fixed 4-floats/texel layout so it goes straight through Texture::createFromFloatPixels unchanged.
void writeTexel(pathtracer::gfx::HdrImage& image, int x, int y, glm::vec3 rgb);

[[nodiscard]] pathtracer::gfx::HdrImage makeImage(int width, int height);

}  // namespace pathtracer::scene
