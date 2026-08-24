#pragma once

#include <array>

#include <glm/glm.hpp>

#include "engine/gfx/hdr_image.h"

namespace engine::scene {

// Order-2 (9-coefficient) spherical-harmonic projection of an
// equirectangular environment map's irradiance (Ramamoorthi & Hanrahan
// 2001, "An Efficient Representation for Irradiance Environment Maps").
// Direction/UV convention (must match assets/shaders/equirect_to_cubemap.frag's
// inverse mapping):
//   theta = v * pi            (polar angle from +Y, v=0 at the image top)
//   phi   = (u - 0.5) * 2*pi  (azimuth, u=0.5 -> phi=0)
//   dir   = (sin(theta)*sin(phi), cos(theta), sin(theta)*cos(phi))
//
// The per-band cosine-lobe convolution constants (A_l, from the same
// paper) are folded into the returned coefficients at projection time,
// so evaluateIrradianceSH9 at shading time is a flat 9-term dot product
// with no separate convolution step. The result is irradiance E(n), not
// yet divided by pi or multiplied by albedo -- shadeBeauty applies
// albedo/pi to match its existing Lambertian diffuse normalization.
[[nodiscard]] std::array<glm::vec3, 9> projectIrradianceSH9(const engine::gfx::HdrImage& equirect);

// Evaluates the order-2 SH irradiance polynomial for direction n
// (unit-length) against coefficients produced by projectIrradianceSH9.
// Exposed for the CPU-side furnace test (tools/furnace_test.cpp), which
// needs the identical evaluator the GLSL side re-implements.
[[nodiscard]] glm::vec3 evaluateIrradianceSH9(const glm::vec3& n,
                                               const std::array<glm::vec3, 9>& coeffs);

}  // namespace engine::scene
