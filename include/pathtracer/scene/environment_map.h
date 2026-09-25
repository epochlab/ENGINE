#pragma once

#include <cmath>
#include <vector>

#include <glm/glm.hpp>

#include "pathtracer/gfx/hdr_image.h"

namespace pathtracer::scene {

// Sin/cos of the environment's Y rotation. The angle is constant for a whole pass, so its trig is resolved once, not per ray.
struct YRotation {
    float sinAngle = 0.0F;  // default-constructed is the identity rotation, matching an angle of exactly zero
    float cosAngle = 1.0F;
    [[nodiscard]] static YRotation of(float angleRadians) {
        return {std::sin(angleRadians), std::cos(angleRadians)};
    }
    // Exact in IEEE: cos is even and sin odd, so the inverse needs no second pair of trig calls.
    [[nodiscard]] YRotation inverse() const { return {-sinAngle, cosAngle}; }
};

// CPU-resident equirect env map for path-traced miss rays and NEE light sampling.
class EnvironmentMap {
public:
    // Builds the 2D piecewise-constant importance-sampling CDFs (marginal over rows, conditional over columns) once at load.
    explicit EnvironmentMap(pathtracer::gfx::ImageTexture image);

    // Direction -> equirect UV -> bilinear sample, matching sky.frag. The renderer's only Le: pdf() below is merely its importance.
    [[nodiscard]] glm::vec3 sampleDirection(const glm::vec3& direction,
                                             YRotation rotation = {}) const;

    struct EnvSample {
        glm::vec3 direction;
        float pdf;  // solid-angle pdf, > 0
    };

    // Importance-samples a direction from the map's luminance distribution, for NEE. u is two independent uniforms.
    [[nodiscard]] EnvSample importanceSampleDirection(glm::vec2 u, YRotation rotation = {}) const;

    // Solid-angle pdf of importanceSampleDirection() producing this direction, for MIS-weighting a BSDF-sampled environment miss.
    [[nodiscard]] float pdf(const glm::vec3& direction, YRotation rotation = {}) const;

private:
    // The CDFs below are built from these stored values, so sampling density matches the radiance returned.
    pathtracer::gfx::ImageTexture image_;
    std::vector<float> marginalCdf_;     // size height+1, marginalCdf_[0]=0, marginalCdf_[height]=1
    std::vector<float> conditionalCdf_;  // size height*(width+1), row-major, each row's slice sums to 1 at its last entry
};

}  // namespace pathtracer::scene
