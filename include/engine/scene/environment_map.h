#pragma once

#include <vector>

#include <glm/glm.hpp>

#include "engine/gfx/hdr_image.h"

namespace engine::scene {

// CPU-resident equirect env map for path-traced miss rays and NEE light sampling.
class EnvironmentMap {
public:
    // Builds the 2D piecewise-constant importance-sampling CDFs (marginal over rows, conditional over
    // columns per row) once here, at load time -- not per-trace, not per-pass. Weighted by sin(theta)
    // so the sampling density corrects for the equirect projection's polar over-representation
    // (a pixel row near a pole covers far less solid angle than one at the equator despite occupying
    // the same image-space area).
    explicit EnvironmentMap(engine::gfx::HdrImage image);

    // Direction -> equirect UV -> bilinear sample; mapping matches sky.frag (theta=acos(y), phi=atan2(x,z)). envRotationRadians matches uEnvRotationRadians.
    [[nodiscard]] glm::vec3 sampleDirection(const glm::vec3& direction,
                                             float envRotationRadians = 0.0F) const;

    struct EnvSample {
        glm::vec3 direction;
        float pdf;  // solid-angle pdf, > 0
    };

    // Importance-samples a direction from the map's luminance distribution -- for NEE light
    // sampling. u: two independent uniform [0,1) values.
    [[nodiscard]] EnvSample importanceSampleDirection(glm::vec2 u, float envRotationRadians = 0.0F) const;

    // Solid-angle pdf of importanceSampleDirection() having produced this direction -- for MIS
    // weighting a BSDF-sampled ray's environment-miss contribution against NEE's light-sampling
    // strategy.
    [[nodiscard]] float pdf(const glm::vec3& direction, float envRotationRadians = 0.0F) const;

private:
    engine::gfx::HdrImage image_;
    std::vector<float> marginalCdf_;     // size height+1, marginalCdf_[0]=0, marginalCdf_[height]=1
    std::vector<float> conditionalCdf_;  // size height*(width+1), row-major, each row's slice sums to 1 at its last entry
};

}  // namespace engine::scene
