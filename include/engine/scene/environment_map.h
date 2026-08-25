#pragma once

#include <glm/glm.hpp>

#include "engine/gfx/hdr_image.h"

namespace engine::scene {

// CPU-resident equirect env map for path-traced miss rays -- the rasterizer's IBL bake discards the raw HdrImage after startup.
class EnvironmentMap {
public:
    explicit EnvironmentMap(engine::gfx::HdrImage image) : image_(std::move(image)) {}

    // Direction -> equirect UV -> bilinear sample; mapping matches sky.frag (theta=acos(y), phi=atan2(x,z)). envRotationRadians matches uEnvRotationRadians.
    [[nodiscard]] glm::vec3 sampleDirection(const glm::vec3& direction,
                                             float envRotationRadians = 0.0F) const;

private:
    engine::gfx::HdrImage image_;
};

}  // namespace engine::scene
