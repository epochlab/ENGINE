#include "engine/scene/material.h"

namespace engine::scene {

Material makeDefaultMaterial() {
    return Material{
        {1, 1, {1.0F, 1.0F, 1.0F, 1.0F}},  // baseColorTexture: white
        {1, 1, {0.5F, 0.5F, 1.0F, 1.0F}},  // normalTexture: decodes to (0,0,1) tangent-space up
        {1, 1, {0.5F, 0.5F, 0.5F, 1.0F}},  // bumpTexture: any constant -> zero finite-difference
        {1, 1, {1.0F, 1.0F, 1.0F, 1.0F}},  // roughnessTexture: roughnessFactor/min/max fully control the result
        {1, 1, {0.04F, 0.04F, 0.04F, 1.0F}},  // specularTexture: standard dielectric f0, inert whenever metallicFactor=0
        {1, 1, {1.0F, 1.0F, 1.0F, 1.0F}},  // aoTexture: fully unoccluded
    };
}

}  // namespace engine::scene
