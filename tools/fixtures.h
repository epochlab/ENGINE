#pragma once

#include <algorithm>
#include <cmath>
#include <optional>
#include <random>
#include <vector>

#include <glm/glm.hpp>

#include "engine/gfx/hdr_image.h"
#include "engine/scene/bsdf.h"
#include "engine/scene/environment_map.h"
#include "engine/scene/material.h"
#include "engine/scene/sampler.h"

// Scenes, materials and analytic references shared by the validators that drive the renderer. Each of these existed in
// two or three validators as an independent transcription; where two copies of an ORACLE drift, the checks built on
// them silently stop measuring the same thing, which is the failure this header removes.
//
// One thing deliberately NOT collapsed: the oracles that re-derive shipped math (bsdf_validate's referenceEon,
// referenceConductorIor, cosineAverageFresnel) stay independent transcriptions of the literature, written so a
// transcription error in src/ surfaces here instead of cancelling. Nothing in this header may include or call into
// src/scene/bsdf.* on a reference path -- an oracle that shares code with the thing it measures proves nothing.
namespace tools::fixtures {

constexpr float kPi = 3.14159265F;

// Uniform hemisphere direction about +z, pdf = 1/(2*pi). Draws cosTheta then phi, in that order: the order is part of
// the contract, because a caller reproducing a reference value depends on the rng consumption sequence.
inline glm::vec3 sampleUniformHemisphere(std::mt19937& rng) {
    std::uniform_real_distribution<float> unit(0.0F, 1.0F);
    const float cosTheta = unit(rng);
    const float sinTheta = std::sqrt(std::max(0.0F, 1.0F - (cosTheta * cosTheta)));
    const float phi = 2.0F * kPi * unit(rng);
    return {sinTheta * std::cos(phi), sinTheta * std::sin(phi), cosTheta};
}

// Independent ground truth: Lo(wo) = integral over the hemisphere of evaluateBsdf(wo,wi)*wi.z dwi, with L0 = 1.
// Uniform-hemisphere Monte Carlo, so it under-samples a sharp GGX peak -- callers restrict the tight comparison to
// roughness values where it converges, and say so at the call site.
inline float referenceLo(const engine::scene::BsdfParams& params, const glm::vec3& wo, int sampleCount,
                          std::mt19937& rng) {
    constexpr float kUniformPdf = 1.0F / (2.0F * kPi);
    glm::vec3 accum(0.0F);
    for (int i = 0; i < sampleCount; ++i) {
        const glm::vec3 wi = sampleUniformHemisphere(rng);
        accum += engine::scene::evaluateBsdf(params, wo, wi) * wi.z / kUniformPdf;
    }
    return std::max({accum.x, accum.y, accum.z}) / static_cast<float>(sampleCount);
}

// A white slab of infinite extent under a uniform L0 = 1 environment, estimated by BSDF sampling alone: a camera ray at normal incidence enters the top face and each vertex continues along sampleBsdf's draw until it leaves either face with L0 = 1 times its throughput.
// No geometry, NEE or MIS: in an infinite homogeneous slab only direction matters, so this reads the BSDF's own round-trip energy closure, and is the integrator-free value integrator_validate's Embree slab must reproduce.
// The bottom face's frame is the top face's mirrored in z, which an isotropic BSDF cannot distinguish. The depth cap only bounds the loop: truncated counts paths that reached it, and callers assert it is zero so the estimate is exact rather than truncated.
// verticesPerPath counts the interfaces a path actually scattered at, which is how far a per-vertex error in the escape table can accumulate along it.
struct SlabWalk {
    double mean;
    double verticesPerPath;
    long long truncated;
};

inline SlabWalk slabWalkLo(const engine::scene::BsdfParams& params, int paths, std::uint32_t seed) {
    constexpr int kMaxVertices = 256;
    double sum = 0.0;
    long long vertices = 0;
    long long truncated = 0;
    for (int i = 0; i < paths; ++i) {
        engine::scene::Sampler sampler(0, 0, i, paths, seed);
        glm::vec3 direction(0.0F, 0.0F, -1.0F);
        glm::vec3 throughput(1.0F);
        bool top = true;
        int vertex = 0;
        for (; vertex < kMaxVertices; ++vertex) {
            const float zSign = top ? 1.0F : -1.0F;
            const std::optional<engine::scene::BsdfSample> sample =
                engine::scene::sampleBsdf(params, glm::vec3(-direction.x, -direction.y, -direction.z * zSign), sampler);
            if (!sample.has_value()) {
                break;
            }
            ++vertices;
            throughput *= sample->throughputWeight;
            direction = glm::vec3(sample->wiLocal.x, sample->wiLocal.y, sample->wiLocal.z * zSign);
            if (top ? direction.z > 0.0F : direction.z < 0.0F) {
                sum += std::max({throughput.x, throughput.y, throughput.z});
                break;
            }
            top = !top;
        }
        truncated += vertex == kMaxVertices ? 1 : 0;
    }
    const auto count = static_cast<double>(paths);
    return {sum / count, static_cast<double>(vertices) / count, truncated};
}

// N-channel float test data stored at type, rounded to nearest even as loadImageTexture's OpenEXR read does.
template <int N>
engine::gfx::ImageTexture<N> makeImageTexture(int width, int height, const std::vector<float>& texels,
                                               engine::gfx::ScalarType type) {
    if (type == engine::gfx::ScalarType::Float32) {
        return {width, height, texels};
    }
    return {width, height, std::vector<engine::gfx::Half>(texels.begin(), texels.end())};
}

// Uniform-radiance (L0 = 1) equirect environment: constant regardless of resolution, but a real image so
// EnvironmentMap's CDF machinery runs its normal (non-degenerate) path rather than the all-black fallback.
inline engine::scene::EnvironmentMap makeUniformEnvironment() {
    constexpr int kWidth = 64;
    constexpr int kHeight = 32;
    return engine::scene::EnvironmentMap(makeImageTexture<3>(
        kWidth, kHeight, std::vector<float>(static_cast<std::size_t>(kWidth) * kHeight * 3, 1.0F),
        engine::gfx::ScalarType::Float32));
}

inline engine::gfx::ImageTexture<3> makeConstantTexture(glm::vec3 rgb) {
    return makeImageTexture<3>(1, 1, {rgb.x, rgb.y, rgb.z}, engine::gfx::ScalarType::Float32);
}

inline engine::gfx::ImageTexture<1> makeConstantTexture(float value) {
    return makeImageTexture<1>(1, 1, {value}, engine::gfx::ScalarType::Float32);
}

// 1x1 textures carrying the neutral values resolveBsdfParams/buildShadingFrame expect: a flat tangent-space normal
// (0.5,0.5,1), the requested roughness, and f0 in the specular slot. Callers set bumpStrength to 0, so the bump
// texture's value is irrelevant.
inline engine::scene::Material makeMaterial(float roughness, glm::vec3 f0) {
    return engine::scene::Material{
        makeConstantTexture(glm::vec3(1.0F)),              // baseColor -- white, worst case
        makeConstantTexture(glm::vec3(0.5F, 0.5F, 1.0F)),  // normal -- flat
        makeConstantTexture(0.5F),                         // bump -- unused, bumpStrength 0
        makeConstantTexture(roughness),                    // roughness
        makeConstantTexture(f0),                           // specular -> f0
    };
}

}  // namespace tools::fixtures
