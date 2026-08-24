// Standalone energy-conservation check for Phase 4's shading model
// (pbr.frag): a furnace test illuminates a surface uniformly from every
// direction with constant radiance L0 and asserts the outgoing radiance
// never exceeds L0 -- a surface must never reflect more energy than it
// received. Also checks the SH-9 diffuse IBL path against its one exact
// closed-form case (a uniform environment). No test framework exists in
// this repo -- follows test_pattern.cpp's convention of a plain CLI tool
// with a non-zero exit on failure.
//
// The GGX BRDF terms below are a hand-duplicated C++ re-implementation
// of pbr.frag's GLSL functions of the same name: GL4.1/C++20 have no
// shared-source mechanism without a codegen step, so this is the
// concrete backstop for the DFG polynomial fit and the multi-scatter
// compensation formula -- keep it in sync with pbr.frag by hand.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <random>
#include <vector>

#include <glm/glm.hpp>

#include "engine/gfx/hdr_image.h"
#include "engine/scene/sh_irradiance.h"

namespace {

constexpr float kPi = 3.14159265F;

float distributionGGX(float ndotH, float alpha) {
    const float alpha2 = alpha * alpha;
    const float d = ((ndotH * ndotH) * (alpha2 - 1.0F)) + 1.0F;
    return alpha2 / std::max(kPi * d * d, 1e-8F);
}

float visibilitySmithGGXCorrelated(float ndotV, float ndotL, float alpha) {
    const float alpha2 = alpha * alpha;
    const float lambdaV = ndotL * std::sqrt(((ndotV * ndotV) * (1.0F - alpha2)) + alpha2);
    const float lambdaL = ndotV * std::sqrt(((ndotL * ndotL) * (1.0F - alpha2)) + alpha2);
    return 0.5F / std::max(lambdaV + lambdaL, 1e-8F);
}

glm::vec3 fresnelSchlick(float vdotH, const glm::vec3& f0) {
    return f0 + ((1.0F - f0) * std::pow(std::clamp(1.0F - vdotH, 0.0F, 1.0F), 5.0F));
}

glm::vec2 envBRDFApprox(float roughness, float ndotV) {
    const glm::vec4 c0(-1.0F, -0.0275F, -0.572F, 0.022F);
    const glm::vec4 c1(1.0F, 0.0425F, 1.04F, -0.04F);
    const glm::vec4 r = (roughness * c0) + c1;
    const float a004 = (std::min(r.x * r.x, std::exp2(-9.28F * ndotV)) * r.x) + r.y;
    return (glm::vec2(-1.04F, 1.04F) * a004) + glm::vec2(r.z, r.w);
}

glm::vec3 multiScatterCompensation(const glm::vec3& f0, const glm::vec2& dfg) {
    const float ess = std::clamp(dfg.x + dfg.y, 1e-3F, 1.0F);
    const glm::vec3 compensation = glm::vec3(1.0F) + (f0 * ((1.0F / ess) - 1.0F));
    // Must match pbr.frag's multiScatterCompensation exactly, including
    // the safety ceiling -- see its comment for why it's needed.
    return glm::min(compensation, glm::vec3(1.1F));
}

// Monte Carlo furnace test: uniform hemisphere sampling (pdf=1/(2*pi))
// of a unit-radiance (L0=1) incident field, integrating the full
// diffuse+specular BRDF response. Returns Lo(v), which must not exceed
// 1.0 for an energy-conserving BRDF.
float furnaceReflectance(float roughness, float metallic, float ndotV, std::mt19937& rng) {
    constexpr int kSampleCount = 200000;
    const glm::vec3 baseColor(1.0F);  // worst case: full white albedo
    const glm::vec3 f0 = glm::mix(glm::vec3(0.04F), baseColor, metallic);
    const float alpha = roughness * roughness;

    const glm::vec3 v(std::sqrt(std::max(0.0F, 1.0F - (ndotV * ndotV))), 0.0F, ndotV);
    const glm::vec3 n(0.0F, 0.0F, 1.0F);
    const glm::vec2 dfg = envBRDFApprox(roughness, ndotV);
    const glm::vec3 msComp = multiScatterCompensation(f0, dfg);
    // NdotV-based (not per-sample VdotH-based) diffuse/specular energy
    // split -- must match pbr.frag's evaluateDirectLighting exactly.
    const glm::vec3 kd = (glm::vec3(1.0F) - fresnelSchlick(ndotV, f0)) * (1.0F - metallic);
    const glm::vec3 diffuseTerm = baseColor * kd / kPi;

    std::uniform_real_distribution<float> unit(0.0F, 1.0F);
    glm::vec3 accum(0.0F);
    for (int i = 0; i < kSampleCount; ++i) {
        // Uniform hemisphere sampling (PBRT-style inversion): z=u1,
        // r=sqrt(1-u1^2), phi=2*pi*u2, pdf=1/(2*pi).
        const float cosTheta = unit(rng);
        const float sinTheta = std::sqrt(std::max(0.0F, 1.0F - (cosTheta * cosTheta)));
        const float phi = 2.0F * kPi * unit(rng);
        const glm::vec3 l(sinTheta * std::cos(phi), sinTheta * std::sin(phi), cosTheta);

        const float ndotL = l.z;
        if (ndotL <= 0.0F) {
            continue;
        }
        const glm::vec3 h = glm::normalize(v + l);
        const float ndotH = std::max(glm::dot(n, h), 0.0F);
        const float vdotH = std::max(glm::dot(v, h), 0.0F);

        const float d = distributionGGX(ndotH, alpha);
        const float vis = visibilitySmithGGXCorrelated(ndotV, ndotL, alpha);
        const glm::vec3 fr = fresnelSchlick(vdotH, f0);
        const glm::vec3 specularTerm = (d * vis) * fr * msComp;

        // Estimator: (1/N) * sum[BRDF * L0 * ndotL / pdf], pdf=1/(2*pi), L0=1.
        accum += (diffuseTerm + specularTerm) * ndotL * (2.0F * kPi);
    }
    const glm::vec3 lo = accum / static_cast<float>(kSampleCount);
    return std::max({lo.x, lo.y, lo.z});
}

bool checkPunctualSweep() {
    std::mt19937 rng(42);
    // Sized to the analytic DFG's measured Ess underestimate at high
    // roughness/near-1 F0 (see multiScatterCompensation's comment), not
    // to Monte Carlo noise: uniform hemisphere sampling under-samples a
    // sharp GGX lobe at low roughness (e.g. 0.05), so those rows read
    // low rather than converge tightly -- harmless for this check (it
    // only asserts an upper bound, and under-reporting can't produce a
    // false failure), but not a precise reading either.
    constexpr float kTolerance = 0.12F;
    const std::array<float, 5> roughnesses = {0.05F, 0.25F, 0.5F, 0.75F, 1.0F};
    const std::array<float, 2> metallics = {0.0F, 1.0F};
    const std::array<float, 4> ndotVs = {0.15F, 0.4F, 0.7F, 1.0F};

    bool ok = true;
    for (float roughness : roughnesses) {
        for (float metallic : metallics) {
            for (float ndotV : ndotVs) {
                const float reflectance = furnaceReflectance(roughness, metallic, ndotV, rng);
                if (reflectance > 1.0F + kTolerance) {
                    std::cerr << "furnace_test: FAILED energy conservation at roughness="
                              << roughness << " metallic=" << metallic << " ndotV=" << ndotV
                              << " reflectance=" << reflectance << '\n';
                    ok = false;
                }
            }
        }
    }
    return ok;
}

// Uniform-environment SH check: with constant environment radiance L0,
// irradiance must equal exactly pi*L0 (the hemispherical integral of a
// constant times cos(theta)), independent of surface normal -- the one
// exact closed-form case for projectIrradianceSH9/evaluateIrradianceSH9.
bool checkUniformEnvironmentSH() {
    constexpr int kWidth = 256;
    constexpr int kHeight = 128;
    constexpr float kL0 = 2.0F;

    engine::gfx::HdrImage image;
    image.width = kWidth;
    image.height = kHeight;
    image.rgba.assign(static_cast<std::size_t>(kWidth) * static_cast<std::size_t>(kHeight) * 4,
                       kL0);

    const std::array<glm::vec3, 9> coeffs = engine::scene::projectIrradianceSH9(image);
    const glm::vec3 expected(kPi * kL0);

    bool ok = true;
    const std::array<glm::vec3, 3> testNormals = {glm::vec3(0.0F, 1.0F, 0.0F),
                                                    glm::vec3(1.0F, 0.0F, 0.0F),
                                                    glm::vec3(0.0F, 0.0F, -1.0F)};
    for (const glm::vec3& n : testNormals) {
        const glm::vec3 irradiance = engine::scene::evaluateIrradianceSH9(n, coeffs);
        const glm::vec3 diff = glm::abs(irradiance - expected);
        // 2% relative tolerance for the grid quadrature's discretization
        // error at this resolution.
        if (diff.x > 0.02F * expected.x || diff.y > 0.02F * expected.y ||
            diff.z > 0.02F * expected.z) {
            std::cerr << "furnace_test: FAILED uniform-environment SH check, expected "
                      << expected.x << " got (" << irradiance.x << ", " << irradiance.y << ", "
                      << irradiance.z << ")\n";
            ok = false;
        }
    }
    return ok;
}

}  // namespace

int main() {
    const bool punctualOk = checkPunctualSweep();
    const bool shOk = checkUniformEnvironmentSH();

    if (!punctualOk || !shOk) {
        std::cerr << "furnace_test: FAILED\n";
        return EXIT_FAILURE;
    }
    std::cout << "furnace_test: PASSED\n";
    return EXIT_SUCCESS;
}
