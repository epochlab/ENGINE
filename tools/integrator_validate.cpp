// Standalone correctness check for path_tracer.cpp's INTEGRATOR (renderPathTraced/tracePath), as distinct from bsdf_validate.cpp/nee_validate.cpp which exercise the BSDF and the MIS weighting in isolation and never run a real trace. Same standalone-CLI convention: no test framework, non-zero exit on failure.
//
// Reference configuration: one large unoccluded quad under a uniform-radiance (L0=1) environment. Because nothing else is in the scene, every ray leaving the surface reaches the environment directly -- there is NO indirect light -- so the converged radiance is exactly the single-scatter direct lighting, Lo(wo) = integral over the hemisphere of evaluateBsdf(wo,wi)*cos(wi) dwi, the same analytic quantity nee_validate.cpp's referenceLo computes.
//
// Two invariants follow, and they catch different classes of integrator bug than a BSDF-level furnace test can:
//
//   1. DEPTH INVARIANCE. With no indirect light, maxBounces=0 and maxBounces=1 must produce the SAME image. A depth cap that drops the terminal BSDF-sampled ray breaks this: at maxBounces=0 the ray built at bounce 0 is never intersected, so NEE's MIS weight (lightPdf^2/(lightPdf^2+bsdfPdf^2)) is never complemented by the BSDF-sampling half and the surface renders too dark, while maxBounces=1 traces that ray at bounce 1 and is complete. The gap is exactly bsdfPdf^2/(bsdfPdf^2+lightPdf^2) of the direct lighting -- large on a glossy surface.
//
//   2. ABSOLUTE AGREEMENT with the analytic reference, which no self-consistency check between two renderer settings can give on its own.
//
// Russian roulette is exercised as a third case: it reweights by 1/p on survival, so an RR-enabled render must return the same answer as an RR-disabled one. RR lives in tracePath, so this is the only place it can be tested.
//
// Note: Material/MeshInstance are plain data (six HdrImage members and a mat4) and need no GL context -- an earlier comment in nee_validate.cpp claimed otherwise, which is why the suite had no integrator-level test until now.

#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "engine/gfx/hdr_image.h"
#include "engine/scene/bsdf.h"
#include "engine/scene/camera.h"
#include "engine/scene/embree_accel.h"
#include "engine/scene/environment_map.h"
#include "engine/scene/gltf_loader.h"
#include "engine/scene/path_tracer.h"
#include "engine/scene/row_thread_pool.h"
#include "engine/scene/shading_scene.h"

namespace {

using engine::scene::BsdfParams;
using engine::scene::Camera;
using engine::scene::EmbreeAccel;
using engine::scene::EnvironmentMap;
using engine::scene::MeshInstance;
using engine::scene::PathTraceSettings;
using engine::scene::ShadingTriangle;
using engine::scene::ShadingVertex;
using engine::scene::Triangle;

constexpr float kPi = 3.14159265F;

// Quad half-extent: large enough that every primary ray in the narrow test FOV lands on it, so no pixel sees the environment directly and the measured value is purely surface radiance.
constexpr float kQuadExtent = 1000.0F;
// Narrow FOV (200mm on a 36x24 gate, ~6.9 degrees vertical) so every pixel's view direction is within a fraction of a degree of the quad normal -- lets one analytic reference at wo = the normal stand for the whole probed region.
constexpr float kFocalLengthMm = 200.0F;
constexpr int kImageSize = 16;
constexpr int kSamplesPerPixel = 512;

engine::gfx::HdrImage makeConstantTexture(glm::vec3 rgb) {
    engine::gfx::HdrImage image;
    image.width = 1;
    image.height = 1;
    image.rgba = {rgb.x, rgb.y, rgb.z, 1.0F};
    return image;
}

// 1x1 textures carrying the neutral values resolveBsdfParams/buildShadingFrame expect: a flat tangent-space normal (0.5,0.5,1), the requested roughness in .r, and f0 in the specular slot. bumpStrength is set to 0 in the settings below, so the bump texture's value is irrelevant.
engine::scene::Material makeMaterial(float roughness, glm::vec3 f0) {
    return engine::scene::Material{
        makeConstantTexture(glm::vec3(1.0F)),                  // baseColor -- white, worst case
        makeConstantTexture(glm::vec3(0.5F, 0.5F, 1.0F)),      // normal -- flat
        makeConstantTexture(glm::vec3(0.5F)),                  // bump -- unused, bumpStrength 0
        makeConstantTexture(glm::vec3(roughness)),             // roughness
        makeConstantTexture(f0),                               // specular -> f0
        makeConstantTexture(glm::vec3(1.0F)),                  // AO -- unoccluded
    };
}

// One quad in the z=0 plane facing +Z, wound counter-clockwise as seen from +Z so geometricNormalOf gives (0,0,1). The camera sits at +Z looking down -Z (yaw=0/pitch=0, this codebase's default orientation), so the centre pixel's view direction is exactly the surface normal.
struct QuadScene {
    std::vector<Triangle> worldTriangles;
    std::vector<ShadingTriangle> shadingTriangles;
    std::vector<MeshInstance> instances;
};

QuadScene makeQuadScene(float roughness, glm::vec3 f0) {
    const glm::vec3 normal(0.0F, 0.0F, 1.0F);
    const glm::vec4 tangent(1.0F, 0.0F, 0.0F, 1.0F);
    const auto vertex = [&](float x, float y) {
        return ShadingVertex{glm::vec3(x, y, 0.0F), normal, glm::vec2(0.5F, 0.5F), tangent};
    };
    const ShadingVertex v0 = vertex(-kQuadExtent, -kQuadExtent);
    const ShadingVertex v1 = vertex(kQuadExtent, -kQuadExtent);
    const ShadingVertex v2 = vertex(kQuadExtent, kQuadExtent);
    const ShadingVertex v3 = vertex(-kQuadExtent, kQuadExtent);

    QuadScene scene;
    scene.worldTriangles = {Triangle{v0.position, v1.position, v2.position},
                             Triangle{v0.position, v2.position, v3.position}};
    scene.shadingTriangles = {ShadingTriangle{v0, v1, v2, 0}, ShadingTriangle{v0, v2, v3, 0}};
    scene.instances = {MeshInstance{makeMaterial(roughness, f0), glm::mat4(1.0F)}};
    return scene;
}

EnvironmentMap makeUniformEnvironment() {
    engine::gfx::HdrImage image;
    image.width = 64;
    image.height = 32;
    image.rgba.assign(static_cast<std::size_t>(image.width) * static_cast<std::size_t>(image.height) * 4,
                       1.0F);
    return EnvironmentMap(std::move(image));
}

// Only `metallic` travels through PathTraceSettings; roughness and f0 reach the renderer through the
// material's 1x1 roughness and specular textures, which resolveBsdfParams samples (gbuffer_shading.cpp).
PathTraceSettings makeSettings(int maxBounces, int rrStartBounce, float metallic) {
    PathTraceSettings settings{};
    settings.samplesPerPixel = kSamplesPerPixel;
    settings.maxBounces = maxBounces;
    settings.russianRouletteStartBounce = rrStartBounce;
    settings.bumpStrength = 0.0F;
    settings.roughnessMin = 0.0F;
    settings.roughnessMax = 1.0F;
    settings.diffuseColour = glm::vec3(1.0F);
    settings.ior = 1.5F;
    settings.transmissionFactor = 0.0F;
    settings.metallicFactor = metallic;
    settings.roughnessFactor = 1.0F;
    return settings;
}

Camera makeCamera() {
    return Camera(glm::vec3(0.0F, 0.0F, 5.0F), 0.0F, 0.0F, Camera::FilmBack{36.0F, 24.0F},
                   kFocalLengthMm, 0.01F, 1000.0F, 2.8F, 1.0F / 125.0F, 100.0F);
}

// Mean radiance over the centre 4x4 block -- averaging several pixels tightens the estimate without widening the view-direction spread enough to matter at this FOV.
glm::vec3 centreMean(const engine::gfx::HdrImage& image) {
    glm::vec3 sum(0.0F);
    int count = 0;
    for (int y = (kImageSize / 2) - 2; y < (kImageSize / 2) + 2; ++y) {
        for (int x = (kImageSize / 2) - 2; x < (kImageSize / 2) + 2; ++x) {
            const std::size_t idx =
                ((static_cast<std::size_t>(y) * static_cast<std::size_t>(image.width)) +
                 static_cast<std::size_t>(x)) *
                4;
            sum += glm::vec3(image.rgba[idx + 0], image.rgba[idx + 1], image.rgba[idx + 2]);
            ++count;
        }
    }
    return sum / static_cast<float>(count);
}

// Independent ground truth, identical in form to nee_validate.cpp's referenceLo: Lo(wo) = integral over the hemisphere of evaluateBsdf(wo,wi)*wi.z dwi, with L0=1. Uniform-hemisphere Monte Carlo, so it under-samples a sharp GGX peak -- callers restrict the tight comparison to roughness values where it converges.
float referenceLo(const BsdfParams& params, const glm::vec3& wo, int sampleCount, std::mt19937& rng) {
    std::uniform_real_distribution<float> unit(0.0F, 1.0F);
    constexpr float kUniformPdf = 1.0F / (2.0F * kPi);
    glm::vec3 accum(0.0F);
    for (int i = 0; i < sampleCount; ++i) {
        const float cosTheta = unit(rng);
        const float sinTheta = std::sqrt(std::max(0.0F, 1.0F - (cosTheta * cosTheta)));
        const float phi = 2.0F * kPi * unit(rng);
        const glm::vec3 wi(sinTheta * std::cos(phi), sinTheta * std::sin(phi), cosTheta);
        accum += engine::scene::evaluateBsdf(params, wo, wi) * wi.z / kUniformPdf;
    }
    return std::max({accum.x, accum.y, accum.z}) / static_cast<float>(sampleCount);
}

// Runs one full renderPathTraced pass over the quad scene and returns the centre-region mean radiance.
float renderCentre(const QuadScene& scene, const EnvironmentMap& env, const PathTraceSettings& settings,
                    EmbreeAccel& accel, engine::scene::RowThreadPool& pool) {
    const std::atomic<std::uint64_t> generation{1};
    const engine::scene::PathTraceResult result = engine::scene::renderPathTraced(
        makeCamera(), accel, scene.shadingTriangles, scene.instances, env, kImageSize, kImageSize,
        /*envRotationRadians=*/0.0F, /*showSky=*/true, /*envExposure=*/1.0F, settings,
        /*runSeed=*/7U, generation, /*requestedGeneration=*/1U, pool);
    const glm::vec3 mean = centreMean(result.beauty);
    return std::max({mean.x, mean.y, mean.z});
}

struct Case {
    const char* name;
    float roughness;
    float metallic;
    glm::vec3 f0;
};

bool runCases() {
    // Roughness restricted to values where the uniform-hemisphere reference converges (same limitation nee_validate.cpp documents for its own tight two-sided check); a sharp low-roughness lobe biases the reference low and would produce false failures.
    const std::array<Case, 4> cases{{
        {"diffuse (metallic 0, rough 1.0)", 1.0F, 0.0F, glm::vec3(0.04F)},
        {"glossy dielectric (rough 0.35)", 0.35F, 0.0F, glm::vec3(0.04F)},
        {"rough conductor (rough 0.5)", 0.5F, 1.0F, glm::vec3(1.0F)},
        {"rough conductor (rough 0.25)", 0.25F, 1.0F, glm::vec3(1.0F)},
    }};

    // Depth invariance is exact up to Monte Carlo noise, so it gets the tighter bound. Absolute agreement is looser: the uniform-hemisphere reference converges slowly, and single-scatter GGX legitimately loses energy at high roughness (Heitz et al. 2016), which the renderer reproduces faithfully and the reference does not correct for -- both sides compute the same single-scatter BSDF, so they agree, but only to within the reference's own noise.
    constexpr float kDepthInvarianceTolerance = 0.02F;
    constexpr float kReferenceTolerance = 0.06F;
    constexpr int kReferenceSamples = 400000;

    const EnvironmentMap env = makeUniformEnvironment();
    engine::scene::RowThreadPool pool;
    std::mt19937 referenceRng(99);
    bool ok = true;

    std::cout << "integrator_validate: quad under uniform L0=1 environment, wo = surface normal\n";
    std::cout << "  case                              maxB=0    maxB=1    RR on    reference\n";

    for (const Case& testCase : cases) {
        const QuadScene scene = makeQuadScene(testCase.roughness, testCase.f0);
        std::optional<EmbreeAccel> accel = EmbreeAccel::build(scene.worldTriangles);
        if (!accel.has_value()) {
            std::cerr << "integrator_validate: FAILED to build Embree scene for " << testCase.name << '\n';
            return false;
        }

        // rrStartBounce far above maxBounces disables Russian roulette for the first two renders, so depth invariance is measured without RR's extra variance folded in.
        const float loDepth0 = renderCentre(
            scene, env, makeSettings(0, 999, testCase.metallic),
            *accel, pool);
        const float loDepth1 = renderCentre(
            scene, env, makeSettings(1, 999, testCase.metallic),
            *accel, pool);
        // RR from bounce 0, same scene: reweighting by 1/p must leave the expectation unchanged.
        const float loRoulette = renderCentre(
            scene, env, makeSettings(1, 0, testCase.metallic),
            *accel, pool);

        const BsdfParams params{glm::vec3(1.0F), testCase.metallic, testCase.roughness, testCase.f0,
                                 1.5F, 0.0F};
        const float reference = referenceLo(params, glm::vec3(0.0F, 0.0F, 1.0F), kReferenceSamples,
                                             referenceRng);

        std::cout << "  " << testCase.name;
        for (std::size_t pad = std::string(testCase.name).size(); pad < 34; ++pad) {
            std::cout << ' ';
        }
        std::cout << loDepth0 << "  " << loDepth1 << "  " << loRoulette << "  " << reference << '\n';

        const float scale = std::max(reference, 0.05F);
        if (std::fabs(loDepth0 - loDepth1) > kDepthInvarianceTolerance * scale) {
            std::cerr << "integrator_validate: FAILED depth invariance at " << testCase.name
                      << " -- maxBounces=0 gave " << loDepth0 << ", maxBounces=1 gave " << loDepth1
                      << ". With no indirect light these must match; a gap means the terminal "
                         "BSDF-sampled ray is not being traced, so NEE's MIS weight is never "
                         "complemented.\n";
            ok = false;
        }
        if (std::fabs(loRoulette - loDepth1) > kDepthInvarianceTolerance * scale) {
            std::cerr << "integrator_validate: FAILED Russian roulette invariance at " << testCase.name
                      << " -- RR on gave " << loRoulette << ", RR off gave " << loDepth1
                      << ". RR reweights by 1/p and must not change the expectation.\n";
            ok = false;
        }
        if (std::fabs(loDepth1 - reference) > kReferenceTolerance * scale) {
            std::cerr << "integrator_validate: FAILED reference agreement at " << testCase.name
                      << " -- rendered " << loDepth1 << ", analytic " << reference << '\n';
            ok = false;
        }
    }
    return ok;
}

}  // namespace

int main() {
    if (!runCases()) {
        std::cerr << "integrator_validate: FAILED\n";
        return EXIT_FAILURE;
    }
    std::cout << "integrator_validate: PASSED\n";
    return EXIT_SUCCESS;
}
