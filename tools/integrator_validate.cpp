// Standalone correctness check for path_tracer.cpp's integrator (renderPathTraced/tracePath), distinct from bsdf_validate.cpp/nee_validate.cpp which exercise the BSDF and MIS weighting in isolation and never run a real trace. Same standalone-CLI convention: no test framework, non-zero exit on failure.
// Reference configuration: one large unoccluded quad under a uniform-radiance (L0=1) environment. Nothing else is in the scene, so every ray leaving the surface reaches the environment directly (no indirect light), and the converged radiance is exactly the single-scatter direct lighting, Lo(wo) = integral over the hemisphere of evaluateBsdf(wo,wi)*cos(wi) dwi, the same quantity nee_validate.cpp's referenceLo computes.
// Two invariants follow, catching different integrator bugs than a BSDF-level furnace test can:
//   1. Depth invariance: with no indirect light, maxBounces=0 and maxBounces=1 must produce the same image. A depth cap dropping the terminal BSDF-sampled ray breaks this: at maxBounces=0 the ray built at bounce 0 is never intersected, so NEE's MIS weight (lightPdf^2/(lightPdf^2+bsdfPdf^2)) is never complemented by the BSDF-sampling half and the surface renders too dark, while maxBounces=1 traces that ray at bounce 1 and is complete; the gap is exactly bsdfPdf^2/(bsdfPdf^2+lightPdf^2) of the direct lighting, large on a glossy surface.
//   2. Absolute agreement with the analytic reference, which no self-consistency check between two renderer settings can give on its own.
// Russian roulette is exercised as a third case: it reweights by 1/p on survival, so an RR-enabled render must return the same answer as an RR-disabled one; RR lives in tracePath, so this is the only place it can be tested.
// Material/MeshInstance are plain data (six HdrImage members and a mat4) and need no GL context: an earlier comment in nee_validate.cpp claimed otherwise, which is why this suite had no integrator-level test until now.

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
#include "engine/scene/shading_scene.h"
#include "engine/scene/thread_pool.h"

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
// Sphere test geometry, shared by the two checks that use it so their tessellations cannot drift apart -- checkBeerLambert's expected transmittance is a function of the chord, so the two must agree on the radius.
constexpr float kSphereRadius = 1.0F;
constexpr int kSphereSlices = 64;
constexpr int kSphereStacks = 32;
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

struct TestScene {
    std::vector<Triangle> worldTriangles;
    std::vector<ShadingTriangle> shadingTriangles;
    std::vector<MeshInstance> instances;
};

// One quad in the z=0 plane facing +Z, wound counter-clockwise as seen from +Z so geometricNormalOf gives (0,0,1). The camera sits at +Z looking down -Z (yaw=0/pitch=0, this codebase's default orientation), so the centre pixel's view direction is exactly the surface normal.
TestScene makeQuadScene(float roughness, glm::vec3 f0) {
    const glm::vec3 normal(0.0F, 0.0F, 1.0F);
    const glm::vec4 tangent(1.0F, 0.0F, 0.0F, 1.0F);
    const auto vertex = [&](float x, float y) {
        return ShadingVertex{glm::vec3(x, y, 0.0F), normal, glm::vec2(0.5F, 0.5F), tangent};
    };
    const ShadingVertex v0 = vertex(-kQuadExtent, -kQuadExtent);
    const ShadingVertex v1 = vertex(kQuadExtent, -kQuadExtent);
    const ShadingVertex v2 = vertex(kQuadExtent, kQuadExtent);
    const ShadingVertex v3 = vertex(-kQuadExtent, kQuadExtent);

    TestScene scene;
    scene.worldTriangles = {Triangle{v0.position, v1.position, v2.position},
                             Triangle{v0.position, v2.position, v3.position}};
    scene.shadingTriangles = {ShadingTriangle{v0, v1, v2, 0}, ShadingTriangle{v0, v2, v3, 0}};
    scene.instances = {MeshInstance{makeMaterial(roughness, f0), glm::mat4(1.0F), ""}};
    return scene;
}

// Two parallel quads with opposing geometric normals: the front facing the camera at +Z, the back facing away at -Z. A camera ray enters at the front face (woLocal.z>0) and leaves at the back (woLocal.z<0), the only configuration that exercises bsdf.cpp's exiting side and the far-side NEE guard. A single quad cannot: it is entered from the front and every hit reads as entering.
TestScene makeSlabScene(float roughness, glm::vec3 f0, float thickness) {
    const glm::vec4 tangent(1.0F, 0.0F, 0.0F, 1.0F);
    const auto vertex = [&](float x, float y, float z, float nz) {
        return ShadingVertex{glm::vec3(x, y, z), glm::vec3(0.0F, 0.0F, nz), glm::vec2(0.5F, 0.5F),
                              tangent};
    };
    // Front wound counter-clockwise as seen from +Z, back clockwise, so geometricNormalOf gives +Z and -Z.
    const ShadingVertex fv0 = vertex(-kQuadExtent, -kQuadExtent, 0.0F, 1.0F);
    const ShadingVertex fv1 = vertex(kQuadExtent, -kQuadExtent, 0.0F, 1.0F);
    const ShadingVertex fv2 = vertex(kQuadExtent, kQuadExtent, 0.0F, 1.0F);
    const ShadingVertex fv3 = vertex(-kQuadExtent, kQuadExtent, 0.0F, 1.0F);
    const ShadingVertex bv0 = vertex(-kQuadExtent, -kQuadExtent, -thickness, -1.0F);
    const ShadingVertex bv1 = vertex(-kQuadExtent, kQuadExtent, -thickness, -1.0F);
    const ShadingVertex bv2 = vertex(kQuadExtent, kQuadExtent, -thickness, -1.0F);
    const ShadingVertex bv3 = vertex(kQuadExtent, -kQuadExtent, -thickness, -1.0F);

    TestScene scene;
    scene.worldTriangles = {Triangle{fv0.position, fv1.position, fv2.position},
                             Triangle{fv0.position, fv2.position, fv3.position},
                             Triangle{bv0.position, bv1.position, bv2.position},
                             Triangle{bv0.position, bv2.position, bv3.position}};
    scene.shadingTriangles = {ShadingTriangle{fv0, fv1, fv2, 0}, ShadingTriangle{fv0, fv2, fv3, 0},
                               ShadingTriangle{bv0, bv1, bv2, 0}, ShadingTriangle{bv0, bv2, bv3, 0}};
    scene.instances = {MeshInstance{makeMaterial(roughness, f0), glm::mat4(1.0F), ""}};
    return scene;
}

// The quad above plus an opaque wall at x=1 facing -X, the only scene here where a non-transmissive path reaches a second surface: the wall sits outside the narrow view frustum (primary rays land within |x|<0.31 at z=0) so it is never primary-visible, but it catches the floor's +X-going bounce rays, and NEE fires there at bounce>=1, the only way anything reaches the Indirect buckets.
TestScene makeCornerScene(float roughness, glm::vec3 f0) {
    TestScene scene = makeQuadScene(roughness, f0);
    const glm::vec3 wallNormal(-1.0F, 0.0F, 0.0F);
    const glm::vec4 wallTangent(0.0F, 1.0F, 0.0F, 1.0F);
    const auto vertex = [&](float y, float z) {
        return ShadingVertex{glm::vec3(1.0F, y, z), wallNormal, glm::vec2(0.5F, 0.5F), wallTangent};
    };
    // Wound so geometricNormalOf gives -X, i.e. facing back across the floor rather than away from it.
    const ShadingVertex w0 = vertex(-kQuadExtent, 0.0F);
    const ShadingVertex w1 = vertex(kQuadExtent, 0.0F);
    const ShadingVertex w2 = vertex(kQuadExtent, kQuadExtent);
    const ShadingVertex w3 = vertex(-kQuadExtent, kQuadExtent);

    scene.worldTriangles.push_back(Triangle{w0.position, w3.position, w2.position});
    scene.worldTriangles.push_back(Triangle{w0.position, w2.position, w1.position});
    scene.shadingTriangles.push_back(ShadingTriangle{w0, w3, w2, 0});
    scene.shadingTriangles.push_back(ShadingTriangle{w0, w2, w1, 0});
    return scene;
}

// UV sphere at the origin, poles on Y so the camera (at +Z looking down -Z) reads the well-tessellated equator rather than the degenerate pole fan. Vertex normals are the exact analytic outward normals and the tangent is d(position)/d(theta), non-degenerate everywhere including the poles.
// THE POINT IS THE CURVATURE. Every other scene here is flat quads, whose coplanar vertex normals make transmissionOffsetEpsilon's curvature factor exactly zero and shadowTerminatorOffset a no-op -- so the two integrator bugs those two mechanisms actually had (a far-side NEE shadow ray on the flat kRayEpsilon, and the shadow-terminator projection pushing an inward-going ray back out through the interface and desynchronising the medium stack) were invisible to all four suites while plainly visible in a cornell render. Here both are non-zero.
// The tessellation sets the offset epsilon (max edge length * sin of the vertex-normal divergence, both taken across the quad's diagonal), which backs each ray origin that much into the medium and so shortens every measured in-medium segment. At 64x32 on a unit sphere that is 1.93e-2. Coarse enough that the curvature terms are firmly non-zero, fine enough that checkBeerLambert's analytic chord stays accurate; the detection strength this costs was measured, not assumed, by re-running the checks at 24x12 (528 triangles, cornell's own tessellation), where they behave the same.
TestScene makeSphereScene(float roughness, glm::vec3 f0) {
    const auto vertexAt = [&](int stack, int slice) {
        const float phi = kPi * static_cast<float>(stack) / static_cast<float>(kSphereStacks);
        const float theta = 2.0F * kPi * static_cast<float>(slice) / static_cast<float>(kSphereSlices);
        const glm::vec3 normal(std::sin(phi) * std::sin(theta), std::cos(phi),
                                std::sin(phi) * std::cos(theta));
        return ShadingVertex{normal * kSphereRadius, normal, glm::vec2(0.5F, 0.5F),
                              glm::vec4(std::cos(theta), 0.0F, -std::sin(theta), 1.0F)};
    };

    TestScene scene;
    for (int stack = 0; stack < kSphereStacks; ++stack) {
        for (int slice = 0; slice < kSphereSlices; ++slice) {
            const ShadingVertex v00 = vertexAt(stack, slice);
            const ShadingVertex v10 = vertexAt(stack + 1, slice);
            const ShadingVertex v11 = vertexAt(stack + 1, slice + 1);
            const ShadingVertex v01 = vertexAt(stack, slice + 1);
            // Wound so geometricNormalOf points outward. Each pole row contributes one triangle, not two: the half-quad whose two vertices collapse onto the pole is dropped rather than emitted with zero area, which would normalize(0) to NaN in geometricNormalOf. Guarding the wrong half of each row leaves an annular hole at both poles (measured: 128 zero-area triangles, surface area 12.4808 against 4*pi) and silently unseals the medium.
            if (stack + 1 < kSphereStacks) {
                scene.worldTriangles.push_back(Triangle{v00.position, v10.position, v11.position});
                scene.shadingTriangles.push_back(ShadingTriangle{v00, v10, v11, 0});
            }
            if (stack > 0) {
                scene.worldTriangles.push_back(Triangle{v00.position, v11.position, v01.position});
                scene.shadingTriangles.push_back(ShadingTriangle{v00, v11, v01, 0});
            }
        }
    }
    scene.instances = {MeshInstance{makeMaterial(roughness, f0), glm::mat4(1.0F), ""}};
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

// Only `metallic` travels through PathTraceSettings; roughness and f0 reach the renderer through the material's 1x1 roughness and specular textures, which resolveBsdfParams samples (gbuffer_shading.cpp).
PathTraceSettings makeSettings(int maxBounces, int rrStartBounce, float metallic,
                               float transmission = 0.0F) {
    PathTraceSettings settings{};
    settings.samplesPerPixel = kSamplesPerPixel;
    settings.maxBounces = maxBounces;
    settings.russianRouletteStartBounce = rrStartBounce;
    settings.bumpStrength = 0.0F;
    settings.roughnessMin = 0.0F;
    settings.roughnessMax = 1.0F;
    settings.diffuseColour = glm::vec3(1.0F);
    settings.ior = 1.5F;
    settings.transmissionFactor = transmission;
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

// Runs one full renderPathTraced pass over the quad scene. showSky gates only the primary ray's own miss, so turning it off zeroes the background term the transport buckets deliberately exclude.
engine::scene::PathTraceResult renderPass(const TestScene& scene, const EnvironmentMap& env,
                                           const PathTraceSettings& settings, EmbreeAccel& accel,
                                           engine::scene::ThreadPool& pool, bool showSky) {
    const std::atomic<std::uint64_t> generation{1};
    engine::scene::PathTraceResult result =
        engine::scene::makePathTraceResult(kImageSize, kImageSize);
    const std::vector<PathTraceSettings> perInstanceSettings(scene.instances.size(), settings);
    engine::scene::renderPathTraced(makeCamera(), accel, scene.shadingTriangles, scene.instances, env,
                                     kImageSize, kImageSize, /*envRotationRadians=*/0.0F, showSky,
                                     /*envExposure=*/1.0F, settings, perInstanceSettings, /*runSeed=*/7U,
                                     generation, /*requestedGeneration=*/1U, pool, result);
    return result;
}

// Centre-region mean radiance of one pass.
float renderCentre(const TestScene& scene, const EnvironmentMap& env, const PathTraceSettings& settings,
                    EmbreeAccel& accel, engine::scene::ThreadPool& pool) {
    const glm::vec3 mean = centreMean(renderPass(scene, env, settings, accel, pool, true).beauty);
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
    engine::scene::ThreadPool pool;
    std::mt19937 referenceRng(99);
    bool ok = true;

    std::cout << "integrator_validate: quad under uniform L0=1 environment, wo = surface normal\n";
    std::cout << "  case                              maxB=0    maxB=1    RR on    reference\n";

    for (const Case& testCase : cases) {
        const TestScene scene = makeQuadScene(testCase.roughness, testCase.f0);
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
                                 1.5F, 0.0F, 0.0F};
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

// A white, non-absorbing dielectric slab in a uniform L0=1 environment is invisible: the camera must read exactly 1.0 through it. Every photon entering the front face leaves somewhere, and the non-symmetric eta^2 radiance compression applied on entering is undone on exiting, so the round trip is lossless.
// This is the only case in the suite that reaches a transmissive exiting vertex, gating the far-side NEE guard against the miss branch's MIS weight. Weighting a rough transmission sample at 1.0 (correct only for a delta lobe) while NEE also evaluates the transmission lobe toward the same directions double-counts their overlap, reading above 1.0 here. Both bounds matter: the same test catches a transmissive vertex that loses energy instead.
bool checkTransmissiveSlab() {
    // Enough depth for internally reflected paths to converge; truncation only ever darkens.
    constexpr int kSlabBounces = 12;
    constexpr float kThickness = 0.5F;
    constexpr float kTolerance = 0.03F;
    // 0.02 is below bsdf.cpp's smooth-roughness threshold, so it exercises the delta transmission path; the rest take the Walter lobe.
    const std::array<float, 5> roughnesses = {0.02F, 0.05F, 0.4F, 0.7F, 1.0F};

    const EnvironmentMap env = makeUniformEnvironment();
    engine::scene::ThreadPool pool;
    bool ok = true;

    std::cout << "integrator_validate: white non-absorbing slab, uniform L0=1 (1.0 = invisible)\n";
    for (float roughness : roughnesses) {
        const TestScene scene = makeSlabScene(roughness, glm::vec3(0.04F), kThickness);
        std::optional<EmbreeAccel> accel = EmbreeAccel::build(scene.worldTriangles);
        if (!accel.has_value()) {
            std::cerr << "integrator_validate: FAILED to build Embree slab scene\n";
            return false;
        }
        const float lo = renderCentre(
            scene, env, makeSettings(kSlabBounces, 999, /*metallic=*/0.0F, /*transmission=*/1.0F),
            *accel, pool);
        std::cout << "  roughness " << roughness << "   Lo " << lo << '\n';
        if (std::fabs(lo - 1.0F) > kTolerance) {
            std::cerr << "integrator_validate: FAILED slab transparency at roughness=" << roughness
                      << " -- rendered " << lo
                      << ", expected 1.0. A white non-absorbing slab must neither add nor remove "
                         "energy; above 1.0 means NEE and the BSDF-sampled miss are double-counting "
                         "the transmission lobe, below means a transmissive vertex is losing energy.\n";
            ok = false;
        }
    }
    return ok;
}

// The curved counterpart of checkTransmissiveSlab, on the same invariant for the same reason: a white, non-absorbing dielectric under a uniform L0=1 environment is invisible WHATEVER ITS SHAPE, since every photon entering leaves again and the eta^2 radiance compression cancels over the round trip. Only the geometry changes, and the geometry is the whole point -- this is the suite's only case where a transmissive vertex sees non-zero curvature, so it is the only one that can see a bug in transmissionOffsetEpsilon or in shadowTerminatorOffset's projection side (see makeSphereScene).
// A sphere traps far more light than a slab: past the critical angle every internal hit totally internally reflects, so paths ring around the inside for many bounces. Truncation only ever darkens, which is why the depth is well above the slab's and the band is two-sided.
bool checkTransmissiveSphere() {
    // Measured convergence point, not a guess: 32 and 96 bounces are bit-identical to this, and 12 is not.
    constexpr int kSphereBounces = 16;
    constexpr float kTolerance = 0.03F;
    // 0.02 is below bsdf.cpp's smooth-roughness threshold, so it takes the delta transmission path; the rest take the Walter lobe, where far-side NEE is the estimator that actually lights the far side.
    const std::array<float, 4> roughnesses = {0.02F, 0.2F, 0.4F, 0.7F};

    const EnvironmentMap env = makeUniformEnvironment();
    engine::scene::ThreadPool pool;
    bool ok = true;

    std::cout << "integrator_validate: white non-absorbing glass sphere, uniform L0=1 (1.0 = invisible)\n";
    for (float roughness : roughnesses) {
        const TestScene scene =
            makeSphereScene(roughness, glm::vec3(0.04F));
        std::optional<EmbreeAccel> accel = EmbreeAccel::build(scene.worldTriangles);
        if (!accel.has_value()) {
            std::cerr << "integrator_validate: FAILED to build Embree sphere scene\n";
            return false;
        }
        const float lo = renderCentre(
            scene, env, makeSettings(kSphereBounces, 999, /*metallic=*/0.0F, /*transmission=*/1.0F),
            *accel, pool);
        std::cout << "  roughness " << roughness << "   Lo " << lo << '\n';
        if (std::fabs(lo - 1.0F) > kTolerance) {
            std::cerr << "integrator_validate: FAILED glass sphere transparency at roughness=" << roughness
                      << " -- rendered " << lo
                      << ", expected 1.0. Same invariant as the flat slab above, so a flat slab that "
                         "passes while this fails localises the fault to a curvature-driven term: the "
                         "transmission offset epsilon, or the shadow-terminator projection's side.\n";
            ok = false;
        }
    }
    return ok;
}

// Beer-Lambert volumetric absorption -- the newest thing in the pipeline and, until now, the only part of the transmissive path with no coverage at all: both slabs above are white and non-absorbing, so sigmaAFromTransmission and tracePath's medium attenuation were never once evaluated by this suite.
// Asserts the documented contract rather than restating its formula: transmissionDepth is "the distance at which transmittance reaches transmissionColor" (scene_config.h), so setting transmissionDepth to the actual traversal distance makes the expected reading exactly transmissionColor, with no exp() written in the test at all. Halving transmissionDepth squares it, which is what separates a true exponential from anything linear in distance -- a test at one depth cannot tell the two apart.
// ior 1.0 is what makes this exact rather than approximate, and it is a physically real configuration (an index-matched pure absorber), not a test-only dodge. The interface neither bends the ray -- so the traversal distance is the slab thickness or the sphere chord, both known in closed form -- nor reflects any of it, since fresnelDielectric is identically zero at eta 1. The delta transmission lobe contributes nothing through NEE either. Absorption is then the only mechanism left that can move the reading off 1.0.
// Per-channel colour, distinct in every channel: a swapped or luminance-collapsed sigmaA passes a grey test and fails this one.
bool checkBeerLambert() {
    constexpr int kBounces = 8;
    constexpr float kSlabThickness = 0.5F;
    // Relative, since the squared row's green channel is 0.0625 and an absolute band would be vacuous there.
    // The flat rows are exact to 3e-4 relative. The sphere row reads systematically HIGH, from two named biases that both shorten its path and so under-absorb: the offset epsilon backs the origin 1.93e-2 into the medium (see makeSphereScene), and the probed block's outermost ray has an impact parameter of 0.075 rather than 0, a chord of 1.9944 rather than 2.0. Measured 0.89/1.82/0.35% across the three channels, which divided by each channel's own sigma_a give the same 0.025 path deficit -- one shortened path, matching the 0.019+0.006 predicted, and not a per-channel error. Worst row is therefore 1.8% against this band.
    constexpr float kRelativeTolerance = 0.03F;
    const glm::vec3 colour(0.5F, 0.25F, 0.75F);

    struct AbsorptionCase {
        const char* name;
        bool sphere;
        float depth;
        glm::vec3 expected;
    };
    const std::array<AbsorptionCase, 3> cases{{
        {"flat slab, depth == thickness", false, kSlabThickness, colour},
        {"flat slab, depth == half thickness", false, kSlabThickness * 0.5F, colour * colour},
        {"sphere, depth == chord", true, 2.0F * kSphereRadius, colour},
    }};

    const EnvironmentMap env = makeUniformEnvironment();
    engine::scene::ThreadPool pool;
    bool ok = true;

    std::cout << "integrator_validate: Beer-Lambert absorption through an index-matched medium\n";
    for (const AbsorptionCase& testCase : cases) {
        const TestScene scene =
            testCase.sphere
                ? makeSphereScene(0.02F, glm::vec3(0.04F))
                : makeSlabScene(0.02F, glm::vec3(0.04F), kSlabThickness);
        std::optional<EmbreeAccel> accel = EmbreeAccel::build(scene.worldTriangles);
        if (!accel.has_value()) {
            std::cerr << "integrator_validate: FAILED to build Embree scene for " << testCase.name << '\n';
            return false;
        }
        PathTraceSettings settings = makeSettings(kBounces, 999, /*metallic=*/0.0F, /*transmission=*/1.0F);
        settings.ior = 1.0F;
        settings.transmissionColor = colour;
        settings.transmissionDepth = testCase.depth;
        const glm::vec3 lo = centreMean(renderPass(scene, env, settings, *accel, pool, true).beauty);

        std::cout << "  " << testCase.name;
        for (std::size_t pad = std::string(testCase.name).size(); pad < 36; ++pad) {
            std::cout << ' ';
        }
        std::cout << "measured [" << lo.x << ", " << lo.y << ", " << lo.z << "]   expected ["
                  << testCase.expected.x << ", " << testCase.expected.y << ", "
                  << testCase.expected.z << "]\n";
        for (int c = 0; c < 3; ++c) {
            if (std::fabs(lo[c] - testCase.expected[c]) > kRelativeTolerance * testCase.expected[c]) {
                std::cerr << "integrator_validate: FAILED Beer-Lambert at " << testCase.name
                          << " channel " << c << " -- measured " << lo[c] << ", expected "
                          << testCase.expected[c]
                          << ". transmissionDepth is the distance at which transmittance reaches "
                             "transmissionColor, so with the medium index-matched to vacuum nothing "
                             "but exp(-sigma_a*d) can move this reading.\n";
                ok = false;
            }
        }
    }
    return ok;
}

// The five transport buckets are a partition of beauty, not a set of related-looking images: with the background term zeroed (showSky off), DirectDiffuse + IndirectDiffuse + DirectSpecular + IndirectSpecular + Refraction must equal Beauty at every pixel, to float error.
// Every radiance contribution tracePath adds is written to exactly one bucket at its own physical value, so any gap means a contribution was bucketed twice, dropped, or rescaled, exactly what the previous delighted buckets did by construction (they stripped baseColor at bounce 0 only, leaving direct and indirect in different units and neither summing to anything).
// The slab rows carry the load: a single quad reaches only the Direct buckets, while the slab's internal reflections populate Indirect and Refraction and exercise the transmissive exiting vertex.
bool checkTransportPartition() {
    // Relative to beauty, since the absolute scale differs by case; float error over kSamplesPerPixel accumulations of ~1e-3 each is orders of magnitude below this.
    constexpr float kTolerance = 1e-4F;

    enum class Geometry { Quad, Corner, Slab };
    struct PartitionCase {
        const char* name;
        Geometry geometry;
        float roughness;
        float metallic;
        float transmission;
        int maxBounces;
    };
    const std::array<PartitionCase, 6> cases{{
        {"quad diffuse (rough 1.0)", Geometry::Quad, 1.0F, 0.0F, 0.0F, 1},
        {"quad glossy dielectric (rough 0.35)", Geometry::Quad, 0.35F, 0.0F, 0.0F, 1},
        {"quad rough conductor (rough 0.5)", Geometry::Quad, 0.5F, 1.0F, 0.0F, 2},
        {"corner diffuse (rough 1.0)", Geometry::Corner, 1.0F, 0.0F, 0.0F, 4},
        {"slab smooth glass (rough 0.02)", Geometry::Slab, 0.02F, 0.0F, 1.0F, 8},
        {"slab rough glass (rough 0.4)", Geometry::Slab, 0.4F, 0.0F, 1.0F, 8},
    }};

    const EnvironmentMap env = makeUniformEnvironment();
    engine::scene::ThreadPool pool;
    bool ok = true;

    std::cout << "integrator_validate: transport buckets partition beauty (showSky off)\n";
    for (const PartitionCase& testCase : cases) {
        const TestScene scene =
            testCase.geometry == Geometry::Slab
                ? makeSlabScene(testCase.roughness, glm::vec3(0.04F), 0.5F)
                : testCase.geometry == Geometry::Corner
                      ? makeCornerScene(testCase.roughness, glm::vec3(0.04F))
                      : makeQuadScene(testCase.roughness, glm::vec3(0.04F));
        std::optional<EmbreeAccel> accel = EmbreeAccel::build(scene.worldTriangles);
        if (!accel.has_value()) {
            std::cerr << "integrator_validate: FAILED to build Embree scene for " << testCase.name << '\n';
            return false;
        }
        const engine::scene::PathTraceResult result = renderPass(
            scene, env,
            makeSettings(testCase.maxBounces, 999, testCase.metallic, testCase.transmission), *accel,
            pool, /*showSky=*/false);

        float worstGap = 0.0F;
        float worstBeauty = 0.0F;
        float maxIndirect = 0.0F;
        for (std::size_t i = 0; i < result.beauty.rgba.size(); i += 4) {
            for (std::size_t c = 0; c < 3; ++c) {
                const float beauty = result.beauty.rgba[i + c];
                const float sum = result.directDiffuse.rgba[i + c] + result.indirectDiffuse.rgba[i + c] +
                                   result.directSpecular.rgba[i + c] +
                                   result.indirectSpecular.rgba[i + c] + result.refraction.rgba[i + c];
                const float gap = std::fabs(beauty - sum) / std::max(std::fabs(beauty), 1e-3F);
                if (gap > worstGap) {
                    worstGap = gap;
                    worstBeauty = beauty;
                }
                maxIndirect = std::max({maxIndirect, result.indirectDiffuse.rgba[i + c],
                                         result.indirectSpecular.rgba[i + c]});
            }
        }

        std::cout << "  " << testCase.name;
        for (std::size_t pad = std::string(testCase.name).size(); pad < 38; ++pad) {
            std::cout << ' ';
        }
        std::cout << "worst relative gap " << worstGap << "   max indirect " << maxIndirect << '\n';
        // A flat quad's continuation ray always escapes at bounce 1 (Direct) and the slab's every multi-vertex path is transmission-sticky (Refraction), so without the corner the two Indirect buckets are identically zero in every case and the identity guards nothing about them.
        if (testCase.geometry == Geometry::Corner && maxIndirect <= 0.0F) {
            std::cerr << "integrator_validate: FAILED transport partition coverage -- the corner scene "
                         "produced no Indirect bucket energy, so the identity below is not testing "
                         "them. Check that the wall is being hit by bounce rays.\n";
            ok = false;
        }
        if (worstGap > kTolerance) {
            std::cerr << "integrator_validate: FAILED transport partition at " << testCase.name
                      << " -- worst pixel is off by " << (worstGap * 100.0F) << "% of its beauty value "
                      << worstBeauty
                      << ". The five buckets must sum to beauty exactly once the background term is "
                         "zeroed; a gap means a contribution is unbucketed, double-bucketed or "
                         "rescaled.\n";
            ok = false;
        }
    }
    return ok;
}

}  // namespace

int main() {
    const bool casesOk = runCases();
    const bool slabOk = checkTransmissiveSlab();
    const bool sphereOk = checkTransmissiveSphere();
    const bool absorptionOk = checkBeerLambert();
    const bool partitionOk = checkTransportPartition();
    if (!casesOk || !slabOk || !sphereOk || !absorptionOk || !partitionOk) {
        std::cerr << "integrator_validate: FAILED\n";
        return EXIT_FAILURE;
    }
    std::cout << "integrator_validate: PASSED\n";
    return EXIT_SUCCESS;
}
