// Standalone correctness check for engine::scene::renderRasterGBuffer (rasterizer.h): synthetic dependency-free scene (no glTF/EXR asset -- constant-color 1x1 textures), cross-checked against a fresh pixel-center Embree primary-ray intersection resolved through the same gbuffer_shading.h sampling functions, deliberately independent of renderRasterGBuffer's own code path so agreement is a real cross-check, not a tautology. Same standalone-CLI convention as embree_validate.cpp/bsdf_validate.cpp/nee_validate.cpp: no test framework, non-zero exit on failure. A small fraction of coverage/value mismatches at triangle silhouette edges is tolerated -- an inherent rasterizer-vs-raytracer tie-break difference, not a bug.

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <random>
#include <vector>

#include <glm/glm.hpp>

#include "engine/gfx/hdr_image.h"
#include "engine/scene/camera.h"
#include "engine/scene/embree_accel.h"
#include "engine/scene/false_color.h"
#include "engine/scene/gbuffer_shading.h"
#include "engine/scene/gltf_loader.h"
#include "engine/scene/rasterizer.h"
#include "engine/scene/ray_types.h"
#include "engine/scene/row_thread_pool.h"
#include "engine/scene/shading_scene.h"

namespace {

using namespace engine::scene;  // NOLINT(google-build-using-namespace) -- tool-local convenience, mirrors embree_validate.cpp's using-declarations

constexpr int kWidth = 96;
constexpr int kHeight = 96;
constexpr int kTriangleCount = 150;
constexpr int kMaterialCount = 4;
constexpr float kPosEpsilon = 5e-2F;    // world-space units (worldPos, depth)
constexpr float kUnitEpsilon = 1e-2F;   // unit-vector/[0,1]-range fields (normal, uv, albedo, ...)
constexpr float kMaxCoverageMismatchFraction = 0.02F;
constexpr float kMaxValueMismatchFraction = 0.02F;

engine::gfx::HdrImage constantTexture(glm::vec4 color) {
    engine::gfx::HdrImage image;
    image.width = 1;
    image.height = 1;
    image.rgba = {color.r, color.g, color.b, color.a};
    return image;
}

Material makeMaterial(glm::vec3 baseColor, float roughness, float ao) {
    Material material;
    material.baseColorTexture = constantTexture(glm::vec4(baseColor, 1.0F));
    material.normalTexture = constantTexture(glm::vec4(0.5F, 0.5F, 1.0F, 1.0F));  // tangent-space (0,0,1)
    material.bumpTexture = constantTexture(glm::vec4(0.5F));
    material.roughnessTexture = constantTexture(glm::vec4(roughness));
    material.specularTexture = constantTexture(glm::vec4(0.04F));
    material.aoTexture = constantTexture(glm::vec4(ao));
    return material;
}

glm::vec4 tangentFor(const glm::vec3& normal) {
    const glm::vec3 up = std::fabs(normal.y) < 0.99F ? glm::vec3(0.0F, 1.0F, 0.0F) : glm::vec3(1.0F, 0.0F, 0.0F);
    return glm::vec4(glm::normalize(glm::cross(up, normal)), 1.0F);
}

// Random triangles in [-8,8]x[-8,8] x/y, [-1,-16] z (in front of the origin, looking down -Z) -- overlapping in depth (exercises the z-buffer), dense enough near z=-1 that a camera placed inside the cluster (clipTest below) clips some at the near plane.
std::vector<ShadingTriangle> makeSyntheticTriangles(std::mt19937& rng) {
    std::uniform_real_distribution<float> centerXY(-8.0F, 8.0F);
    std::uniform_real_distribution<float> centerZ(-16.0F, -1.0F);
    std::uniform_real_distribution<float> offset(-1.5F, 1.5F);

    std::vector<ShadingTriangle> triangles;
    triangles.reserve(kTriangleCount);
    for (int i = 0; i < kTriangleCount; ++i) {
        const glm::vec3 center(centerXY(rng), centerXY(rng), centerZ(rng));
        const glm::vec3 p0 = center + glm::vec3(offset(rng), offset(rng), offset(rng) * 0.1F);
        const glm::vec3 p1 = center + glm::vec3(offset(rng), offset(rng), offset(rng) * 0.1F);
        const glm::vec3 p2 = center + glm::vec3(offset(rng), offset(rng), offset(rng) * 0.1F);
        const glm::vec3 normal = glm::normalize(glm::cross(p1 - p0, p2 - p0));
        if (!std::isfinite(normal.x)) {
            continue;  // degenerate (near-collinear) draw -- skip, next iteration redraws a fresh center
        }
        const glm::vec4 tangent = tangentFor(normal);
        ShadingTriangle tri;
        tri.v0 = ShadingVertex{p0, normal, glm::vec2(0.0F, 0.0F), tangent};
        tri.v1 = ShadingVertex{p1, normal, glm::vec2(1.0F, 0.0F), tangent};
        tri.v2 = ShadingVertex{p2, normal, glm::vec2(0.0F, 1.0F), tangent};
        tri.instanceIndex = i % kMaterialCount;
        triangles.push_back(tri);
    }
    return triangles;
}

std::vector<Triangle> worldTrianglesOf(const std::vector<ShadingTriangle>& shadingTriangles) {
    std::vector<Triangle> triangles;
    triangles.reserve(shadingTriangles.size());
    for (const ShadingTriangle& tri : shadingTriangles) {
        triangles.push_back(Triangle{tri.v0.position, tri.v1.position, tri.v2.position});
    }
    return triangles;
}

glm::vec3 texelAt(const engine::gfx::HdrImage& image, int x, int y) {
    const std::size_t idx = ((static_cast<std::size_t>(y) * static_cast<std::size_t>(image.width)) +
                              static_cast<std::size_t>(x)) *
                             4;
    return {image.rgba[idx], image.rgba[idx + 1], image.rgba[idx + 2]};
}

struct FieldCheck {
    const char* name;
    glm::vec3 raster;
    glm::vec3 oracle;
    float epsilon;
};

int checkFields(const std::vector<FieldCheck>& fields, int x, int y, const char* poseName) {
    int mismatches = 0;
    for (const FieldCheck& f : fields) {
        if (glm::any(glm::greaterThan(glm::abs(f.raster - f.oracle), glm::vec3(f.epsilon)))) {
            std::cerr << "rasterizer_validate: " << poseName << " (" << x << "," << y << ") " << f.name
                      << " mismatch (raster=" << f.raster.x << "," << f.raster.y << "," << f.raster.z
                      << " oracle=" << f.oracle.x << "," << f.oracle.y << "," << f.oracle.z << ")\n";
            ++mismatches;
        }
    }
    return mismatches;
}

// Oracle: pixel-center (unjittered) Embree primary-ray intersection, resolved through the same gbuffer_shading.h functions renderRasterGBuffer itself calls -- independent of the rasterizer's own scan-conversion/clipping code, so agreement is a real cross-check.
bool checkPose(const char* poseName, const Camera& camera, const EmbreeAccel& accel,
               const std::vector<ShadingTriangle>& shadingTriangles,
               const std::vector<MeshInstance>& instances, const PathTraceSettings& settings,
               RowThreadPool& threadPool) {
    RasterGBuffer raster;
    renderRasterGBuffer(camera, accel, shadingTriangles, instances, settings, kWidth, kHeight, threadPool, raster);
    const float aspect = static_cast<float>(kWidth) / static_cast<float>(kHeight);
    const glm::vec3 camPos = camera.position();
    const glm::vec3 camForward = camera.forward();

    int coverageMismatches = 0;
    int valueMismatches = 0;
    int hitPixels = 0;
    for (int y = 0; y < kHeight; ++y) {
        for (int x = 0; x < kWidth; ++x) {
            const float ndcX = (((static_cast<float>(x) + 0.5F) / static_cast<float>(kWidth)) * 2.0F) - 1.0F;
            const float ndcY = 1.0F - (((static_cast<float>(y) + 0.5F) / static_cast<float>(kHeight)) * 2.0F);
            const Ray ray = camera.primaryRay(ndcX, ndcY, aspect);
            const std::optional<Hit> hit = accel.intersect(ray);
            const float rasterAlpha = texelAt(raster.alpha, x, y).x;
            const bool rasterHit = rasterAlpha > 0.5F;

            if (rasterHit != hit.has_value()) {
                ++coverageMismatches;
                continue;
            }
            if (!hit.has_value()) {
                continue;
            }
            ++hitPixels;

            const ShadingTriangle& triangle = shadingTriangles[static_cast<std::size_t>(hit->triangleIndex)];
            const Material& material = instances[static_cast<std::size_t>(triangle.instanceIndex)].material;
            const ShadingVertex shading = interpolateShading(triangle, hit->u, hit->v);
            const ShadingFrame frame = buildShadingFrame(shading, material, settings);
            const BsdfParams params = resolveBsdfParams(material, shading.uv, settings);
            const glm::vec3 woWorld = -ray.dir;
            const float ndotV = std::max(glm::dot(frame.normal, woWorld), 1e-4F);
            const float fresnelVal = fresnelSchlick(ndotV, params.f0).x;
            const float aoVal = engine::gfx::sampleBilinear(material.aoTexture, shading.uv).r;
            const float depth = glm::dot(shading.position - camPos, camForward);

            const std::vector<FieldCheck> fields{
                {"depth", texelAt(raster.depth, x, y), glm::vec3(depth), kPosEpsilon},
                {"worldPos", texelAt(raster.worldPos, x, y), shading.position, kPosEpsilon},
                {"uv", texelAt(raster.uv, x, y), glm::vec3(glm::fract(shading.uv), 0.0F), kUnitEpsilon},
                {"normal", texelAt(raster.normal, x, y), frame.normal, kUnitEpsilon},
                {"geomNormal", texelAt(raster.geomNormal, x, y), glm::normalize(shading.normal), kUnitEpsilon},
                {"albedo", texelAt(raster.albedo, x, y), params.baseColor, kUnitEpsilon},
                {"metallic", texelAt(raster.metallic, x, y), glm::vec3(params.metallic), kUnitEpsilon},
                {"roughness", texelAt(raster.roughness, x, y), glm::vec3(params.roughness), kUnitEpsilon},
                {"tangent", texelAt(raster.tangent, x, y), frame.tangent, kUnitEpsilon},
                {"objectId", texelAt(raster.objectId, x, y), falseColorForId(triangle.instanceIndex), kUnitEpsilon},
                {"fresnel", texelAt(raster.fresnel, x, y), glm::vec3(fresnelVal, 1.0F - fresnelVal, 0.0F), kUnitEpsilon},
                {"ao", texelAt(raster.ao, x, y), glm::vec3(aoVal), kUnitEpsilon},
                {"iorAov", texelAt(raster.iorAov, x, y), glm::vec3(settings.ior), kUnitEpsilon},
            };
            valueMismatches += checkFields(fields, x, y, poseName);
        }
    }

    const int totalPixels = kWidth * kHeight;
    const float coverageFraction = static_cast<float>(coverageMismatches) / static_cast<float>(totalPixels);
    const float valueFraction =
        hitPixels > 0 ? static_cast<float>(valueMismatches) / static_cast<float>(hitPixels) : 0.0F;
    std::cout << "rasterizer_validate: " << poseName << " -- " << hitPixels << " hit pixels, "
              << coverageMismatches << " coverage mismatches, " << valueMismatches << " value mismatches\n";

    if (coverageFraction > kMaxCoverageMismatchFraction || valueFraction > kMaxValueMismatchFraction) {
        std::cerr << "rasterizer_validate: " << poseName << " FAILED (coverage " << coverageFraction * 100.0F
                  << "%, value " << valueFraction * 100.0F << "%)\n";
        return false;
    }
    return true;
}

// A single tiny (non-degenerate) triangle near `center` -- plants a known point into accel.sceneBounds() without a real visible surface.
ShadingTriangle makeTinyTriangle(glm::vec3 center, float eps, int instanceIndex) {
    const glm::vec3 normal(0.0F, 0.0F, 1.0F);
    const glm::vec4 tangent(1.0F, 0.0F, 0.0F, 1.0F);
    ShadingTriangle tri;
    tri.v0 = ShadingVertex{center, normal, glm::vec2(0.0F, 0.0F), tangent};
    tri.v1 = ShadingVertex{center + glm::vec3(eps, 0.0F, 0.0F), normal, glm::vec2(1.0F, 0.0F), tangent};
    tri.v2 = ShadingVertex{center + glm::vec3(0.0F, eps, 0.0F), normal, glm::vec2(0.0F, 1.0F), tangent};
    tri.instanceIndex = instanceIndex;
    return tri;
}

// A flat quad (2 triangles) in the XY plane at depth z, centered at the origin.
std::array<ShadingTriangle, 2> makeQuad(float halfExtent, float z, int instanceIndex) {
    const glm::vec3 normal(0.0F, 0.0F, 1.0F);
    const glm::vec4 tangent(1.0F, 0.0F, 0.0F, 1.0F);
    const glm::vec3 a(-halfExtent, -halfExtent, z);
    const glm::vec3 b(halfExtent, -halfExtent, z);
    const glm::vec3 c(halfExtent, halfExtent, z);
    const glm::vec3 d(-halfExtent, halfExtent, z);
    ShadingTriangle t0;
    t0.v0 = ShadingVertex{a, normal, glm::vec2(0.0F, 0.0F), tangent};
    t0.v1 = ShadingVertex{b, normal, glm::vec2(1.0F, 0.0F), tangent};
    t0.v2 = ShadingVertex{c, normal, glm::vec2(1.0F, 1.0F), tangent};
    t0.instanceIndex = instanceIndex;
    ShadingTriangle t1;
    t1.v0 = ShadingVertex{a, normal, glm::vec2(0.0F, 0.0F), tangent};
    t1.v1 = ShadingVertex{c, normal, glm::vec2(1.0F, 1.0F), tangent};
    t1.v2 = ShadingVertex{d, normal, glm::vec2(0.0F, 1.0F), tangent};
    t1.instanceIndex = instanceIndex;
    return {t0, t1};
}

// Wireframe is a combined AOV: white (1,1,1) mesh edges, yellow (1,1,0) box edges. Yellow is R>0.5 && B<0.5 -- unique among {black, white, yellow}.
bool isBoundingBoxColor(glm::vec3 c) {
    return c.x > 0.5F && c.z < 0.5F;
}

bool isWireframeColor(glm::vec3 c) {
    return c.z > 0.5F;
}

int countBoundingBoxPixels(const RasterGBuffer& raster) {
    int count = 0;
    for (int y = 0; y < kHeight; ++y) {
        for (int x = 0; x < kWidth; ++x) {
            if (isBoundingBoxColor(texelAt(raster.wireframe, x, y))) {
                ++count;
            }
        }
    }
    return count;
}

// Regression check: box edges are real line segments z-tested against scene geometry, so a solid occluder in front of the box should hide edges behind it. Case 1: two tiny (non-occluding) corner markers define the box -- all 12 edges eligible. Case 2: an occluder nearer than the box's near corner, same x/y extent as the marker (not larger -- it's part of the scene, so a larger extent would enlarge accel.sceneBounds() and test a different box). Tiny markers, not a filled quad, define the box corners: a filled quad would self-occlude in both cases.
bool checkBoundingBoxOcclusion(RowThreadPool& threadPool) {
    const Camera::FilmBack filmBack{36.0F, 24.0F};
    const Camera camera(glm::vec3(0.0F), 0.0F, 0.0F, filmBack, 35.0F, 0.1F, 100.0F, 2.8F, 1.0F / 125.0F,
                        100.0F);
    PathTraceSettings settings{};
    settings.samplesPerPixel = 1;
    settings.maxBounces = 0;
    settings.russianRouletteStartBounce = 1;
    settings.bumpStrength = 1.0F;
    settings.roughnessMin = 0.045F;
    settings.roughnessMax = 1.0F;
    settings.diffuseColour = glm::vec3(1.0F);
    settings.ior = 1.5F;
    settings.transmissionFactor = 0.0F;
    settings.metallicFactor = 0.2F;
    settings.roughnessFactor = 1.0F;

    std::vector<MeshInstance> instances;
    instances.push_back(MeshInstance{makeMaterial(glm::vec3(0.5F), 0.5F, 1.0F), glm::mat4(1.0F)});

    const ShadingTriangle farMarker = makeTinyTriangle(glm::vec3(-0.5F, -0.5F, -15.0F), 0.05F, 0);
    const ShadingTriangle nearMarker = makeTinyTriangle(glm::vec3(0.5F, 0.5F, -5.0F), 0.05F, 0);

    // Case 1: box spans x/y in [-0.5,0.5], z in [-15,-5], nothing solid anywhere -- all edges eligible.
    {
        const std::vector<ShadingTriangle> scene{farMarker, nearMarker};
        std::vector<Triangle> world = worldTrianglesOf(scene);
        std::optional<EmbreeAccel> accel = EmbreeAccel::build(std::move(world));
        if (!accel) {
            std::cerr << "rasterizer_validate: checkBoundingBoxOcclusion FAILED -- accel build (case 1)\n";
            return false;
        }
        RasterGBuffer raster;
        renderRasterGBuffer(camera, *accel, scene, instances, settings, kWidth, kHeight, threadPool, raster);
        const int unoccluded = countBoundingBoxPixels(raster);
        std::cout << "rasterizer_validate: boundingBox unoccluded -- " << unoccluded << " pixels\n";
        if (unoccluded == 0) {
            std::cerr << "rasterizer_validate: checkBoundingBoxOcclusion FAILED -- no box edges "
                         "visible with nothing in front of the box\n";
            return false;
        }

        // Case 2: occluder at z=-4 (nearer than the near corner at z=-5), same 0.5 x/y extent -- shifts near-z by a small, predictable amount (-5 to -4) instead of enlarging x/y. Its own surface becomes the box's new near face, so that face's 4 edges stay visible (nearest thing there); far face + pillars are now behind it. Expect a large, not total, reduction.
        const std::array<ShadingTriangle, 2> occluder = makeQuad(0.5F, -4.0F, 0);
        const std::vector<ShadingTriangle> occludedScene{farMarker, occluder[0], occluder[1]};
        std::vector<Triangle> occludedWorld = worldTrianglesOf(occludedScene);
        std::optional<EmbreeAccel> occludedAccel = EmbreeAccel::build(std::move(occludedWorld));
        if (!occludedAccel) {
            std::cerr << "rasterizer_validate: checkBoundingBoxOcclusion FAILED -- accel build (case 2)\n";
            return false;
        }
        RasterGBuffer occludedRaster;
        renderRasterGBuffer(camera, *occludedAccel, occludedScene, instances, settings, kWidth, kHeight,
                             threadPool, occludedRaster);
        const int occluded = countBoundingBoxPixels(occludedRaster);
        std::cout << "rasterizer_validate: boundingBox occluded -- " << occluded << " pixels\n";
        if (occluded * 2 >= unoccluded) {
            std::cerr << "rasterizer_validate: checkBoundingBoxOcclusion FAILED -- occluded pixel "
                         "count ("
                      << occluded << ") not substantially lower than unoccluded (" << unoccluded << ")\n";
            return false;
        }
    }
    return true;
}

// Regression check: must draw something (not silently empty) but only a thin fraction of hit pixels, not most of the mesh.
bool checkWireframeSanity(const Camera& camera, const EmbreeAccel& accel,
                           const std::vector<ShadingTriangle>& shadingTriangles,
                           const std::vector<MeshInstance>& instances, const PathTraceSettings& settings,
                           RowThreadPool& threadPool) {
    RasterGBuffer raster;
    renderRasterGBuffer(camera, accel, shadingTriangles, instances, settings, kWidth, kHeight, threadPool, raster);
    int hitPixels = 0;
    int wirePixels = 0;
    for (int y = 0; y < kHeight; ++y) {
        for (int x = 0; x < kWidth; ++x) {
            if (texelAt(raster.alpha, x, y).x > 0.5F) {
                ++hitPixels;
                if (isWireframeColor(texelAt(raster.wireframe, x, y))) {
                    ++wirePixels;
                }
            }
        }
    }
    std::cout << "rasterizer_validate: wireframe -- " << wirePixels << " / " << hitPixels << " hit pixels\n";
    if (wirePixels == 0) {
        std::cerr << "rasterizer_validate: checkWireframeSanity FAILED -- no wireframe pixels drawn\n";
        return false;
    }
    if (hitPixels > 0 && (wirePixels * 2) > hitPixels) {
        std::cerr << "rasterizer_validate: checkWireframeSanity FAILED -- wireframe covers over "
                     "half of all hit pixels, expected thin edges only\n";
        return false;
    }
    return true;
}

}  // namespace

int main() {
    std::mt19937 rng(42);
    const std::vector<ShadingTriangle> shadingTriangles = makeSyntheticTriangles(rng);
    const std::vector<Triangle> worldTriangles = worldTrianglesOf(shadingTriangles);

    std::optional<EmbreeAccel> accel = EmbreeAccel::build(worldTriangles);
    if (!accel) {
        std::cerr << "rasterizer_validate: EmbreeAccel::build failed\n";
        return EXIT_FAILURE;
    }

    std::vector<MeshInstance> instances;
    instances.push_back(MeshInstance{makeMaterial(glm::vec3(0.8F, 0.2F, 0.2F), 0.2F, 1.0F), glm::mat4(1.0F)});
    instances.push_back(MeshInstance{makeMaterial(glm::vec3(0.2F, 0.8F, 0.2F), 0.5F, 0.6F), glm::mat4(1.0F)});
    instances.push_back(MeshInstance{makeMaterial(glm::vec3(0.2F, 0.2F, 0.8F), 0.8F, 0.3F), glm::mat4(1.0F)});
    instances.push_back(MeshInstance{makeMaterial(glm::vec3(0.8F, 0.8F, 0.2F), 1.0F, 0.9F), glm::mat4(1.0F)});

    PathTraceSettings settings{};
    settings.samplesPerPixel = 1;
    settings.maxBounces = 0;
    settings.russianRouletteStartBounce = 1;
    settings.bumpStrength = 1.0F;
    settings.roughnessMin = 0.045F;
    settings.roughnessMax = 1.0F;
    settings.diffuseColour = glm::vec3(1.0F);
    settings.ior = 1.5F;
    settings.transmissionFactor = 0.0F;
    settings.metallicFactor = 0.2F;
    settings.roughnessFactor = 1.0F;

    const Camera::FilmBack filmBack{36.0F, 24.0F};
    const Camera straightOn(glm::vec3(0.0F, 0.0F, 0.0F), 0.0F, 0.0F, filmBack, 35.0F, 0.1F, 100.0F, 2.8F,
                             1.0F / 125.0F, 100.0F);
    const Camera angled(glm::vec3(3.0F, 2.0F, 1.0F), 20.0F, -10.0F, filmBack, 35.0F, 0.1F, 100.0F, 2.8F,
                         1.0F / 125.0F, 100.0F);
    // Near clip 5.0 (vs. the other poses' 0.1) puts it mid-cluster: centers in (-16,-5) stay fully in front, centers in (-5,-1) straddle/sit behind -- exercises the Sutherland-Hodgman clip path instead of coincidentally skipping it.
    const Camera clipTest(glm::vec3(0.0F, 0.0F, 0.0F), 5.0F, 5.0F, filmBack, 35.0F, 5.0F, 100.0F, 2.8F,
                           1.0F / 125.0F, 100.0F);

    RowThreadPool threadPool;
    bool ok = true;
    ok = checkPose("straightOn", straightOn, *accel, shadingTriangles, instances, settings, threadPool) && ok;
    ok = checkPose("angled", angled, *accel, shadingTriangles, instances, settings, threadPool) && ok;
    ok = checkPose("clipTest", clipTest, *accel, shadingTriangles, instances, settings, threadPool) && ok;
    ok = checkWireframeSanity(straightOn, *accel, shadingTriangles, instances, settings, threadPool) && ok;
    ok = checkBoundingBoxOcclusion(threadPool) && ok;

    if (!ok) {
        std::cerr << "rasterizer_validate: FAILED\n";
        return EXIT_FAILURE;
    }
    std::cout << "rasterizer_validate: PASSED\n";
    return EXIT_SUCCESS;
}
