// Timing harness for engine::scene::renderRasterGBuffer (rasterizer.h), the synchronous per-frame render-thread work behind the 15 primary-hit AOVs. Synthetic dependency-free scene (no glTF/EXR asset, constant-color 1x1 textures), same standalone-CLI convention as rasterizer_validate.cpp: no test framework, non-zero exit on bad input. Deliberately NOT an add_test: a benchmark is not a correctness gate, and the rasterizer's correctness gate is rasterizer_validate.
// Synthetic rather than asset-driven so the two variables the rasterizer's cost is actually a function of are independently controllable: --triangles sweeps the sub-triangle array past cache (the shipped scene is 20561 triangles = 1.73 MB, resident; the 5M-triangle tier is 420 MB, not), and --layers sweeps depth complexity, which is what a depth prepass is a function of. Neither is adjustable in a fixed asset.
// Reports best-of-N, not the mean: run-to-run spread on this hardware is +/-10%, wide enough to hide a single change. A/B one change at a time against the same build with only that change stashed.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>
#include <optional>
#include <random>
#include <vector>

#include <glm/glm.hpp>

#include "engine/gfx/hdr_image.h"
#include "engine/scene/camera.h"
#include "engine/scene/embree_accel.h"
#include "engine/scene/gltf_loader.h"
#include "engine/scene/path_tracer.h"
#include "engine/scene/rasterizer.h"
#include "engine/scene/ray_types.h"
#include "engine/scene/row_thread_pool.h"
#include "engine/scene/shading_scene.h"

namespace {

using namespace engine::scene;  // NOLINT(google-build-using-namespace) -- tool-local convenience, mirrors rasterizer_validate.cpp

constexpr int kMaterialCount = 4;
constexpr float kNearestLayerZ = 4.0F;   // world units in front of the camera; > nearClip so no layer is clipped away
constexpr float kLayerSpacing = 1.0F;
constexpr float kLayerCoverage = 1.05F;  // centers spread slightly past the frustum cross-section so triangles reach the screen edges
constexpr float kVertexAngleStep = 2.0943951F;  // 2*pi/3, the three vertices of an equilateral triangle

struct Options {
    int triangleCount = 20561;  // scene.json's rkswd_tier_2.gltf, the shipped default
    int width = 2048;           // profile.json's 1024x576 window on a Retina display
    int height = 1152;
    int frames = 5;
    int layers = 1;
    unsigned int seed = 42;
};

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

// `layers` screen-filling shells, each holding an equal share of the triangle budget. Per-shell triangle radius is derived from that shell's frustum cross-section so every shell covers the screen once regardless of triangle count: raising --triangles shrinks triangles rather than piling up overdraw, keeping the two axes independent.
// Emitted furthest shell first, so every shell improves the depth record and a pixel accumulates `layers` of them -- the quantity a depth prepass trades against. Submission order, not shell count, is what decides that: nearest-first would have every deeper shell rejected on arrival by the z-test, leaving one record per pixel however high --layers goes. This is therefore the worst case; an unsorted real scene averages the harmonic number of records (~2.7 at 8 shells), and a front-to-back one none at all.
std::vector<ShadingTriangle> makeLayeredTriangles(const Options& options, const Camera& camera) {
    const float aspect = static_cast<float>(options.width) / static_cast<float>(options.height);
    const Camera::ViewBasis basis = camera.viewBasis(aspect);
    const int perLayer = options.triangleCount / options.layers;  // >= 1: parseOptions rejects layers > triangleCount

    std::mt19937 rng(options.seed);
    std::uniform_real_distribution<float> unit(-1.0F, 1.0F);

    std::vector<ShadingTriangle> triangles;
    triangles.reserve(static_cast<std::size_t>(options.triangleCount));
    for (int layer = options.layers - 1; layer >= 0; --layer) {
        const float depth = kNearestLayerZ + (static_cast<float>(layer) * kLayerSpacing);
        const float halfWidth = depth * basis.halfWidth * kLayerCoverage;
        const float halfHeight = depth * basis.halfHeight * kLayerCoverage;
        // Circumradius giving `perLayer` triangles a combined area of ~1.3x the cross-section: an equilateral triangle of circumradius r has area (3*sqrt(3)/4)r^2.
        const float radius = std::sqrt((4.0F * halfWidth * halfHeight) / static_cast<float>(perLayer));
        // The furthest shell (layer layers-1, emitted first) absorbs the integer-division remainder so the total matches --triangles exactly.
        const int count = layer == options.layers - 1
                              ? options.triangleCount - (perLayer * (options.layers - 1))
                              : perLayer;

        for (int i = 0; i < count; ++i) {
            const glm::vec3 center(unit(rng) * halfWidth, unit(rng) * halfHeight, -depth);
            std::array<glm::vec3, 3> p{};
            for (int k = 0; k < 3; ++k) {
                const float angle = kVertexAngleStep * static_cast<float>(k);
                // Per-vertex depth jitter tilts each triangle off screen-parallel, so perspective-correct interpolation and the z-test do real work.
                p[static_cast<std::size_t>(k)] =
                    center + glm::vec3(radius * std::cos(angle), radius * std::sin(angle), unit(rng) * radius * 0.25F);
            }
            const glm::vec3 normal = glm::normalize(glm::cross(p[1] - p[0], p[2] - p[0]));
            if (!std::isfinite(normal.x)) {
                continue;  // degenerate draw -- skipped, leaving the total marginally under --triangles
            }
            const glm::vec4 tangent = tangentFor(normal);
            ShadingTriangle tri;
            tri.v0 = ShadingVertex{p[0], normal, glm::vec2(0.0F, 0.0F), tangent};
            tri.v1 = ShadingVertex{p[1], normal, glm::vec2(1.0F, 0.0F), tangent};
            tri.v2 = ShadingVertex{p[2], normal, glm::vec2(0.0F, 1.0F), tangent};
            tri.instanceIndex = i % kMaterialCount;
            triangles.push_back(tri);
        }
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

// Returns nullopt on an unrecognized flag, a missing value, or a value outside its usable range -- argv is a system boundary, so a bad value is surfaced rather than clamped around.
std::optional<Options> parseOptions(int argc, char** argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        if (i + 1 >= argc) {
            std::cerr << "raster_bench: " << argv[i] << " expects a value\n";
            return std::nullopt;
        }
        const char* flag = argv[i];
        const char* text = argv[++i];
        char* end = nullptr;
        const long value = std::strtol(text, &end, 10);
        // strtol reports non-numeric input as 0 and stops at the first bad character, so the terminator check is what makes "--seed foo" an error rather than a silent seed of 0.
        if (end == text || *end != '\0') {
            std::cerr << "raster_bench: " << flag << " expects an integer, got " << text << '\n';
            return std::nullopt;
        }
        // Bounded before the narrowing casts below, so an out-of-range argument is an error rather than an implementation-defined wrap.
        if (value < 0 || value > std::numeric_limits<int>::max()) {
            std::cerr << "raster_bench: " << flag << " out of range: " << text << '\n';
            return std::nullopt;
        }
        if (std::strcmp(flag, "--triangles") == 0) {
            options.triangleCount = static_cast<int>(value);
        } else if (std::strcmp(flag, "--width") == 0) {
            options.width = static_cast<int>(value);
        } else if (std::strcmp(flag, "--height") == 0) {
            options.height = static_cast<int>(value);
        } else if (std::strcmp(flag, "--frames") == 0) {
            options.frames = static_cast<int>(value);
        } else if (std::strcmp(flag, "--layers") == 0) {
            options.layers = static_cast<int>(value);
        } else if (std::strcmp(flag, "--seed") == 0) {
            options.seed = static_cast<unsigned int>(value);
        } else {
            std::cerr << "raster_bench: unknown flag " << flag
                       << "\n  usage: raster_bench [--triangles N] [--width N] [--height N] [--frames N] [--layers N] [--seed N]\n";
            return std::nullopt;
        }
    }
    if (options.triangleCount < 1 || options.width < 1 || options.height < 1 || options.frames < 1 ||
        options.layers < 1) {
        std::cerr << "raster_bench: --triangles/--width/--height/--frames/--layers must all be >= 1\n";
        return std::nullopt;
    }
    if (options.layers > options.triangleCount) {
        std::cerr << "raster_bench: --layers cannot exceed --triangles\n";
        return std::nullopt;
    }
    return options;
}

}  // namespace

int main(int argc, char** argv) {
    const std::optional<Options> options = parseOptions(argc, argv);
    if (!options) {
        return EXIT_FAILURE;
    }

    const Camera::FilmBack filmBack{36.0F, 24.0F};
    const Camera camera(glm::vec3(0.0F), 0.0F, 0.0F, filmBack, 35.0F, 0.1F, 100.0F, 2.8F, 1.0F / 125.0F,
                         100.0F);

    const std::vector<ShadingTriangle> shadingTriangles = makeLayeredTriangles(*options, camera);
    std::optional<EmbreeAccel> accel = EmbreeAccel::build(worldTrianglesOf(shadingTriangles));
    if (!accel) {
        std::cerr << "raster_bench: EmbreeAccel::build failed\n";
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

    RowThreadPool threadPool;
    // One buffer for the whole run, matching how the app owns it: renderRasterGBuffer reuses it in place, so the timed frames measure steady-state cost with no allocation in them.
    RasterGBuffer gbuffer;
    // Discarded warm-up pass, absorbing the costs that happen once rather than per frame: spinning up and parking the pool's workers, and the buffer's only allocation.
    renderRasterGBuffer(camera, *accel, shadingTriangles, instances, settings, options->width,
                         options->height, threadPool, gbuffer);
    if (gbuffer.depth.width != options->width) {
        std::cerr << "raster_bench: warm-up produced a " << gbuffer.depth.width << "px-wide buffer\n";
        return EXIT_FAILURE;
    }

    std::vector<double> milliseconds;
    milliseconds.reserve(static_cast<std::size_t>(options->frames));
    for (int frame = 0; frame < options->frames; ++frame) {
        const auto start = std::chrono::steady_clock::now();
        renderRasterGBuffer(camera, *accel, shadingTriangles, instances, settings, options->width,
                             options->height, threadPool, gbuffer);
        const auto end = std::chrono::steady_clock::now();
        milliseconds.push_back(std::chrono::duration<double, std::milli>(end - start).count());
        // Reading one texel keeps the optimizer from treating the whole call as dead; the result is otherwise unused.
        if (!std::isfinite(gbuffer.depth.rgba[0])) {
            std::cerr << "raster_bench: non-finite depth at texel 0\n";
            return EXIT_FAILURE;
        }
    }

    const auto [best, worst] = std::minmax_element(milliseconds.begin(), milliseconds.end());
    double total = 0.0;
    for (const double ms : milliseconds) {
        total += ms;
    }

    std::cout << "raster_bench: " << shadingTriangles.size() << " tris, " << options->width << "x"
              << options->height << ", " << options->layers << " layer(s), best-of-" << options->frames
              << ": " << *best << " ms  (mean " << total / static_cast<double>(options->frames)
              << ", worst " << *worst << ")\n";
    return EXIT_SUCCESS;
}
