#include "engine/scene/path_tracer.h"

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <limits>
#include <optional>
#include <thread>

#include "engine/scene/bsdf.h"
#include "engine/scene/sampler.h"
#include "engine/scene/shading_scene.h"

namespace engine::scene {

namespace {

constexpr float kRayEpsilon = 1e-4F;

glm::vec3 resolveBaseColor(const Material& material, glm::vec2 uv) {
    const glm::vec4 sample = engine::gfx::sampleBilinear(material.baseColorTexture.cpu, uv);
    return glm::vec3(sample) * glm::vec3(material.baseColorFactor);
}

float resolveRoughness(const Material& material, glm::vec2 uv) {
    const float sample = engine::gfx::sampleBilinear(material.roughnessTexture.cpu, uv).r;
    // 0.045 floor matches pbr.frag's own minimum-roughness clamp (UE4/Frostbite convention).
    return std::clamp(sample * material.roughnessFactor, 0.045F, 1.0F);
}

// Tangent-space normal map only -- pbr.frag additionally blends a bump-derived detail normal
// (central-difference height gradient); omitted here as a deliberate scope simplification, not a
// correctness concern (it only adds fine surface micro-detail on top of this).
BsdfParams resolveBsdfParams(const Material& material, glm::vec2 uv) {
    const glm::vec3 baseColor = resolveBaseColor(material, uv);
    const float roughness = resolveRoughness(material, uv);
    const glm::vec3 specular = glm::vec3(engine::gfx::sampleBilinear(material.specularTexture.cpu, uv));
    const glm::vec3 f0 = glm::mix(specular, baseColor, material.metallicFactor);
    return BsdfParams{baseColor, material.metallicFactor, roughness, f0, material.ior,
                       material.transmissionFactor};
}

// Gram-Schmidt re-orthogonalized tangent frame, normal-mapped -- same construction as pbr.frag's
// computeShadingInputs, ported to CPU.
ShadingFrame buildShadingFrame(const ShadingVertex& shading, const Material& material) {
    const glm::vec3 normal = glm::normalize(shading.normal);
    glm::vec3 tangent = glm::vec3(shading.tangent);
    tangent = glm::normalize(tangent - (glm::dot(tangent, normal) * normal));
    const glm::vec3 bitangent = glm::cross(normal, tangent) * shading.tangent.w;

    const glm::vec4 normalSample = engine::gfx::sampleBilinear(material.normalTexture.cpu, shading.uv);
    const glm::vec3 tangentSpaceNormal = glm::normalize((glm::vec3(normalSample) * 2.0F) - 1.0F);
    const glm::vec3 mappedNormal = glm::normalize(
        (tangentSpaceNormal.x * tangent) + (tangentSpaceNormal.y * bitangent) +
        (tangentSpaceNormal.z * normal));

    const glm::vec3 finalTangent =
        glm::normalize(tangent - (glm::dot(tangent, mappedNormal) * mappedNormal));
    const glm::vec3 finalBitangent = glm::cross(mappedNormal, finalTangent) * shading.tangent.w;
    return ShadingFrame{finalTangent, finalBitangent, mappedNormal};
}

glm::vec3 geometricNormalOf(const ShadingTriangle& tri) {
    return glm::normalize(
        glm::cross(tri.v1.position - tri.v0.position, tri.v2.position - tri.v0.position));
}

struct TraceResult {
    glm::vec3 radiance;
    float firstHitIor;      // -1 if the primary ray never hit geometry
    int terminationBounce;  // bounce index the path stopped at (== maxBounces if depth-capped)
};

TraceResult tracePath(const Ray& primaryRay, const Bvh& bvh,
                       const std::vector<ShadingTriangle>& shadingTriangles,
                       const std::vector<MeshInstance>& instances,
                       const EnvironmentMap& environmentMap, float envRotationRadians,
                       const PathTraceSettings& settings, Sampler& sampler) {
    glm::vec3 radiance(0.0F);
    glm::vec3 throughput(1.0F);
    Ray ray = primaryRay;
    float firstHitIor = -1.0F;
    int bounce = 0;

    for (; bounce < settings.maxBounces; ++bounce) {
        const std::optional<Hit> hit = bvh.intersect(ray);
        if (!hit.has_value()) {
            radiance += throughput * environmentMap.sampleDirection(ray.dir, envRotationRadians);
            break;
        }

        const ShadingTriangle& triangle =
            shadingTriangles[static_cast<std::size_t>(hit->triangleIndex)];
        const Material& material =
            instances[static_cast<std::size_t>(triangle.instanceIndex)].material;
        if (bounce == 0) {
            firstHitIor = material.ior;
        }

        const ShadingVertex shading = interpolateShading(triangle, hit->u, hit->v);
        const ShadingFrame frame = buildShadingFrame(shading, material);
        const BsdfParams params = resolveBsdfParams(material, shading.uv);

        const glm::vec3 woWorld = -ray.dir;
        const std::optional<BsdfSample> sample = sampleBsdf(params, frame.toLocal(woWorld), sampler);
        if (!sample.has_value()) {
            break;
        }
        const glm::vec3 wiWorld = frame.toWorld(sample->wiLocal);

        // Geometric-normal-consistency rejection (normal-map robustness -- simpler stand-in for
        // Schussler et al. 2017's full two-facet microsurface reconstruction): a reflection/diffuse
        // sample crossing to the wrong side of the true triangle plane is a normal-map light-leak
        // artifact, not a physical bounce.
        const glm::vec3 geoNormal = geometricNormalOf(triangle);
        if (!sample->specular) {
            const bool woAbove = glm::dot(woWorld, geoNormal) > 0.0F;
            const bool wiAbove = glm::dot(wiWorld, geoNormal) > 0.0F;
            if (woAbove != wiAbove) {
                break;
            }
        }

        throughput *= sample->throughputWeight;

        if (bounce >= settings.russianRouletteStartBounce) {
            const float continueProb = std::clamp(
                std::max({throughput.x, throughput.y, throughput.z}), settings.rrMinProb,
                settings.rrMaxProb);
            if (sampler.next1D() >= continueProb) {
                break;
            }
            throughput /= continueProb;
        }

        // Chiang/Li/Burley shadow-terminator-corrected origin, nudged off the true triangle plane
        // along the geometric normal (toward wi's side) to avoid self-intersection.
        const glm::vec3 offsetOrigin =
            shadowTerminatorOffset(triangle, hit->u, hit->v) +
            (geoNormal * kRayEpsilon * (glm::dot(wiWorld, geoNormal) > 0.0F ? 1.0F : -1.0F));
        ray = Ray{offsetOrigin, wiWorld, kRayEpsilon, std::numeric_limits<float>::max()};
    }

    return {radiance, firstHitIor, bounce};
}

void writeTexel(engine::gfx::HdrImage& image, int x, int y, glm::vec3 rgb) {
    const std::size_t idx = ((static_cast<std::size_t>(y) * static_cast<std::size_t>(image.width)) +
                              static_cast<std::size_t>(x)) *
                             4;
    image.rgba[idx + 0] = rgb.x;
    image.rgba[idx + 1] = rgb.y;
    image.rgba[idx + 2] = rgb.z;
    image.rgba[idx + 3] = 1.0F;
}

engine::gfx::HdrImage makeImage(int width, int height) {
    engine::gfx::HdrImage image;
    image.width = width;
    image.height = height;
    image.rgba.assign(static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 4, 0.0F);
    return image;
}

}  // namespace

PathTraceResult renderPathTraced(const Camera& camera, const Bvh& bvh,
                                  const std::vector<ShadingTriangle>& shadingTriangles,
                                  const std::vector<MeshInstance>& instances,
                                  const EnvironmentMap& environmentMap, int width, int height,
                                  float envRotationRadians, const PathTraceSettings& settings,
                                  std::uint32_t runSeed) {
    PathTraceResult result{makeImage(width, height), makeImage(width, height),
                            makeImage(width, height)};
    const float aspect = static_cast<float>(width) / static_cast<float>(height);
    const float sppInv = 1.0F / static_cast<float>(settings.samplesPerPixel);

    const auto renderRow = [&](int y) {
        for (int x = 0; x < width; ++x) {
            glm::vec3 colorAccum(0.0F);
            float iorSample = -1.0F;
            float bounceAccum = 0.0F;
            for (int s = 0; s < settings.samplesPerPixel; ++s) {
                Sampler sampler(x, y, s, runSeed);
                const glm::vec2 jitter = sampler.next2D();
                const float ndcX =
                    (((static_cast<float>(x) + jitter.x) / static_cast<float>(width)) * 2.0F) - 1.0F;
                // HdrImage row 0 is the top (EXR/glTF convention); NDC +Y is up -- flip.
                const float ndcY =
                    1.0F -
                    (((static_cast<float>(y) + jitter.y) / static_cast<float>(height)) * 2.0F);
                const Ray primary = camera.primaryRay(ndcX, ndcY, aspect);
                const TraceResult trace = tracePath(primary, bvh, shadingTriangles, instances,
                                                     environmentMap, envRotationRadians, settings,
                                                     sampler);
                colorAccum += trace.radiance;
                if (s == 0) {
                    iorSample = trace.firstHitIor;
                }
                bounceAccum += static_cast<float>(trace.terminationBounce);
            }
            writeTexel(result.beauty, x, y, colorAccum * sppInv);
            writeTexel(result.iorAov, x, y, glm::vec3(iorSample));
            writeTexel(result.bounceHeatmap, x, y, glm::vec3(bounceAccum * sppInv));
        }
    };

    const unsigned int threadCount = std::max(1U, std::thread::hardware_concurrency());
    std::atomic<int> nextRow{0};
    std::vector<std::thread> threads;
    threads.reserve(threadCount);
    for (unsigned int t = 0; t < threadCount; ++t) {
        threads.emplace_back([&nextRow, height, &renderRow]() {
            int y = 0;
            while ((y = nextRow.fetch_add(1)) < height) {
                renderRow(y);
            }
        });
    }
    for (std::thread& thread : threads) {
        thread.join();
    }

    return result;
}

}  // namespace engine::scene
