#include "engine/scene/gbuffer_shading.h"

#include <algorithm>
#include <cstddef>

namespace engine::scene {

namespace {

// glTF core spec order: baseColorFactor * baseColorTexture * COLOR_0 (commutative, so the order here
// is documentation, not a correctness requirement). vertexColour is white (1,1,1) when the primitive
// has no COLOR_0 attribute, making this a no-op multiply for every asset that doesn't use it.
glm::vec3 resolveBaseColor(const Material& material, glm::vec2 uv, const glm::vec3& vertexColour,
                            const PathTraceSettings& settings) {
    const glm::vec4 sample = engine::gfx::sampleBilinear(material.baseColorTexture, uv);
    return glm::vec3(sample) * settings.diffuseColour * vertexColour;
}

float resolveRoughness(const Material& material, glm::vec2 uv, const PathTraceSettings& settings) {
    const float sample = engine::gfx::sampleBilinear(material.roughnessTexture, uv).r;
    // Floor (UE4/Frostbite convention) avoids a near-zero-roughness GGX singularity.
    return std::clamp(sample * settings.roughnessFactor, settings.roughnessMin, settings.roughnessMax);
}

}  // namespace

LineProximity nearLineSegmentPx(glm::vec2 p, glm::vec2 a, glm::vec2 b, float thicknessPx) {
    const glm::vec2 ab = b - a;
    const float abLenSq = glm::dot(ab, ab);
    const float t = abLenSq > 1e-12F ? std::clamp(glm::dot(p - a, ab) / abLenSq, 0.0F, 1.0F) : 0.0F;
    const glm::vec2 closest = a + (t * ab);
    return LineProximity{glm::length(p - closest) < thicknessPx, t};
}

BsdfParams resolveBsdfParams(const Material& material, glm::vec2 uv, const glm::vec3& vertexColour,
                              const PathTraceSettings& settings) {
    const glm::vec3 baseColor = resolveBaseColor(material, uv, vertexColour, settings);
    const float roughness = resolveRoughness(material, uv, settings);
    const glm::vec3 specular = glm::vec3(engine::gfx::sampleBilinear(material.specularTexture, uv));
    const glm::vec3 f0 = glm::mix(specular, baseColor, settings.metallicFactor);
    return BsdfParams{baseColor,          settings.metallicFactor, roughness,
                       f0,                settings.edgeTint,       settings.ior,
                       settings.transmissionFactor, settings.diffuseRoughness};
}

ShadingFrame buildShadingFrame(const ShadingVertex& shading, const Material& material,
                                const PathTraceSettings& settings) {
    const glm::vec3 normal = glm::normalize(shading.normal);
    glm::vec3 tangent = glm::vec3(shading.tangent);
    tangent = glm::normalize(tangent - (glm::dot(tangent, normal) * normal));
    const glm::vec3 bitangent = glm::cross(normal, tangent) * shading.tangent.w;

    const glm::vec4 normalSample = engine::gfx::sampleBilinear(material.normalTexture, shading.uv);
    const glm::vec3 tangentSpaceNormal = glm::normalize((glm::vec3(normalSample) * 2.0F) - 1.0F);
    const glm::vec3 mappedNormal = glm::normalize(
        (tangentSpaceNormal.x * tangent) + (tangentSpaceNormal.y * bitangent) +
        (tangentSpaceNormal.z * normal));

    // Blinn 1978 bump mapping: perturbs mappedNormal further using the bump texture's height difference between adjacent texels -- a texture-space (not screen-space) derivative, so no ray-differential tracking is needed. Applied on top of the normal map (not the base geometric normal), since this asset ships both: the normal map carries the sculpted macro surface direction, bump adds a finer wrinkle on top. Deliberately NOT divided by texel size into a true per-UV-unit derivative: at this asset's 4096px resolution that divisor is ~4096, which amplifies even tiny neighboring-texel differences into a huge tilt -- settings.bumpStrength instead scales the raw (small, well-behaved) per-texel height difference directly.
    const glm::vec2 texel(1.0F / static_cast<float>(material.bumpTexture.width),
                           1.0F / static_cast<float>(material.bumpTexture.height));
    const float dHdu =
        engine::gfx::sampleBilinear(material.bumpTexture, shading.uv + glm::vec2(texel.x, 0.0F)).r -
        engine::gfx::sampleBilinear(material.bumpTexture, shading.uv - glm::vec2(texel.x, 0.0F)).r;
    const float dHdv =
        engine::gfx::sampleBilinear(material.bumpTexture, shading.uv + glm::vec2(0.0F, texel.y)).r -
        engine::gfx::sampleBilinear(material.bumpTexture, shading.uv - glm::vec2(0.0F, texel.y)).r;
    const glm::vec3 bumpedNormal = glm::normalize(
        mappedNormal - (settings.bumpStrength * dHdu * tangent) -
        (settings.bumpStrength * dHdv * bitangent));

    const glm::vec3 finalTangent =
        glm::normalize(tangent - (glm::dot(tangent, bumpedNormal) * bumpedNormal));
    const glm::vec3 finalBitangent = glm::cross(bumpedNormal, finalTangent) * shading.tangent.w;
    return ShadingFrame{finalTangent, finalBitangent, bumpedNormal};
}

glm::vec3 geometricNormalOf(const ShadingTriangle& tri) {
    return glm::normalize(
        glm::cross(tri.v1.position - tri.v0.position, tri.v2.position - tri.v0.position));
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

}  // namespace engine::scene
