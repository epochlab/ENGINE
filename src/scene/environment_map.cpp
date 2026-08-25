#include "engine/scene/environment_map.h"

#include <algorithm>
#include <cmath>

#include <glm/gtc/constants.hpp>

namespace engine::scene {

namespace {

// Matches sky.frag's rotateAboutY.
glm::vec3 rotateAboutY(const glm::vec3& v, float angleRadians) {
    const float c = std::cos(angleRadians);
    const float s = std::sin(angleRadians);
    return {(v.x * c) + (v.z * s), v.y, (-v.x * s) + (v.z * c)};
}

// Same Rec.709 weights as edge_filter.frag's sampleLuminance.
float luminanceOf(const engine::gfx::HdrImage& image, int x, int y) {
    const std::size_t idx = ((static_cast<std::size_t>(y) * static_cast<std::size_t>(image.width)) +
                              static_cast<std::size_t>(x)) *
                             4;
    return (0.2126F * image.rgba[idx + 0]) + (0.7152F * image.rgba[idx + 1]) +
           (0.0722F * image.rgba[idx + 2]);
}

// Inverts a piecewise-constant CDF slice [cdf[0], cdf[count]) (cdf[0]==0, cdf[count]==1) at u,
// returning the bin index and the fractional offset within that bin's probability mass -- shared by
// both the marginal (row) and conditional (column) inversion steps.
struct CdfSample {
    int index;
    float fraction;  // [0,1) position within the selected bin
};
CdfSample invertCdf(const float* cdf, int count, float u) {
    // upper_bound finds the first entry > u; the bin just before it is the one u falls into.
    // cdf[0]==0 is never > u (u>=0) and searching it is harmless, so no need to special-case it out.
    const float* it = std::upper_bound(cdf, cdf + count + 1, u);
    const int index = std::clamp(static_cast<int>(it - cdf) - 1, 0, count - 1);
    const float lo = cdf[index];
    const float hi = cdf[index + 1];
    const float fraction = hi > lo ? std::clamp((u - lo) / (hi - lo), 0.0F, 1.0F) : 0.5F;
    return {index, fraction};
}

}  // namespace

EnvironmentMap::EnvironmentMap(engine::gfx::HdrImage image) : image_(std::move(image)) {
    const int width = image_.width;
    const int height = image_.height;
    marginalCdf_.assign(static_cast<std::size_t>(height) + 1, 0.0F);
    conditionalCdf_.assign(static_cast<std::size_t>(height) * (static_cast<std::size_t>(width) + 1),
                            0.0F);

    float total = 0.0F;
    for (int y = 0; y < height; ++y) {
        // Row-center theta, sin(theta)-weighted so sampling density corrects for the equirect
        // projection's polar over-representation (see class doc comment).
        const float theta = glm::pi<float>() * (static_cast<float>(y) + 0.5F) / static_cast<float>(height);
        const float sinTheta = std::max(std::sin(theta), 1e-6F);

        float* row = &conditionalCdf_[static_cast<std::size_t>(y) * (static_cast<std::size_t>(width) + 1)];
        float rowSum = 0.0F;
        for (int x = 0; x < width; ++x) {
            rowSum += luminanceOf(image_, x, y) * sinTheta;
            row[x + 1] = rowSum;
        }
        if (rowSum > 0.0F) {
            for (int x = 0; x <= width; ++x) {
                row[x] /= rowSum;
            }
        } else {
            // Degenerate (fully black) row -- uniform fallback so later division/inversion stays well-defined.
            for (int x = 0; x <= width; ++x) {
                row[x] = static_cast<float>(x) / static_cast<float>(width);
            }
        }

        total += rowSum;
        marginalCdf_[static_cast<std::size_t>(y) + 1] = total;
    }

    if (total > 0.0F) {
        for (float& v : marginalCdf_) {
            v /= total;
        }
    } else {
        // Degenerate (fully black) map -- uniform fallback, same reasoning as the per-row case above.
        for (int y = 0; y <= height; ++y) {
            marginalCdf_[static_cast<std::size_t>(y)] = static_cast<float>(y) / static_cast<float>(height);
        }
    }
}

glm::vec3 EnvironmentMap::sampleDirection(const glm::vec3& direction,
                                           float envRotationRadians) const {
    const glm::vec3 rotated = rotateAboutY(direction, -envRotationRadians);
    const float theta = std::acos(glm::clamp(rotated.y, -1.0F, 1.0F));
    const float phi = std::atan2(rotated.x, rotated.z);
    const glm::vec2 uv((phi / (2.0F * glm::pi<float>())) + 0.5F, theta / glm::pi<float>());
    return glm::vec3(engine::gfx::sampleBilinear(image_, uv));
}

EnvironmentMap::EnvSample EnvironmentMap::importanceSampleDirection(glm::vec2 u,
                                                                     float envRotationRadians) const {
    const int width = image_.width;
    const int height = image_.height;

    const CdfSample rowSample = invertCdf(marginalCdf_.data(), height, u.x);
    const float v = (static_cast<float>(rowSample.index) + rowSample.fraction) / static_cast<float>(height);

    const float* row =
        &conditionalCdf_[static_cast<std::size_t>(rowSample.index) * (static_cast<std::size_t>(width) + 1)];
    const CdfSample colSample = invertCdf(row, width, u.y);
    const float uCoord = (static_cast<float>(colSample.index) + colSample.fraction) / static_cast<float>(width);

    const float theta = v * glm::pi<float>();
    const float phi = (uCoord - 0.5F) * 2.0F * glm::pi<float>();
    const float sinTheta = std::sin(theta);
    const glm::vec3 rotated(sinTheta * std::sin(phi), std::cos(theta), sinTheta * std::cos(phi));
    const glm::vec3 direction = rotateAboutY(rotated, envRotationRadians);

    const float pdfV = (marginalCdf_[static_cast<std::size_t>(rowSample.index) + 1] -
                         marginalCdf_[static_cast<std::size_t>(rowSample.index)]) *
                        static_cast<float>(height);
    const float pdfU = (row[colSample.index + 1] - row[colSample.index]) * static_cast<float>(width);
    // Jacobian from (u,v) density to solid-angle density: dw = sin(theta) * (pi dv) * (2pi du), so
    // pdf_solid_angle = pdf_uv / (2 * pi^2 * sin(theta)) -- see class doc comment / plan notes.
    const float pdfSolidAngle =
        (pdfU * pdfV) / std::max(2.0F * glm::pi<float>() * glm::pi<float>() * sinTheta, 1e-6F);
    return {direction, std::max(pdfSolidAngle, 1e-8F)};
}

float EnvironmentMap::pdf(const glm::vec3& direction, float envRotationRadians) const {
    const int width = image_.width;
    const int height = image_.height;
    const glm::vec3 rotated = rotateAboutY(direction, -envRotationRadians);
    const float theta = std::acos(glm::clamp(rotated.y, -1.0F, 1.0F));
    const float phi = std::atan2(rotated.x, rotated.z);
    const float uCoord = (phi / (2.0F * glm::pi<float>())) + 0.5F;
    const float v = theta / glm::pi<float>();

    const int y = std::clamp(static_cast<int>(v * static_cast<float>(height)), 0, height - 1);
    const int x = std::clamp(static_cast<int>(uCoord * static_cast<float>(width)), 0, width - 1);
    const float* row =
        &conditionalCdf_[static_cast<std::size_t>(y) * (static_cast<std::size_t>(width) + 1)];

    const float pdfV = (marginalCdf_[static_cast<std::size_t>(y) + 1] - marginalCdf_[static_cast<std::size_t>(y)]) *
                        static_cast<float>(height);
    const float pdfU = (row[x + 1] - row[x]) * static_cast<float>(width);
    const float sinTheta = std::max(std::sin(theta), 1e-6F);
    const float pdfSolidAngle =
        (pdfU * pdfV) / std::max(2.0F * glm::pi<float>() * glm::pi<float>() * sinTheta, 1e-6F);
    return std::max(pdfSolidAngle, 1e-8F);
}

}  // namespace engine::scene
