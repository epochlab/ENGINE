// Standalone correctness check for engine::scene::bsdf (bsdf.h): verifies the combined specular+diffuse pdf integrates to the expected continuous-lobe probability mass, and runs a furnace test (uniform incident radiance from every direction, including through transmission) via BSDF importance sampling -- must never reflect/transmit more energy than received. Same standalone-CLI convention as embree_validate.cpp/furnace_test.cpp: no test framework, non-zero exit on failure.

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <random>

#include <glm/glm.hpp>

#include "engine/scene/bsdf.h"
#include "engine/scene/sampler.h"

namespace {

using engine::scene::BsdfParams;

constexpr float kPi = 3.14159265F;

BsdfParams makeParams(float roughness, float metallic, float transmissionFactor) {
    const glm::vec3 baseColor(1.0F);  // worst case: full white albedo
    const glm::vec3 f0 = glm::mix(glm::vec3(0.04F), baseColor, metallic);
    return BsdfParams{baseColor, metallic, roughness, f0, 1.5F, transmissionFactor};
}

// A colored/dark conductor (f0=0.5, not the white f0=baseColor=1 makeParams gives at metallic=1) -- specifically the case that caught evaluateDiffuseLobe's pdf-gating bug: a white f0 clamps specularProb to 0.95, leaving only 5% diffuse selection mass to hide a diffuse-pdf bug under this test's tolerance; f0=0.5 leaves ~50%, large enough for the same bug to fail loudly.
BsdfParams makeColoredMetalParams(float roughness) {
    return BsdfParams{glm::vec3(1.0F), 1.0F, roughness, glm::vec3(0.5F), 1.5F, 0.0F};
}

// Uniform-solid-angle hemisphere sample (PBRT-style inversion, same as furnace_test.cpp): z=u1, r=sqrt(1-u1^2), phi=2*pi*u2, pdf=1/(2*pi).
glm::vec3 sampleUniformHemisphere(std::mt19937& rng) {
    std::uniform_real_distribution<float> unit(0.0F, 1.0F);
    const float cosTheta = unit(rng);
    const float sinTheta = std::sqrt(std::max(0.0F, 1.0F - (cosTheta * cosTheta)));
    const float phi = 2.0F * kPi * unit(rng);
    return {sinTheta * std::cos(phi), sinTheta * std::sin(phi), cosTheta};
}

// Integrates pdfBsdf(wo, .) over the hemisphere via uniform-hemisphere Monte Carlo; for an opaque material (transmissionFactor=0) this must not exceed 1.0 -- all sampling probability mass is in the two continuous lobes the pdf covers. Upper-bound only, not a tight equality check: uniform hemisphere sampling under-samples a sharp GGX lobe at low roughness (same accepted limitation as furnace_test.cpp's checkPunctualSweep), so a low reading there is expected noise, not a bug -- only exceeding 1.0 would indicate a real double-counted pdf.
bool checkPdfNormalization() {
    std::mt19937 rng(7);
    constexpr int kSampleCount = 200000;
    constexpr float kTolerance = 0.05F;
    constexpr float kUniformPdf = 1.0F / (2.0F * kPi);
    const std::array<float, 4> roughnesses = {0.05F, 0.25F, 0.5F, 1.0F};
    const std::array<float, 2> metallics = {0.0F, 1.0F};
    const std::array<float, 4> ndotVs = {0.2F, 0.6F, 1.0F, -0.6F};

    bool ok = true;
    for (float roughness : roughnesses) {
        for (float metallic : metallics) {
            for (float ndotV : ndotVs) {
                const BsdfParams params = makeParams(roughness, metallic, 0.0F);
                const glm::vec3 wo(std::sqrt(std::max(0.0F, 1.0F - (ndotV * ndotV))), 0.0F, ndotV);
                double integral = 0.0;
                for (int i = 0; i < kSampleCount; ++i) {
                    const glm::vec3 wi = sampleUniformHemisphere(rng);
                    integral += engine::scene::pdfBsdf(params, wo, wi) / kUniformPdf;
                }
                integral /= kSampleCount;
                if (integral > 1.0 + kTolerance) {
                    std::cerr << "bsdf_validate: FAILED pdf normalization at roughness=" << roughness
                              << " metallic=" << metallic << " ndotV=" << ndotV
                              << " integral=" << integral << " (expected <= 1.0)\n";
                    ok = false;
                }
            }
        }
    }
    return ok;
}

float furnaceLo(const BsdfParams& params, const glm::vec3& wo, int sampleCount, std::uint32_t seed) {
    glm::vec3 accum(0.0F);
    for (int i = 0; i < sampleCount; ++i) {
        engine::scene::Sampler sampler(0, 0, i, seed);
        const std::optional<engine::scene::BsdfSample> sample =
            engine::scene::sampleBsdf(params, wo, sampler);
        if (sample.has_value()) {
            accum += sample->throughputWeight;  // L0=1
        }
    }
    const glm::vec3 lo = accum / static_cast<float>(sampleCount);
    return std::max({lo.x, lo.y, lo.z});
}

// Furnace test through sampleBsdf: uniform radiance L0=1 from every direction (both hemispheres -- transmission can receive from the far side), estimator Lo = mean(throughputWeight) since throughputWeight already folds in f*cosTheta/pdf. ndotV sweep includes negative values (woLocal.z<0, the exiting side of a transmissive dielectric) and a value past the ior=1.5 critical angle (~41.8deg, cosTheta~0.745) to force total internal reflection. Energy bound: 1.0 (L0) everywhere EXCEPT the exiting side (ndotV<0) of a transmissive material below the critical angle, where sampleBsdf's eta^2 non-symmetric radiance-compression factor (Veach 1997 sec. 5.2 -- see bsdf.cpp's transmission branch) legitimately raises Lo above L0: L/n^2 is the invariant along a ray, so radiance increases going from a denser medium (ior=1.5, inside) into a rarer one (1.0, outside) by up to ior^2. The naive Lo<=1 bound only holds for eta==1 interfaces (pure reflection) or the entering side, where this same factor is < 1 -- exactly compensating so a round trip through the surface loses no net energy.
bool checkFurnace() {
    constexpr int kSampleCount = 200000;
    constexpr float kTolerance = 0.1F;
    constexpr float kIor = 1.5F;  // matches makeParams/makeColoredMetalParams
    const std::array<float, 4> roughnesses = {0.05F, 0.25F, 0.5F, 1.0F};
    const std::array<float, 2> metallics = {0.0F, 1.0F};
    const std::array<float, 3> transmissions = {0.0F, 0.5F, 1.0F};
    const std::array<float, 5> ndotVs = {0.2F, 0.6F, 1.0F, -0.9F, -0.3F};  // last two: exiting/TIR

    bool ok = true;
    std::uint32_t seed = 0;
    for (float roughness : roughnesses) {
        for (float metallic : metallics) {
            for (float transmission : transmissions) {
                for (float ndotV : ndotVs) {
                    ++seed;
                    const BsdfParams params = makeParams(roughness, metallic, transmission);
                    const glm::vec3 wo(std::sqrt(std::max(0.0F, 1.0F - (ndotV * ndotV))), 0.0F,
                                        ndotV);
                    const float maxLo = furnaceLo(params, wo, kSampleCount, seed);
                    const bool exitingTransmissive = ndotV < 0.0F && transmission > 0.0F;
                    const float energyBound = exitingTransmissive ? kIor * kIor : 1.0F;
                    if (maxLo > energyBound + kTolerance) {
                        std::cerr << "bsdf_validate: FAILED furnace test at roughness=" << roughness
                                  << " metallic=" << metallic << " transmission=" << transmission
                                  << " ndotV=" << ndotV << " Lo=" << maxLo
                                  << " (expected <= " << energyBound << ")\n";
                        ok = false;
                    }
                }
            }
        }
    }

    // Colored conductor (f0=0.5): the case that caught evaluateDiffuseLobe's pdf-gating bug (a white f0's clamped 95% specular probability hid it under this test's tolerance).
    for (float roughness : roughnesses) {
        for (float ndotV : ndotVs) {
            ++seed;
            const BsdfParams params = makeColoredMetalParams(roughness);
            const glm::vec3 wo(std::sqrt(std::max(0.0F, 1.0F - (ndotV * ndotV))), 0.0F, ndotV);
            const float maxLo = furnaceLo(params, wo, kSampleCount, seed);
            if (maxLo > 1.0F + kTolerance) {
                std::cerr << "bsdf_validate: FAILED colored-metal furnace test at roughness="
                          << roughness << " ndotV=" << ndotV << " Lo=" << maxLo
                          << " (expected <= 1.0)\n";
                ok = false;
            }
        }
    }
    return ok;
}

}  // namespace

int main() {
    const bool pdfOk = checkPdfNormalization();
    const bool furnaceOk = checkFurnace();

    if (!pdfOk || !furnaceOk) {
        std::cerr << "bsdf_validate: FAILED\n";
        return EXIT_FAILURE;
    }
    std::cout << "bsdf_validate: PASSED\n";
    return EXIT_SUCCESS;
}
