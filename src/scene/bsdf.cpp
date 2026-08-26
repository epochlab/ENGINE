#include "engine/scene/bsdf.h"

#include <algorithm>
#include <cmath>

namespace engine::scene {

namespace {

constexpr float kPi = 3.14159265F;
constexpr float kMinAlpha = 0.02F * 0.02F;  // roughness floor, avoids a degenerate GGX delta lobe

float distributionGGX(float ndotH, float alpha) {
    const float alpha2 = alpha * alpha;
    const float d = ((ndotH * ndotH) * (alpha2 - 1.0F)) + 1.0F;
    return alpha2 / std::max(kPi * d * d, 1e-8F);
}

float smithLambda(float ndotV, float alpha) {
    const float ndotV2 = std::max(ndotV * ndotV, 1e-8F);
    const float tan2 = std::max(0.0F, 1.0F - ndotV2) / ndotV2;
    return 0.5F * (-1.0F + std::sqrt(1.0F + (alpha * alpha * tan2)));
}

float smithG1(float ndotV, float alpha) { return 1.0F / (1.0F + smithLambda(ndotV, alpha)); }

float smithG2(float ndotV, float ndotL, float alpha) {
    return 1.0F / (1.0F + smithLambda(ndotV, alpha) + smithLambda(ndotL, alpha));
}

// Exact unpolarized dielectric Fresnel reflectance (PBRT's FrDielectric); 1.0 past total internal reflection.
float fresnelDielectric(float cosThetaI, float etaI, float etaT) {
    cosThetaI = std::clamp(cosThetaI, -1.0F, 1.0F);
    if (cosThetaI < 0.0F) {
        std::swap(etaI, etaT);
        cosThetaI = -cosThetaI;
    }
    const float sinThetaI = std::sqrt(std::max(0.0F, 1.0F - (cosThetaI * cosThetaI)));
    const float sinThetaT = (etaI / etaT) * sinThetaI;
    if (sinThetaT >= 1.0F) {
        return 1.0F;
    }
    const float cosThetaT = std::sqrt(std::max(0.0F, 1.0F - (sinThetaT * sinThetaT)));
    const float rParallel =
        ((etaT * cosThetaI) - (etaI * cosThetaT)) / ((etaT * cosThetaI) + (etaI * cosThetaT));
    const float rPerpendicular =
        ((etaI * cosThetaI) - (etaT * cosThetaT)) / ((etaI * cosThetaI) + (etaT * cosThetaT));
    return ((rParallel * rParallel) + (rPerpendicular * rPerpendicular)) * 0.5F;
}

// Heitz 2018 VNDF sampling. wo.z > 0 required (caller pre-flips into the +z hemisphere).
glm::vec3 sampleGGXVNDF(const glm::vec3& wo, float alpha, glm::vec2 u) {
    const glm::vec3 vh = glm::normalize(glm::vec3(alpha * wo.x, alpha * wo.y, wo.z));
    const float lensq = (vh.x * vh.x) + (vh.y * vh.y);
    const glm::vec3 t1 = lensq > 0.0F ? glm::vec3(-vh.y, vh.x, 0.0F) * (1.0F / std::sqrt(lensq))
                                       : glm::vec3(1.0F, 0.0F, 0.0F);
    const glm::vec3 t2 = glm::cross(vh, t1);
    const float r = std::sqrt(u.x);
    const float phi = 2.0F * kPi * u.y;
    const float t1p = r * std::cos(phi);
    float t2p = r * std::sin(phi);
    const float s = 0.5F * (1.0F + vh.z);
    t2p = ((1.0F - s) * std::sqrt(std::max(0.0F, 1.0F - (t1p * t1p)))) + (s * t2p);
    const glm::vec3 nh = (t1p * t1) + (t2p * t2) +
                          (std::sqrt(std::max(0.0F, 1.0F - (t1p * t1p) - (t2p * t2p))) * vh);
    return glm::normalize(glm::vec3(alpha * nh.x, alpha * nh.y, std::max(0.0F, nh.z)));
}

glm::vec3 sampleCosineHemisphere(glm::vec2 u) {
    const float r = std::sqrt(u.x);
    const float phi = 2.0F * kPi * u.y;
    return {r * std::cos(phi), r * std::sin(phi), std::sqrt(std::max(0.0F, 1.0F - u.x))};
}

struct LobeEval {
    glm::vec3 f;
    float pdf;
};

// kd = (1-F)(1-metallic)(1-transmissionFactor), fixed at wo.z (not per-wi) -- NdotV-based energy split avoids diffuse+specular jointly exceeding received energy.
// pdf must NOT be gated on kd: sampleBsdf still selects this lobe with probability lobes.diffuse
// (independent of kd, see computeLobeProbabilities), so the pdf side of the MIS mixture must match
// that selection density regardless of how little/no value the lobe carries -- gating pdf on kd
// starves the mixture denominator and inflates throughput for metals (kd=0 but diffuseProb>0).
LobeEval evaluateDiffuseLobe(const BsdfParams& params, const glm::vec3& wi, float kd) {
    if (wi.z <= 0.0F) {
        return {glm::vec3(0.0F), 0.0F};
    }
    return {params.baseColor * std::max(kd, 0.0F) / kPi, wi.z / kPi};
}

// D*G2*F/(4*ndotV*ndotL) plus VNDF pdf (Heitz 2018 eq.3, Jacobian 1/(4*dot(wo,nh))).
LobeEval evaluateSpecularLobe(const BsdfParams& params, const glm::vec3& wo, const glm::vec3& wi,
                               float alpha, float etaI, float etaT) {
    if (wo.z <= 0.0F || wi.z <= 0.0F) {
        return {glm::vec3(0.0F), 0.0F};
    }
    const glm::vec3 nh = glm::normalize(wo + wi);
    const float woDotNh = std::max(glm::dot(wo, nh), 0.0F);
    if (woDotNh <= 0.0F) {
        return {glm::vec3(0.0F), 0.0F};
    }
    const float d = distributionGGX(nh.z, alpha);
    const float g2 = smithG2(wo.z, wi.z, alpha);
    const float fDielectric = fresnelDielectric(woDotNh, etaI, etaT);
    const glm::vec3 f =
        glm::mix(glm::vec3(fDielectric), fresnelSchlick(woDotNh, params.f0), params.metallic);
    const glm::vec3 value = (d * g2 * f) / std::max(4.0F * wo.z * wi.z, 1e-6F);
    const float g1 = smithG1(wo.z, alpha);
    const float pdf = (g1 * woDotNh * d) / std::max(wo.z, 1e-6F) / (4.0F * woDotNh);
    return {value, pdf};
}

struct LobeProbabilities {
    float specular;
    float diffuse;
    float transmit;
    float etaI;
    float etaT;
    float diffuseKd;              // evaluateDiffuseLobe's energy factor, 0 on the exiting side
    float transmitPhysicalValue;  // transmission's true (1-F)*t energy fraction -- see below
};

// specular = Fresnel reflectance probability (exact dielectric via ior, Schlick via f0 for
// conductors, blended by metallic). "Exiting" (transmissive material, sign<0, already inside): no
// diffuse substrate, transmit takes everything specular didn't -- reflect internally or exit, no
// third option. Everything else -- entering (sign>0), or an opaque material's woLocal.z pushed
// negative by grazing-angle normal mapping -- uses the entering split: diffuse/transmit divide the
// remainder by transmissionFactor (0 for opaque, so transmit vanishes and this reduces to
// diffuse+specular regardless of which side of the interpolated normal wo landed on).
// transmitPhysicalValue != transmit: throughput = physicalValue/transmit, so physicalValue must
// independently carry the same transmissionFactor/metallic factors transmit's probability used, or
// they cancel out of the throughput and silently erase their effect on energy (caught by
// tools/bsdf_validate.cpp's furnace test).
LobeProbabilities computeLobeProbabilities(const BsdfParams& params, const glm::vec3& wo,
                                            float sign) {
    const bool exiting = sign < 0.0F && params.transmissionFactor > 0.0F;
    const float etaI = exiting ? params.ior : 1.0F;
    const float etaT = exiting ? 1.0F : params.ior;
    const float fresnelAtNormal = fresnelDielectric(wo.z, etaI, etaT);
    const glm::vec3 fresnelConductor = fresnelSchlick(wo.z, params.f0);
    const float conductorLuma = (fresnelConductor.x + fresnelConductor.y + fresnelConductor.z) / 3.0F;
    const float specularProb =
        std::clamp(glm::mix(fresnelAtNormal, conductorLuma, params.metallic), 0.05F, 0.95F);
    const float transmittance = (1.0F - fresnelAtNormal) * (1.0F - params.metallic);
    float diffuseProb = 0.0F;
    float transmitProb = 0.0F;
    float diffuseKd = 0.0F;
    float transmitPhysicalValue = 0.0F;
    if (!exiting) {
        diffuseProb = (1.0F - specularProb) * (1.0F - params.transmissionFactor);
        transmitProb = (1.0F - specularProb) * params.transmissionFactor;
        diffuseKd = transmittance * (1.0F - params.transmissionFactor);
        transmitPhysicalValue = transmittance * params.transmissionFactor;
    } else {
        transmitProb = 1.0F - specularProb;
        transmitPhysicalValue = transmittance;
    }
    return {specularProb, diffuseProb, transmitProb, etaI, etaT, diffuseKd, transmitPhysicalValue};
}

glm::vec3 evaluateContinuousLobes(const BsdfParams& params, const glm::vec3& wo, const glm::vec3& wi,
                                   float alpha, const LobeProbabilities& lobes, float& outPdf) {
    const LobeEval specular = evaluateSpecularLobe(params, wo, wi, alpha, lobes.etaI, lobes.etaT);
    const LobeEval diffuse = evaluateDiffuseLobe(params, wi, lobes.diffuseKd);
    outPdf = (lobes.specular * specular.pdf) + (lobes.diffuse * diffuse.pdf);
    return specular.f + diffuse.f;
}

}  // namespace

glm::vec3 fresnelSchlick(float cosTheta, const glm::vec3& f0) {
    return f0 + ((glm::vec3(1.0F) - f0) * std::pow(std::clamp(1.0F - cosTheta, 0.0F, 1.0F), 5.0F));
}

float pdfBsdf(const BsdfParams& params, const glm::vec3& woLocal, const glm::vec3& wiLocal) {
    const float sign = woLocal.z >= 0.0F ? 1.0F : -1.0F;
    const glm::vec3 wo(woLocal.x, woLocal.y, woLocal.z * sign);
    const glm::vec3 wi(wiLocal.x, wiLocal.y, wiLocal.z * sign);
    const float alpha = std::max(params.roughness * params.roughness, kMinAlpha);
    const LobeProbabilities lobes = computeLobeProbabilities(params, wo, sign);
    float pdf = 0.0F;
    evaluateContinuousLobes(params, wo, wi, alpha, lobes, pdf);
    return pdf;
}

glm::vec3 evaluateBsdf(const BsdfParams& params, const glm::vec3& woLocal, const glm::vec3& wiLocal) {
    const float sign = woLocal.z >= 0.0F ? 1.0F : -1.0F;
    const glm::vec3 wo(woLocal.x, woLocal.y, woLocal.z * sign);
    const glm::vec3 wi(wiLocal.x, wiLocal.y, wiLocal.z * sign);
    const float alpha = std::max(params.roughness * params.roughness, kMinAlpha);
    const LobeProbabilities lobes = computeLobeProbabilities(params, wo, sign);
    float pdf = 0.0F;
    return evaluateContinuousLobes(params, wo, wi, alpha, lobes, pdf);
}

glm::vec3 evaluateDiffuseRaw(const BsdfParams& params, const glm::vec3& woLocal,
                              const glm::vec3& wiLocal) {
    const float sign = woLocal.z >= 0.0F ? 1.0F : -1.0F;
    const glm::vec3 wo(woLocal.x, woLocal.y, woLocal.z * sign);
    const glm::vec3 wi(wiLocal.x, wiLocal.y, wiLocal.z * sign);
    if (wi.z <= 0.0F) {
        return glm::vec3(0.0F);
    }
    const LobeProbabilities lobes = computeLobeProbabilities(params, wo, sign);
    return glm::vec3(std::max(lobes.diffuseKd, 0.0F) / kPi);
}

glm::vec3 evaluateSpecularOnly(const BsdfParams& params, const glm::vec3& woLocal,
                                const glm::vec3& wiLocal) {
    const float sign = woLocal.z >= 0.0F ? 1.0F : -1.0F;
    const glm::vec3 wo(woLocal.x, woLocal.y, woLocal.z * sign);
    const glm::vec3 wi(wiLocal.x, wiLocal.y, wiLocal.z * sign);
    const float alpha = std::max(params.roughness * params.roughness, kMinAlpha);
    const LobeProbabilities lobes = computeLobeProbabilities(params, wo, sign);
    return evaluateSpecularLobe(params, wo, wi, alpha, lobes.etaI, lobes.etaT).f;
}

std::optional<BsdfSample> sampleBsdf(const BsdfParams& params, const glm::vec3& woLocal,
                                      Sampler& sampler) {
    const float sign = woLocal.z >= 0.0F ? 1.0F : -1.0F;
    const glm::vec3 wo(woLocal.x, woLocal.y, woLocal.z * sign);
    const float alpha = std::max(params.roughness * params.roughness, kMinAlpha);
    const LobeProbabilities lobes = computeLobeProbabilities(params, wo, sign);

    const float lobeU = sampler.next1D();

    if (lobeU < lobes.specular + lobes.diffuse) {
        const bool sampledSpecular = lobeU < lobes.specular;
        glm::vec3 wi;
        if (sampledSpecular) {
            const glm::vec3 nh = sampleGGXVNDF(wo, alpha, sampler.next2D());
            wi = glm::reflect(-wo, nh);
        } else {
            wi = sampleCosineHemisphere(sampler.next2D());
        }
        if (wi.z <= 0.0F) {
            return std::nullopt;
        }
        float pdf = 0.0F;
        const glm::vec3 f = evaluateContinuousLobes(params, wo, wi, alpha, lobes, pdf);
        if (pdf <= 1e-8F) {
            return std::nullopt;
        }
        const glm::vec3 throughput = (f * wi.z) / pdf;
        const glm::vec3 rawThroughput =
            sampledSpecular ? throughput : glm::vec3(std::max(lobes.diffuseKd, 0.0F));
        return BsdfSample{glm::vec3(wi.x, wi.y, wi.z * sign), throughput,
                           sampledSpecular ? LobeType::SpecularReflection : LobeType::Diffuse,
                           rawThroughput};
    }

    // Smooth specular transmission (delta lobe): Snell's law, TIR already folded into
    // lobes.specular via fresnelDielectric returning 1.0 past the critical angle.
    if (lobes.transmit <= 0.0F) {
        return std::nullopt;
    }
    const float eta = lobes.etaI / lobes.etaT;
    const float sin2ThetaT = eta * eta * std::max(0.0F, 1.0F - (wo.z * wo.z));
    if (sin2ThetaT >= 1.0F) {
        return std::nullopt;
    }
    const float cosThetaT = std::sqrt(1.0F - sin2ThetaT);
    const glm::vec3 wt(-eta * wo.x, -eta * wo.y, -cosThetaT);
    // Non-symmetric radiance-compression factor for camera-originated (Veach 1997 sec. 5.2, PBRT's
    // SpecularTransmission::Sample_f under TransportMode::Radiance) transport: eta^2 = (etaI/etaT)^2,
    // the squared ratio of the medium the ray is leaving to the medium it's entering. Self-consistent
    // under round trips -- entering (eta=1/ior) times exiting (eta=ior/1) squared multiplies to 1, so
    // a ray that enters and exits the same surface loses no net energy (tools/bsdf_validate.cpp's
    // furnace test).
    const glm::vec3 throughput =
        params.baseColor * (lobes.transmitPhysicalValue / lobes.transmit) * (eta * eta);
    return BsdfSample{glm::vec3(wt.x, wt.y, wt.z * sign), throughput, LobeType::SpecularTransmission,
                       throughput};
}

}  // namespace engine::scene
