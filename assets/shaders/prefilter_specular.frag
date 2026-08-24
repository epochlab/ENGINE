#version 410 core

// GGX-importance-sampled specular prefilter, one mip level of one
// cubemap face per draw (Karis 2013, "Real Shading in Unreal Engine 4",
// split-sum specular prefilter). uEnvCubemap is the sharp base cubemap
// (mip 0, built by equirect_to_cubemap.frag); this shader is never used
// for mip 0 itself (env_prefilter_pass.cpp copies the base directly —
// roughness 0 is a delta BRDF, prefiltering it would just reproduce the
// source with extra cost). The standard split-sum simplification
// V = R = N is used throughout, matching pbr.frag's IBL specular sample
// direction at shading time.

in vec2 vUv;

out vec4 fragColor;

uniform samplerCube uEnvCubemap;
uniform vec3 uFaceRight;
uniform vec3 uFaceUp;
uniform vec3 uFaceForward;
uniform float uRoughness;

const float kPi = 3.14159265;
const uint kSampleCount = 64u;

// Van der Corput radical inverse, base 2 (bit-reversal trick).
float radicalInverseVdc(uint bits) {
    bits = (bits << 16u) | (bits >> 16u);
    bits = ((bits & 0x55555555u) << 1u) | ((bits & 0xAAAAAAAAu) >> 1u);
    bits = ((bits & 0x33333333u) << 2u) | ((bits & 0xCCCCCCCCu) >> 2u);
    bits = ((bits & 0x0F0F0F0Fu) << 4u) | ((bits & 0xF0F0F0F0u) >> 4u);
    bits = ((bits & 0x00FF00FFu) << 8u) | ((bits & 0xFF00FF00u) >> 8u);
    return float(bits) * 2.3283064365386963e-10;
}

vec2 hammersley(uint i, uint n) {
    return vec2(float(i) / float(n), radicalInverseVdc(i));
}

// Maps a low-discrepancy 2D sample to a half-vector distributed
// proportionally to the GGX NDF around n.
vec3 importanceSampleGGX(vec2 xi, vec3 n, float roughness) {
    float alpha = roughness * roughness;
    float phi = 2.0 * kPi * xi.x;
    float cosTheta = sqrt((1.0 - xi.y) / (1.0 + (((alpha * alpha) - 1.0) * xi.y)));
    float sinTheta = sqrt(1.0 - (cosTheta * cosTheta));

    vec3 h = vec3(sinTheta * cos(phi), sinTheta * sin(phi), cosTheta);

    vec3 up = abs(n.z) < 0.999 ? vec3(0.0, 0.0, 1.0) : vec3(1.0, 0.0, 0.0);
    vec3 tangent = normalize(cross(up, n));
    vec3 bitangent = cross(n, tangent);
    return normalize((tangent * h.x) + (bitangent * h.y) + (n * h.z));
}

void main() {
    vec2 ndc = (vUv * 2.0) - 1.0;
    vec3 n = normalize(uFaceForward + (ndc.x * uFaceRight) + (ndc.y * uFaceUp));
    vec3 v = n;

    vec3 prefiltered = vec3(0.0);
    float totalWeight = 0.0;
    for (uint i = 0u; i < kSampleCount; ++i) {
        vec2 xi = hammersley(i, kSampleCount);
        vec3 h = importanceSampleGGX(xi, n, uRoughness);
        vec3 l = normalize((2.0 * dot(v, h) * h) - v);
        float ndotL = max(dot(n, l), 0.0);
        if (ndotL > 0.0) {
            // Explicit LOD 0: uEnvCubemap is this same cubemap object
            // mid-bake (higher mips not yet written when this draw
            // targets an early mip) -- automatic derivative-based LOD
            // selection could sample a not-yet-populated level.
            prefiltered += textureLod(uEnvCubemap, l, 0.0).rgb * ndotL;
            totalWeight += ndotL;
        }
    }
    fragColor = vec4(prefiltered / max(totalWeight, 1e-4), 1.0);
}
