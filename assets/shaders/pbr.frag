#version 410 core

in vec2 vUv;
in vec3 vWorldNormal;
in vec3 vWorldPos;
in vec4 vWorldTangent;

uniform sampler2D uBaseColor;
uniform sampler2D uRoughness;
uniform sampler2D uAo;
uniform float uMetallicFactor;
uniform vec3 uBoundsMin;  // world-space instance AABB, for the World position AOV
uniform vec3 uBoundsMax;
uniform vec3 uObjectIdColor;

// 0=Beauty 1=BaseColor 2=Normal 3=GeomNormal 4=Roughness 5=UV 6=WorldPos
// 7=Tangent 8=Metallic 9=ObjectID 10=AO -- order must match main.cpp's
// AOV combo list.
uniform int uAov;
uniform int uChannelView;  // 0=off 1=R 2=G 3=B

out vec4 fragColor;

void main() {
    vec3 color;
    if (uAov == 1) {
        color = texture(uBaseColor, vUv).rgb;
    } else if (uAov == 2 || uAov == 3) {
        // Normal (shading) and Geometric normal are the same value today --
        // no normal-map perturbation exists yet to distinguish them.
        color = normalize(vWorldNormal) * 0.5 + 0.5;
    } else if (uAov == 4) {
        color = vec3(texture(uRoughness, vUv).r);
    } else if (uAov == 5) {
        color = vec3(fract(vUv), 0.0);
    } else if (uAov == 6) {
        vec3 extent = max(uBoundsMax - uBoundsMin, vec3(1e-6));
        color = clamp((vWorldPos - uBoundsMin) / extent, 0.0, 1.0);
    } else if (uAov == 7) {
        color = normalize(vWorldTangent.xyz) * 0.5 + 0.5;
    } else if (uAov == 8) {
        color = vec3(uMetallicFactor);
    } else if (uAov == 9) {
        color = uObjectIdColor;
    } else if (uAov == 10) {
        color = vec3(texture(uAo, vUv).r);
    } else {
        // Beauty (0): unlit base color until lighting lands.
        color = texture(uBaseColor, vUv).rgb;
    }

    if (uChannelView == 1) {
        color = vec3(color.r);
    } else if (uChannelView == 2) {
        color = vec3(color.g);
    } else if (uChannelView == 3) {
        color = vec3(color.b);
    }

    fragColor = vec4(color, 1.0);
}
