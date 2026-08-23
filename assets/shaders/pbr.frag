#version 410 core

in vec2 vUv;
in vec3 vWorldNormal;
in vec3 vWorldPos;
in vec4 vWorldTangent;

uniform sampler2D uBaseColor;

out vec4 fragColor;

void main() {
    // First increment: unlit base color only, to visually verify
    // mesh/UV/texture correctness before layering in normal mapping
    // and lighting.
    vec3 baseColor = texture(uBaseColor, vUv).rgb;
    fragColor = vec4(baseColor, 1.0);
}
