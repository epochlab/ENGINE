// Placeholder only — replaced by the real OCIO display transform in
// Stage F. TASK.md's Stage G removes/marks-unused this file once Stage F
// lands; don't over-invest here.
#version 410 core

in vec2 vUv;
out vec4 fragColor;

uniform sampler2D uHdrColor;

void main() {
    fragColor = vec4(texture(uHdrColor, vUv).rgb, 1.0);
}
