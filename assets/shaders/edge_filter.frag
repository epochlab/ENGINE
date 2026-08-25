#version 410 core

in vec2 vUv;

uniform sampler2D uHdrColor;  // holds Luminance (pbr.frag's Sobel/Gabor branch), not Beauty
uniform int uFilterMode;      // 0=Sobel 1=Gabor
uniform float uGaborKernel[100];  // 4 orientations x 5x5 taps, see main.cpp's buildGaborKernel

out vec4 fragColor;

float sampleLuminance(vec2 uv, vec2 texel, vec2 offset) {
    return texture(uHdrColor, uv + offset * texel).r;
}

// Fixed 3x3 Sobel Gx/Gy kernels, gradient magnitude of the Luminance AOV.
float sobel(vec2 texel) {
    float tl = sampleLuminance(vUv, texel, vec2(-1.0, 1.0));
    float t = sampleLuminance(vUv, texel, vec2(0.0, 1.0));
    float tr = sampleLuminance(vUv, texel, vec2(1.0, 1.0));
    float l = sampleLuminance(vUv, texel, vec2(-1.0, 0.0));
    float r = sampleLuminance(vUv, texel, vec2(1.0, 0.0));
    float bl = sampleLuminance(vUv, texel, vec2(-1.0, -1.0));
    float b = sampleLuminance(vUv, texel, vec2(0.0, -1.0));
    float br = sampleLuminance(vUv, texel, vec2(1.0, -1.0));

    float gx = -tl - 2.0 * l - bl + tr + 2.0 * r + br;
    float gy = -bl - 2.0 * b - br + tl + 2.0 * t + tr;
    return length(vec2(gx, gy));
}

// 4-orientation Gabor bank (0/45/90/135 degrees), max |response| across orientations. Each of the 25 neighborhood texels is fetched once and reused across all 4 orientations -- uGaborKernel's weights already bake in the per-orientation envelope*carrier, computed once on the CPU (main.cpp) rather than re-derived per-fragment.
float gabor(vec2 texel) {
    float response[4] = float[4](0.0, 0.0, 0.0, 0.0);
    int tapIndex = 0;
    for (int dy = -2; dy <= 2; ++dy) {
        for (int dx = -2; dx <= 2; ++dx) {
            float lum = sampleLuminance(vUv, texel, vec2(float(dx), float(dy)));
            response[0] += uGaborKernel[0 * 25 + tapIndex] * lum;
            response[1] += uGaborKernel[1 * 25 + tapIndex] * lum;
            response[2] += uGaborKernel[2 * 25 + tapIndex] * lum;
            response[3] += uGaborKernel[3 * 25 + tapIndex] * lum;
            ++tapIndex;
        }
    }
    float magnitude = max(max(abs(response[0]), abs(response[1])),
                           max(abs(response[2]), abs(response[3])));
    return magnitude;
}

void main() {
    vec2 texel = 1.0 / vec2(textureSize(uHdrColor, 0));
    float value = uFilterMode == 1 ? gabor(texel) : sobel(texel);
    fragColor = vec4(vec3(clamp(value, 0.0, 1.0)), 1.0);
}
