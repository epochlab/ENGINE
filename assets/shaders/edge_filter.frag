#version 410 core

in vec2 vUv;

uniform sampler2D uHdrColor;  // the path tracer's Beauty image (real RGB)
uniform int uFilterMode;      // 0=Sobel 1=Gabor 2=Luminance passthrough (no neighborhood filter -- see main())
uniform float uGaborKernel[100];  // 4 orientations x 5x5 taps, see main.cpp's buildGaborKernel
uniform int uChannelView;     // 0=off 1=R 2=G 3=B -- isolation was previously baked into the uploaded texels by a CPU copy of the whole image; as a uniform, toggling it costs nothing and needs no re-upload
uniform float uExposure;     // pow(2, relativeExposureEv()) -- same multiplier Beauty itself displays at (OcioDisplayTransform::bind); applied per-tap in sampleLuminance so Sobel/Gabor's gradient of an already-exposed signal scales by the same factor as Beauty, rather than staying frozen at unity gain
uniform bool uInvert;        // 1.0 - value, applied to the final output colour -- the 'I' debug toggle

out vec4 fragColor;

// Rec.709 luminance. Not just texture(...).r: uHdrColor is real RGB, where .r alone would isolate the red channel, not luminance.
// Channel isolation applies here, before the dot, exactly where the CPU-side bake used to sit in the pipeline. Isolating broadcasts the channel to grey and the Rec.709 weights sum to 1, so the dot then returns that channel unchanged -- the previous behaviour reproduced exactly, not approximated.
float sampleLuminance(vec2 uv, vec2 texel, vec2 offset) {
    vec3 color = texture(uHdrColor, uv + offset * texel).rgb * uExposure;
    if (uChannelView == 1) {
        color = vec3(color.r);
    } else if (uChannelView == 2) {
        color = vec3(color.g);
    } else if (uChannelView == 3) {
        color = vec3(color.b);
    }
    return dot(color, vec3(0.2126, 0.7152, 0.0722));
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
    // Mode 2 (Luminance): the path tracer has no per-AOV Luminance buffer -- reusing this shader's existing uHdrColor/sampleLuminance plumbing for a plain center-tap read is cheaper than a whole new ShaderProgram just to broadcast one dot product.
    float value = uFilterMode == 2   ? sampleLuminance(vUv, texel, vec2(0.0))
                  : uFilterMode == 1 ? gabor(texel)
                                     : sobel(texel);
    float outValue = clamp(value, 0.0, 1.0);
    if (uInvert) {
        outValue = 1.0 - outValue;
    }
    fragColor = vec4(vec3(outValue), 1.0);
}
