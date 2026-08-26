#version 410 core

in vec2 vUv;

uniform sampler2D uHdrColor;  // the path tracer's Beauty image
uniform int uChannelView;  // 0=off(HSV) 1=Hue 2=Saturation 3=Value -- applied to the HSV output, not uHdrColor's RGB (isolating a source RGB channel first would broadcast it to grey, which always converts to H=0/S=0 -- destroying the very thing this AOV exists to show).

out vec4 fragColor;

vec3 rgb2hsv(vec3 c) {
    vec4 k = vec4(0.0, -1.0 / 3.0, 2.0 / 3.0, -1.0);
    vec4 p = mix(vec4(c.bg, k.wz), vec4(c.gb, k.xy), step(c.b, c.g));
    vec4 q = mix(vec4(p.xyw, c.r), vec4(c.r, p.yzx), step(p.x, c.r));
    float d = q.x - min(q.w, q.y);
    float e = 1.0e-10;
    return vec3(abs(q.z + (q.w - q.y) / (6.0 * d + e)), d / (q.x + e), q.x);
}

void main() {
    vec3 beauty = texture(uHdrColor, vUv).rgb;
    vec3 hsv = rgb2hsv(beauty);
    vec3 outColor = vec3(hsv.x, hsv.y, clamp(hsv.z, 0.0, 1.0));
    if (uChannelView == 1) {
        outColor = vec3(outColor.r);  // Hue
    } else if (uChannelView == 2) {
        outColor = vec3(outColor.g);  // Saturation
    } else if (uChannelView == 3) {
        outColor = vec3(outColor.b);  // Value
    }
    fragColor = vec4(outColor, 1.0);
}
