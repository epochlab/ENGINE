#version 410 core

out vec2 vUv;

void main() {
    vec2 p = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2);
    // v inverted so v=0 lands at the top of the window, matching HdrImage's row-0-is-top convention (hdr_image.h) directly. The CPU used to reconcile the two by row-reversing the whole image into a scratch buffer before every upload; two interpolated values do it for free. Every program sharing this vertex shader samples the same display texture, so the convention change is total.
    vUv = vec2(p.x, 1.0 - p.y);
    gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0);
}
