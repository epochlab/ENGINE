#version 410 core

out vec2 vUv;

void main() {
    vec2 p = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2);
    // v inverted so v=0 is the window top, matching HdrImage's row-0-is-top; two interpolated values replace a CPU row-reverse.
    vUv = vec2(p.x, 1.0 - p.y);
    gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0);
}
