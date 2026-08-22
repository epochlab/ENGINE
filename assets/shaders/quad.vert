#version 410 core

layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec2 aUv;

out vec2 vUv;

void main() {
    vUv = aUv;
    // No view/projection this stage: Mesh::createQuad()'s vertices are
    // already authored directly in clip space, so passing position
    // straight through is correct. Camera's matrices gain their first
    // real consumer once Phase 2/3 needs to place geometry relative to a
    // moving viewpoint.
    gl_Position = vec4(aPosition, 1.0);
}
