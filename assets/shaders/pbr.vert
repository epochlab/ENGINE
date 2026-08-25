#version 410 core

layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec2 aUv;
layout(location = 2) in vec3 aNormal;
layout(location = 3) in vec4 aTangent;

uniform mat4 uModel;
uniform mat4 uView;
uniform mat4 uProjection;
// Inverse-transpose of uModel's upper 3x3, corrects for non-uniform scale -- computed once per draw call on the CPU (main.cpp), not per-vertex: it's the same matrix for every vertex in the mesh.
uniform mat3 uNormalMatrix;

out vec2 vUv;
out vec3 vWorldNormal;
out vec3 vWorldPos;
out vec4 vWorldTangent;

void main() {
    vec4 worldPos = uModel * vec4(aPosition, 1.0);
    vWorldPos = worldPos.xyz;
    vUv = aUv;

    vWorldNormal = uNormalMatrix * aNormal;
    vWorldTangent = vec4(mat3(uModel) * aTangent.xyz, aTangent.w);

    gl_Position = uProjection * uView * worldPos;
}
