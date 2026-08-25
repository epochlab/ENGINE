#version 410 core

// Background pass: reconstructs a world-space view ray per pixel and samples the raw (unprefiltered) equirectangular environment map -- sharp, not the blurred prefiltered specular mips, since this is meant to look like an actual sky/background. Drawn with depth test and depth write both disabled, before the depth-tested opaque geometry pass, so geometry simply overwrites it wherever it covers a pixel; left untouched, uncovered background pixels show the environment.

in vec2 vUv;

out vec4 fragColor;

uniform sampler2D uEquirect;
uniform mat4 uInvViewProj;
uniform vec3 uCameraPos;
// User-controlled Y-axis (world up) rotation of the environment, in radians -- see pbr.frag's matching uEnvRotationRadians/rotateAboutY for why this rotates the query direction rather than re-baking anything (same trick keeps sky/diffuse/specular all consistent).
uniform float uEnvRotationRadians;

const float kPi = 3.14159265;

// Rotates v by angleRadians about the world Y axis (right-handed). Sampling the *unrotated* environment at rotateAboutY(d, -angle) gives the same result as sampling the environment rotated by +angle at the original direction d -- standard inverse-rotate-the-query trick, applied identically in pbr.frag for SH irradiance and the prefiltered specular cubemap, so all three stay in sync without ever re-baking the SH coefficients or the cubemap.
vec3 rotateAboutY(vec3 v, float angleRadians) {
    float c = cos(angleRadians);
    float s = sin(angleRadians);
    return vec3((v.x * c) + (v.z * s), v.y, (-v.x * s) + (v.z * c));
}

void main() {
    vec2 ndc = (vUv * 2.0) - 1.0;
    // NDC z=1 -- the far plane -- unprojected back to a world-space point, giving a ray direction from the camera through this pixel.
    vec4 farPoint4 = uInvViewProj * vec4(ndc, 1.0, 1.0);
    vec3 farPoint = farPoint4.xyz / farPoint4.w;
    vec3 dir = normalize(farPoint - uCameraPos);
    vec3 rotatedDir = rotateAboutY(dir, -uEnvRotationRadians);

    // Direction -> equirect UV, identical to equirect_to_cubemap.frag's mapping (and sh_irradiance.cpp's inverse of it) -- theta=v*pi from +Y, phi=(u-0.5)*2*pi.
    float theta = acos(clamp(rotatedDir.y, -1.0, 1.0));
    float phi = atan(rotatedDir.x, rotatedDir.z);
    vec2 uv = vec2((phi / (2.0 * kPi)) + 0.5, theta / kPi);

    // Explicit LOD 0, not automatic derivative-based texture(): at the phi=+/-180deg seam, uv.x jumps from ~1 to ~0 between adjacent screen pixels, so the hardware's dFdx/dFdy-derived UV derivative spikes to ~1.0 there -- read as "this texel spans the whole texture" and forcing a heavily blurred/bright mip level, visible as a vertical seam regardless of wrap mode. The sky is meant to be sharp anyway (see file header), so forcing mip 0 both fixes the seam and matches the intent.
    fragColor = vec4(textureLod(uEquirect, uv, 0.0).rgb, 1.0);
}
