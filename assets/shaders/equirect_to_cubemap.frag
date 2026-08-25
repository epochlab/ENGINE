#version 410 core

// Resamples an equirectangular (latlong) HDR environment map into one face of a cubemap. Draw once per face with fullscreen_triangle.vert; uFaceRight/uFaceUp/uFaceForward are the CPU-supplied per-face basis (see env_prefilter_pass.cpp) that turns this face's screen-space UV into a world-space direction, matching the OpenGL cubemap face convention (so later texture(samplerCube, dir) reads land on the texel this shader wrote for that direction).

in vec2 vUv;

out vec4 fragColor;

uniform sampler2D uEquirect;
uniform vec3 uFaceRight;
uniform vec3 uFaceUp;
uniform vec3 uFaceForward;

const float kPi = 3.14159265;

void main() {
    vec2 ndc = (vUv * 2.0) - 1.0;
    vec3 dir = normalize(uFaceForward + (ndc.x * uFaceRight) + (ndc.y * uFaceUp));

    // Direction -> equirect UV. Must match sh_irradiance.cpp's inverse (UV -> direction) mapping: theta=v*pi from +Y, phi=(u-0.5)*2*pi.
    float theta = acos(clamp(dir.y, -1.0, 1.0));
    float phi = atan(dir.x, dir.z);
    vec2 uv = vec2((phi / (2.0 * kPi)) + 0.5, theta / kPi);

    // Explicit LOD 0, not automatic derivative-based texture(): at the phi=+/-180deg seam, uv.x jumps from ~1 to ~0 between adjacent fragments, spiking the hardware's dFdx/dFdy-derived UV derivative and forcing a heavily blurred mip -- baking a visible seam into this cubemap face that would persist for the whole run (see sky.frag's identical fix for the same underlying issue).
    fragColor = vec4(textureLod(uEquirect, uv, 0.0).rgb, 1.0);
}
