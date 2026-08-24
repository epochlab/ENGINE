#version 410 core

in vec2 vUv;
in vec3 vWorldNormal;
in vec3 vWorldPos;
in vec4 vWorldTangent;

uniform sampler2D uBaseColor;
uniform sampler2D uNormal;
uniform sampler2D uRoughness;
uniform sampler2D uBump;
uniform sampler2D uSpecular;
uniform sampler2D uAo;
uniform vec3 uBaseColorFactor;  // glTF baseColorFactor.rgb (alpha unused, opaque-only renderer)
uniform float uMetallicFactor;
uniform float uRoughnessFactor;
uniform vec3 uBoundsMin;  // world-space instance AABB, for the World position AOV
uniform vec3 uBoundsMax;
uniform vec3 uObjectIdColor;
uniform vec3 uLightDir;    // world-space, points toward the light
uniform vec3 uLightColor;
uniform vec3 uCameraPos;
uniform float uNearClip;
uniform float uFarClip;

// 0=Beauty 1=Alpha 2=Depth 3=HSV 4=Luminance 5=Sobel 6=Gabor 7=WorldPos
// 8=UV 9=Normal 10=GeomNormal 11=Albedo 12=Metallic 13=Roughness
// 14=Tangent 15=ObjectID 16=AO -- order matches the README's §3 AOV
// reference table, and must match main.cpp's AOV combo list.
// Sobel/Gabor (5/6) only need this shader to output Luminance (same as
// 4) -- the actual edge filtering is a second pass, see
// edge_filter.frag.
uniform int uAov;
uniform int uChannelView;  // 0=off 1=R 2=G 3=B

out vec4 fragColor;

const float kPi = 3.14159265;

// Rec.709 luma weights -- matches OcioDisplayTransform's "Linear
// Rec.709 (sRGB)" working color space.
float luminance(vec3 c) {
    return dot(c, vec3(0.2126, 0.7152, 0.0722));
}

// Standard branchless RGB->HSV (h, s in [0,1); v unbounded, matching
// input scale).
vec3 rgb2hsv(vec3 c) {
    vec4 k = vec4(0.0, -1.0 / 3.0, 2.0 / 3.0, -1.0);
    vec4 p = mix(vec4(c.bg, k.wz), vec4(c.gb, k.xy), step(c.b, c.g));
    vec4 q = mix(vec4(p.xyw, c.r), vec4(c.r, p.yzx), step(p.x, c.r));
    float d = q.x - min(q.w, q.y);
    float e = 1.0e-10;
    return vec3(abs(q.z + (q.w - q.y) / (6.0 * d + e)), d / (q.x + e), q.x);
}

// Lambertian diffuse + Schlick-Fresnel specular against one fixed test
// light. No microfacet D/G terms (importance sampling is Phase 5), no
// punctual-light system (Phase 4). AO is multiplied in as a blanket
// occlusion term -- not physically exact with no ambient/indirect term
// to occlude, but a common approximation for grounding contact shadows.
vec3 shadeBeauty(vec3 n, vec3 worldPos, vec3 baseColor, float roughness, float metallic,
                  vec3 specularSample, float aoSample) {
    vec3 v = normalize(uCameraPos - worldPos);
    vec3 l = normalize(uLightDir);
    vec3 h = normalize(v + l);
    float ndotL = max(dot(n, l), 0.0);
    float ndotH = max(dot(n, h), 0.0);
    float vdotH = max(dot(v, h), 0.0);

    vec3 f0 = mix(specularSample, baseColor, metallic);
    vec3 fresnel = f0 + (1.0 - f0) * pow(1.0 - vdotH, 5.0);

    // Roughness -> Blinn-Phong exponent, standard alpha^2 equivalence
    // (Real-Time Rendering 4th ed. eq. 9.35).
    float alpha = roughness * roughness;
    float shininess = 2.0 / (alpha * alpha) - 2.0;

    vec3 diffuse = baseColor * (1.0 - metallic) / kPi;
    // max(ndotH, 1e-4): pow(0, shininess) is undefined when
    // shininess<=0 (roughness at its 1.0 ceiling), which some drivers
    // evaluate to NaN.
    vec3 specular = fresnel * pow(max(ndotH, 1e-4), shininess);
    return (diffuse + specular) * uLightColor * ndotL * aoSample;
}

void main() {
    // Bump detail normal: central-difference height gradient, UV-space
    // (texel-offset, not screen-space dFdx/dFdy -- independent of mesh
    // density).
    vec2 texel = 1.0 / vec2(textureSize(uBump, 0));
    float hL = texture(uBump, vUv - vec2(texel.x, 0.0)).r;
    float hR = texture(uBump, vUv + vec2(texel.x, 0.0)).r;
    float hD = texture(uBump, vUv - vec2(0.0, texel.y)).r;
    float hU = texture(uBump, vUv + vec2(0.0, texel.y)).r;
    vec3 detailNormalTs = normalize(vec3(hL - hR, hD - hU, 1.0));

    // Tangent-space normal map (3-channel, no Z-reconstruction needed),
    // blended with the bump detail (additive XY, base Z).
    vec3 normalTs = texture(uNormal, vUv).rgb * 2.0 - 1.0;
    vec3 combinedTs = normalize(vec3(normalTs.xy + detailNormalTs.xy, normalTs.z));

    // TBN, re-orthogonalized (Gram-Schmidt) against the interpolated
    // vertex normal.
    vec3 geoNormal = normalize(vWorldNormal);
    vec3 tangent = normalize(vWorldTangent.xyz);
    tangent = normalize(tangent - dot(tangent, geoNormal) * geoNormal);
    vec3 bitangent = cross(geoNormal, tangent) * vWorldTangent.w;
    vec3 shadingNormal = normalize(mat3(tangent, bitangent, geoNormal) * combinedTs);

    vec3 baseColor = texture(uBaseColor, vUv).rgb * uBaseColorFactor;
    // 0.045 floor avoids a near-zero alpha driving the specular exponent
    // to infinity (common UE4/Frostbite minimum-roughness clamp).
    float roughness = clamp(texture(uRoughness, vUv).r * uRoughnessFactor, 0.045, 1.0);
    float metallic = uMetallicFactor;

    vec3 color;
    if (uAov == 1) {
        // Alpha: every fragment invocation is a hit in this opaque-only
        // rasterizer, and the background clear color is already
        // (0,0,0) -- a flat white silhouette is already a correct
        // coverage mask, no separate alpha channel needed.
        color = vec3(1.0);
    } else if (uAov == 2) {
        // Depth: planar camera-space Z (Arnold/RenderMan/OpenEXR "Z"
        // convention -- distance along the view axis, not radial
        // distance to the point), linearized from the non-linear
        // gl_FragCoord.z via the standard perspective un-projection
        // (matches epochlab/KODAK's shaders/geometry/pbr.frag depth
        // mode). Written raw, in metres, with no display normalization
        // or clamp baked into the value -- a production Z AOV is
        // scene-referred data, not a pre-tonemapped [0,1] image; this
        // buffer is RGBA16F and holds it losslessly. (The window's own
        // backbuffer is still an 8-bit target, so anything >=1m will
        // read as saturated white on screen -- that's a display
        // limitation, not something baked into the AOV itself.)
        float ndc = (gl_FragCoord.z * 2.0) - 1.0;
        float linearDepth =
            (2.0 * uNearClip * uFarClip) / (uFarClip + uNearClip - (ndc * (uFarClip - uNearClip)));
        color = vec3(linearDepth);
    } else if (uAov == 3) {
        vec3 beauty = shadeBeauty(shadingNormal, vWorldPos, baseColor, roughness, metallic,
                                   texture(uSpecular, vUv).rgb, texture(uAo, vUv).r);
        vec3 hsv = rgb2hsv(beauty);
        color = vec3(hsv.x, hsv.y, clamp(hsv.z, 0.0, 1.0));
    } else if (uAov == 4 || uAov == 5 || uAov == 6) {
        // Luminance (4), and Sobel/Gabor's (5/6) first-pass input -- the
        // edge filters themselves run as a second pass over this
        // buffer, see edge_filter.frag.
        vec3 beauty = shadeBeauty(shadingNormal, vWorldPos, baseColor, roughness, metallic,
                                   texture(uSpecular, vUv).rgb, texture(uAo, vUv).r);
        color = vec3(luminance(beauty));
    } else if (uAov == 7) {
        vec3 extent = max(uBoundsMax - uBoundsMin, vec3(1e-6));
        color = clamp((vWorldPos - uBoundsMin) / extent, 0.0, 1.0);
    } else if (uAov == 8) {
        color = vec3(fract(vUv), 0.0);
    } else if (uAov == 9) {
        color = shadingNormal * 0.5 + 0.5;
    } else if (uAov == 10) {
        color = geoNormal * 0.5 + 0.5;
    } else if (uAov == 11) {
        color = baseColor;
    } else if (uAov == 12) {
        color = vec3(metallic);
    } else if (uAov == 13) {
        color = vec3(roughness);
    } else if (uAov == 14) {
        color = normalize(vWorldTangent.xyz) * 0.5 + 0.5;
    } else if (uAov == 15) {
        color = uObjectIdColor;
    } else if (uAov == 16) {
        color = vec3(texture(uAo, vUv).r);
    } else {
        // Beauty (0).
        color = shadeBeauty(shadingNormal, vWorldPos, baseColor, roughness, metallic,
                             texture(uSpecular, vUv).rgb, texture(uAo, vUv).r);
    }

    if (uChannelView == 1) {
        color = vec3(color.r);
    } else if (uChannelView == 2) {
        color = vec3(color.g);
    } else if (uChannelView == 3) {
        color = vec3(color.b);
    }

    fragColor = vec4(color, 1.0);
}
