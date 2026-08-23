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
uniform float uMetallicFactor;
uniform float uRoughnessFactor;
uniform vec3 uBoundsMin;  // world-space instance AABB, for the World position AOV
uniform vec3 uBoundsMax;
uniform vec3 uObjectIdColor;
uniform vec3 uLightDir;    // world-space, points toward the light
uniform vec3 uLightColor;
uniform vec3 uCameraPos;

// 0=Beauty 1=Albedo 2=Normal 3=GeomNormal 4=Roughness 5=UV 6=WorldPos
// 7=Tangent 8=Metallic 9=ObjectID 10=AO -- order must match main.cpp's
// AOV combo list.
uniform int uAov;
uniform int uChannelView;  // 0=off 1=R 2=G 3=B

out vec4 fragColor;

const float kPi = 3.14159265;

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

    vec3 baseColor = texture(uBaseColor, vUv).rgb;
    // 0.045 floor avoids a near-zero alpha driving the specular exponent
    // to infinity (common UE4/Frostbite minimum-roughness clamp).
    float roughness = clamp(texture(uRoughness, vUv).r * uRoughnessFactor, 0.045, 1.0);
    float metallic = uMetallicFactor;

    vec3 color;
    if (uAov == 1) {
        color = baseColor;
    } else if (uAov == 2) {
        color = shadingNormal * 0.5 + 0.5;
    } else if (uAov == 3) {
        color = geoNormal * 0.5 + 0.5;
    } else if (uAov == 4) {
        color = vec3(roughness);
    } else if (uAov == 5) {
        color = vec3(fract(vUv), 0.0);
    } else if (uAov == 6) {
        vec3 extent = max(uBoundsMax - uBoundsMin, vec3(1e-6));
        color = clamp((vWorldPos - uBoundsMin) / extent, 0.0, 1.0);
    } else if (uAov == 7) {
        color = normalize(vWorldTangent.xyz) * 0.5 + 0.5;
    } else if (uAov == 8) {
        color = vec3(metallic);
    } else if (uAov == 9) {
        color = uObjectIdColor;
    } else if (uAov == 10) {
        color = vec3(texture(uAo, vUv).r);
    } else {
        // Beauty (0): Lambertian diffuse + Schlick-Fresnel specular
        // against one fixed test light. No microfacet D/G terms
        // (importance sampling is Phase 5), no punctual-light system
        // (Phase 4). AO is multiplied in as a blanket occlusion term --
        // not physically exact with no ambient/indirect term to occlude,
        // but a common approximation for grounding contact shadows.
        vec3 n = shadingNormal;
        vec3 v = normalize(uCameraPos - vWorldPos);
        vec3 l = normalize(uLightDir);
        vec3 h = normalize(v + l);
        float ndotL = max(dot(n, l), 0.0);
        float ndotH = max(dot(n, h), 0.0);
        float vdotH = max(dot(v, h), 0.0);

        vec3 specularSample = texture(uSpecular, vUv).rgb;
        vec3 f0 = mix(specularSample, baseColor, metallic);
        vec3 fresnel = f0 + (1.0 - f0) * pow(1.0 - vdotH, 5.0);

        // Roughness -> Blinn-Phong exponent, standard alpha^2
        // equivalence (Real-Time Rendering 4th ed. eq. 9.35).
        float alpha = roughness * roughness;
        float shininess = 2.0 / (alpha * alpha) - 2.0;

        vec3 diffuse = baseColor * (1.0 - metallic) / kPi;
        // max(ndotH, 1e-4): pow(0, shininess) is undefined when
        // shininess<=0 (roughness at its 1.0 ceiling), which some
        // drivers evaluate to NaN.
        vec3 specular = fresnel * pow(max(ndotH, 1e-4), shininess);
        color = (diffuse + specular) * uLightColor * ndotL * texture(uAo, vUv).r;
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
