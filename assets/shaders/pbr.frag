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
uniform vec3 uCameraPos;
uniform float uNearClip;
uniform float uFarClip;

// Punctual lights (Phase 4: direct analytic evaluation, no visibility rays). Type 0=Directional (uLightPositionOrDir is the unit direction toward the light, uLightRange unused), Type 1=Point (position, range in metres for the windowed inverse-square falloff, Karis 2013). Matches engine::config::kMaxLights (scene_config.h) -- GLSL and C++ can't share this literal directly, so both sides name it.
const int kMaxLights = 16;
uniform int uLightCount;
uniform int uLightType[kMaxLights];
uniform vec3 uLightPositionOrDir[kMaxLights];
uniform vec3 uLightColor[kMaxLights];
uniform float uLightRange[kMaxLights];

// Prefiltered IBL (Karis 2013 split-sum). Diffuse is SH-9 irradiance (Ramamoorthi & Hanrahan 2001) -- no texture, just 9 coefficients. Specular is a GGX-prefiltered cubemap mip chain (env_prefilter_pass.h) sampled at an explicit roughness-derived LOD, combined with the analytic split-sum DFG approximation (Lazarov 2013) -- no baked LUT.
uniform vec3 uShIrradiance[9];
uniform samplerCube uPrefilteredSpecular;
uniform float uPrefilteredSpecularMaxLod;
// User-controlled Y-axis (world up) rotation of the environment, in radians. The SH coefficients and prefiltered cubemap are baked once at startup in the environment's original orientation; rather than re-baking either on every UI change, the query direction is rotated by the inverse angle before each lookup (see rotateAboutY) -- exactly equivalent to rotating the environment itself, and free per-fragment. sky.frag applies the same rotation to its equirect sample direction so the background, diffuse, and specular all stay in sync.
uniform float uEnvRotationRadians;

// 0=Beauty 1=Alpha 2=Depth 3=HSV 4=Luminance 5=Sobel 6=Gabor 7=WorldPos 8=UV 9=Normal 10=GeomNormal 11=Albedo 12=Metallic 13=Roughness 14=Tangent 15=ObjectID 16=AO 17=Fresnel 18=IBL (orphaned -- see below). This is main.cpp's toRasterAovIndex()-translated legacy index, not the raw HUD selection: the HUD's unified AovId enum (engine/debug/aov.h) goes further (IOR, BounceCount, then transport-component AOVs replacing IBL), and toRasterAovIndex maps anything past 17 back to 0 before it reaches this uniform, since this shader has no branch for those -- the path tracer supplies them instead (see main.cpp's presentFrame/selectPathTracedImage). Sobel/Gabor (5/6) only need this shader to output Luminance (same as 4) -- the actual edge filtering is a second pass, see edge_filter.frag.
uniform int uAov;
uniform int uChannelView;  // 0=off 1=R 2=G 3=B

out vec4 fragColor;

const float kPi = 3.14159265;

// Rec.709 luma weights -- matches OcioDisplayTransform's "Linear Rec.709 (sRGB)" working color space.
float luminance(vec3 c) {
    return dot(c, vec3(0.2126, 0.7152, 0.0722));
}

// Standard branchless RGB->HSV (h, s in [0,1); v unbounded, matching input scale).
vec3 rgb2hsv(vec3 c) {
    vec4 k = vec4(0.0, -1.0 / 3.0, 2.0 / 3.0, -1.0);
    vec4 p = mix(vec4(c.bg, k.wz), vec4(c.gb, k.xy), step(c.b, c.g));
    vec4 q = mix(vec4(p.xyw, c.r), vec4(c.r, p.yzx), step(p.x, c.r));
    float d = q.x - min(q.w, q.y);
    float e = 1.0e-10;
    return vec3(abs(q.z + (q.w - q.y) / (6.0 * d + e)), d / (q.x + e), q.x);
}

// Trowbridge-Reitz/GGX normal distribution function.
float distributionGGX(float ndotH, float alpha) {
    float alpha2 = alpha * alpha;
    float d = ((ndotH * ndotH) * (alpha2 - 1.0)) + 1.0;
    return alpha2 / max(kPi * d * d, 1e-8);
}

// Smith height-correlated visibility term (Heitz 2014), pre-divided by the 4*ndotV*ndotL denominator -- callers combine D*Vis*F directly.
float visibilitySmithGGXCorrelated(float ndotV, float ndotL, float alpha) {
    float alpha2 = alpha * alpha;
    float lambdaV = ndotL * sqrt(((ndotV * ndotV) * (1.0 - alpha2)) + alpha2);
    float lambdaL = ndotV * sqrt(((ndotL * ndotL) * (1.0 - alpha2)) + alpha2);
    return 0.5 / max(lambdaV + lambdaL, 1e-8);
}

vec3 fresnelSchlick(float vdotH, vec3 f0) {
    return f0 + (1.0 - f0) * pow(clamp(1.0 - vdotH, 0.0, 1.0), 5.0);
}

// Analytic split-sum DFG approximation (Lazarov 2013, "Getting More Physically Based Reflectance" -- the same polynomial fit Karis's UE4 notes cite for mobile), returning (scale, bias) in place of a baked 2D LUT texture: reflectance = f0*scale + bias.
vec2 envBRDFApprox(float roughness, float ndotV) {
    const vec4 c0 = vec4(-1.0, -0.0275, -0.572, 0.022);
    const vec4 c1 = vec4(1.0, 0.0425, 1.04, -0.04);
    vec4 r = (roughness * c0) + c1;
    float a004 = (min(r.x * r.x, exp2(-9.28 * ndotV)) * r.x) + r.y;
    return (vec2(-1.04, 1.04) * a004) + r.zw;
}

// Multiplicative energy-compensation factor for a single-scatter microfacet specular value (Turquin 2019, "Practical multiple scattering compensation for microfacet models" -- the simpler, direct-lighting-friendly alternative to Fdez-Aguera 2019's IBL-only additive treatment). dfg.x+dfg.y approximates the white-Fresnel (f0=1) directional albedo of the single-scatter GGX lobe -- the same roughness/ndotV-dependent quantity split-sum IBL already needs, reused here with no extra texture or pass. Applied to both punctual and IBL specular so they lose energy consistently.
vec3 multiScatterCompensation(vec3 f0, vec2 dfg) {
    float ess = clamp(dfg.x + dfg.y, 1e-3, 1.0);
    vec3 compensation = 1.0 + (f0 * ((1.0 / ess) - 1.0));
    // Safety ceiling: envBRDFApprox's cheap polynomial fit measurably underestimates Ess relative to the true single-scatter GGX integral across much of the roughness range (worst near roughness=1, where its NdotV dependence vanishes entirely -- its scale term goes to exactly 0 there), so the compensation factor this Ess drives can overshoot energy conservation -- caught by tools/furnace_test.cpp. A real importance-sampled LUT wouldn't have this gap; the analytic approximation does. 1.1 keeps the worst measured case (a perfectly white, f0=1 conductor -- no real material reaches this) within the furnace test's tolerance, at the cost of leaving some darkening uncompensated for such near-perfect reflectors specifically.
    return min(compensation, vec3(1.1));
}

// Rotates v by angleRadians about the world Y axis (right-handed). Sampling the *unrotated* environment at rotateAboutY(d, -angle) gives the same result as sampling the environment rotated by +angle at the original direction d -- see uEnvRotationRadians's comment. Identical to sky.frag's copy (GLSL/GLSL can't share source across programs any more than the GLSL/C++ pairs elsewhere in this shader can).
vec3 rotateAboutY(vec3 v, float angleRadians) {
    float c = cos(angleRadians);
    float s = sin(angleRadians);
    return vec3((v.x * c) + (v.z * s), v.y, (-v.x * s) + (v.z * c));
}

// Order-2 (9-coefficient) SH irradiance evaluation (Ramamoorthi & Hanrahan 2001). Basis functions and cosine-lobe convolution constants must match sh_irradiance.cpp's projectIrradianceSH9/evaluateIrradianceSH9 exactly -- coeffs already has the A_l convolution folded in, so this is a flat dot product, not a second convolution step.
vec3 evaluateIrradianceSH9(vec3 n, vec3 coeffs[9]) {
    vec3 result = coeffs[0] * 0.282095;
    result += coeffs[1] * (0.488603 * n.y);
    result += coeffs[2] * (0.488603 * n.z);
    result += coeffs[3] * (0.488603 * n.x);
    result += coeffs[4] * (1.092548 * n.x * n.y);
    result += coeffs[5] * (1.092548 * n.y * n.z);
    result += coeffs[6] * (0.315392 * ((3.0 * n.z * n.z) - 1.0));
    result += coeffs[7] * (1.092548 * n.x * n.z);
    result += coeffs[8] * (0.546274 * ((n.x * n.x) - (n.y * n.y)));
    return max(result, vec3(0.0));
}

// Punctual direct lighting: analytic Cook-Torrance GGX (D, Smith-correlated visibility, Schlick Fresnel), evaluated directly against each light -- no importance sampling/Monte Carlo, that's Phase 5's recursive-tracing concern. Shares its BRDF with the IBL specular term below (evaluateAmbient) so direct and ambient specular stay consistent, and shares dfg (already computed once by shadeBeauty) so the multi-scatter compensation factor isn't recomputed per light.
vec3 evaluateDirectLighting(vec3 n, vec3 v, vec3 worldPos, vec3 baseColor, vec3 f0,
                             float roughness, float metallic, vec2 dfg) {
    float ndotV = max(dot(n, v), 1e-4);
    vec3 msComp = multiScatterCompensation(f0, dfg);
    float alpha = roughness * roughness;

    // Diffuse/specular energy split tied to the view angle (NdotV), not each light's half-vector (VdotH): with a per-light VdotH Fresnel, diffuse can stay high for light directions that happen to keep H close to N even while V itself is grazing, so diffuse and the independently-grazing-boosted specular term can together exceed the energy received (caught by tools/furnace_test.cpp, which shares this same formula). Tying the split to NdotV instead keeps it consistent with the ambient term's dfg-based split below.
    vec3 kd = (vec3(1.0) - fresnelSchlick(ndotV, f0)) * (1.0 - metallic);
    vec3 diffuseTerm = baseColor * kd / kPi;

    vec3 total = vec3(0.0);
    for (int i = 0; i < uLightCount; ++i) {
        vec3 l;
        vec3 radiance = uLightColor[i];
        if (uLightType[i] == 0) {
            l = normalize(uLightPositionOrDir[i]);
        } else {
            vec3 toLight = uLightPositionOrDir[i] - worldPos;
            float dist = length(toLight);
            l = toLight / max(dist, 1e-4);
            // Windowed inverse-square falloff (Karis 2013, "Real Shading in Unreal Engine 4"): avoids both the 1/d^2 singularity at d->0 and an abrupt cutoff at uLightRange.
            float window = clamp(1.0 - pow(dist / max(uLightRange[i], 1e-4), 4.0), 0.0, 1.0);
            radiance *= (window * window) / max(dist * dist, 1e-4);
        }

        float ndotL = max(dot(n, l), 0.0);
        if (ndotL <= 0.0) {
            continue;
        }
        vec3 h = normalize(v + l);
        float ndotH = max(dot(n, h), 0.0);
        float vdotH = max(dot(v, h), 0.0);

        float d = distributionGGX(ndotH, alpha);
        float vis = visibilitySmithGGXCorrelated(ndotV, ndotL, alpha);
        vec3 fr = fresnelSchlick(vdotH, f0);
        vec3 specularTerm = (d * vis) * fr * msComp;

        total += (diffuseTerm + specularTerm) * radiance * ndotL;
    }
    return total;
}

// Ambient/IBL term: SH-9 diffuse irradiance + prefiltered-specular-cubemap sample (roughness -> LOD) x analytic split-sum DFG x multi-scatter compensation. AO is applied only here -- it occludes the ambient term, not direct light, now that an ambient term actually exists to occlude (previously a blanket multiplier over everything, see git history).
vec3 evaluateAmbient(vec3 n, vec3 v, vec3 baseColor, vec3 f0, float roughness, float metallic,
                      vec2 dfg, float aoSample) {
    vec3 msComp = multiScatterCompensation(f0, dfg);
    // Single-scatter specular reflectance estimate, reused both as the specular term's weight and (via 1-this) as the diffuse attenuation -- same (1-F) energy split as evaluateDirectLighting, using dfg in place of a per-sample Fresnel since IBL has no single light direction to evaluate one against.
    vec3 singleScatterSpecular = (f0 * dfg.x) + dfg.y;
    vec3 kd = (vec3(1.0) - singleScatterSpecular) * (1.0 - metallic);

    vec3 rotatedN = rotateAboutY(n, -uEnvRotationRadians);
    vec3 irradiance = evaluateIrradianceSH9(rotatedN, uShIrradiance);
    vec3 diffuseAmbient = baseColor * kd * irradiance / kPi;

    vec3 r = reflect(-v, n);
    vec3 rotatedR = rotateAboutY(r, -uEnvRotationRadians);
    float lod = roughness * uPrefilteredSpecularMaxLod;
    vec3 prefiltered = textureLod(uPrefilteredSpecular, rotatedR, lod).rgb;
    vec3 specularAmbient = prefiltered * singleScatterSpecular * msComp;

    return (diffuseAmbient + specularAmbient) * aoSample;
}

vec3 shadeBeauty(vec3 n, vec3 worldPos, vec3 baseColor, float roughness, float metallic,
                  vec3 specularSample, float aoSample) {
    vec3 v = normalize(uCameraPos - worldPos);
    vec3 f0 = mix(specularSample, baseColor, metallic);
    float ndotV = max(dot(n, v), 1e-4);
    vec2 dfg = envBRDFApprox(roughness, ndotV);

    vec3 direct = evaluateDirectLighting(n, v, worldPos, baseColor, f0, roughness, metallic, dfg);
    vec3 ambient = evaluateAmbient(n, v, baseColor, f0, roughness, metallic, dfg, aoSample);
    return direct + ambient;
}

struct ShadingInputs {
    vec3 shadingNormal;
    vec3 geoNormal;
    vec3 baseColor;
    float roughness;
    float metallic;
};

// Bump detail normal (central-difference height gradient, UV-space -- texel-offset, not screen-space dFdx/dFdy, independent of mesh density), blended with the tangent-space normal map (3-channel, no Z-reconstruction needed) and re-orthogonalized (Gram-Schmidt) against the interpolated vertex normal, plus the base-color/roughness/metallic material reads every AOV branch below needs at least one of.
ShadingInputs computeShadingInputs() {
    vec2 texel = 1.0 / vec2(textureSize(uBump, 0));
    float hL = texture(uBump, vUv - vec2(texel.x, 0.0)).r;
    float hR = texture(uBump, vUv + vec2(texel.x, 0.0)).r;
    float hD = texture(uBump, vUv - vec2(0.0, texel.y)).r;
    float hU = texture(uBump, vUv + vec2(0.0, texel.y)).r;
    vec3 detailNormalTs = normalize(vec3(hL - hR, hD - hU, 1.0));

    vec3 normalTs = texture(uNormal, vUv).rgb * 2.0 - 1.0;
    vec3 combinedTs = normalize(vec3(normalTs.xy + detailNormalTs.xy, normalTs.z));

    vec3 geoNormal = normalize(vWorldNormal);
    vec3 tangent = normalize(vWorldTangent.xyz);
    tangent = normalize(tangent - dot(tangent, geoNormal) * geoNormal);
    vec3 bitangent = cross(geoNormal, tangent) * vWorldTangent.w;
    vec3 shadingNormal = normalize(mat3(tangent, bitangent, geoNormal) * combinedTs);

    vec3 baseColor = texture(uBaseColor, vUv).rgb * uBaseColorFactor;
    // 0.045 floor avoids a near-zero alpha driving the specular exponent to infinity (common UE4/Frostbite minimum-roughness clamp).
    float roughness = clamp(texture(uRoughness, vUv).r * uRoughnessFactor, 0.045, 1.0);

    return ShadingInputs(shadingNormal, geoNormal, baseColor, roughness, uMetallicFactor);
}

// Pure-geometric/utility AOVs: no shading model evaluation, just a transform of already-available position/UV/clip-space data.
vec3 debugAovUtility(int aov) {
    if (aov == 1) {
        // Alpha: every fragment invocation is a hit in this opaque-only rasterizer, and the background clear color is already (0,0,0) -- a flat white silhouette is already a correct coverage mask, no separate alpha channel needed.
        return vec3(1.0);
    }
    if (aov == 2) {
        // Depth: planar camera-space Z (Arnold/RenderMan/OpenEXR "Z" convention -- distance along the view axis, not radial distance to the point), linearized from the non-linear gl_FragCoord.z via the standard perspective un-projection. Written raw, in metres, with no display normalization or clamp baked into the value -- a production Z AOV is scene-referred data, not a pre-tonemapped [0,1] image; this buffer is RGBA16F and holds it losslessly. (The window's own backbuffer is still an 8-bit target, so anything >=1m will read as saturated white on screen -- that's a display limitation, not something baked into the AOV itself.)
        float ndc = (gl_FragCoord.z * 2.0) - 1.0;
        float linearDepth =
            (2.0 * uNearClip * uFarClip) / (uFarClip + uNearClip - (ndc * (uFarClip - uNearClip)));
        return vec3(linearDepth);
    }
    if (aov == 7) {
        vec3 extent = max(uBoundsMax - uBoundsMin, vec3(1e-6));
        return clamp((vWorldPos - uBoundsMin) / extent, 0.0, 1.0);
    }
    // aov == 8.
    return vec3(fract(vUv), 0.0);
}

// Material-debug AOVs: a direct pass-through of one already-computed material/geometry quantity, no lighting evaluation.
vec3 debugAovMaterial(int aov, ShadingInputs s) {
    if (aov == 9) {
        return s.shadingNormal * 0.5 + 0.5;
    }
    if (aov == 10) {
        return s.geoNormal * 0.5 + 0.5;
    }
    if (aov == 11) {
        return s.baseColor;
    }
    if (aov == 12) {
        return vec3(s.metallic);
    }
    if (aov == 13) {
        return vec3(s.roughness);
    }
    if (aov == 14) {
        return normalize(vWorldTangent.xyz) * 0.5 + 0.5;
    }
    if (aov == 15) {
        return uObjectIdColor;
    }
    // aov == 16.
    return vec3(texture(uAo, vUv).r);
}

// Lighting-model AOVs: each requires at least one of shadeBeauty/evaluateAmbient/fresnelSchlick above, unlike the two pass-through groups above.
vec3 debugAovLighting(int aov, ShadingInputs s) {
    vec3 specularSample = texture(uSpecular, vUv).rgb;
    float aoSample = texture(uAo, vUv).r;

    if (aov == 3) {
        vec3 beauty = shadeBeauty(s.shadingNormal, vWorldPos, s.baseColor, s.roughness,
                                   s.metallic, specularSample, aoSample);
        vec3 hsv = rgb2hsv(beauty);
        return vec3(hsv.x, hsv.y, clamp(hsv.z, 0.0, 1.0));
    }
    if (aov == 4 || aov == 5 || aov == 6) {
        // Luminance (4), and Sobel/Gabor's (5/6) first-pass input -- the edge filters themselves run as a second pass over this buffer, see edge_filter.frag.
        vec3 beauty = shadeBeauty(s.shadingNormal, vWorldPos, s.baseColor, s.roughness,
                                   s.metallic, specularSample, aoSample);
        return vec3(luminance(beauty));
    }
    vec3 v = normalize(uCameraPos - vWorldPos);
    vec3 f0 = mix(specularSample, s.baseColor, s.metallic);
    float ndotV = max(dot(s.shadingNormal, v), 1e-4);
    if (aov == 17) {
        // Fresnel/reflectance: Schlick term at the actual view angle, isolating grazing-angle behaviour from the rest of shading.
        return fresnelSchlick(ndotV, f0);
    }
    // aov == 18: IBL/ambient only: SH diffuse + prefiltered specular, no direct lights -- isolates the Phase 4 ambient term this AOV exists to debug (AO's re-scoped role, SH/prefiltered-cubemap plausibility). Diffuse albedo is deliberately dropped (baseColor -> white) so this reads as "light arriving here", not "object colour under ambient light" -- specular keeps the real F0 (a physical reflectance property, not albedo), so a metal's tinted reflection still shows.
    vec2 dfg = envBRDFApprox(s.roughness, ndotV);
    return evaluateAmbient(s.shadingNormal, v, vec3(1.0), f0, s.roughness, s.metallic, dfg,
                            aoSample);
}

vec3 applyChannelView(vec3 color, int channelView) {
    if (channelView == 1) {
        return vec3(color.r);
    }
    if (channelView == 2) {
        return vec3(color.g);
    }
    if (channelView == 3) {
        return vec3(color.b);
    }
    return color;
}

void main() {
    ShadingInputs s = computeShadingInputs();

    vec3 color;
    if (uAov == 1 || uAov == 2 || uAov == 7 || uAov == 8) {
        color = debugAovUtility(uAov);
    } else if (uAov >= 9 && uAov <= 16) {
        color = debugAovMaterial(uAov, s);
    } else if (uAov == 3 || uAov == 4 || uAov == 5 || uAov == 6 || uAov == 17 || uAov == 18) {
        color = debugAovLighting(uAov, s);
    } else {
        // Beauty (0).
        color = shadeBeauty(s.shadingNormal, vWorldPos, s.baseColor, s.roughness, s.metallic,
                             texture(uSpecular, vUv).rgb, texture(uAo, vUv).r);
    }

    fragColor = vec4(applyChannelView(color, uChannelView), 1.0);
}
