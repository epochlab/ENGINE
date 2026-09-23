# Material library

[← README](../README.md)


Named presets live in `assets/materials/*.json`, parsed into `MaterialConfig` (`scene_config.h`). Each scene picks one as its default via `SceneConfig::materialPath`, with optional per-object overrides via `SceneConfig::materialOverrides` (glTF node name → material JSON path) — e.g. `assets/scenes/cornell.json`'s `{"sphere01": "materials/chrome.json", "sphere02": "materials/glass.json"}`, everything else left on the scene default. Overrides build a per-instance settings vector, indexed by `ShadingTriangle::instanceIndex` through both render paths ([Per-object materials](components.md#materials--lighting)).

Add a new material by dropping a JSON file in `assets/materials/` and pointing `materialPath`/`materialOverrides` at it — no code or schema change needed.

## Shipped presets

| File | metallic | transmission | roughness (factor / min) | Notes |
|---|---|---|---|---|
| `principled.json` | 0.0 | 0.0 | 1.0 / 0.045 | Rough dielectric, heavy bump (`bumpStrength: 10.0`). The reference preset: the only one declaring every field, including the otherwise-optional `transmissionColor`/`transmissionDepth`/`edgeTint`. No shipped scene uses it |
| `clay.json` | 0.0 | 0.0 | 0.5 / 0.045 | Neutral matte dielectric, no bump |
| `chrome.json` | 1.0 | 0.0 | 0.05 / 0.045 | Measured chromium (Johnson & Christy 1974): `diffuseColour` (which at `metallic=1` *is* `f0`, Gulbrandsen's reflectivity `r`) `[0.5496, 0.5560, 0.5542]` and `edgeTint` `[0.5417, 0.5692, 0.6942]`, both verbatim `tools/metal_fit` output (CIE 1931 2°, D65, linear Rec.709), so the grazing reflectance dip is the measured one. `colour.chrome_matches_measured_chromium` fails if this file drifts from the fit |
| `glass.json` | 0.0 | 1.0 | 0.02 / 0.01 | Schott N-BK7 crown glass, `ior: 1.5168` at the d line with `abbe: 64.17` for dispersion; tinted via Beer-Lambert `transmissionColor: [0.96, 0.98, 1.0]` over `transmissionDepth: 0.4` world units — `diffuseColour` cannot tint transmission at all (see below) |

## `MaterialConfig` fields

| Field | Meaning |
|---|---|
| `diffuseColour` | Multiplies `baseColorTexture`; also the conductor lobe's `f0` tint. A **reflection** quantity throughout — it never tints transmitted light, which `transmissionColor` alone does. On the diffuse lobe it's the **observed** albedo (OpenPBR's reading of `base_color`), not EON's ρ; the two differ once `diffuseRoughness > 0`, and `eonAlbedoInversion` (`bsdf.cpp`) maps one to the other |
| `metallicFactor` / `transmissionFactor` | Lobe selection — a material is dielectric, conductor, or transmissive, not blended between (every shipped file uses 0.0/1.0) |
| `roughnessFactor` | Multiplies the roughness texture sample, before the `roughnessMin`/`roughnessMax` clamp |
| `roughnessMin` / `roughnessMax` | Per-material clamp on the roughness sample; a material can floor below the shared 0.045 (e.g. glass's 0.01) for a genuinely smooth GGX lobe |
| `ior` | Dielectric IOR, non-metal lobes only |
| `abbe` | Abbe number, dispersion strength for the dielectric/transmissive lobes; 0 = no dispersion |
| `diffuseRoughness` | EON rough-diffuse parameter r ∈ [0,1] (Portsmouth, Kutz, Hill 2025, revised 2026-02-04); 0 = Lambertian, and the roughness at which `diffuseColour` and EON's ρ coincide |
| `bumpStrength` | Scales the bump texture's per-texel height difference |
| `transmissionColor` / `transmissionDepth` | The **only** tint on transmitted light (Arnold `standard_surface`/OpenPBR convention). `transmissionDepth > 0`: interior-medium Beer-Lambert absorption, `sigmaA = -log(transmissionColor)/transmissionDepth`, applied while a path is inside a `transmissionFactor>0` material. `transmissionDepth 0`: no interior medium — a constant on-surface tint applied once per crossing, so a closed solid reads its square. Default `[1,1,1]`/`0.0` is Arnold/OpenPBR's own no-op default in either regime ([Volumetric absorption](components.md#materials--lighting)) |
| `edgeTint` | Gulbrandsen 2014 edge tint for the conductor lobe, `metallicFactor>0` only. Default `[1,1,1]`: white is the no-dip edge Schlick always produced |
