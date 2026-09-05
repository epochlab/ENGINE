# Physically based path tracer

*A CPU, unidirectional brute-force Monte Carlo path tracer with real-time progressive display: Embree-accelerated, stochastic BSDF sampling combined with environment-map NEE via MIS, behind a thin OpenGL display/HUD layer.*

![Sample render](sample.png)

## Contents

- [Build](#build)
- [1. Pipeline](#1-pipeline)
- [2. Component reference](#2-component-reference)
- [3. Material library](#3-material-library)
- [4. AOV reference](#4-aov-reference)
- [5. Roadmap](#5-roadmap)
- [6. References](#6-references)

## Build

C++20, built with CMake. Currently developed against macOS only.

### Prerequisites

```
brew install cmake glfw glew glm imath openexr opencolorio embree
```

Homebrew's `embree` formula pulls in `tbb` automatically (Embree's parallelism dependency) — no separate install step needed.

Dear ImGui is vendored as a git submodule (`third_party/imgui`); initialise it before configuring:

```
git submodule update --init --recursive
```

### Configure & build

```
cmake -B build
cmake --build build
```

This produces nine targets:

- `build/engine`: the path tracer
- `build/test_pattern`: EXR calibration-pattern generator (`tools/test_pattern.cpp`)
- `build/downsample`: EXR downsampling tool (`tools/downsample.cpp`)
- `build/embree_validate`: headless Embree ray-scene intersection correctness check (`tools/embree_validate.cpp`)
- `build/bsdf_validate`: headless BSDF pdf-normalization and furnace-test check (`tools/bsdf_validate.cpp`)
- `build/nee_validate`: headless NEE/MIS unbiasedness check against a brute-force reference (`tools/nee_validate.cpp`)
- `build/rasterizer_validate`: headless CPU-rasterizer-vs-Embree G-buffer correctness check (`tools/rasterizer_validate.cpp`)
- `build/integrator_validate`: headless full-integrator depth-invariance and transport-partition correctness check (`tools/integrator_validate.cpp`)
- `build/raster_bench`: rasterizer timing harness, not run under `ctest` (`tools/raster_bench.cpp`)

`-Wall -Wextra -Werror` gates every target. `clang-tidy` (see `.clang-tidy`) and `cppcheck` (`cmake --build build --target cppcheck`, over `src/`) both run if installed, skipped otherwise.

`cmake -B build -DENGINE_SANITIZE=ON` builds with AddressSanitizer + UndefinedBehaviorSanitizer instead (off by default); worth running after touching the glTF/JSON/EXR loading paths.

### Run

```
./build/engine [--scene path/to/scene.json]
```

Defaults to `assets/scenes/cornell.json` if `--scene` is omitted — currently the only scene shipped; the tree scene's geometry, textures and `tree.json` have been pulled from the repo for the time being.

## 1. Pipeline

glTF geometry/materials load once at startup into a CPU-resident scene: per-vertex shading data (`ShadingTriangle`) and world-space triangles feed an Embree scene (`EmbreeAccel`); materials keep only CPU `HdrImage` textures, sampled per-ray. An equirectangular HDR environment map loads alongside it, with its own luminance-importance-sampling CDF for NEE. Area lights (`scene.json`'s `lights`) inject emitting geometry into the same Embree scene at load (`appendQuadLights`), so they occlude and are camera/BSDF-hittable with no second intersection path.

Every frame, on any camera/scene-state change, `PathTraceDriver` hands a fresh request to a background thread pool (one worker/core, row-parallel, dynamic scheduling), restarting progressive accumulation:

- Camera ray generation (pinhole) → Embree intersection (`rtcIntersect1`/`rtcOccluded1`)
- BSDF eval/sampling (Heitz 2018 GGX VNDF specular; EON rough-diffuse, Portsmouth/Kutz/Hill 2025; Walter 2007 rough dielectric transmission with exact Fresnel/TIR, falling back to a Snell delta lobe below the smooth-roughness threshold; Kulla-Conty multiple-scattering compensation on both interfaces; exact complex-IOR conductor Fresnel via Gulbrandsen 2014's reflectivity/edge-tint parameterisation) → NEE against a `LightSet` (environment map, optionally excluded via the HUD's "Environment Light" checkbox, plus any area lights — uniform selection, quads importance-sampled by solid angle via Ureña/Fajardo/King 2013's spherical-rectangle parametrisation), MIS-combined with BSDF sampling (power heuristic, Veach 1997)
- Recursive bounce loop with Russian roulette, Chiang/Li/Burley 2019 shadow-terminator-corrected secondary-ray origins, Beer-Lambert extinction inside transmissive media
- Radiance + full G-buffer/transport-component AOV set accumulated per pass, published lock-free for the render thread

Independently, on the same trigger, a synchronous CPU rasterizer (`rasterizer.cpp`) computes the 15 primary-hit-only AOVs (§4) every frame on the render thread: edge-function rasterization (Pineda 1988) with near-plane Sutherland-Hodgman clipping, sharing `gbuffer_shading.h`'s material sampling with the path tracer but no Embree/BSDF/recursion. This gives those AOVs instant, glitch-free feedback during camera movement, decoupled from Beauty's own progressive convergence; the path-traced request above only restarts when the selected AOV needs light-transport data.

The render thread blits whichever AOV is selected through OCIO's display transform (exposure/tone-mapping) and the debug HUD, converging over subsequent passes rather than blocking on one long render. No GPU rasterization anywhere: OpenGL exists only for the window, the post-process/OCIO blit, and ImGui; the primary-hit rasterizer is CPU-only.

## 2. Component reference

| Feature | Mechanism | Role / why it matters |
|---|---|---|
| Camera / lens | Position/yaw/pitch, film-back + focal length → derived vertical FOV; pinhole primary rays derived directly from this basis | Geometric ground truth the ray-intersection/BSDF math is measured against |
| Photographic exposure | EV100 from aperture/shutter/ISO, applied as a relative-stops delta against profile.json's default triple (Filament/Frostbite EV100 formula) | Familiar photographic brightness control -- not an absolute photometric quantity, since the scene isn't calibrated to real-world radiance |
| OpenEXR linear pipeline | `HdrImage`/`loadExr`; all shading/compositing in linear light, OCIO display-encodes only at the final blit | Precondition for correct PBR colour math |
| Display transform | OCIO Display/View API (sRGB, Rec.709, Raw), cycled at runtime ('L') -- a colourimetric encode (sRGB / Rec.1886 OETF) only, no tone mapping | Scene-referred radiance throughout; values above 1.0 clip at the display by design -- lookdev sees clipping honestly, not masked by a hidden filmic shoulder |
| GPU/system readout | Device name, driver/API version, refresh rate, RAM at startup | Confirms the actual GPU/backend before a wrong-adapter bug masquerades as a render bug |
| Frame-timing HUD | Ring buffer of recent frame times; rolling FPS/avg/min/max, GPU timer query around the post-process blit | Makes blit cost measurable frame to frame |
| Memory HUD | Live RAM readout plus GPU allocation tracked at alloc/free (the path-traced display texture is the only GPU allocation left; OCIO uses zero LUT textures) | Surfaces a memory regression immediately, not after VRAM exhaustion |
| Scene stats | Object/triangle/point counts, viewport resolution | Scene-complexity readout |
| Debug camera controls | WASD/QE fly, R reset, LMB-drag orbit around a pivot read directly from the path tracer's own G-buffer (world-space hit position + hit mask at its centre pixel) | Interactive navigation without hand-editing camera parameters between runs |
| Camera framing overlays | Centre crosshair, always on, drawn on the foreground overlay over the viewport | Composition aid that never contaminates the AOV buffers being debugged |
| AOV selector | Dropdown across the full AOV set (§4), plus R/G/B channel-isolation hotkeys | Isolates one signal at a time for debugging |
| Live histogram | Per-channel (R/G/B) histogram of the currently displayed image | Catches exposure/clipping and colour-space bugs a single still frame can hide |
| glTF loading | cgltf; per-primitive vertices baked to world-space triangles/shading data at load time, materials' textures decoded once to `HdrImage` | Standard interchange format; nothing GPU-resident is needed once the CPU Embree scene/shading data exists |
| Tangent-space normal mapping | Per-vertex tangent (glTF-supplied only), Gram-Schmidt re-orthogonalized per-ray | Surface micro-detail without extra geometry |
| Ray acceleration | Intel Embree (SIMD BVH build/traversal), CPU, built once at load | Sub-linear ray-scene intersection, required before recursion is affordable |
| Primary-hit rasterizer | CPU edge-function rasterization (Pineda 1988) + near-plane clip (Sutherland-Hodgman 1974), row-parallel, synchronous every frame | Instant primary-hit G-buffer AOVs (§4), decoupled from Beauty's progressive convergence |
| Area lights | Rectangular emitters (`scene.json`'s `lights`), one-sided by default; own geometry in the BVH, solid-angle NEE sampling (Ureña, Fajardo & King 2013), MIS against BSDF sampling exactly like the environment | The classic emissive-panel Cornell box (§5 Large #2); ReSTIR's (§5) prerequisite light set |
| Environment-light toggle | HUD "Environment Light" checkbox (`environment.lightEnabled` in `scene.json`, `--env-light` on `render_beauty`) | Removes the environment from NEE/MIS/every miss entirely, background included -- distinct from "Show/Hide Background", which only hides the camera-visible sky. Lets an HDRI+area-light scene isolate the panel-only look |
| Stochastic BSDF | EON rough-diffuse (Portsmouth, Kutz, Hill 2025), GGX microfacet specular, Walter 2007 rough dielectric transmission (delta Snell + TIR below the smooth-roughness threshold), Kulla-Conty multiple-scattering compensation on the reflective and transmissive interface alike; four-lobe stochastic selection, one-sample mixture estimator | Materials respond to light with real physical behaviour, including rough and smooth glass, and conserve energy at every roughness |
| Volumetric absorption | Beer-Lambert extinction (`transmissionColor` over `transmissionDepth`, Arnold `standard_surface`/OpenPBR convention) applied to a single-level medium stack while a path is inside a `transmissionFactor>0` material; at `transmissionDepth 0` there is no medium and the same colour is a constant on-surface tint instead | Tinted/coloured glass, thick or thin, without participating-media in-scattering (§5 Large #1) |
| Per-object materials | `SceneConfig::materialOverrides` (glTF node name → `materials/*.json` path) builds a per-instance settings vector, indexed by `ShadingTriangle::instanceIndex` through both render paths | Different objects in one scene can carry different materials (e.g. a chrome sphere and a glass sphere in the same Cornell box) |
| Environment lighting | Equirect HDR map, BSDF-sampled misses + luminance-importance-sampled NEE, MIS-combined | Image-based lighting; one member of `LightSet` alongside any area lights (still no punctual/directional lights -- those have no hittable geometry) |
| Russian roulette | Survival probability clamped from running throughput from `russianRouletteStartBounce`, reweighted by `1/p` | Keeps recursion finite without biasing the estimator |
| Progressive accumulation | Each background pass re-traces at the current camera/settings and averages into the displayed result; any camera/scene change restarts accumulation | Real-time-interactive without waiting for a single long render to finish |

## 3. Material library

Named presets (`assets/materials/*.json`), parsed into `MaterialConfig` (`scene_config.h`) and assigned per-scene via `SceneConfig::materialPath` (the scene's default material) plus an optional per-object override, `SceneConfig::materialOverrides` (glTF node name → material JSON path) -- e.g. `assets/scenes/cornell.json`'s `{"sphere01": "materials/chrome.json", "sphere02": "materials/glass.json"}`, cornell box left on the scene default. Overrides build a per-instance settings vector indexed by `ShadingTriangle::instanceIndex` through both render paths (§2 Per-object materials).

| File | metallic | transmission | roughness (factor / min) | Role |
|---|---|---|---|---|
| `principled.json` | 0.0 | 0.0 | 1.0 / 0.045 | Was the tree scene's material (`tree.json`, its geometry/textures currently pulled from the repo); rough dielectric, heavy bump (`bumpStrength: 10.0`). The one file that declares every field, including the otherwise-optional `transmissionColor`/`transmissionDepth`/`edgeTint` |
| `clay.json` | 0.0 | 0.0 | 0.5 / 0.045 | Neutral matte dielectric, no bump |
| `chrome.json` | 1.0 | 0.0 | 0.05 / 0.045 | Idealised near-white mirror: `diffuseColour` (which at `metallic=1` *is* `f0`, Gulbrandsen's reflectivity `r`) is `[0.95, 0.95, 0.97]` and `edgeTint` is white, i.e. **no reflectance dip** — the one edge tint at which the conductor reproduces Schlick's grazing behaviour, so this preset exercises the transport rather than the parameterisation. A measured metal is what the `(r, edgeTint)` basis exists for: chromium's Johnson & Christy 1974 triples, their provenance and their error budget are recorded at `conductorIorFromReflectivity` (`bsdf.cpp`) and can be pasted straight back into this file |
| `glass.json` | 0.0 | 1.0 | 0.02 / 0.01 | Schott N-BK7 crown glass, `ior: 1.5168` at the d line with `abbe: 64.17` giving it dispersion; tinted via Beer-Lambert `transmissionColor: [0.96, 0.98, 1.0]` over `transmissionDepth: 0.4` world units -- `diffuseColour` cannot tint transmission at all, see below |

`MaterialConfig` fields:

| Field | Meaning |
|---|---|
| `diffuseColour` | Multiplies `baseColorTexture`; also the conductor lobe's `f0` tint. A **reflection** quantity throughout: it does not tint transmitted light, which `transmissionColor` alone does (below). On the diffuse lobe it is the **observed** albedo — the reflection colour seen at normal incidence under uniform illumination, OpenPBR's reading of `base_color` — not EON's ρ. The two differ once `diffuseRoughness > 0`; `eonAlbedoInversion` (`bsdf.cpp`) maps one to the other |
| `metallicFactor` / `transmissionFactor` | Lobe selection -- a material is dielectric, conductor, or transmissive, not blended between (every shipped file uses 0.0/1.0) |
| `roughnessFactor` | Multiplies the roughness texture sample, before the `roughnessMin`/`roughnessMax` clamp |
| `roughnessMin` / `roughnessMax` | Per-material clamp on the roughness sample; a material can floor below the shared 0.045 (e.g. glass's 0.01) for a genuinely smooth GGX lobe |
| `ior` | Dielectric IOR, non-metal lobes only |
| `diffuseRoughness` | EON rough-diffuse parameter r ∈ [0,1] (Portsmouth, Kutz, Hill 2025, revised 2026-02-04); 0 = Lambertian, and the roughness at which `diffuseColour` and EON's ρ coincide |
| `bumpStrength` | Scales the bump texture's per-texel height difference |
| `transmissionColor` / `transmissionDepth` | The **only** tint on transmitted light (Arnold `standard_surface`/OpenPBR convention), realised in one of two mutually exclusive regimes picked by the depth. `transmissionDepth > 0`: interior-medium Beer-Lambert absorption, `sigmaA = -log(transmissionColor)/transmissionDepth`, applied while a path is inside a `transmissionFactor>0` material. `transmissionDepth 0`: no interior medium -- a constant on-surface tint applied once per crossing, so a closed solid reads its square. Optional, default `[1,1,1]`/`0.0` -- Arnold/OpenPBR's own default, a true no-op in either regime (§2 Volumetric absorption) |
| `edgeTint` | Gulbrandsen 2014 edge tint for the conductor lobe, `metallicFactor>0` only. Optional, default `[1,1,1]`: white is the no-dip edge Schlick always produced |

`diffuseColour`/`roughnessFactor`/`roughnessMin`/`roughnessMax`/`bumpStrength` are required (`j.at`, missing/malformed fails the load and logs to stderr); `ior`/`abbe`/`metallicFactor`/`transmissionFactor`/`diffuseRoughness`/`transmissionColor`/`transmissionDepth`/`edgeTint` are parsed optional-with-default (`j.value`), each defaulting to the value that makes it a no-op in its owning lobe -- so a bespoke material only declares the fields its archetype actually uses (see `clay.json`/`glass.json`/`chrome.json` above vs. `principled.json`, which declares all thirteen). Add a new material by dropping a JSON file in `assets/materials/` and pointing `materialPath`/`materialOverrides` at it -- no code or schema change needed.

## 4. AOV reference

Every AOV below is computed by the path tracer each pass, except: the 15 primary-hit-only AOVs (Alpha, Depth, WorldPos, UV, Normal, GeomNormal, Albedo, Metallic, Roughness, Tangent, ObjectID, Fresnel, IOR, AO, Wireframe), which come from the synchronous CPU rasterizer (§1, §2) instead, refreshed every frame; and HSV/Luminance/Sobel/Gabor, GPU post-filters of the Beauty image (shared `PostProcessPass`, run once per selection, not per-pass).

| AOV | Category | Mechanism | Role / why it matters |
|---|---|---|---|
| Beauty | Utility | Final accumulated radiance, post tone-mapping | The primary output |
| Wireframe | Utility | Screen-space line rasterization (Pineda 1988), z-tested against the scene's own depth: white mesh-triangle edges, yellow scene-bounding-box edges (drawn on top, so yellow wins) | Visualizes triangle density/topology and sanity-checks scene extent/framing in one combined view |
| Alpha | Utility | 1.0 on a primary hit, 0.0 on a primary miss | Real coverage mask (this renderer isn't opaque-only-by-construction) |
| Depth | Utility | Planar camera-space Z (Arnold/RenderMan/EXR "Z" convention) at the primary hit | Depth-based compositing/debugging |
| HSV | Utility | Colour-space transform of Beauty | Isolates hue/saturation shifts a pure RGB view can hide |
| Luminance | Utility | Rec.709 luminance of Beauty | Isolates perceived brightness from colour |
| Sobel | Utility | 3×3 Sobel gradient magnitude of Luminance | Cheap edge/gradient signal |
| Gabor | Utility | 4-orientation Gabor kernel bank, max response, of Luminance | Directional edge/texture response Sobel's isotropic magnitude can't distinguish |
| WorldPos | Utility | Raw world-space primary-hit position | Debugging geometry/UV placement independent of shading |
| UV | Utility | Primary-hit interpolated UV (fractional part) | Visualizes the texture-space mapping directly |
| Normal | Material | Shading (normal-mapped) normal at the primary hit | The normal actually used in shading |
| GeomNormal | Material | Smooth interpolated vertex normal, before normal-mapping | Separates a bad normal map from a bad base mesh |
| Albedo | Material | Base-colour texture sample at the primary hit | Isolates texture data from lighting |
| Metallic | Material | Per-instance metallic factor (`settings.metallicFactor`, `materials/*.json` or `SceneConfig::materialOverrides`), uniform within one object's triangles but no longer whole-image-constant now that per-object material assignment exists | Debug which instance carries which metallic value |
| Roughness | Material | Roughness texture × a per-instance factor, floored at that material's own `roughnessMin` (materials can set their own floor, e.g. glass's below diffuse/chrome's shared 0.045); the texture varies per hit, the multiplying factor/floor vary per instance | Debug material authoring independent of shading |
| Tangent | Material | Shading tangent basis at the primary hit | Debugs the tangent-space basis used for normal mapping |
| ObjectID | Material | Per-instance index, false-coloured (`falseColorForId`) | Isolation mask for compositing/debugging |
| AO | Material | Authored ambient-occlusion texture sample at the primary hit | Debug baked AO independent of lighting |
| Fresnel | Transport | `mix(exact dielectric Fresnel, exact complex-IOR conductor Fresnel, metallic)` at the primary hit's view angle — the same term shading evaluates (`fresnelAtViewAngle`, `bsdf.h`), against the macro normal rather than a microfacet half-vector | Debug grazing-angle reflectance behaviour in isolation, including the conductor dip an authored `edgeTint` produces, which the previous Schlick term could not represent at any `f0` |
| IOR | Transport | Per-instance dielectric IOR (`settings.ior`), -1 on a miss | Isolates the raw refractive-index input driving Fresnel/transmission |
| BounceCount | Transport | Mean path termination depth across samples, per pixel | Debug Russian roulette/termination behaviour |
| DirectDiffuse | Lighting | Diffuse-bucketed radiance from a path's first (bounce-0) surface, physical (base colour included) | Isolates direct diffuse light arrival, in the same units as Beauty |
| IndirectDiffuse | Lighting | Diffuse-bucketed radiance from later bounces | Isolates indirect (bounced) diffuse contribution |
| DirectSpecular | Lighting | Specular-reflection-bucketed radiance, one bounce from camera | Isolates direct specular contribution |
| IndirectSpecular | Lighting | Specular-reflection-bucketed radiance, later bounces | Isolates indirect specular (reflections) |
| Refraction | Lighting | Radiance from any path that sampled a transmission lobe (sticky bucket) | Isolates glass/transmissive transport |
| Shadow | Lighting | Binary NEE occlusion test toward the sampled light (environment or an area light, per `LightSet`'s uniform selection) at the primary hit, re-averaged across progressive passes into continuous shadow/penumbra density | Isolates direct-light visibility from material/lighting colour |

## 5. Roadmap

Grouped by area of design, each group ordered by importance (most important first). **Large**, the closing group, is instead a strict dependency chain (each item needs the ones before it) reflecting build order rather than priority, and must land in that order -- it closes the roadmap as the long-horizon/strategic track, not because it matters least. Items elsewhere have no hard blocker on one another.

### 1. Rendering correctness (materials & transport physics)

- **An instrument that can see the coat half of `F_avg`**: `coatAlbedo` taking `dielectricFresnelAvg(ior)` rather than Karis' mean of Schlick is worth at most 1.9e-4 on any furnace reading, below every `bsdf_validate` tolerance -- reverting that half alone leaves all five validators **green**, yet it's the half that actually moves the cornell picture (1882 of 1886 changed channels).
  - `checkAverageFresnel` asserts `dielectricFresnelAvg` directly, pinning the function, but nothing pins the *call site*: a revert there is invisible.
  - Needs either a coat-specific energy identity tight enough to resolve 2e-4, or a direct assertion on `coatAlbedo`'s output against an independently integrated reference, as `checkAverageFresnel` does for the average itself.
- **Gate the EON sampling shape on `diffuseKd`**: `diffuseRoughness` conflates the EON diffuse *value* with the *sampling strategy* for the borrowed cosine multi-scatter lobe.
  - They decouple whenever `diffuseKd` is zero -- `metallic=1`/`transmissionFactor=1` zeroes it (`bsdf.cpp:643`) while `diffuseProb` stays non-zero (`bsdf.cpp:641`) -- so a conductor still draws CLTC (`sampleEon`, `bsdf.cpp:531`) to shape a lobe EON doesn't describe.
  - Measured on `chrome.json`: 1976/691200 channels affected, max 11/255, a 1.2% variance penalty; worked around there via `diffuseRoughness: 0`, which fixes one asset, not the mechanism.
  - Fix: gate the *shape* on `diffuseKd` at both `sampleEon` and `pdfEon` (`bsdf.cpp:543`) -- `diffuseKd` is deterministic from params, so this stays consistent. Not the *density*: `bsdf.cpp:362` forbids gating the pdf on kd, since selection probability is independent of it and starving the mixture denominator inflates throughput for metals.
- **Rough-transmission coverage at `transmissionDepth 0`**: `checkOnSurfaceTransmissionTint` (`tools/integrator_validate.cpp`) authors both its slab and sphere at roughness 0.02 (`alpha` 4e-4, below the smooth threshold), so every reading comes from `sampleBsdf`'s delta branch.
  - Measured, not assumed: tinting *only* that branch and reverting both continuous sites fails `bsdf_validate` 54 times yet leaves `integrator_validate` **green** -- the rough transmission path's tint has no integrator-level coverage at all, resting entirely on `bsdf_validate`'s analytic rows.
  - A rough row may be a one-line change -- at `ior 1.0` Walter refraction is straight through for any microfacet normal, so the expected reading should still be exactly `transmissionColor^2` -- but that needs measuring first: whether the transmissive multiple-scattering lobe activates at `eta = 1` and moves the expectation off the square. Not Quick until that reads back.
- **Occlusion-sensitive curved transmissive test**: `checkTransmissiveSphere` (`tools/integrator_validate.cpp:443`) asserts a white non-absorbing glass sphere is invisible under a uniform environment.
  - Closes one of two curvature-driven blind spots: reverting the shadow-terminator projection's side reads 0.955/0.877 there while the flat slab stays bit-identical. Cannot close the other -- a furnace measures how much energy arrives, not where from, so it can't see a light *leak*.
  - Reverting either curvature-scaled epsilon (`transmissionOffsetEpsilon`, `path_tracer.cpp:43`, used at the far-side NEE shadow ray `path_tracer.cpp:265` and the transmission continuation ray `path_tracer.cpp:335`) leaves the whole suite green: measured deltas 2e-4 and 4e-3 at 64x32 and again at cornell's own 24x12 tessellation -- not a tessellation artefact.
  - Needs a configuration where the far-side shadow ray *ought* to be occluded (rough glass against an opaque backdrop). Not Quick: that has no closed-form answer, so it needs a converged-reference or two-estimator invariant rather than an analytic one -- every transmissive assertion in the suite today is analytic.
- **CIE-integrated fit for the chromium reflectivity/edge-tint triples**: the measured chromium at `conductorIorFromReflectivity` (`bsdf.cpp`) -- reflectivity `[0.552, 0.555, 0.558]`, `edgeTint` `[0.555, 0.558, 0.672]`, from Johnson & Christy 1974 via Gulbrandsen eq 14/15 -- is sampled at three representative wavelengths rather than integrated against the CIE colour-matching functions, which is what an RGB channel actually is. Its 0.0048 round-trip error against the source `(n, k)`, and its sensitivity to the chosen wavelength triple, are both documented there. Proper integration needs the CIE curves and a chosen RGB primary set, giving every future measured metal one principled path from tabulated `(n, k)` to an authored `(r, g)` pair instead of a hand-picked triple.

### 2. Lighting

- **Light temperature (Kelvin/blackbody)**: a quad light's emission is authored as a raw `color` x `intensity` RGB triple (`QuadLightConfig`, `scene_config.h`), no colour-temperature input anywhere; the environment carries whatever white balance its HDRI was captured at. Unblocked now a synthetic light source exists (§5 Large #2): a Planckian-locus-to-RGB conversion (CIE 1931 colour-matching functions) so a light can be authored by Kelvin rather than a hand-picked triple.
- **Photometric calibration**: tie radiometric output to real photometric units (lux/candela/lumen) so `ev100()` (`camera.h:54-56`) and light intensities can be checked against a light meter instead of eyeballed. Complements the Macbeth chart scene.

### 3. Rendering performance & sampling

- **Low-discrepancy sampler upgrade**: Sobol/hash-based Owen scrambling (Burley 2020, §6), replacing randomised Halton (`sampler.cpp`).
- **Blue-noise dithered sampling**: `Sampler` (`sampler.h:21-33`) is randomised Halton (Cranley-Patterson rotated) + PCG32 beyond 32 dimensions; no blue-noise mask exists anywhere. Complementary to, not a replacement for, the Low-discrepancy sampler upgrade above: Sobol/Owen improves asymptotic convergence rate, blue noise improves the visual error distribution at low sample counts (a single-sample preview, or dithering before temporal accumulation).
- **Render-mode selector + adaptive tiling**: Single Sample / Progressive / Adaptive Tiling. Today the path tracer dispatches fixed 96x96 tiles and the rasterizer rows, both via `ThreadPool`; neither adapts to residual noise, and the mode isn't selectable.
- **Adaptive per-pixel sample budget**: variance-driven, builds on tiling above; `samplesPerPixel` (`profile.json`) is one fixed global today, no per-pixel allocation.
- **Texture minification filtering (MIP-mapping)**: point/bilinear only today (`sampleBilinear`, `hdr_image.h:22`); grazing/distant surfaces alias. No mip chain exists; needs ray differentials to pick a level per ray.
- **Texture bit depth (16/32) via JSON**: hardcoded `GL_RGBA16F` today (`texture.cpp:45`); 32F ~doubles VRAM/buffer.
- **High-frequency binary noise texture/material**: none exists yet. Add as a stress test for the Texture minification filtering item above -- high-frequency content exposes aliasing before/after mip-mapping lands, since sampling is point/bilinear only today (`sampleBilinear`, `hdr_image.h:22`).
- **Frustum/backface culling**: skip `buildSubTriangles`'s per-frame full-scene walk (`rasterizer.cpp:153,274`) and the equivalent Embree traversal when out of view.
- **Packet tracing**: `EmbreeAccel` calls `rtcIntersect1`/`rtcOccluded1` exclusively (`embree_accel.cpp:116,144`), single-ray only -- no `rtcIntersect4/8/16` packet API.
- **Ray reordering before shading**: the tile loop traces in raster order (`path_tracer.cpp:307-320`), no Morton/direction-coherence sort ahead of `tracePath` (`path_tracer.cpp:77`).
- **Deferred/sorted shading by material**: `tracePath` (`path_tracer.cpp:77`) evaluates the BSDF inline per ray; no material-bucketed shading pass.

### 4. Scene & geometry

- **Scene-graph foundation**: the only ordered chain outside **Large**; prerequisite for Motion blur (Camera, below).
  1. *Indexed geometry.* `gltf_loader.cpp:94-129` de-indexes every mesh into soup, storing positions twice (once in `Triangle` for Embree, once in `ShadingVertex.position` for shading) -- 184 B/tri, ~920 MB on a 5M-triangle asset. `rtcSetSharedGeometryBuffer` takes a byte stride, so Embree can read positions in place from the indexed shading vertices instead: deletes the `Triangle` array, keeps glTF's own index buffer rather than filling one with the identity sequence (`embree_accel.cpp:77-84`), drops the `reserve(size+1)` padding hack. ~5x smaller; traversal locality matters more than the footprint.
  2. *Object instancing.* `RTC_GEOMETRY_TYPE_INSTANCE` over one scene per unique mesh, replacing today's single flattened geometry (`embree_accel.cpp:65`), finally giving `MeshInstance::transform` (`gltf_loader.h:18`, stored and never read) a consumer. `Hit` gains `geomID`/`instID`. Rays trace in object space against shared BVH leaves instead of today's per-instance duplication. Needs (1)'s indexed layout.
  3. ~~*Per-material factors.*~~ **Solved a different way.** `SceneConfig::materialOverrides` (glTF node name → `materials/*.json` path) already builds a per-instance settings vector, independent of (1)/(2), no indexed-geometry or instancing prerequisite. `Material` itself still holds only six textures -- `metallic_factor`/`roughness_factor`/etc. are still parsed by cgltf and discarded, and `extrasTextureIndex` (`gltf_loader.cpp:35`) is still the hand-rolled substring scanner it always was. Per-object factors are authored in `scene.json`/`materials/*.json` instead of the glTF file itself; moving them onto `Material` for glTF-native authoring remains open.
- **Per-object bounds**: `EmbreeAccel::sceneBounds()` (`embree_accel.cpp:124-129`) computes one `rtcGetSceneBounds()` over a single flattened `RTC_GEOMETRY_TYPE_TRIANGLE` geometry holding every instance's triangles combined (`embree_accel.cpp:53-100`); the only consumer, the Wireframe AOV's box overlay, draws one scene-wide box (`rasterizer.cpp:435-436`). Every triangle already carries `instanceIndex` (`shading_scene.h:21`), so per-object AABBs can be computed by grouping `worldTriangles` by it -- no dependency on Scene-graph foundation above.

### 5. Camera

- **Fisheye lens**: `Camera::primaryRay` (`camera.h:64,67`; `camera.cpp:51-59`) derives ray direction from a linear NDC-to-view-plane mapping -- pinhole only; thin-lens/DoF doesn't exist either (Depth of field, below). Not a glass-simulation problem: the equidistant/equisolid-angle/orthographic/stereographic projection families (Kannala & Brandt 2006) are themselves the physically-correct angle-to-image-radius mapping real fisheye lenses use. Implementable as an alternative analytic ray-generation function alongside `primaryRay`, no lens-element simulation needed.
- **Physical camera filters**: no filter simulation exists; the only "filter" hits are the AA reconstruction filter (`filterWeight`, `path_tracer.cpp:89`, applied at splat time `path_tracer.cpp:458-473`) and unrelated GLSL texture filtering. Three types, rising in cost: ND (uniform attenuation across the visible spectrum -- a straightforward exposure-path multiplier); UV (blocks near-UV, effectively a no-op on visible-spectrum RGB radiance as modelled today); CPL/polarising (Malus's-law attenuation dependent on polarisation angle) needs the renderer to track polarisation state (Stokes vector) at all, which it doesn't -- unpolarised scalar radiance throughout (Chandrasekhar 1960's polarised radiative-transfer treatment is the classical reference). ND/UV are Quick-sized; CPL is its own structural item.
- **Depth of field**: thin-lens sampling in `primaryRay` + focus distance; technically unblocked today, cheaper once adaptive sampling lands.
- **Motion blur**: blocked on Scene-graph foundation (Scene & geometry, above) + Embree multi-timestep geometry.

### 6. Debug tooling, AOVs & UX

- **Recover the delighted DirectDiffuse/DirectSpecular view**: `bdecb41` made the five transport AOVs physical (each contribution written once) so they exactly partition Beauty (`checkTransportPartition`, `integrator_validate.cpp:387`, ±1e-4) -- the prior delighted view (base colour divided out) was deliberately dropped, not a regression; its replacement is `DirectDiffuse / Albedo`.
  - Blocked, not Quick: Albedo is a `RasterGBuffer` field (`rasterizer.cpp:321`); `aovNeedsLightTransport` (`main.cpp:773-808`) makes the rasterizer/path-tracer mutually exclusive, so the two are never simultaneously fresh.
  - Needs: a path-traced albedo buffer (+3 accumulator lanes), or relaxing that gating.
- **Optic flow AOV**: per-pixel motion vectors, appended to `AovId` (`aov.h:7-40`).
  - No scene-graph animation yet (see Scene-graph foundation above), camera-only motion over static geometry -- this reprojects `WorldPos` (§4) through the previous frame's camera transform, not true motion capture.
  - `Camera` (`camera.h:10`) exposes only current `position()` (`camera.h:23`), no stored prior-frame matrix; needs one new persisted matrix, no new ray/sample work.
- **Contact sheet export (grid of every AOV)**: tile thumbnails of all 27 `AovId` (`aov.h:7-40`) at once, vs. the HUD's single `ImGui::Combo` (`hud_overlay.cpp:336`) feeding one `pathTraceDisplayTexture` blit (`presentFrame`, `main.cpp:739`). Same mutual-exclusion blocker as the delighted-view item above: `aovNeedsLightTransport` (`main.cpp:93-108`) splits the 27 into 12 light-transport / 15 rasterizer AOVs, only one side fresh per frame -- needs that gating relaxed, not just N reads of one cached buffer.
- **Expand terminal output (launch + loop)**: startup logs GL/camera/model/BVH stats (`main.cpp:286-361`); no per-frame stdout during the loop (`main.cpp:990`) -- sample/pass/convergence stats reach only the HUD (`hud_overlay.cpp`).
- **Expand reset (`0`) to full launch state**: `DebugCameraController::resetToDefault()` (`debug_camera_controller.cpp:122-127`, bound at `main.cpp:494-495`) resets only camera position/yaw/pitch/orbiting.
  - Exposure-triangle defaults are already stored but never applied (`defaultAperture_`/`defaultShutterSeconds_`/`defaultIso_`, `debug_camera_controller.h:77-79`).
  - Also not reset: `app.aov`, `userLut`, `channelView`, `invert`, `showHud`, `envRotationDegrees`, `showSky`, `envExposureStops`, `envLightEnabled` (`main.cpp:153-233`).
- **PNG capture tool hardening**: `tools/render_beauty.cpp`'s `writePng` (lines 68-111) pipeline order is correct (exposure → OCIO display transform → dither → clamp → quantize, lines 198-234). Two gaps: RGB-only, no alpha (`colour type 2`, line 95); scanline filter hardcoded to type 0/None despite a comment claiming adaptive filtering (lines 70-74). Fix the comment or implement real adaptive filtering; add alpha output if a future consumer needs it.
- **Benchmark log**: `raster_bench` (`tools/raster_bench.cpp:252`) prints one-shot timing to stdout only, nothing persists between runs. Append each run (git SHA, CLI args, scene, timing) to a log so regressions are visible across runs.
- **Example images of engine technology**: no gallery of the renderer's own output in this README. Build small demo scenes each isolating one feature (a material, GI behaviour, transmission), render with `render_beauty`, embed a curated set here.
- **Nuke-equivalent exposure/gamma control**: extend `OcioDisplayTransform` with live numeric control, beyond today's LUT cycling (`L`).

### 7. Testing & validation infrastructure

- **Test suite hardening**: `ctest` wires 5 correctness validators (`bsdf_validate`, `embree_validate`, `integrator_validate`, `nee_validate`, `rasterizer_validate`; `enable_testing()`/`add_test` loop, `CMakeLists.txt:229-232`), but no unit-test framework exists for logic that doesn't need a full scene (sampler, BSDF math in isolation), and no automated regression-image gate -- `render_beauty`'s `--compare` exists but is deliberately excluded from `add_test` (`CMakeLists.txt:199`, human-judged visual comparison). Add a lightweight unit-test framework for the former, and/or a threshold-based promotion of the image diff into `ctest` for the latter.

### 8. Engineering & maintenance

- **Memory efficiency pass**: only monitoring exists today (Memory HUD, §2), no active reduction initiative. Candidates once profiled: the indexed-geometry duplication under Scene-graph foundation (Scene & geometry, above), and texture bit depth (Rendering performance & sampling, above).
- **Documentation pass**: three parts -- a user/build guide beyond this README's Build section; an architecture/API reference for the `src/`/`include/engine/` module layout; write-ups of the physically-based techniques in use (BSDF model, sampling, GI), separate from inline code comments.
- **Code-quality audit**: `rotateAboutY` (`environment_map.cpp:13`) → `glm::rotate`; `ShadingFrame::toLocal`/`toWorld` (`bsdf.h:26`) → `glm::mat3`. (BSDF math in `bsdf.cpp` -- GGX/Smith/Fresnel/VNDF -- is standard domain logic, not an offload candidate.)

### 9. Large: strict dependency order

1. **Volumetric & subsurface transport**: participating media (in-scattering, phase functions) + BSSRDF/random-walk subsurface.
   - Beer-Lambert *extinction* (no in-scattering) already ships for tinted glass (`transmissionColor`/`transmissionDepth`, `path_tracer.cpp`'s `mediumSigmaA`) -- a simpler subset of this item, not the item itself.
   - The multiple-scattering transmission lobe's energy-orientation bug (`multiScatterShape` evaluating the transmitted-wi escape probability at the wrong eta) is fixed. What remains is structural, not buggy: that lobe's strict per-direction reciprocity (`f(wo→wi) ≠ f(wi→wo)`) is inherent to the transmissive multi-scatter formulation itself (`checkTransmissionReciprocity`, `tools/bsdf_validate.cpp`, deliberately scoped to single scatter) -- full bidirectional participating-media/subsurface transport is blocked on whatever that transport algorithm needs from this lobe, not on a fix already available.
2. **Global illumination**: area lights + shadow rays to them now ship -- `LightSet` (`light.h`) generalises NEE from the environment map alone to a uniformly-selected set of lights, and a rectangular emitter (`QuadLight`, solid-angle sampled via Ureña, Fajardo & King 2013's spherical-rectangle parametrisation, edges validated perpendicular at scene load) is the first light type, its two triangles injected into the Embree scene so it occludes and is BSDF-hittable. The classic Goral et al. 1984 emissive-panel Cornell box is reachable from `assets/scenes/cornell.json` with the HUD's "Environment Light" checkbox off (or `render_beauty --env-light 0`).
   - Still needed: ReSTIR (Bitterli et al. 2020) needs *multiple* area lights to resample across -- one light selected uniformly has nothing to resample; ray-traced AO to replace today's baked-texture AO AOV (§4); disk/sphere lights; emissive-mesh lights; power-weighted light selection.
   - Caustics do not fall out of this: unidirectional path tracing structurally cannot sample specular-diffuse-specular paths regardless of light count -- that needs (4).
3. **GPU ray-tracing backend**: Embree SYCL or CUDA-OptiX, to raise achievable sample budget beyond CPU Embree. Today's single-ray `rtcIntersect1`/`rtcOccluded1` calls and per-ray inline shading in `tracePath` (`path_tracer.cpp:77`) are the opposite of a wavefront/streaming GPU kernel design -- this item is that rearchitecture, not just a backend swap.
4. **Bidirectional path tracing with MIS (caustics)**: light-subpath/eye-subpath vertex connection (Veach & Guibas 1995; Veach 1997) -- the transport algorithm caustics need, since unidirectional path tracing (2) can't produce them at all. Blocked on (2) plus the transmissive multi-scatter lobe's structural non-reciprocity from (1): connection needs BSDF agreement in both directions, which that lobe doesn't provide by construction, not because of a bug.
5. **Spectral upgrade**: per-wavelength transport and hero-wavelength sampling (Wilkie et al. 2014). Dispersion already ships in RGB form (`MaterialConfig::abbe` → `bsdf.cpp`'s `cauchyIor`, one hero *channel* per path); what remains is the genuinely spectral part.
   - (a) `kRgbWavelengthsNm` is one fixed wavelength per channel -- OpenPBR's own reference implementation notes this produces "discrete RGB bands" rather than continuous rainbows. Cheap fix: a stochastically drawn lambda per path, just replacing that constant lookup with a draw from each channel's spectral sensitivity, no transport change. Quick-sized.
   - (b) Radiance itself is still RGB, so a true spectral integrator additionally needs spectral upsampling of RGB textures and environment maps (Jakob & Hanika 2019). This is the item itself; likely offline-only given sample-budget cost.
6. **Denoising**: needs (1)-(5) transport correctness first -- denoising an incorrect image just smooths the error.
7. **Upscaling**: spatial/temporal supersampling (neural, Xiao et al. 2020, §6, or classical).
8. **GenAI diffusion channel**: img2img refinement AOV + raw latent/embedding output for HOST's cognitive pipeline. Needs (6)'s converged image.

## 6. References

- Khronos Group. glTF 2.0 specification: scene/mesh/material interchange format.
- Mikkelsen, M.S. (2008). Simulation of wrinkled surfaces revisited: MikkTSpace tangent space standard; not implemented (tangent mapping uses glTF-supplied tangents only, no MikkTSpace generation).
- Goral, C.M., Torrance, K.E., Greenberg, D.P., Battaile, B. (1984). Modeling the interaction of light between diffuse surfaces. SIGGRAPH: the classic emissive-panel Cornell box -- implemented (`assets/scenes/cornell.json`'s ceiling panel, reachable with the HUD's "Environment Light" checkbox off).
- Kannala, J., Brandt, S.S. (2006). A generic camera model and calibration method for conventional, wide-angle, and fish-eye lenses. IEEE TPAMI: the equidistant/equisolid-angle/orthographic/stereographic fisheye projection families -- §5, fisheye lens, not yet implemented.
- Chandrasekhar, S. (1960). Radiative Transfer. Dover: the classical polarised radiative-transfer treatment underlying a CPL/polarising camera filter -- §5, physical camera filters, not yet implemented.
- Kajiya, J.T. (1986). The rendering equation. SIGGRAPH.
- Veach, E. (1997). Robust Monte Carlo Methods for Light Transport Simulation. PhD thesis, Stanford: multiple importance sampling, next-event estimation, and the MIS-weighted vertex connection -- §5 Large #4, not yet implemented.
- Veach, E., Guibas, L.J. (1995). Bidirectional estimators for light transport. Eurographics Rendering Workshop: light-subpath/eye-subpath vertex connection -- §5 Large #4, not yet implemented.
- Pharr, M., Jakob, W., Humphreys, G. Physically Based Rendering: From Theory to Implementation (PBRT); source of `FrDielectric`, the exact unpolarised dielectric Fresnel formula the BSDF uses.
- Cook, R.L., Torrance, K.E. (1982). A reflectance model for computer graphics. ACM ToG: BRDF and Fresnel foundations.
- Walter, B. et al. (2007). Microfacet models for refraction through rough surfaces: the GGX distribution, and the rough-refraction BTDF (value eq. 21, half-vector eq. 16, Jacobian eq. 17) the transmission lobe implements in PBRT-v3's radiance-transport form.
- Debevec, P. (1998). Rendering synthetic objects into real scenes: HDR image-based lighting.
- Wilkie, A. et al. (2014). Hero wavelength spectral sampling. Computer Graphics Forum: the sample-budget-bounding scheme -- §5 Large #5, not yet implemented. The shipped dispersion commits a path to one RGB *channel* at its first dispersive interface rather than to a wavelength at path start -- the same one-sample idea at RGB resolution, strictly cheaper, since a path that never meets dispersive glass keeps all three channels.
- Khronos Group. `KHR_materials_dispersion`: the normative statement of Cauchy's `n(lambda) = A + B/lambda^2` inverted from an Abbe number at the Fraunhofer d/F/C lines, which `bsdf.cpp`'s `cauchyIor` implements in the spec's general form rather than its pre-multiplied composite one.
- OpenPBR Surface specification, and Autodesk's Arnold `standard_surface`: the transmission-tint convention. `transmission_color` is the sole tint on transmitted light -- OpenPBR's `transmission_depth` "controls the depth into the volume at which the `transmission_color` is realized; if zero, acts as a constant (on-surface) transmission tint", Arnold rendering that zero case as a flat filter colour -- while `base_color` is "the observed reflection color (viewed at normal incidence under uniform illumination)" and leaves transmitted light unaffected. glTF `KHR_materials_transmission` instead tints transmission with `baseColor`, but only for want of a transmission colour of its own: it is scoped to "infinitely thin surfaces" whose absorption "is constant and equal to `1.0 - baseColor`", which is OpenPBR's zero-depth case under another name, and `KHR_materials_volume` then adds absorption alongside it as a separate effect ("Base color changes the color of light at the volume boundary. Absorption occurs while the light is traveling through the volume."). This pipeline follows OpenPBR/Arnold, the standard it already takes `edgeTint`, EON, `base_color`'s meaning and its RGB wavelength triple from.
- Adobe. OpenPBR BSDF reference implementation (`adobe/openpbr-bsdf`), `openpbr_constants.h`: `OpenPBR_BaseRgbWavelengths_nm` = 620/540/450 nm, the representative per-channel wavelengths `kRgbWavelengthsNm` takes, and the source of the "discrete RGB bands" limitation (§5 Large #5).
- OpenPBR: Novel Features and Implementation Details (arXiv:2512.23696): the throughput-weighted colour-channel selection the dispersive path uses, which keeps a path's throughput magnitude balanced instead of varying by 3x with the draw.
- OpenEXR / Academy Software Foundation technical documentation: linear HDR pipeline, exposure.
- Wald, I., Woop, S., Benthin, C., Johnson, G.S., Ernst, M. (2014). Embree: A Kernel Framework for Efficient CPU Ray Tracing. ACM ToG (SIGGRAPH); the CPU ray-scene intersection kernel library (`EmbreeAccel`) backing BVH build/traversal, replacing an earlier hand-rolled binned-SAH implementation.
- Heitz, E. (2014). Understanding the Masking-Shadowing Function in Microfacet-Based BRDFs: the Smith height-correlated visibility term (`bsdf.cpp`'s `smithG1`/`smithG2`).
- Christensen, P.H., Jarosz, W. (2016). The Path to Path-Traced Movies. Foundations and Trends in Computer Graphics and Vision: production path-tracing grounding.
- Heitz, E. (2018). Sampling the GGX Distribution of Visible Normals. JCGT 7(4): the VNDF importance-sampling routine the specular lobe uses.
- Sobel filtering: edge-detection AOV computed from Luminance (§4), arXiv:2601.16806.
- Chiang, M.J.-Y., Li, Y., Burley, B. (2019). Taming the Shadow Terminator. JCGT 8(4): the shading-point correction used for secondary-ray origins.
- Halton, J.H. (1960). On the efficiency of certain quasi-random sequences of points in evaluating multi-dimensional integrals; Cranley, R., Patterson, T.N.L. (1976). Randomization of number theoretic methods for multiple integration: `Sampler`'s radical inverse + per-pixel rotation.
- O'Neill, M.E. (2014). PCG: A Family of Simple Fast Space-Efficient Statistically Good Algorithms for Random Number Generation; `Sampler`'s fallback generator beyond its low-discrepancy dimension budget.
- Arvo, J., Kirk, D. (1990). Particle Transport and Image Synthesis: Russian roulette path termination.
- Gulbrandsen, O. (2014). Artist Friendly Metallic Fresnel. JCGT 3(4): the conductor Fresnel parameterisation, ported from the paper's Appendix A. Reflectivity `r` (the existing `f0`) plus an authored `edgeTint` `g` invert to a complex IOR via eq 12 and eq 2, replacing Schlick, which is monotone in `cos` by construction and so forces every metal to exactly white at grazing. Two documented departures from the listing: `k^2` is evaluated in the factored form `(nMax-n)(n-nLow)` (the same expression through eq 2's own roots) because the literal form loses all precision in float32 near `r=1`, returning `-1.28e6` where the truth is 0; and the reflectance itself is the exact unpolarised complex-IOR Fresnel rather than the listing's large-|eta| approximation, which deviates from it by up to 0.094 absolute at mid reflectivity. The cosine mean `F_avg` the Kulla-Conty tint needs has no closed form on this basis and is a 3-node quadrature rule over the same Fresnel, fitted against 128-point Gauss-Legendre to a measured 4.0e-4 (`conductorFresnelAvg`, asserted by `checkAverageFresnel`); a rational fit over `(r, g)` was measured and rejected, since `n` collapses from 39.8 to 0.005 across the last tenth of `g` at `r=0.99` and 49 terms reached only 9e-3.
- Portsmouth, J., Kutz, P., Hill, S. (2025, **revised 2026-02-04**). EON: A Practical Energy-Preserving Rough Diffuse BRDF. JCGT 14(1): the rough-diffuse lobe, its analytic multiple-scattering compensation, and CLTC importance sampling, all ported from the paper's GLSL listings. The 2026 revision added Appendix A's albedo inversion, ported at `eonAlbedoInversion`: without it `rho` is the authored colour directly and the multiple-scattering term's saturation drags the *observed* albedo below it, measured at -12.2% for `diffuseColour 0.5` at `diffuseRoughness 1` and -20.6% on the darkest channel of `[0.8, 0.3, 0.1]`. The appendix gives two inversions and they cannot both hold; this pipeline takes the normal-incidence one (eq. 30/31), so `diffuseColour` means what OpenPBR says `base_color` means -- the observed reflection colour at normal incidence under uniform illumination -- a meaning OpenPBR declares but does not enforce, since it sets `rho = C` directly. Eq. 30's stated root is the unstable one, diverging as `r -> 0`, and the paper's remedy is a Taylor form switched in below some roughness; the conjugate-multiplied root is algebraically identical, needs no threshold, and recovers `rho = C` at `r = 0` from the algebra (Press et al., *Numerical Recipes* 5.6). The authors' `EON-diffuse` repository ships no inversion code, so this is a transcription of the equations rather than of a listing.
- Dupuy, J., Benyoub, A. (2023). Sampling Visible GGX Normals with Spherical Caps; Tokuyoshi, Y., Eto, K. (2023). Bounded VNDF Sampling for Smith-GGX Reflections: newer VNDF refinements surveyed, not implemented (Heitz 2018 used instead; better-established, lower risk to reproduce correctly from reference material alone).
- Heitz, E., Hanika, J., d'Eon, E., Dachsbacher, C. (2016). Multiple-scattering microfacet BSDFs with the Smith model. The source of the problem, not the solution used: it establishes the energy single-scatter GGX discards (a white conductor returned 0.31 of the light it received at roughness 1.0), but the implemented compensation is Kulla & Conty's cheaper directional-albedo form below rather than this paper's stochastic microsurface evaluation.
- Kulla, C., Conty, A. (2017). Revisiting Physically Based Shading at Imageworks. SIGGRAPH course. The multiple-scattering energy compensation the BSDF implements: a directional-albedo table drives a compensation lobe returning exactly the deficit `smithG2` masks away, on the reflective and transmissive interface alike, with its own cosine sampling strategy on the transmit side.
- Burley, B. (2020). Practical Hash-based Owen Scrambling. JCGT 9(4): §5, low-discrepancy sampler upgrade (Sobol + Owen scrambling), not yet implemented (depends on precomputed direction-number tables; randomised Halton used instead).
- Schüßler, V., Heitz, E., Hanika, J., Dachsbacher, C. (2017). Microfacet-based normal mapping for robust Monte Carlo path tracing: considered for normal-map robustness, not implemented (a simpler geometric-normal-consistency rejection is used instead: a normal-map-induced light-leak sample is absorbed rather than reconstructed via the full two-facet microsurface model).
- Jensen, H.W., Marschner, S.R., Levoy, M., Hanrahan, P. (2001). A Practical Model for Subsurface Light Transport. SIGGRAPH; Christensen, P.H., Burley, B. (2015). Approximate Reflectance Profiles for Efficient Subsurface Scattering: BSSRDF and its practical diffusion-profile approximation -- §5 Large #1, not yet implemented.
- Novák, J., Georgiev, I., Hanika, J., Jarosz, W. (2018). Monte Carlo Methods for Volumetric Light Transport Simulation. Computer Graphics Forum (EG STAR): participating-media survey -- §5 Large #1, not yet implemented.
- Cook, R.L., Porter, T., Carpenter, L. (1984). Distributed Ray Tracing. SIGGRAPH: the stochastic-sampling origin of depth of field and motion blur -- §5, depth of field and motion blur, not yet implemented (primary rays are pinhole and instantaneous; the camera's aperture and shutter drive exposure only).
- Williams, L. (1983). Pyramidal Parametrics. SIGGRAPH: MIP-mapping -- §5, texture minification filtering, not yet implemented (textures are point/bilinear-sampled per ray only).
- Zwicker, M. et al. (2015). Recent Advances in Adaptive Sampling and Reconstruction for Monte Carlo Rendering. Computer Graphics Forum (EG STAR): denoising/reconstruction survey -- §5 Large #6, not yet implemented.
- Belcour, L. (2018). Efficient Rendering of Layered Materials using an Atomic Decomposition with Statistical Operators. ACM ToG: layered-BSDF approach, not yet implemented (materials are single-layer metallic-roughness only).
- Bitterli, B., Wyman, C., Pharr, M., Shirley, P., Lefohn, A., Jarosz, W. (2020). Spatiotemporal reservoir resampling for real-time ray tracing with dynamic direct lighting (ReSTIR). SIGGRAPH -- §5 Large #2, not yet implemented: `LightSet` now selects among multiple lights uniformly, but ReSTIR's reservoir resampling needs many lights to be worth resampling across -- one area light plus the environment isn't yet that regime.
- Ureña, C., Fajardo, M., King, A. (2013). An Area-Preserving Parametrization for Spherical Rectangles. Computer Graphics Forum (EGSR): the constant-solid-angle-density sampler `light.cpp`'s `SphericalRectangle` implements for a rectangular area light's NEE sampling, matching Arnold's own `quad_light` sampler; as given in Pharr/Jakob/Humphreys, *Physically Based Rendering* (4th ed.) Sec 12.5.3.
- Lambert, J.H. (1760). Photometria; Baum, D.R., Rushmeier, H.E., Winget, J.M. (1989). Improving radiosity solutions through the use of analytically determined form-factors. SIGGRAPH: the closed-form Lambertian-polygon irradiance formula `tools/integrator_validate.cpp`'s `checkQuadLightIrradianceOneSidedOcclusion` uses as its analytic reference -- independent of `SphericalRectangle`'s own solid-angle formula (Girard's theorem on the polygon's internal vertex angles) since it instead sums the projected solid angle directly over the polygon's edges, so the two cannot share a transcription bug.
- Miller, G. (1994). Efficient algorithms for local and global accessibility shading. SIGGRAPH: ambient occlusion -- §5 Large #2 (ray-traced AO note), not yet implemented (the AO AOV currently samples a baked texture, §4).
- Xiao, L., Nouri, S., Chapman, M., Fix, A., Lanman, D., Kaplanyan, A. (2020). Neural supersampling for real-time rendering. SIGGRAPH: upscaling -- §5 Large #7, not yet implemented.
- Ho, J., Jain, A., Abbeel, P. (2020). Denoising diffusion probabilistic models. NeurIPS; Rombach, R., Blattmann, A., Lorenz, D., Esser, P., Ommer, B. (2022). High-resolution image synthesis with latent diffusion models. CVPR: diffusion-model foundations -- §5 Large #8, not yet implemented.
- Pineda, J. (1988). A parallel algorithm for polygon rasterization. SIGGRAPH: the edge-function incremental rasterization technique behind the primary-hit rasterizer (§1, §2, `rasterizer.cpp`).
- Sutherland, I.E., Hodgman, G.W. (1974). Reentrant polygon clipping. CACM: the near-plane polygon clip the primary-hit rasterizer's triangle setup uses (`rasterizer.cpp`).
