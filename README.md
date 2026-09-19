# Physically based path tracer

*A CPU, unidirectional Monte Carlo path tracer with real-time progressive display: Embree-accelerated, stochastic BSDF sampling combined with environment-map NEE via MIS, behind a thin OpenGL display/HUD layer.*

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

```
git submodule update --init --recursive
```

### Configure & build

```
cmake -B build
cmake --build build
```

### Run

```
./build/engine [-scene path/to/scene.json] [-stats] [-bench log.jsonl]
```

Defaults to `assets/scenes/cornell.json` if `-scene` is omitted — currently the only scene shipped.

### Benchmark

Timing runs append one JSON Lines record each to a local log (§2 Benchmark log): `engine -bench PATH`, `render_beauty --bench-log PATH`, `raster_bench --bench-log PATH`. A performance claim is made with a randomized interleaved A/B of two builds, not by comparing two runs by eye:

```
./build/bench_compare run --a buildA/render_beauty --b buildB/render_beauty --rounds 12 --log renders/bench.jsonl \
    -- --scene scenes/cornell.json --out renders/ab.png --passes 32 --width 640 --height 360 --bench-log renders/bench.jsonl
```

It prints B/A with a distribution-free confidence interval, and says "not resolved" when that interval contains 1. `bench_compare compare` and `bench_compare history --tool T` read records already in the log (unpaired, so drift is not controlled). `--metric` picks any samples column (its mean per event -- per frame, per upload or per pass, since frame count scales with run duration) or rusage field such as `user_s`.

## 1. Pipeline

**Startup.** glTF geometry/materials load once into a CPU-resident scene: per-vertex shading data (`ShadingTriangle`) and world-space triangles feed an Embree scene (`EmbreeAccel`); materials keep only CPU `HdrImage` textures, sampled per-ray. An equirectangular HDR environment map loads alongside it, with its own luminance-importance-sampling CDF for NEE. Area lights (`scene.json`'s `lights`) are injected into the same Embree scene at load (`appendQuadLights`), so they occlude and are camera/BSDF-hittable with no second intersection path.

**Per frame**, on any camera/scene-state change, two paths run independently off the same trigger:

- **Path tracer** — `PathTraceDriver` hands a fresh request to a background thread pool (one worker/core, row-parallel, dynamic scheduling), restarting progressive accumulation:
  1. Camera ray generation (pinhole) → Embree intersection (`rtcIntersect1`/`rtcOccluded1`)
  2. BSDF eval/sampling (Heitz 2018 GGX VNDF specular; EON rough-diffuse, Portsmouth/Kutz/Hill 2025; Walter 2007 rough dielectric transmission with exact Fresnel/TIR, falling back to a Snell delta lobe below the smooth-roughness threshold and at an index-matched interface; Kulla-Conty multiple-scattering compensation on both interfaces; exact complex-IOR conductor Fresnel via Gulbrandsen 2014's reflectivity/edge-tint parameterisation)
  3. NEE against a `LightSet` (environment map, optionally excluded via the HUD's "Environment Light" checkbox, plus any area lights — uniform selection, quads importance-sampled by solid angle via Ureña/Fajardo/King 2013's spherical-rectangle parametrisation), MIS-combined with BSDF sampling (power heuristic, Veach 1997)
  4. Recursive bounce loop with Russian roulette, Chiang/Li/Burley 2019 shadow-terminator-corrected secondary-ray origins, Beer-Lambert extinction inside transmissive media
  5. Radiance + full G-buffer/transport-component AOV set accumulated per pass, published lock-free for the render thread

- **Rasterizer** — a synchronous CPU pass (`rasterizer.cpp`) computes the 14 primary-hit-only AOVs (§4) every frame on the render thread: watertight edge-function rasterization (Pineda 1988) -- view-frustum Sutherland-Hodgman clipping, vertices snapped to a fixed-point grid whose precision is derived per frame for exact int64 edge functions, and the top-left fill rule, so triangles sharing an edge cover every pixel centre on it exactly once -- sharing `gbuffer_shading.h`'s material sampling with the path tracer but no Embree/BSDF/recursion. This gives those AOVs instant, glitch-free feedback during camera movement, decoupled from Beauty's own progressive convergence — the path-traced request above only restarts when the selected AOV needs light-transport data.

**Display.** The render thread blits whichever AOV is selected through OCIO's display transform (exposure/tone-mapping) and the debug HUD, converging over subsequent passes rather than blocking on one long render. No GPU rasterization anywhere: OpenGL exists only for the window, the post-process/OCIO blit, and ImGui; the primary-hit rasterizer is CPU-only.

## 2. Component reference

### Camera & display

| Feature | Mechanism |
|---|---|
| Camera / lens | Position/yaw/pitch, film-back + focal length → derived vertical FOV, feeding pinhole primary rays directly — the geometric ground truth the ray/BSDF math is measured against |
| Photographic exposure | EV100 from aperture/shutter/ISO, applied as a relative-stops delta against profile.json's default triple (Filament/Frostbite formula) — a familiar brightness control, not an absolute photometric quantity |
| OpenEXR linear pipeline | `HdrImage`/`loadExr`; all shading/compositing in linear light, OCIO display-encodes only at the final blit — a precondition for correct PBR colour math |
| Display transform | OCIO Display/View API (sRGB, Rec.709, Raw), cycled at runtime ('L') — a colourimetric encode only, no tone mapping, so values above 1.0 clip honestly rather than being masked by a hidden filmic shoulder |

### Scene loading & geometry

| Feature | Mechanism |
|---|---|
| glTF loading | cgltf; per-primitive vertices baked to world-space triangles/shading data at load time, materials' textures decoded once to `HdrImage` — nothing GPU-resident is needed once the CPU Embree scene exists |
| Tangent-space normal mapping | Per-vertex tangent (glTF-supplied only), Gram-Schmidt re-orthogonalized per-ray, for surface micro-detail without extra geometry |
| Ray acceleration | Intel Embree (SIMD BVH build/traversal), CPU, built once at load — sub-linear ray-scene intersection, required before recursion is affordable |
| Primary-hit rasterizer | Watertight CPU edge-function rasterization (Pineda 1988, incremental along each row): view-frustum clip (Sutherland-Hodgman 1974, Blinn-Newell 1978 outcodes) with canonical-order intersections, fixed-point snapping at the most sub-pixel bits that keep int64 edge functions exact, top-left fill rule (D3D11.3 functional spec; Giesen 2013), row-parallel, synchronous every frame, giving instant primary-hit G-buffer AOVs (§4) decoupled from Beauty's progressive convergence |

### Materials & lighting

| Feature | Mechanism |
|---|---|
| Stochastic BSDF | EON rough-diffuse (Portsmouth, Kutz, Hill 2025), GGX microfacet specular, Walter 2007 rough dielectric transmission (delta Snell + TIR below the smooth-roughness threshold, and at `ior` 1 at every roughness), Kulla-Conty multiple-scattering compensation on both interfaces; four-lobe stochastic selection, one-sample mixture estimator — real physical response, energy-conserving at every roughness |
| Volumetric absorption | Beer-Lambert extinction (`transmissionColor` over `transmissionDepth`, Arnold/OpenPBR convention) on a single-level medium stack inside a `transmissionFactor>0` material (a constant on-surface tint at `transmissionDepth 0`) — tinted glass, thick or thin, without participating-media in-scattering (§5 Large #1) |
| Per-object materials | `SceneConfig::materialOverrides` (glTF node name → `materials/*.json` path) builds a per-instance settings vector, indexed by `ShadingTriangle::instanceIndex` through both render paths — different objects in one scene can carry different materials |
| Area lights | Rectangular emitters (`scene.json`'s `lights`), one-sided by default, with their own geometry in the BVH and solid-angle NEE sampling (Ureña, Fajardo & King 2013), MIS'd against BSDF sampling like the environment — the classic emissive-panel Cornell box (§5 Large #2), and ReSTIR's (§5) prerequisite light set |
| Environment lighting | Equirect HDR map, BSDF-sampled misses + luminance-importance-sampled NEE, MIS-combined — image-based lighting, one member of `LightSet` alongside any area lights (still no punctual/directional lights, which have no hittable geometry) |
| Environment-light toggle | HUD "Environment Light" checkbox (`environment.lightEnabled` in `scene.json`, `--env-light` on `render_beauty`) — removes the environment from NEE/MIS/every miss including the background, unlike "Show/Hide Background" which only hides the camera-visible sky; lets an HDRI+area-light scene isolate the panel-only look |
| Russian roulette | Survival probability clamped from running throughput past `russianRouletteStartBounce`, reweighted by `1/p` — keeps recursion finite without biasing the estimator |
| Progressive accumulation | Each background pass re-traces at the current camera/settings and averages into the displayed result, restarting on any camera/scene change — real-time-interactive without waiting for one long render to finish |

### Debug tooling & telemetry

| Feature | Mechanism |
|---|---|
| Startup spec block | One plain-text provenance block on stdout: GPU/driver/refresh rate, host CPU topology and cache line from `sysctl`, compiler/build type/`-march`/IPO/git SHA, runtime-queried library versions, and the scene's load/BVH-build cost — confirms the actual GPU/backend before a wrong-adapter bug masquerades as a render bug, and makes every timing number attributable |
| Render telemetry (`-stats`) | 78-column terminal dashboard redrawn in place at 3 Hz via a single `write(2)`: render-thread stages with share-of-frame bars, path-trace phases, ray counts by type with Mray/s, and a `cpu total` / `frame measured` / `unaccounted` reconciliation — shows where every millisecond goes, and what it can't account for |
| Frame pacing | `DisplayLink` (`display_link.mm`): `-[NSView displayLinkWithTarget:selector:]` on a user-interactive-QoS thread wakes the render loop once per vblank of the window's own display, at swap interval 0 with a one-frame GPU fence -- NSGL's swap interval lets two swaps through per refresh on current macOS and GLFW substitutes a fixed 60 Hz `usleep` while occluded, so neither is used. A minimised window, whose link stops ticking, free-runs on the display's period grid. The measured period is the refresh rate every consumer reports. `swap_ms` still has a tail outside the engine: NSOpenGL's flush makes a synchronous WindowServer query (`SLSFlushSurfaceWithOptionsAndIndex` -> `_CGSWindowIsOrderedIn`), and in 7 visible convergences (88k frames) 808 of the 827 1 ms samples that found the render thread blocked inside a swap over a quarter period were in it, and none found it runnable-waiting for a core; swaps over half a period were 96-98% off-CPU, unchanged with the trace and driver threads at utility QoS (13 vs 9, P = 0.52) and over-represented right after display-texture uploads (7 vs 1.3 expected). A per-frame maximum of `swap_ms` or `frame_ms` therefore measures WindowServer, not the engine |
| Frame-timing HUD | Ring buffer of recent frame times; rolling FPS/avg/min/max, GPU timer query around the post-process blit — makes blit cost measurable frame to frame |
| Memory HUD | Live RAM readout plus GPU allocation tracked at alloc/free (the path-traced display texture is the only GPU allocation left) — surfaces a memory regression immediately, not after VRAM exhaustion |
| Scene stats | Object/triangle/point counts, viewport resolution — a scene-complexity readout |
| Debug camera controls | WASD/QE fly, R reset, LMB-drag orbit around a pivot read from the path tracer's own G-buffer (world-space hit position + hit mask at its centre pixel) — interactive navigation without hand-editing camera parameters |
| Camera framing overlays | Centre crosshair, always on, drawn on the foreground overlay — a composition aid that never contaminates the AOV buffers being debugged |
| AOV selector | Dropdown across the full AOV set (§4), plus R/G/B channel-isolation hotkeys, to isolate one signal at a time |
| Live histogram | Per-channel (R/G/B) histogram of the currently displayed image — catches exposure/clipping and colour-space bugs a single still frame can hide |
| Benchmark log | Append-only JSON Lines, one record per timing run (`bench_log.h`, after Arnold's `stats_file` append mode): build-time git SHA plus the binary's Mach-O `LC_UUID`, host topology, argv, the resolved config, raw per-iteration samples, rusage, and a CRC of the output. `bench_compare` runs randomized multiple interleaved trials (Abedi & Brecht 2017) and reports the Hodges-Lehmann ratio with its exact Wilcoxon interval — makes a single-change timing claim resolvable when run-to-run drift exceeds the effect |

## 3. Material library

Named presets live in `assets/materials/*.json`, parsed into `MaterialConfig` (`scene_config.h`). Each scene picks one as its default via `SceneConfig::materialPath`, with optional per-object overrides via `SceneConfig::materialOverrides` (glTF node name → material JSON path) — e.g. `assets/scenes/cornell.json`'s `{"sphere01": "materials/chrome.json", "sphere02": "materials/glass.json"}`, everything else left on the scene default. Overrides build a per-instance settings vector, indexed by `ShadingTriangle::instanceIndex` through both render paths (§2 Per-object materials).

Add a new material by dropping a JSON file in `assets/materials/` and pointing `materialPath`/`materialOverrides` at it — no code or schema change needed.

### Shipped presets

| File | metallic | transmission | roughness (factor / min) | Notes |
|---|---|---|---|---|
| `principled.json` | 0.0 | 0.0 | 1.0 / 0.045 | The tree scene's material (`tree.json`); rough dielectric, heavy bump (`bumpStrength: 10.0`). The only preset that declares every field, including the otherwise-optional `transmissionColor`/`transmissionDepth`/`edgeTint` |
| `clay.json` | 0.0 | 0.0 | 0.5 / 0.045 | Neutral matte dielectric, no bump |
| `chrome.json` | 1.0 | 0.0 | 0.05 / 0.045 | Idealised near-white mirror: `diffuseColour` (which at `metallic=1` *is* `f0`, Gulbrandsen's reflectivity `r`) is `[0.95, 0.95, 0.97]` with a white `edgeTint` — **no reflectance dip**, the one edge tint at which the conductor reproduces Schlick's grazing behaviour, so this preset exercises the transport rather than the parameterisation. A measured metal is what the `(r, edgeTint)` basis exists for: chromium's Johnson & Christy 1974 triples are recorded at `conductorIorFromReflectivity` (`bsdf.cpp`) and can be pasted straight back into this file |
| `glass.json` | 0.0 | 1.0 | 0.02 / 0.01 | Schott N-BK7 crown glass, `ior: 1.5168` at the d line with `abbe: 64.17` for dispersion; tinted via Beer-Lambert `transmissionColor: [0.96, 0.98, 1.0]` over `transmissionDepth: 0.4` world units — `diffuseColour` cannot tint transmission at all (see below) |

### `MaterialConfig` fields

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
| `transmissionColor` / `transmissionDepth` | The **only** tint on transmitted light (Arnold `standard_surface`/OpenPBR convention). `transmissionDepth > 0`: interior-medium Beer-Lambert absorption, `sigmaA = -log(transmissionColor)/transmissionDepth`, applied while a path is inside a `transmissionFactor>0` material. `transmissionDepth 0`: no interior medium — a constant on-surface tint applied once per crossing, so a closed solid reads its square. Default `[1,1,1]`/`0.0` is Arnold/OpenPBR's own no-op default in either regime (§2 Volumetric absorption) |
| `edgeTint` | Gulbrandsen 2014 edge tint for the conductor lobe, `metallicFactor>0` only. Default `[1,1,1]`: white is the no-dip edge Schlick always produced |

## 4. AOV reference

The four Direct/Indirect Diffuse/Specular buckets key on the **sampling strategy** a bounce drew from (`LobeType`, `path_tracer.cpp`), not on the surface's material class. A rough surface's Kulla-Conty multiple-scattering energy is therefore Specular in both senses that matter -- it is repeated scattering on the GGX microsurface, and since it has its own `(1-E)cos` strategy (`msReflect`, `bsdf.cpp`) it is drawn as such. A conductor contributes to the Diffuse buckets not at all, having no diffuse lobe.

Every AOV below is computed by the path tracer each pass, except: the 14 primary-hit-only AOVs (Alpha, Depth, WorldPos, UV, Normal, GeomNormal, Albedo, Metallic, Roughness, Tangent, ObjectID, Fresnel, IOR, Wireframe), which come from the synchronous CPU rasterizer (§1, §2) instead, refreshed every frame; and HSV/Luminance/Sobel/Gabor, GPU post-filters of the Beauty image (shared `PostProcessPass`, re-run every displayed frame over the completed texture -- not cached across frames, see §5 roadmap Parked).

### Utility

| AOV | Mechanism |
|---|---|
| Beauty | Final accumulated radiance, post tone-mapping — the primary output |
| Wireframe | Screen-space line rasterization (Pineda 1988), z-tested against the scene's own depth: white mesh-triangle edges, plus one bounding box per instance in that instance's `falseColorForId` hue, the same hue ObjectID gives it (drawn on top, so the box wins) — visualizes triangle density/topology and each object's extent/placement in one view |
| Alpha | 1.0 on a primary hit, 0.0 on a primary miss — a real coverage mask (this renderer isn't opaque-only-by-construction) |
| Depth | Planar camera-space Z (Arnold/RenderMan/EXR "Z" convention) at the primary hit, for depth-based compositing/debugging |
| HSV | Colour-space transform of Beauty — isolates hue/saturation shifts a pure RGB view can hide |
| Luminance | Rec.709 luminance of Beauty — isolates perceived brightness from colour |
| Sobel | 3×3 Sobel gradient magnitude of Luminance — a cheap edge/gradient signal |
| Gabor | 4-orientation Gabor kernel bank, max response, of Luminance — directional edge/texture response Sobel's isotropic magnitude can't distinguish |
| WorldPos | Raw world-space primary-hit position, for debugging geometry/UV placement independent of shading |
| UV | Primary-hit interpolated UV (fractional part) — visualizes the texture-space mapping directly |

### Material

| AOV | Mechanism |
|---|---|
| Normal | Shading (normal-mapped) normal at the primary hit — the normal actually used in shading |
| GeomNormal | Smooth interpolated vertex normal, before normal-mapping — separates a bad normal map from a bad base mesh |
| Albedo | Base-colour texture sample at the primary hit — isolates texture data from lighting |
| Metallic | Per-instance metallic factor (`settings.metallicFactor`, `materials/*.json` or `SceneConfig::materialOverrides`), uniform within one object's triangles but no longer whole-image-constant now that per-object material assignment exists — debugs which instance carries which metallic value |
| Roughness | Roughness texture × a per-instance factor, floored at that material's own `roughnessMin` (materials can set their own floor, e.g. glass's below diffuse/chrome's shared 0.045); texture varies per hit, factor/floor vary per instance — debugs material authoring independent of shading |
| Tangent | Shading tangent basis at the primary hit — debugs the tangent-space basis used for normal mapping |
| ObjectID | Per-instance index, false-coloured (`falseColorForId`) — an isolation mask for compositing/debugging |
| AO | Cosine-weighted obscurance (Zhukov et al. 1998; Iones et al. 2003), the distance-weighted generalisation of ambient occlusion (Miller 1994; Landis 2002): one hemisphere ray per sample bounded by `aoMaxDistance` (`profile.json`), each hit weighted `1 - (1 - t/aoMaxDistance)^2` so occlusion grades with proximity and reaches full visibility smoothly at the bound; 1.0 = unoccluded, the opposite polarity to Shadow — reads contact/corner darkening off the actual geometry, independent of material and lighting |

### Transport

| AOV | Mechanism |
|---|---|
| Fresnel | `mix(exact dielectric Fresnel, exact complex-IOR conductor Fresnel, metallic)` at the primary hit's view angle — the same term shading evaluates (`fresnelAtViewAngle`, `bsdf.h`), against the macro normal rather than a microfacet half-vector; debugs grazing-angle reflectance in isolation, including the conductor dip an authored `edgeTint` produces |
| IOR | Per-instance dielectric IOR (`settings.ior`), -1 on a miss — isolates the raw refractive-index input driving Fresnel/transmission |
| BounceCount | Mean path termination depth across samples, per pixel — debugs Russian roulette/termination behaviour |

### Lighting

| AOV | Mechanism |
|---|---|
| DirectDiffuse | Diffuse-bucketed radiance from a path's first (bounce-0) surface, physical (base colour included) — isolates direct diffuse light arrival, in the same units as Beauty |
| IndirectDiffuse | Diffuse-bucketed radiance from later bounces — isolates indirect (bounced) diffuse contribution |
| DirectSpecular | Specular-reflection-bucketed radiance, one bounce from camera — isolates direct specular contribution |
| IndirectSpecular | Specular-reflection-bucketed radiance, later bounces — isolates indirect specular (reflections) |
| Refraction | Radiance from any path that sampled a transmission lobe (sticky bucket) — isolates glass/transmissive transport |
| Shadow | Binary NEE occlusion test toward the sampled light (environment or an area light, per `LightSet`'s uniform selection) at the primary hit, re-averaged across progressive passes into continuous shadow/penumbra density — isolates direct-light visibility from material/lighting colour |

## 5. Roadmap

Execution order by waves, each wave enabling/measuring the next. **Large**, the closing section, is a strict dependency chain reflecting transport order rather than priority. Items within each wave have no hard blocker on one another; a wave ships once all its items pass their per-item verification (measure, never infer).

### Wave 1: Transmissive multi-scatter lobe (one workstream, same code in `bsdf.cpp`)

- **The transmit lobe's angular distribution is discontinuous across `eta = 1`** (distribution only): `transmissionIsRough` (`bsdf.cpp`) routes an index-matched interface to the delta branch at every roughness, PBRT-v4's own `eta == 1 || EffectivelySmooth()`, which is what keeps Walter's half-vector `normalize(wo + eta*wi)` off the zero vector it forms there. The tabulated escape deficit either side of that point does not tend to zero to meet it: measured 0.20865 at `eta` 0.999 and 0.21014 at 1.001 at roughness 1, against exactly 0 at 1.
  - So within an arbitrarily small neighbourhood of index match, a fifth of the transmitted energy jumps between being spread over the whole far hemisphere by `kMsTransmitDensity` and being a delta at `-wo`. The total is 1 on both sides, so this is a distribution discontinuity and not an energy one -- which is exactly why no furnace row can see it, and why `transmissive_slab_energy` and `transmissive_sphere_energy` stay green across it.
  - Not a transcription error: the deficit is the single-scatter GGX lobe's own Smith G2 masking loss, which the microsurface model reports even where the facets deviate the ray by nothing at all. That is a known property of microfacet BTDFs approaching index match, and PBRT's exact-equality branch is the standard pragmatic handling of it rather than a fix for it.
  - Not Quick, and possibly not worth doing: the alternative is a compensation shape that collapses toward the refraction direction as `eta -> 1`, which is a change to the rough-transmission lobe itself. Measure the reachable band first -- nothing shipped sits near index match (`glass.json` is 1.5168), so this may be a region no authored material occupies.

**Separately from Wave 1 (isolated, moderate):**
- **`dielectricFresnelAvg`'s rational fit for the coat**: `checkCoatFresnelAvg` (`tools/bsdf_validate.cpp`) recovers the `F_avg` the coat actually used -- by dividing the diffuse channel by `referenceEon` and inverting the coupling -- and lands 6.6e-5 from `dielectricFresnelAvg`'s own value. What the check then spends its tolerance on is not the instrument but the fit: 0.00228 at ior 1.5 falling to 0.00149 at 1.55, against Karis' 0.00606 and 0.00714.
  - So the open item is a better `F_avg` function, not a better instrument. The fit is two constants over `ior` in [1.1, 3.0] with worst error 0.00597; it is also not uniformly better than the Schlick mean it replaced -- it is worse below `ior` ~1.42 (0.00544 vs 0.00269 at 1.3) and only wins above, which is why the check sweeps 1.5-1.8 and why `clay.json`'s 1.55 and `glass.json`'s 1.5168 are the values that matter.
  - This became measurable only when the albedo table moved offline. `F_avg` reaches `coatAlbedo` through `multiScatterTint(F_avg, Eavg)*(1-E(mu))`, so an error `e` in `E` recovers as `e/0.054` in `F_avg`: at the old 32x32 startup table's ~1.5e-3 that is 0.028, eight times the tolerance the check now runs at.

### Wave 2: CIE 1931 module (one enabler, four consumers)

Build the CMF tables and primary-set handling once, then apply to three pending items now, plus one later:

- **CIE-integrated fit for the chromium reflectivity/edge-tint triples**: the measured chromium at `conductorIorFromReflectivity` (`bsdf.cpp`) -- reflectivity `[0.552, 0.555, 0.558]`, `edgeTint` `[0.555, 0.558, 0.672]`, from Johnson & Christy 1974 via Gulbrandsen eq 14/15 -- is sampled at three representative wavelengths rather than integrated against the CIE colour-matching functions, which is what an RGB channel actually is. Its 0.0048 round-trip error against the source `(n, k)`, and its sensitivity to the chosen wavelength triple, are both documented there. Proper integration needs the CIE curves and a chosen RGB primary set, giving every future measured metal one principled path from tabulated `(n, k)` to an authored `(r, g)` pair instead of a hand-picked triple.

- **Kelvin light temperature** (Wave 2 consumer): a quad light's emission is authored as a raw `color` x `intensity` RGB triple (`QuadLightConfig`, `scene_config.h`), no colour-temperature input anywhere; the environment carries whatever white balance its HDRI was captured at. Unblocked once Wave 2's CIE module ships: a Planckian-locus-to-RGB conversion (CIE 1931 colour-matching functions) so a light can be authored by Kelvin rather than a hand-picked triple.
- **Photometric calibration** (Wave 2 consumer): tie radiometric output to real photometric units (lux/candela/lumen) so `ev100()` (`camera.h:54-56`) and light intensities can be checked against a light meter instead of eyeballed. The luminous efficiency is the CIE ȳ curve. Complements the Macbeth chart scene.
- **Later, once the transport groundwork lands**: Large #5a's stochastic per-path wavelength draw (below) is this module's fourth consumer.

### Wave 3: Non-analytic validation infrastructure

Validation infrastructure needed for large/complex changes that have no closed-form reference. Place this directly before Large #1 since volumetric SSS has no closed forms either.

- **Occlusion-sensitive curved transmissive test**: `checkTransmissiveSphere` (`tools/integrator_validate.cpp:443`) asserts a white non-absorbing glass sphere is invisible under a uniform environment.
  - Closes one of two curvature-driven blind spots: reverting the shadow-terminator projection's side reads 0.955/0.877 there while the flat slab stays bit-identical. Cannot close the other -- a furnace measures how much energy arrives, not where from, so it can't see a light *leak*.
  - Reverting either curvature-scaled epsilon (`transmissionOffsetEpsilon`, `path_tracer.cpp:43`, used at the far-side NEE shadow ray `path_tracer.cpp:265` and the transmission continuation ray `path_tracer.cpp:335`) leaves the whole suite green: measured deltas 2e-4 and 4e-3 at 64x32 and again at cornell's own 24x12 tessellation -- not a tessellation artefact.
  - Needs a configuration where the far-side shadow ray *ought* to be occluded (rough glass against an opaque backdrop). Not Quick: that has no closed-form answer, so it needs a converged-reference or two-estimator invariant rather than an analytic one -- every transmissive assertion in the suite today is analytic.
- **Band calibration tool**: derived bands are verified by mutation (each converted band detects a smaller error than the one it replaced), but not yet by a standing false-rejection-rate measurement over many seeds.
- **Histogram coverage**: `debug/histogram.cpp` is FBO/PBO-bound with no CPU-reachable binning function, so it has no validator. Needs the bin arithmetic extracted first.

### Wave 4: Relax `aovNeedsLightTransport` gating (one enabler, two consumers)

Unblocks the path-traced Fresnel AOV and AOV-switch restarts (the last one measured via the benchmark log, §2); the contact sheet is a third, parked.

- **Reduce AOV-switch restarts** (measured via the benchmark log, §2): `aovNeedsLightTransport` (`main.cpp:100-120`) plus the trigger-state comparison (`main.cpp:942-958`) force a full progressive-accumulation restart on every AOV switch that changes producer (rasterizer vs. path tracer), and even between two light-transport AOVs, since there's no mechanism to add a new accumulator bucket onto an already-converged mean today -- full restart or nothing. Measure before committing to the added bookkeeping: `engine -bench` captures one fixed convergence today; this item adds a scripted AOV-switch schedule to that mode and compares time-to-converge with `bench_compare run`.
- **Path-traced Fresnel AOV** (Wave 4 consumer): today's `Fresnel` AOV (`aov.h:29`) is rasterizer-only -- a single primary-hit sample against the macro normal (§4), not progressive. A path-traced version needs a new accumulator lane in `PathTraceResult` (`path_tracer.h:48-64`) and the same gating relaxation as the item above: `aovNeedsLightTransport` (`main.cpp:100-120`) keeps the rasterizer and path tracer mutually exclusive per frame, so neither can freshen a buffer the other owns.

### Wave 5: Performance, each item justified by the benchmark log (§2)

- **Blue-noise sample matrix beyond d = 1**: the dither mask (`bluenoise_mask.cpp`) is a scalar void-and-cluster array, so a pixel's d-dimensional toroidal shift is one value replicated along the diagonal of the d-torus. Relative shifts between pixels therefore lie on a line rather than filling the torus. Georgiev & Fajardo's Sec. 3 anneals a true d-vector-per-element matrix against their energy function; adopting it changes only the baked table and its lookup, not the sampler.
- **Display-texture upload rebuilds the whole image every published pass** (measured via the benchmark log, §2): `ensurePathTraceDisplayTexture` (`main.cpp`) re-uploads the full RGBA32F buffer whenever the cache key changes -- 37.7 MB at 2048x1152, measured at **3.6-7.4 ms, median 4.6 ms** on the render thread over the 128 uploads of one `engine -bench` convergence, firing once per completed pass. Three independent options, none of them tried: RGBA16F for the display copy (the texture is already `GL_RGBA16F`, `texture.cpp:45`, so the 32-bit staging copy is converted on the way in anyway), a PBO so the copy leaves the render thread, or uploading only the tiles the pass actually rewrote. Decide each with `bench_compare run --metric upload_ms` over `engine -bench`.
- **Hierarchical frustum culling**: `clipToFrustum`'s outcodes already reject an out-of-view triangle, but only after `buildSubTriangles` has visited it -- the per-frame walk is still the whole scene. Rejecting per instance against `instanceBounds` first would skip it; the equivalent Embree traversal likewise. The rasterizer is double-sided by design, so backface culling does not apply.
- **Texture bit depth (16/32) via JSON**: hardcoded `GL_RGBA16F` today (`texture.cpp:45`); 32F ~doubles VRAM/buffer.

**Folded into Large #3, don't do on CPU:**
- **Packet tracing**: `EmbreeAccel` calls `rtcIntersect1`/`rtcOccluded1` exclusively (`embree_accel.cpp:116,144`), single-ray only -- no `rtcIntersect4/8/16` packet API.
- **Ray reordering before shading**: the tile loop traces in raster order (`path_tracer.cpp:307-320`), no Morton/direction-coherence sort ahead of `tracePath` (`path_tracer.cpp:77`).
- **Deferred/sorted shading by material**: `tracePath` (`path_tracer.cpp:77`) evaluates the BSDF inline per ray; no material-bucketed shading pass.

### Wave 6: Hygiene, after `bsdf.cpp` churn settles

- **Refactor the 11 over-length functions**: `readability-function-size` still fires on `tracePath`, `renderPathTraced`, `sampleBsdf`, `computeLobeProbabilities`, `driverLoop`, `buildSphericalRectangle`, `loadProfileConfig`, and four in `main.cpp`. Deliberately not bundled with the test work: these are hot-path functions, and splitting them risks the image. Needs its own before/after verification, via the bit-identical stash/revert method. Skip `tracePath`/`renderPathTraced` if Large #3 (GPU backend rewrite) is near -- no point refactoring code about to be replaced.
- **Code-quality audit**: `rotateAboutY` (`environment_map.cpp:13`) → `glm::rotate`; `ShadingFrame::toLocal`/`toWorld` (`bsdf.h:26`) → `glm::mat3`. (BSDF math in `bsdf.cpp` -- GGX/Smith/Fresnel/VNDF -- is standard domain logic, not an offload candidate.)
- **Documentation pass**: three parts -- a user/build guide beyond this README's Build section; an architecture/API reference for the `src/`/`include/engine/` module layout; write-ups of the physically-based techniques in use (BSDF model, sampling, GI), separate from inline code comments.
- **Memory efficiency pass**: only monitoring exists today (Memory HUD, §2), no active reduction initiative. Candidates once profiled: `gltf_loader.cpp`'s de-indexed mesh soup, which stores every position twice (once in `Triangle` for Embree, once in `ShadingVertex` for shading), and texture bit depth (Wave 5, above).

### Camera (not yet sequenced by wave)

- **Depth of field**: thin-lens sampling in `primaryRay` + focus distance; technically unblocked today, cheaper once adaptive sampling lands (Parked).
- **Motion blur**: blocked on the scene-graph/animation foundation the engine does not have yet + Embree multi-timestep geometry.

### Testing infrastructure not yet assigned to a wave

Every validator runs on a shared harness (`tools/check.h`): each check is registered by name and discovered into `ctest` as its own entry (`<suite>.<check>`), labelled by speed (`fast`/`slow`) and kind (`exact`/`statistical`). `ctest -L fast -L exact` is the sub-second pre-commit gate; the full suite is the pre-merge gate. Monte Carlo bands are derived from the run's own variance over independent scramble seeds at one family-wise significance level (`tools/stats.h`), not hand-picked. `render_beauty --assert-deterministic` and `--assert-converged` gate the shipping pipeline with no golden image.

(Band calibration tool and Histogram coverage moved to Wave 3, above; the `-bench` frame-pacing and driver-nondeterminism instrument gaps closed Wave 0, CHANGELOG.)

### Parked (low value, or needs a use-case first)

- **Contact sheet export (grid of every AOV)** (Wave 4 consumer): tile thumbnails of all 27 `AovId` (`aov.h:7-40`) at once, vs. the HUD's single `ImGui::Combo` (`hud_overlay.cpp:336`) feeding one `pathTraceDisplayTexture` blit (`presentFrame`, `main.cpp:739`). Blocked on the rasterizer/path-tracer mutual exclusion: `aovNeedsLightTransport` (`main.cpp:93-108`) splits the 27 into 13 light-transport / 14 rasterizer AOVs, only one side fresh per frame -- needs that gating relaxed, not just N reads of one cached buffer.
- **Render-mode selector**: Single Sample / Progressive. Today the path tracer dispatches fixed 96x96 tiles and the rasterizer rows, both via `ThreadPool`; neither adapts to residual noise, and the mode isn't selectable, define in profile.json.
- **Adaptive per-pixel sample budget**: variance-driven, builds on tiling above; `samplesPerPixel` (`profile.json`) is one fixed global today, no per-pixel allocation.
- **Texture minification filtering (MIP-mapping)**: point/bilinear only today (`sampleBilinear`, `hdr_image.h:22`); grazing/distant surfaces alias. No mip chain exists; needs ray differentials to pick a level per ray.
- **High-frequency binary noise texture/material**: none exists yet. Add as a stress test for the Texture minification filtering item above -- high-frequency content exposes aliasing before/after mip-mapping lands, since sampling is point/bilinear only today (`sampleBilinear`, `hdr_image.h:22`).
- **Reduce the HUD's build cost**: the `hud build` / `hud render` split (`-stats`) now attributes the stage, and the cost is **entirely widget construction in `HudOverlay::draw`**, not draw-list submission: 1.520 ms against 0.107 ms with the HUD shown, 0.000 ms against 0.021 ms with it hidden (cornell, 2048x1152, Apple M1). Any optimisation belongs in `draw`; `ImGui::Render`/`RenderDrawData` has nothing worth attacking.
  - This **disproves** the concern the split was built to test. `render` is unconditional by design (`main.cpp:223`, ImGui frame pairing), so a fixed cost there would have been paid with the HUD hidden -- but it is not fixed: with no widgets built there is no draw list to submit, and it falls to 0.021 ms. Hiding the HUD recovers essentially the whole stage.
  - Deprioritised rather than open: the recovered 1.5 ms goes to the vsync wait, not to frame rate. At 60 Hz the render thread is vsync-bound with 13.35 ms of headroom with the HUD shown (`-stats`, display-link paced), so this is slack being spent, not budget. It becomes worth doing when something else makes the render thread CPU-bound -- most likely the Depth auto-range scan below, which is 5.8x larger.
- **Depth's auto-range maximum is a serial per-texel render-thread scan**: `ensurePathTraceDisplayTexture` (`main.cpp:848-854`) walks the whole Depth image on the render thread to find its normalisation maximum -- the same shape as the over-range scan that has now moved to the driver, and the last one left. Narrower, though: it runs only on the display-texture cache-miss path and only while the Depth AOV is selected, so it is not on the Beauty path at all. Fixing it needs the range published from the producer the way `OverRangeStats` is, which for Depth is `RasterGBuffer`, not `PathTraceResult`.
- **PNG capture tool hardening**: `tools/render_beauty.cpp`'s `writePng` (lines 68-111) pipeline order is correct (exposure → OCIO display transform → dither → clamp → quantize, lines 198-234). Two gaps: RGB-only, no alpha (`colour type 2`, line 95); scanline filter hardcoded to type 0/None despite a comment claiming adaptive filtering (lines 70-74). Fix the comment or implement real adaptive filtering; add alpha output if a future consumer needs it.
- **Spherical Harmonics AOV**: no SH infrastructure exists anywhere in the engine -- environment lighting is a full equirect `HdrImage` plus a 2D luminance-importance-sampling CDF (`environment_map.h`), not an SH projection; the only trace of the idea is a speculative comment at `hdr_image.h:11`. This is new infrastructure (basis/projection code, coefficient storage, evaluation), not a small add -- needs a use-case writeup (fast irradiance preview? export for a downstream baked-lighting consumer?) before it's scoped enough to size.
- **Optic flow AOV**: per-pixel motion vectors, appended to `AovId` (`aov.h:7-40`).
  - No scene-graph animation yet, camera-only motion over static geometry -- this reprojects `WorldPos` (§4) through the previous frame's camera transform, not true motion capture.
  - `Camera` (`camera.h:10`) exposes only current `position()` (`camera.h:23`), no stored prior-frame matrix; needs one new persisted matrix, no new ray/sample work.
- **Bloom PostFX**: doesn't exist yet (zero references repo-wide). Chromatic aberration is the model to follow: a true post-process pass (`kAberrationGlsl`, `ocio_display_transform.cpp:41-51`) baked into the OCIO display shader, applied once per displayed frame over the final accumulated Beauty texture, Beauty-AOV-only. Implement Bloom as a bright-pass threshold + blur + additive composite in that same `PostProcessPass`/OCIO-shader-blit architecture, not a separate mechanism.
- **Cache the post-filter AOVs (Gabor/Sobel/HSV/Luminance) across unchanged frames**: these already run as true post-process passes over the completed Beauty texture (`isPostFilterAov`, `main.cpp:865-867`; `edge_filter.frag`/`hsv_display.frag`), not touching accumulation -- but `presentFrame` (`main.cpp:860`, called every frame from `renderFrame` at `main.cpp:1100`) re-runs the filter shader on every displayed frame regardless of whether Beauty actually changed since the last one (e.g. idle HUD interaction). Skip the re-run when the underlying texture hasn't changed; §4's AOV reference wording was corrected alongside this finding, since it previously read as if a cache already existed.
- **Example images of engine technology (gallery)**: no gallery of the renderer's own output in this README. Build small demo scenes each isolating one feature (a material, GI behaviour, transmission), render with `render_beauty`, embed a curated set here.

### Large: strict dependency order

1. **Volumetric & subsurface transport**: participating media (in-scattering, phase functions) + BSSRDF/random-walk subsurface.
   - Beer-Lambert *extinction* (no in-scattering) already ships for tinted glass (`transmissionColor`/`transmissionDepth`, `path_tracer.cpp`'s `mediumSigmaA`) -- a simpler subset of this item, not the item itself.
   - The multiple-scattering transmission lobe's energy-orientation bug (the transmitted-wi escape evaluated at the wrong eta) is fixed. What remains is structural, not buggy: that lobe's strict per-direction reciprocity (`f(wo→wi) ≠ f(wi→wo)`) is inherent to the transmissive multi-scatter formulation itself (`checkTransmissionReciprocity`, `tools/bsdf_validate.cpp`, deliberately scoped to single scatter) -- full bidirectional participating-media/subsurface transport is blocked on whatever that transport algorithm needs from this lobe, not on a fix already available.
2. **Global illumination**: area lights + shadow rays to them now ship -- `LightSet` (`light.h`) generalises NEE from the environment map alone to a uniformly-selected set of lights, and a rectangular emitter (`QuadLight`, solid-angle sampled via Ureña, Fajardo & King 2013's spherical-rectangle parametrisation, edges validated perpendicular at scene load) is the first light type, its two triangles injected into the Embree scene so it occludes and is BSDF-hittable. The classic Goral et al. 1984 emissive-panel Cornell box is reachable from `assets/scenes/cornell.json` with the HUD's "Environment Light" checkbox off (or `render_beauty --env-light 0`).
   - Still needed: ReSTIR (Bitterli et al. 2020) needs *multiple* area lights to resample across -- one light selected uniformly has nothing to resample; disk/sphere lights; emissive-mesh lights; power-weighted light selection.
   - Caustics do not fall out of this: unidirectional path tracing structurally cannot sample specular-diffuse-specular paths regardless of light count -- that needs (4).
3. **GPU ray-tracing backend**: Embree SYCL or CUDA-OptiX, to raise achievable sample budget beyond CPU Embree. Today's single-ray `rtcIntersect1`/`rtcOccluded1` calls and per-ray inline shading in `tracePath` (`path_tracer.cpp:77`) are the opposite of a wavefront/streaming GPU kernel design -- this item is that rearchitecture, not just a backend swap.
4. **Bidirectional path tracing with MIS (caustics)**: light-subpath/eye-subpath vertex connection (Veach & Guibas 1995; Veach 1997) -- the transport algorithm caustics need, since unidirectional path tracing (2) can't produce them at all. Blocked on (2) plus the transmissive multi-scatter lobe's structural non-reciprocity from (1): connection needs BSDF agreement in both directions, which that lobe doesn't provide by construction, not because of a bug.
5. **Spectral upgrade**: per-wavelength transport and hero-wavelength sampling (Wilkie et al. 2014). Dispersion already ships in RGB form (`MaterialConfig::abbe` → `bsdf.cpp`'s `cauchyIor`, one hero *channel* per path); what remains is the genuinely spectral part.
   - (a) `kRgbWavelengthsNm` (`bsdf.h:92`) is one fixed wavelength per channel -- OpenPBR's own reference implementation notes this produces "discrete RGB bands" rather than continuous rainbows, and its documented remedy is a stochastically drawn lambda per path. Not Quick: the draw needs spectral data this repo does not have.
     - "No transport change" holds *only* if lambda is drawn exactly proportional to the channel's spectral sensitivity `S_c(lambda)` -- the estimator weight `S_c/p` is then a constant, absorbed by normalisation. Draw from anything else and an `S_c/p` factor stays in the throughput: a genuine estimator change, and one that puts `checkTransmissiveSphere`'s dispersive per-channel unbiasedness gate (`integrator_validate.cpp:524`, `|Lo - 1| <= 0.03`) at risk.
     - Today's model is `S_c = delta(lambda - lambda_c)`, which is exactly the thing being replaced. Beyond it the repo holds no spectral data at all: six per-wavelength floats in total (`kRgbWavelengthsNm` plus the three Fraunhofer lines, `bsdf.cpp:402-404`), no colour-matching curves, no XYZ-to-RGB matrix.
     - So the honest version needs CIE 1931 tables and a chosen primary set, and must resolve that the resulting Rec.709 `r` sensitivity has a negative lobe -- unavoidable, since real primaries do not span the spectral locus, and not a density that can be sampled from. That same CIE module is the stated prerequisite of §5 Wave 2's chromium reflectivity/edge-tint item and Kelvin light temperature item, so build it once for all three.
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
- Veach, E. (1997). Robust Monte Carlo Methods for Light Transport Simulation. PhD thesis, Stanford: multiple importance sampling and its support condition (§9.2: every direction with value must carry density), which `bsdf_validate`'s `strategy_coverage` asserts exactly; next-event estimation, and the MIS-weighted vertex connection -- §5 Large #4, not yet implemented.
- Veach, E., Guibas, L.J. (1995). Bidirectional estimators for light transport. Eurographics Rendering Workshop: light-subpath/eye-subpath vertex connection -- §5 Large #4, not yet implemented.
- Pharr, M., Jakob, W., Humphreys, G. Physically Based Rendering: From Theory to Implementation (PBRT); source of `FrDielectric`, the exact unpolarised dielectric Fresnel formula the BSDF uses, of `TrowbridgeReitzDistribution::EffectivelySmooth`, and of the `eta == 1 || EffectivelySmooth()` test `transmissionIsRough` (`bsdf.cpp`) implements: PBRT-v4's `DielectricBxDF` routes value, pdf and sampler alike to the perfect-specular path at an index-matched interface at every roughness. An index-matched interface is not a rough interface, it is no interface -- refraction about any microfacet normal returns `-wo`, so Walter's half-vector `normalize(wo + eta*wi)` is handed the zero vector.
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
- Heitz, E. (2014). Understanding the Masking-Shadowing Function in Microfacet-Based BRDFs: the Smith height-correlated visibility term, evaluated as `bsdf.cpp`'s `smithVisibility` = G2/(4 cosO cosI) and `smithG1OverCos` in the form where the cosines multiply rather than divide (as Filament's `V_SmithGGXCorrelated`), exact to the silhouette with no floor.
- Christensen, P.H., Jarosz, W. (2016). The Path to Path-Traced Movies. Foundations and Trends in Computer Graphics and Vision: production path-tracing grounding.
- Heitz, E. (2018). Sampling the GGX Distribution of Visible Normals. JCGT 7(4): the VNDF importance-sampling routine the specular lobe uses.
- Guy, R., Agopian, M. (2018). Physically Based Rendering in Filament, sec. 4.4.2: the cancellation-free form of the Trowbridge-Reitz denominator `distributionGGX` evaluates. The textbook `ndotH^2*(alpha^2-1)+1` subtracts two near-equal numbers wherever the half-vector is near the normal, which at low roughness is the whole lobe; taking `sin^2` from the half-vector's own tangential components makes every term non-negative and the peak value `1/(pi*alpha^2)` exact. Measured 20% of `D` lost at the peak at `alpha=4e-4` on the textbook form in float32.
- Sobel filtering: edge-detection AOV computed from Luminance (§4), arXiv:2601.16806.
- Chiang, M.J.-Y., Li, Y., Burley, B. (2019). Taming the Shadow Terminator. JCGT 8(4): the shading-point correction used for secondary-ray origins.
- Cranley, R., Patterson, T.N.L. (1976). Randomization of number theoretic methods for multiple integration: the toroidal shift `Sampler` applies per pixel, which is the mechanism blue-noise dithered sampling drives from a mask rather than from a random offset. (Halton's radical inverse, the previous sampler's basis, is no longer used.)
- Arvo, J., Kirk, D. (1990). Particle Transport and Image Synthesis: Russian roulette path termination.
- Gulbrandsen, O. (2014). Artist Friendly Metallic Fresnel. JCGT 3(4): the conductor Fresnel parameterisation, ported from the paper's Appendix A. Reflectivity `r` (the existing `f0`) plus an authored `edgeTint` `g` invert to a complex IOR via eq 12 and eq 2, replacing Schlick, which is monotone in `cos` by construction and so forces every metal to exactly white at grazing. Two documented departures from the listing: `k^2` is evaluated in the factored form `(nMax-n)(n-nLow)` (the same expression through eq 2's own roots) because the literal form loses all precision in float32 near `r=1`, returning `-1.28e6` where the truth is 0; and the reflectance itself is the exact unpolarised complex-IOR Fresnel rather than the listing's large-|eta| approximation, which deviates from it by up to 0.094 absolute at mid reflectivity. The cosine mean `F_avg` the Kulla-Conty tint needs has no closed form on this basis and is a 3-node quadrature rule over the same Fresnel, fitted against 128-point Gauss-Legendre to a measured 4.0e-4 (`conductorFresnelAvg`, asserted by `checkAverageFresnel`); a rational fit over `(r, g)` was measured and rejected, since `n` collapses from 39.8 to 0.005 across the last tenth of `g` at `r=0.99` and 49 terms reached only 9e-3.
- Portsmouth, J., Kutz, P., Hill, S. (2025, **revised 2026-02-04**). EON: A Practical Energy-Preserving Rough Diffuse BRDF. JCGT 14(1): the rough-diffuse lobe, its analytic multiple-scattering compensation, and CLTC importance sampling, all ported from the paper's GLSL listings. The 2026 revision added Appendix A's albedo inversion, ported at `eonAlbedoInversion`: without it `rho` is the authored colour directly and the multiple-scattering term's saturation drags the *observed* albedo below it, measured at -12.2% for `diffuseColour 0.5` at `diffuseRoughness 1` and -20.6% on the darkest channel of `[0.8, 0.3, 0.1]`. The appendix gives two inversions and they cannot both hold; this pipeline takes the normal-incidence one (eq. 30/31), so `diffuseColour` means what OpenPBR says `base_color` means -- the observed reflection colour at normal incidence under uniform illumination -- a meaning OpenPBR declares but does not enforce, since it sets `rho = C` directly. Eq. 30's stated root is the unstable one, diverging as `r -> 0`, and the paper's remedy is a Taylor form switched in below some roughness; the conjugate-multiplied root is algebraically identical, needs no threshold, and recovers `rho = C` at `r = 0` from the algebra (Press et al., *Numerical Recipes* 5.6). The authors' `EON-diffuse` repository ships no inversion code, so this is a transcription of the equations rather than of a listing.
- Dupuy, J., Benyoub, A. (2023). Sampling Visible GGX Normals with Spherical Caps; Tokuyoshi, Y., Eto, K. (2023). Bounded VNDF Sampling for Smith-GGX Reflections: newer VNDF refinements surveyed, not implemented (Heitz 2018 used instead; better-established, lower risk to reproduce correctly from reference material alone).
- Heitz, E., Hanika, J., d'Eon, E., Dachsbacher, C. (2016). Multiple-scattering microfacet BSDFs with the Smith model. The source of the problem, not the solution used: it establishes the energy single-scatter GGX discards (a white conductor returned 0.31 of the light it received at roughness 1.0), but the implemented compensation is Kulla & Conty's cheaper directional-albedo form below rather than this paper's stochastic microsurface evaluation.
- Kulla, C., Conty, A. (2017). Revisiting Physically Based Shading at Imageworks. SIGGRAPH course. The multiple-scattering energy compensation the BSDF implements: a directional-albedo table drives a compensation lobe returning exactly the deficit the Smith G2 masks away, on the reflective and transmissive interface alike, with its own cosine sampling strategy on the transmit side.
- Dupuy, J., Jakob, W. (2018). An Adaptive Parameterization for Efficient Material Acquisition and Rendering. ACM TOG 37(6): tabulated BSDFs evaluated and sampled through one interpolant, so the density is exact and the value matches what is drawn -- the construction both transmissive multiple-scattering shares use (`kMsTransmitDensity`: value = energy x pdf / cos).
- Sobol, I.M. (1967). On the distribution of points in a cube and the approximate evaluation of integrals; Joe, S., Kuo, F.Y. (2008). Constructing Sobol Sequences with Better Two-Dimensional Projections. SIAM J. Sci. Comput. 30(5); Bratley, P., Fox, B.L. (1988). ACM Algorithm 659: the sequence, its direction numbers (`sobol_direction_seeds.inc`) and the recurrence that expands them.
- Burley, B. (2020). Practical Hash-based Owen Scrambling. JCGT 9(4): the hash-based Owen scramble, per-set index shuffling and padding `Sampler` implements. The scramble uses Vegdahl's improved Laine-Karras constants as shipped by Cycles rather than the paper's originals -- same construction, measurably better tree-permutation quality.
- Georgiev, I., Fajardo, M. (2016). Blue-noise Dithered Sampling. SIGGRAPH Talks: the per-pixel toroidal shift looked up in a blue-noise matrix tiled over the image, which distributes error into the high frequencies without changing its magnitude. Adopted for d = 1, where the paper's matrix "is identical to a dither mask"; the d-dimensional annealed matrix is the open item in §5.
- Ulichney, R.A. (1993). The void-and-cluster method for dither array generation. Proc. SPIE 1913: the dither mask's construction -- wrap-around Gaussian filter at sigma 1.5, the initial-binary-pattern generator, and the three ranking phases. Implemented in `tools/bluenoise_mask.cpp` and re-runnable, rather than a lifted binary tile whose provenance cannot be audited.
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
- Miller, G. (1994). Efficient algorithms for local and global accessibility shading. SIGGRAPH; Landis, H. (2002). Production-ready global illumination. SIGGRAPH Course Notes: the cosine-weighted, distance-bounded ambient occlusion the AO AOV path-traces (§4), whose pdf = cos/pi cancels both the 1/pi and the cosine so the estimator is the mean of the visibility term alone.
- Zhukov, S., Iones, A., Kronin, G. (1998). An ambient light illumination model. Eurographics Rendering Workshop; Iones, A., Krupkin, A., Sbert, M., Zhukov, S. (2003). Fast, realistic lighting for video games. IEEE CG&A; surveyed in Mendez-Feliu, A., Sbert, M. (2009). From obscurances to ambient occlusion: a survey. The Visual Computer: obscurance, which replaces AO's binary visibility with a distance falloff rho(t). The AO AOV uses rho(x) = 1 - (1-x)^2 over x = t/aoMaxDistance -- the lowest-degree polynomial with rho(0) = 0, rho(1) = 1 and rho'(1) = 0, the last being what removes the value and gradient steps a hard cutoff leaves at the bound. The cosine-weighted estimator is unchanged, so it stays the mean of the per-ray term; validated against the closed form of that integral in `tools/integrator_validate.cpp`'s `checkAmbientOcclusionAnalytic`.
- Xiao, L., Nouri, S., Chapman, M., Fix, A., Lanman, D., Kaplanyan, A. (2020). Neural supersampling for real-time rendering. SIGGRAPH: upscaling -- §5 Large #7, not yet implemented.
- Ho, J., Jain, A., Abbeel, P. (2020). Denoising diffusion probabilistic models. NeurIPS; Rombach, R., Blattmann, A., Lorenz, D., Esser, P., Ommer, B. (2022). High-resolution image synthesis with latent diffusion models. CVPR: diffusion-model foundations -- §5 Large #8, not yet implemented.
- Kalibera, T., Jones, R. (2013). Rigorous Benchmarking in Reasonable Time. ISMM: the process invocation, not the iteration, as the unit of replication -- why each log record keeps raw samples but `bench_compare` compares one per-invocation summary, each column's mean per event.
- Abedi, A., Brecht, T. (2017). Conducting Repeatable Experiments in Highly Variable Cloud Computing Environments. ICPE: randomized multiple interleaved trials, the design `bench_compare run` automates; Mytkowicz, T., Diwan, A., Hauswirth, M., Sweeney, P.F. (2009). Producing Wrong Data Without Doing Anything Obviously Wrong! ASPLOS: the fixed-order and environment bias that randomization removes.
- Apple (2023). `NSView.displayLink(target:selector:)`, AppKit, macOS 14; Apple, Energy Efficiency Guide for Mac Apps: Prioritize Work with Quality of Service Classes: the vblank-synchronised callback `DisplayLink` paces the render loop with, and the user-interactive QoS class for display-timed work. GLFW issues #1990/#2249 and PR #2277 record `NSOpenGLCPSwapInterval` no longer blocking once per refresh on current macOS, and `nsgl_context.m`'s fixed-60 Hz `usleep` for occluded windows.
- Hodges, J.L., Lehmann, E.L. (1963). Estimates of location based on rank tests. Ann. Math. Stat. 34(2); Hollander, M., Wolfe, D.A., Chicken, E. (2014). Nonparametric Statistical Methods, 3rd ed., Wiley, 3.2 and 4.3: the paired and two-sample shift estimators and their exact signed-rank/rank-sum intervals (`tools/stats.h`), applied on the log scale so a shift is a ratio (Fleming, P.J., Wallace, J.J. (1986). How not to lie with statistics. CACM 29(3)); Hoefler, T., Belli, R. (2015). Scientific Benchmarking of Parallel Computing Systems. SC: nonparametric intervals for performance data.
- Autodesk Arnold `options.stats_file`/`stats_mode`: one appended JSON stats record per render, the benchmark log's format precedent; LLVM `GenerateVersionFromVCS.cmake`: the build-time VCS stamp `cmake/GitSha.cmake` follows.
- Pineda, J. (1988). A parallel algorithm for polygon rasterization. SIGGRAPH: the edge-function incremental rasterization technique behind the primary-hit rasterizer (§1, §2, `rasterizer.cpp`).
- Sutherland, I.E., Hodgman, G.W. (1974). Reentrant polygon clipping. CACM: the view-frustum polygon clip the primary-hit rasterizer's triangle setup uses (`rasterizer.cpp`).
- Blinn, J.F., Newell, M.E. (1978). Clipping using homogeneous coordinates. SIGGRAPH: per-vertex outcodes, the trivial accept/reject ahead of the rasterizer's frustum clip (`clipToFrustum`).
- Microsoft. Direct3D 11.3 Functional Specification, §3.4 (coordinate snapping; triangle rasterization rules): fixed-point vertex snapping and the top-left fill rule the rasterizer's watertight coverage follows. Khronos, Vulkan specification, Rasterization chapter (`subPixelPrecisionBits`): the same snapping, with precision a device limit -- here derived per frame from the int64 exactness bound instead.
- Giesen, F. (2013). Triangle rasterization in practice; Optimizing the basic rasterizer. The ryg blog: integer edge functions, the top-left rule as a bias, and incremental row stepping, as implemented in `rasterizer.cpp`.
- Welford, B.P. (1962). Note on a method for calculating corrected sums of squares and products. Technometrics 4(3); West, D.H.D. (1979). Updating mean and variance estimates: an improved method. CACM 22(9): the incremental running mean `PathTraceDriver` publishes. Chan, T.F., Golub, G.H., LeVeque, R.J. (1983). Algorithms for computing the sample variance: analysis and recommendations. The American Statistician 37(3), and Higham, N.J. (2002). Accuracy and Stability of Numerical Algorithms, 2nd ed., SIAM, §1.9 and ch. 3 (the θ/γ rounding-error calculus): the per-element forward-error bound `driver.running_mean_matches_batch_mean` derives for it (`tools/driver_validate.cpp`).
