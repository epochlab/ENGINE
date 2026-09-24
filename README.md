# PBR Pathtracer

*A CPU, unidirectional Monte Carlo path tracer with real-time progressive display: Embree-accelerated, stochastic BSDF sampling combined with environment-map NEE via MIS, behind a thin OpenGL display/HUD layer.*

![Sample render](sample.png)


## Documentation

| | |
|---|---|
| [Pipeline](#pipeline) | How a frame is produced, below |
| [Components](#components) | Every subsystem, session settings, benchmark tooling |
| [Material Library](#material-library) | Shipped presets and every `MaterialConfig` field |
| [AOV](#aov) | All 28 debug outputs, by category |
| [References](#references) | The literature each technique implements |

## Build

C++20, built with CMake. Currently developed against macOS only.

```
brew install cmake glfw glew glm imath openexr opencolorio embree
git submodule update --init --recursive
cmake -B build
cmake --build build
```

`cmake --build build` builds everything: the `pathtracer` viewer, the `pathtracer_c` shared library the
Python binding loads, twelve validators that `ctest` discovers, four offline codegen/asset tools
(`albedo_table`, `bluenoise_mask`, `metal_fit`, `gltf_tangent`), two image utilities (`test_pattern`,
`downsample`) and three instruments (`render_beauty`, `raster_bench`, `bench_compare`). Name a target to
build just one.

Third-party dependencies and how they are vendored: [`third_party/README.md`](third_party/README.md).

### Static analysis

`-Wall -Wextra -Werror` everywhere, plus clang-tidy on the three shipping targets as a per-compile gate
(non-fatal, so pre-existing debt does not block a build) and cppcheck as its own target:

```
cmake --build build --target cppcheck
```

Both are no-ops if the tool is absent, so a fresh checkout without them still configures and builds.
Vendored sources are exempt from both. Suppressions are documented one-by-one with their reason in
`.cppcheck-suppressions`; anything not listed there is expected to be fixed rather than silenced.

## Run

```
./build/pathtracer [-scene path/to/scene.json] [-stats] [-bench log.jsonl]
```

## Python

Every AOV is reachable headlessly from Python as a numpy array. The renderer is a plain C ABI (`include/pathtracer/api/pathtracer_c.h`) loaded with `ctypes`; no build-time Python dependency and one library serves every interpreter.

```
cmake --build build --target pathtracer_c
pip install -e python
```

```python
from pathtracer import Renderer

renderer = Renderer("scenes/cornell.json")
frame = renderer.render(aovs=("beauty", "depth", "normal"), width=128, height=64, samples=8, seed=1)

frame["beauty"]   # (64, 128, 3) float32, linear Rec.709 radiance
frame["depth"]    # (64, 128, 1) float32, camera-space Z
frame["normal"]   # (64, 128, 3) float32, normal-mapped shading normal
```

Values are scene-referred linear and unclamped, with no display transform: what a model trains on, not what a monitor shows. Use `render_beauty --out` for a display-encoded picture. Arrays are C-contiguous float32, so `torch.from_numpy(...)` shares memory and leaves one explicit `.to(device)`.

Each producer runs at most once per call, so requesting several AOVs together costs far less than requesting them one at a time; a request with no path-traced AOV skips the integrator entirely. Override the camera with `dataclasses.replace(renderer.default_camera, ...)` — note that `aperture`/`shutter_seconds`/`iso` set exposure only, since the camera is a pinhole with no depth of field.

## Benchmark

Timing runs append one JSON Lines record each to a local log: `pathtracer -bench PATH`, `render_beauty --bench-log PATH`, `raster_bench --bench-log PATH`. A performance claim is made with a randomized interleaved A/B of two builds, not by comparing two runs by eye:

```
./build/bench_compare run --a buildA/render_beauty --b buildB/render_beauty --rounds 12 --log renders/bench.jsonl \
    -- --scene scenes/cornell.json --out renders/ab.png --passes 32 --width 640 --height 360 --bench-log renders/bench.jsonl
```

## Pipeline

**Startup.** glTF geometry/materials load once into a CPU-resident scene: per-vertex shading data (`ShadingTriangle`) and world-space triangles feed an Embree scene (`EmbreeAccel`); materials keep only CPU `HdrImage` textures, sampled per-ray. An equirectangular HDR environment map loads alongside it, with its own luminance-importance-sampling CDF for NEE. Area lights (`scene.json`'s `lights`) are injected into the same Embree scene at load (`appendQuadLights`), so they occlude and are camera/BSDF-hittable with no second intersection path.

**Per frame**, on any camera/scene-state change, two paths run independently off the same trigger:

- **Path tracer** — `PathTraceDriver` hands a fresh request to a background thread pool (one worker/core, row-parallel, dynamic scheduling), restarting progressive accumulation:
  1. Camera ray generation (pinhole) → Embree intersection (`rtcIntersect1`/`rtcOccluded1`)
  2. BSDF eval/sampling, combining five techniques:
     - Heitz 2018 GGX VNDF specular
     - EON rough-diffuse (Portsmouth/Kutz/Hill 2025)
     - Walter 2007 rough dielectric transmission with exact Fresnel/TIR, falling back to a Snell delta lobe below the smooth-roughness threshold and at an index-matched interface
     - Kulla-Conty multiple-scattering compensation on both interfaces
     - exact complex-IOR conductor Fresnel via Gulbrandsen 2014's reflectivity/edge-tint parameterisation
  3. NEE against a `LightSet` (environment map, optionally excluded via the HUD's "Environment Light" checkbox, plus any area lights — uniform selection, quads importance-sampled by solid angle via Ureña/Fajardo/King 2013's spherical-rectangle parametrisation), MIS-combined with BSDF sampling (power heuristic, Veach 1997)
  4. Recursive bounce loop with Russian roulette, Chiang/Li/Burley 2019 shadow-terminator-corrected secondary-ray origins, Beer-Lambert extinction inside transmissive media
  5. Radiance + full G-buffer/transport-component AOV set accumulated per pass, published lock-free for the render thread

- **Rasterizer** — a synchronous CPU pass (`rasterizer.cpp`) computes the 14 primary-hit-only AOVs (`aov.h`) every frame on the render thread: watertight edge-function rasterization (Pineda 1988) -- view-frustum Sutherland-Hodgman clipping, vertices snapped to a fixed-point grid whose precision is derived per frame for exact int64 edge functions, and the top-left fill rule, so triangles sharing an edge cover every pixel centre on it exactly once -- sharing `gbuffer_shading.h`'s material sampling with the path tracer but no Embree/BSDF/recursion. This gives those AOVs instant, glitch-free feedback during camera movement, decoupled from Beauty's own progressive convergence — the path-traced request above only restarts when the selected AOV needs light-transport data.

**Display.** The render thread blits whichever AOV is selected through OCIO's display transform (exposure/tone-mapping) and the debug HUD, converging over subsequent passes rather than blocking on one long render. No GPU rasterization anywhere: OpenGL exists only for the window, the post-process/OCIO blit, and ImGui; the primary-hit rasterizer is CPU-only.

# Components

## Camera & display

| Feature | Mechanism |
|---|---|
| Camera / lens | Position/yaw/pitch, film-back + focal length → derived vertical FOV, feeding pinhole primary rays directly — the geometric ground truth the ray/BSDF math is measured against |
| Photographic exposure | EV100 from aperture/shutter/ISO, applied as a relative-stops delta against profile.json's default triple (Filament/Frostbite formula) — a familiar brightness control, not an absolute photometric quantity |
| OpenEXR linear pipeline | `HdrImage`/`loadExr`; all shading/compositing in linear light, OCIO display-encodes only at the final blit — a precondition for correct PBR colour math |
| Display transform | OCIO Display/View API (sRGB, Rec.709, Raw), cycled at runtime ('L') — a colourimetric encode only, no tone mapping, so values above 1.0 clip honestly rather than being masked by a hidden filmic shoulder |

## Scene loading & geometry

| Feature | Mechanism |
|---|---|
| glTF loading | cgltf; per-primitive vertices baked to world-space triangles/shading data at load time, materials' textures decoded once to `HdrImage` — nothing GPU-resident is needed once the CPU Embree scene exists |
| Tangent-space normal mapping | Per-vertex tangent (glTF-supplied only), Gram-Schmidt re-orthogonalized per-ray, for surface micro-detail without extra geometry |
| Ray acceleration | Intel Embree (SIMD BVH build/traversal), CPU, built once at load — sub-linear ray-scene intersection, required before recursion is affordable |
| Primary-hit rasterizer | Watertight CPU edge-function rasterization (Pineda 1988, incremental along each row): view-frustum clip (Sutherland-Hodgman 1974, Blinn-Newell 1978 outcodes) with canonical-order intersections, fixed-point snapping at the most sub-pixel bits that keep int64 edge functions exact, top-left fill rule (D3D11.3 functional spec; Giesen 2013), row-parallel, synchronous every frame, giving instant primary-hit G-buffer AOVs (`aov.h`) decoupled from Beauty's progressive convergence |

## Materials & lighting

| Feature | Mechanism |
|---|---|
| Stochastic BSDF | EON rough-diffuse (Portsmouth, Kutz, Hill 2025), GGX microfacet specular, Walter 2007 rough dielectric transmission (delta Snell + TIR below the smooth-roughness threshold, and at `ior` 1 at every roughness), Kulla-Conty multiple-scattering compensation on both interfaces; four-lobe stochastic selection, one-sample mixture estimator — real physical response, energy-conserving at every roughness |
| Volumetric absorption | Beer-Lambert extinction (`transmissionColor` over `transmissionDepth`, Arnold/OpenPBR convention) on a single-level medium stack inside a `transmissionFactor>0` material (a constant on-surface tint at `transmissionDepth 0`) — tinted glass, thick or thin, without participating-media in-scattering ([roadmap](ROADMAP.md) transport #1) |
| Per-object materials | `SceneConfig::materialOverrides` (glTF node name → `materials/*.json` path) builds a per-instance settings vector, indexed by `ShadingTriangle::instanceIndex` through both render paths — different objects in one scene can carry different materials |
| Area lights | Rectangular emitters (`scene.json`'s `lights`), one-sided by default, with their own geometry in the BVH and solid-angle NEE sampling (Ureña, Fajardo & King 2013), MIS'd against BSDF sampling like the environment — the classic emissive-panel Cornell box ([roadmap](ROADMAP.md) transport #2), and ReSTIR's ([roadmap](ROADMAP.md)) prerequisite light set |
| Environment lighting | Equirect HDR map, BSDF-sampled misses + luminance-importance-sampled NEE, MIS-combined — image-based lighting, one member of `LightSet` alongside any area lights (still no punctual/directional lights, which have no hittable geometry) |
| Environment-light toggle | HUD "Environment Light" checkbox (`environment.lightEnabled` in `scene.json`, `--env-light` on `render_beauty`) — removes the environment from NEE/MIS/every miss including the background, unlike "Show/Hide Background" which only hides the camera-visible sky; lets an HDRI+area-light scene isolate the panel-only look |
| Russian roulette | Survival probability clamped from running throughput past `russianRouletteStartBounce`, reweighted by `1/p` — keeps recursion finite without biasing the estimator |
| Progressive accumulation | Each background pass re-traces at the current camera/settings and averages into the displayed result, restarting on any camera/scene change — real-time-interactive without waiting for one long render to finish |

## Debug tooling & telemetry

| Feature | Mechanism |
|---|---|
| Startup spec block | One plain-text provenance block on stdout: GPU/driver/refresh rate, host CPU topology and cache line from `sysctl`, compiler/build type/`-march`/IPO/git SHA, runtime-queried library versions, and the scene's load/BVH-build cost — confirms the actual GPU/backend before a wrong-adapter bug masquerades as a render bug, and makes every timing number attributable |
| Render telemetry (`-stats`) | 78-column terminal dashboard redrawn in place at 3 Hz via a single `write(2)`: render-thread stages with share-of-frame bars, path-trace phases, ray counts by type with Mray/s, and a `cpu total` / `frame measured` / `unaccounted` reconciliation — shows where every millisecond goes, and what it can't account for |
| Frame pacing | `DisplayLink` (`display_link.mm`): `-[NSView displayLinkWithTarget:selector:]` on a user-interactive-QoS thread wakes the render loop once per vblank of the window's own display, at swap interval 0 with a one-frame GPU fence. NSGL's swap interval lets two swaps through per refresh on current macOS, and GLFW substitutes a fixed 60 Hz `usleep` while occluded, so neither is used. A minimised window, whose link stops ticking, free-runs on the display's period grid. `render.vsync: false` (`profile.json`) skips the vblank wait and runs uncapped, still bounded to one frame in flight by the fence. The measured period is the refresh rate every consumer reports. `swap_ms` still has a tail outside the pathtracer: NSOpenGL's flush makes a synchronous WindowServer query (`SLSFlushSurfaceWithOptionsAndIndex` -> `_CGSWindowIsOrderedIn`). In 7 visible convergences (88k frames), 808 of the 827 1 ms samples that found the render thread blocked inside a swap over a quarter period were in it, and none found it runnable-waiting for a core. Swaps over half a period were 96-98% off-CPU, unchanged with the trace and driver threads at utility QoS (13 vs 9, P = 0.52) and over-represented right after display-texture uploads (7 vs 1.3 expected). A per-frame maximum of `swap_ms` or `frame_ms` therefore measures WindowServer, not the pathtracer |
| Frame-timing HUD | Ring buffer of recent frame times; rolling FPS/avg/min/max, GPU timer query around the post-process blit — makes blit cost measurable frame to frame |
| Memory HUD | Live RAM readout plus GPU allocation tracked at alloc/free (the path-traced display texture is the only GPU allocation left) — surfaces a memory regression immediately, not after VRAM exhaustion |
| Scene stats | Object/triangle/point counts, viewport resolution — a scene-complexity readout |
| Debug camera controls | WASD/QE fly, R reset, LMB-drag orbit around a pivot read from the path tracer's own G-buffer (world-space hit position + hit mask at its centre pixel) — interactive navigation without hand-editing camera parameters |
| Camera framing overlays | Centre crosshair, always on, drawn on the foreground overlay — a composition aid that never contaminates the AOV buffers being debugged |
| AOV selector | Dropdown across the full AOV set (`aov.h`), plus R/G/B channel-isolation hotkeys, to isolate one signal at a time |
| Live histogram | Per-channel (R/G/B) histogram of the currently displayed image — catches exposure/clipping and colour-space bugs a single still frame can hide |
| Benchmark log | Append-only JSON Lines, one record per timing run (`bench_log.h`, after Arnold's `stats_file` append mode): build-time git SHA plus the binary's Mach-O `LC_UUID`, host topology, argv, the resolved config, raw per-iteration samples, rusage, and a CRC of the output. `bench_compare` runs randomized multiple interleaved trials (Abedi & Brecht 2017) and reports the Hodges-Lehmann ratio with its exact Wilcoxon interval — makes a single-change timing claim resolvable when run-to-run drift exceeds the effect |

## Session settings (`profile.json`)

Session settings live in `assets/config/profile.json`.

`render.vsync` caps the frame rate to the display's vblank (`true`) or runs uncapped (`false`); uncapped, the render thread competes with the trace workers for cores, measured at **1.58x** `pass_ms` on cornell and **1.47x** on the stump (8 cores).

`render.textureBitDepth` sets the CPU storage of the scene's input images, the environment HDRI and every material texture: `16` (the shipped default) or `32` (float) (IEEE 754 binary16: unit roundoff 2^-11, finite maximum 65504; a source texel that overflows it becomes Inf and is rejected at load rather than silently clamped). The environment's importance-sampling CDFs are built from the stored values, so sampling stays proportional to the radiance returned at either depth. Every shipped EXR is half on disk, so `16` is lossless for them: renders are bit-identical, and on a local 4K-textured test asset (the stump) peak RSS drops 2103 -> 1281 MB, exactly the analytic 822 MB, and `pass_ms` is **0.978x** (half the bytes per texel fetch).

`render.displayBitDepth` sets the path-traced display texture's GL storage: `16` (`GL_RGBA16F`) or `32` (`GL_RGBA32F`, twice the memory and sampling bandwidth: `present_gpu_ms` **1.44x**, while `upload_ms` drops to 0.54x as the driver's float-to-half conversion disappears).

## Benchmark tooling

`pathtracer -bench` also takes `-bench-aovs "Beauty,Sobel,Direct Diffuse,Normal,Beauty"`, which walks that AOV sequence and times each switch into a `stage_wall_ms` column; the first entry is an unmeasured warm-up, so every switch is timed from an already-converged image. Comparability is exact: `config` carries only configured inputs, never a measured one -- the display refresh the run was paced at is a per-frame `refresh_hz` sample, since a measured double cannot satisfy an equality contract.

It prints B/A with a distribution-free confidence interval, and says "not resolved" when that interval contains 1. `bench_compare compare` and `bench_compare history --tool T` read records already in the log (unpaired, so drift is not controlled). `--metric` picks any samples column (its mean per event -- per frame, per upload or per pass, since frame count scales with run duration) or rusage field such as `user_s`.

## Testing

Every validator runs on a shared harness (`tools/check.h`): each check is registered by name and discovered into `ctest` as its own entry (`<suite>.<check>`), labelled by speed (`fast`/`slow`) and kind (`exact`/`statistical`). `ctest -L fast -L exact` is the sub-second pre-commit gate; the full suite is the pre-merge gate. Monte Carlo bands are derived from the run's own variance over independent scramble seeds at one family-wise significance level (`tools/stats.h`), not hand-picked. `render_beauty --assert-deterministic` and `--assert-converged` gate the shipping pipeline with no golden image.

# Material Library

Named presets live in `assets/materials/*.json`, parsed into `MaterialConfig` (`scene_config.h`). Each scene picks one as its default via `SceneConfig::materialPath`, with optional per-object overrides via `SceneConfig::materialOverrides` (glTF node name → material JSON path) — e.g. `assets/scenes/cornell.json`'s `{"sphere01": "materials/chrome.json", "sphere02": "materials/glass.json"}`, everything else left on the scene default. Overrides build a per-instance settings vector, indexed by `ShadingTriangle::instanceIndex` through both render paths ([Per-object materials](#materials--lighting)).

Add a new material by dropping a JSON file in `assets/materials/` and pointing `materialPath`/`materialOverrides` at it — no code or schema change needed.

## `MaterialConfig`

| File | metallic | transmission | roughness (factor / min) | Notes |
|---|---|---|---|---|
| `principled.json` | 0.0 | 0.0 | 1.0 / 0.045 | Rough dielectric, heavy bump (`bumpStrength: 10.0`). The reference preset: the only one declaring every field, including the otherwise-optional `transmissionColor`/`transmissionDepth`/`edgeTint`. No shipped scene uses it |
| `clay.json` | 0.0 | 0.0 | 0.5 / 0.045 | Neutral matte dielectric, no bump |
| `chrome.json` | 1.0 | 0.0 | 0.05 / 0.045 | Measured chromium (Johnson & Christy 1974): `diffuseColour` (which at `metallic=1` *is* `f0`, Gulbrandsen's reflectivity `r`) `[0.5496, 0.5560, 0.5542]` and `edgeTint` `[0.5417, 0.5692, 0.6942]`, both verbatim `tools/metal_fit` output (CIE 1931 2°, D65, linear Rec.709), so the grazing reflectance dip is the measured one. `colour.chrome_matches_measured_chromium` fails if this file drifts from the fit |
| `glass.json` | 0.0 | 1.0 | 0.02 / 0.01 | Schott N-BK7 crown glass, `ior: 1.5168` at the d line with `abbe: 64.17` for dispersion; tinted via Beer-Lambert `transmissionColor: [0.96, 0.98, 1.0]` over `transmissionDepth: 0.4` world units — `diffuseColour` cannot tint transmission at all (see below) |

| Field | Meaning |
|---|---|
| `diffuseColour` | Multiplies `baseColorTexture`. Also the conductor lobe's `f0` tint. A **reflection** quantity throughout — it never tints transmitted light, which `transmissionColor` alone does. On the diffuse lobe it's the **observed** albedo (OpenPBR's reading of `base_color`), not EON's ρ; the two differ once `diffuseRoughness > 0`, and `eonAlbedoInversion` (`bsdf.cpp`) maps one to the other |
| `metallicFactor` / `transmissionFactor` | Lobe selection — a material is dielectric, conductor, or transmissive, not blended between (every shipped file uses 0.0/1.0) |
| `roughnessFactor` | Multiplies the roughness texture sample, before the `roughnessMin`/`roughnessMax` clamp |
| `roughnessMin` / `roughnessMax` | Per-material clamp on the roughness sample; a material can floor below the shared 0.045 (e.g. glass's 0.01) for a genuinely smooth GGX lobe |
| `ior` | Dielectric IOR, non-metal lobes only |
| `abbe` | Abbe number, dispersion strength for the dielectric/transmissive lobes; 0 = no dispersion |
| `diffuseRoughness` | EON rough-diffuse parameter r ∈ [0,1] (Portsmouth, Kutz, Hill 2025, revised 2026-02-04); 0 = Lambertian, and the roughness at which `diffuseColour` and EON's ρ coincide |
| `bumpStrength` | Scales the bump texture's per-texel height difference |
| `transmissionColor` / `transmissionDepth` | The **only** tint on transmitted light (Arnold `standard_surface`/OpenPBR convention). `transmissionDepth > 0` means interior-medium Beer-Lambert absorption, `sigmaA = -log(transmissionColor)/transmissionDepth`, applied while a path is inside a `transmissionFactor>0` material. `transmissionDepth 0` means no interior medium — a constant on-surface tint applied once per crossing, so a closed solid reads its square. Default `[1,1,1]`/`0.0` is Arnold/OpenPBR's own no-op default in either regime ([Volumetric absorption](#materials--lighting)) |
| `edgeTint` | Gulbrandsen 2014 edge tint for the conductor lobe, `metallicFactor>0` only. Default `[1,1,1]`: white is the no-dip edge Schlick always produced |

# AOV

| AOV | Mechanism |
|---|---|
| Beauty | Final accumulated radiance, post tone-mapping — the primary output |
| Wireframe | Screen-space line rasterization (Pineda 1988), z-tested against the scene's own depth: white mesh-triangle edges, plus one bounding box per instance in that instance's `falseColorForId` hue, the same hue ObjectID gives it (drawn on top, so the box wins) — visualizes triangle density/topology and each object's extent/placement in one view |
| Alpha | 1.0 on a primary hit, 0.0 on a primary miss — a real coverage mask (this renderer isn't opaque-only-by-construction) |
| Depth | Planar camera-space Z (Arnold/RenderMan/EXR "Z" convention) at the primary hit, for depth-based compositing/debugging |
| Lookahead | Depth on a fixed, declared scale: `clamp(1 - Z/lookaheadDistance, 0, 1)`, so it reads 1 at the camera plane and falls linearly to 0 at `lookaheadDistance` (`profile.json`, scene units). Proximity without the consumer having to source a range of its own — unlike Depth, which is unbounded and auto-ranged only at display time. Geometry at or beyond the horizon reads 0, the same value a primary miss reads, so Alpha is what separates "too far" from "nothing there" |
| HSV | Colour-space transform of Beauty — isolates hue/saturation shifts a pure RGB view can hide |
| Luminance | Rec.709 luminance of Beauty — isolates perceived brightness from colour |
| Sobel | 3×3 Sobel gradient magnitude of Luminance — a cheap edge/gradient signal |
| Gabor | 4-orientation Gabor kernel bank, max response, of Luminance — directional edge/texture response Sobel's isotropic magnitude can't distinguish |
| WorldPos | Raw world-space primary-hit position, for debugging geometry/UV placement independent of shading |
| UV | Primary-hit interpolated UV (fractional part) — visualizes the texture-space mapping directly |
| Normal | Shading (normal-mapped) normal at the primary hit — the normal actually used in shading |
| GeomNormal | Smooth interpolated vertex normal, before normal-mapping — separates a bad normal map from a bad base mesh |
| Albedo | Base-colour texture sample at the primary hit — isolates texture data from lighting |
| Metallic | Per-instance metallic factor (`settings.metallicFactor`, `materials/*.json` or `SceneConfig::materialOverrides`), uniform within one object's triangles but no longer whole-image-constant now that per-object material assignment exists — debugs which instance carries which metallic value |
| Roughness | Roughness texture × a per-instance factor, floored at that material's own `roughnessMin` (materials can set their own floor, e.g. glass's below diffuse/chrome's shared 0.045); texture varies per hit, factor/floor vary per instance — debugs material authoring independent of shading |
| Tangent | Shading tangent basis at the primary hit — debugs the tangent-space basis used for normal mapping |
| ObjectID | Per-instance index, false-coloured (`falseColorForId`) — an isolation mask for compositing/debugging |
| AO | Cosine-weighted obscurance (Zhukov et al. 1998; Iones et al. 2003), the distance-weighted generalisation of ambient occlusion (Miller 1994; Landis 2002). One hemisphere ray per sample bounded by `aoMaxDistance` (`profile.json`), each hit weighted `1 - (1 - t/aoMaxDistance)^2` so occlusion grades with proximity and reaches full visibility smoothly at the bound. 1.0 = unoccluded, the opposite polarity to Shadow — reads contact/corner darkening off the actual geometry, independent of material and lighting |
| Fresnel | Expected Fresnel reflectance over the **visible microfacet normal distribution**, `E[F(wo.wh)]` for `wh ~ D_vis(wo)`. One VNDF draw per sample (Heitz 2018, the same `D_vis` and `alpha` `sampleBsdf` draws from), through `mix(exact dielectric Fresnel, exact complex-IOR conductor Fresnel, metallic)` (`fresnelAtMicrofacet`, `bsdf.h`), progressive like every other path-traced lane. This is the angle the microfacet BSDF actually evaluates Fresnel at (Walter et al. 2007), so it is roughness-dependent where a macro-normal value cannot be. At `n.wo = 0.05` on an `ior` 1.5 dielectric it reads 0.7521 at the roughness floor, 0.4406 at roughness 0.3 and 0.1692 at 0.6, against a macro-normal 0.7521 throughout. Full RGB — a conductor's Fresnel is chromatic by construction (`edgeTint` inverts to a per-channel complex IOR), which the rasterizer's `(F, 1-F, 0)` packing discarded. Collapses onto the macro-normal value to 4 decimals as `alpha` reaches its `kMinAlpha` floor, so it is a strict generalisation of the AOV it replaces, not a different quantity |
| IOR | Per-instance dielectric IOR (`settings.ior`), -1 on a miss — isolates the raw refractive-index input driving Fresnel/transmission |
| BounceCount | Mean path termination depth across samples, per pixel — debugs Russian roulette/termination behaviour |
| DirectDiffuse | Diffuse-bucketed radiance from a path's first (bounce-0) surface, physical (base colour included) — isolates direct diffuse light arrival, in the same units as Beauty |
| IndirectDiffuse | Diffuse-bucketed radiance from later bounces — isolates indirect (bounced) diffuse contribution |
| DirectSpecular | Specular-reflection-bucketed radiance, one bounce from camera — isolates direct specular contribution |
| IndirectSpecular | Specular-reflection-bucketed radiance, later bounces — isolates indirect specular (reflections) |
| Refraction | Radiance from any path that sampled a transmission lobe (sticky bucket) — isolates glass/transmissive transport |
| Shadow | Binary NEE occlusion test toward the sampled light (environment or an area light, per `LightSet`'s uniform selection) at the primary hit, re-averaged across progressive passes into continuous shadow/penumbra density — isolates direct-light visibility from material/lighting colour |

# References

- Kajiya, J.T. (1986). The rendering equation. SIGGRAPH.
- Veach, E. (1997). Robust Monte Carlo Methods for Light Transport Simulation. PhD thesis, Stanford: MIS, its support condition (§9.2, asserted by `bsdf_validate`'s `strategy_coverage`), and NEE.
- Veach, E., Guibas, L.J. (1995). Bidirectional estimators for light transport. EGRW: vertex connection — [roadmap](ROADMAP.md) transport #4, not implemented.
- Arvo, J., Kirk, D. (1990). Particle Transport and Image Synthesis: Russian roulette termination.
- Christensen, P.H., Jarosz, W. (2016). The Path to Path-Traced Movies. FnT CGV: production grounding.
- Sobol, I.M. (1967); Joe, S., Kuo, F.Y. (2008). SIAM JSC 30(5); Bratley, P., Fox, B.L. (1988). ACM Alg. 659: the sequence, its direction numbers (`sobol_direction_seeds.inc`) and the expanding recurrence.
- Burley, B. (2020). Practical Hash-based Owen Scrambling. JCGT 9(4): the scramble, set-index shuffling and padding in `Sampler`, using Vegdahl's constants as shipped by Cycles.
- Cranley, R., Patterson, T.N.L. (1976). Randomization of number theoretic methods: the per-pixel toroidal shift blue-noise dithering drives.
- Georgiev, I., Fajardo, M. (2016). Blue-noise Dithered Sampling. SIGGRAPH Talks: the tiled blue-noise shift, adopted at d = 1; the annealed d-dimensional matrix is open in the [roadmap](ROADMAP.md).
- Ulichney, R.A. (1993). The void-and-cluster method for dither array generation. SPIE 1913: the mask's construction, re-runnable in `tools/bluenoise_mask.cpp` rather than a lifted tile.
- Dupuy, J., Jakob, W. (2018). An Adaptive Parameterization for Efficient Material Acquisition and Rendering. ACM ToG 37(6): one interpolant for value and density, the construction both transmissive multiple-scattering shares use.
- Zwicker, M. et al. (2015). Adaptive Sampling and Reconstruction for Monte Carlo Rendering. CGF STAR: denoising survey — [roadmap](ROADMAP.md) transport #6, not implemented.
- Xiao, L. et al. (2020). Neural supersampling for real-time rendering. SIGGRAPH — [roadmap](ROADMAP.md) transport #7, not implemented.
- Ho, J. et al. (2020). NeurIPS; Rombach, R. et al. (2022). CVPR: diffusion foundations — [roadmap](ROADMAP.md) transport #8, not implemented.
- Novák, J. et al. (2018). Monte Carlo Methods for Volumetric Light Transport. CGF STAR — [roadmap](ROADMAP.md) transport #1, not implemented.
- Jensen, H.W. et al. (2001). SIGGRAPH; Christensen, P.H., Burley, B. (2015): BSSRDF and diffusion profiles — [roadmap](ROADMAP.md) transport #1, not implemented.
- Cook, R.L., Torrance, K.E. (1982). ACM ToG: BRDF and Fresnel foundations.
- Walter, B. et al. (2007). Microfacet models for refraction through rough surfaces: GGX, the rough-refraction BTDF, and the per-microfacet reflect/refract choice `facetReflectProbability` implements.
- Heitz, E. (2014). Understanding the Masking-Shadowing Function: the height-correlated Smith term (`smithVisibility`). Its transmissive Beta form is deliberately not used — see the [roadmap](ROADMAP.md)'s Smith-exact entry.
- Heitz, E. (2018). Sampling the GGX Distribution of Visible Normals. JCGT 7(4): the VNDF routine the specular lobe uses.
- Heitz, E. et al. (2016). Multiple-scattering microfacet BSDFs with the Smith model: the reference instrument for exit distributions, and the statement of the energy single scatter discards.
- Kulla, C., Conty, A. (2017). Revisiting Physically Based Shading at Imageworks. SIGGRAPH course: the shipped directional-albedo multiple-scattering compensation, on both interfaces.
- Turquin, E. (2019). Practical multiple scattering compensation: evaluated against Heitz 2016 and not adopted; crossover in the [roadmap](ROADMAP.md).
- Guy, R., Agopian, M. (2018). Filament §4.4.2: the cancellation-free Trowbridge-Reitz denominator `distributionGGX` evaluates.
- Gulbrandsen, O. (2014). Artist Friendly Metallic Fresnel. JCGT 3(4): the conductor reflectivity/edge-tint parameterisation, with two documented departures from its listing (see `bsdf.cpp`).
- Portsmouth, J., Kutz, P., Hill, S. (2025, rev. 2026-02-04). EON: A Practical Energy-Preserving Rough Diffuse BRDF. JCGT 14(1): the rough-diffuse lobe, its compensation, CLTC sampling, and Appendix A's albedo inversion (`eonAlbedoInversion`).
- Pharr, M., Jakob, W., Humphreys, G. Physically Based Rendering: `FrDielectric`, `EffectivelySmooth`, and the index-matched delta routing `transmissionIsRough` implements.
- OpenPBR Surface specification; Autodesk Arnold `standard_surface`: the transmission-tint convention this pipeline follows, and the meaning of `base_color`.
- Adobe. OpenPBR BSDF reference implementation, `openpbr_constants.h`: the 620/540/450 nm triple `kRgbWavelengthsNm` takes, and the "discrete RGB bands" limitation ([roadmap](ROADMAP.md) transport #5).
- OpenPBR: Novel Features and Implementation Details (arXiv:2512.23696): the throughput-weighted channel selection the dispersive path uses.
- Khronos. `KHR_materials_dispersion`: Cauchy's relation inverted from an Abbe number, implemented by `cauchyIor`.
- Dupuy, J., Benyoub, A. (2023); Tokuyoshi, Y., Eto, K. (2023): newer VNDF refinements, surveyed, not implemented (Heitz 2018 used instead).
- Schüßler, V. et al. (2017). Microfacet-based normal mapping: not implemented; a geometric-normal-consistency rejection is used instead.
- Belcour, L. (2018). Layered materials by atomic decomposition. ACM ToG: not implemented (single-layer only).
- Chiang, M.J.-Y., Li, Y., Burley, B. (2019). Taming the Shadow Terminator. JCGT 8(4): the secondary-ray origin correction.
- Debevec, P. (1998). Rendering synthetic objects into real scenes: HDR image-based lighting.
- Goral, C.M. et al. (1984). SIGGRAPH: the emissive-panel Cornell box, implemented (`cornell.json`).
- Ureña, C., Fajardo, M., King, A. (2013). An Area-Preserving Parametrization for Spherical Rectangles. CGF: the quad light's solid-angle NEE sampler, as Arnold's `quad_light` and PBRT 12.5.3.
- Lambert, J.H. (1760). Photometria; Baum, D.R. et al. (1989). SIGGRAPH: the closed-form Lambertian-polygon irradiance `integrator_validate` uses as an independent analytic reference.
- Miller, G. (1994); Landis, H. (2002): the cosine-weighted distance-bounded AO the AO AOV path-traces, whose pdf cancels to the mean of the visibility term.
- Zhukov, S. et al. (1998); Iones, A. et al. (2003); surveyed in Mendez-Feliu, A., Sbert, M. (2009): obscurance, the distance falloff rho(x) = 1 - (1-x)^2 the AO AOV uses.
- Bitterli, B. et al. (2020). ReSTIR. SIGGRAPH — [roadmap](ROADMAP.md) transport #2, not implemented: reservoir resampling needs many lights to be worth it.
- Sobel filtering: the edge-detection AOV computed from Luminance, arXiv:2601.16806.
- CIE 018:2019 Table 6 (ISO/CIE 11664-1:2019): the 1931 2° colour-matching functions at 1 nm, `cie_1931.inc`.
- ISO/CIE 11664-2:2022 Table B.1: D65 at 1 nm, `cie_1931.inc`.
- CIE 015:2018: tristimulus integration at the 1 nm interval, `cie::reflectanceToRec709`.
- ITU-R BT.709-6 (2015); SMPTE RP 177-1993: the primaries and the matrix derivation, `cie::xyzToRec709`.
- Johnson, P.B., Christy, R.W. (1974). Phys. Rev. B 9, 5056: the measured chromium `(n, k)` behind `chrome.json`.
- Wilkie, A. et al. (2014). Hero wavelength spectral sampling. CGF — [roadmap](ROADMAP.md) transport #5, not implemented; the shipped dispersion commits to one RGB channel instead.
- OpenEXR / Academy Software Foundation: the linear HDR pipeline and exposure.
- Chandrasekhar, S. (1960). Radiative Transfer. Dover: polarised transport, underlying a CPL filter — [roadmap](ROADMAP.md), physical camera filters, not implemented.
- Khronos. glTF 2.0 specification: the scene/mesh/material interchange format.
- Mikkelsen, M.S. (2008). MikkTSpace: not implemented (glTF-supplied tangents only).
- Wald, I. et al. (2014). Embree. ACM ToG: the CPU ray-scene intersection kernels behind `EmbreeAccel`.
- Pineda, J. (1988). A parallel algorithm for polygon rasterization. SIGGRAPH: the edge-function rasterizer.
- Sutherland, I.E., Hodgman, G.W. (1974). CACM: the view-frustum polygon clip.
- Blinn, J.F., Newell, M.E. (1978). SIGGRAPH: the per-vertex outcodes ahead of that clip.
- Microsoft. Direct3D 11.3 Functional Spec §3.4; Khronos. Vulkan, Rasterization: fixed-point snapping and the top-left fill rule, with precision derived per frame from the int64 exactness bound.
- Giesen, F. (2013). Triangle rasterization in practice: integer edge functions, the top-left bias, incremental row stepping.
- Williams, L. (1983). Pyramidal Parametrics. SIGGRAPH: MIP-mapping — [roadmap](ROADMAP.md), texture minification, not implemented.
- Cook, R.L., Porter, T., Carpenter, L. (1984). Distributed Ray Tracing. SIGGRAPH — [roadmap](ROADMAP.md), depth of field and motion blur, not implemented.
- Kannala, J., Brandt, S.S. (2006). IEEE TPAMI: the fisheye projection families — [roadmap](ROADMAP.md), fisheye lens, not implemented.
- Kalibera, T., Jones, R. (2013). Rigorous Benchmarking in Reasonable Time. ISMM: the process invocation as the unit of replication.
- Abedi, A., Brecht, T. (2017). ICPE: randomized multiple interleaved trials, which `bench_compare run` automates; Mytkowicz, T. et al. (2009). ASPLOS: the ordering and environment bias randomization removes.
- Hodges, J.L., Lehmann, E.L. (1963); Hollander, M., Wolfe, D.A., Chicken, E. (2014) 3.2/4.3: the shift estimators and exact rank intervals in `tools/stats.h`, on the log scale so a shift is a ratio (Fleming, P.J., Wallace, J.J. (1986). CACM 29(3)); Hoefler, T., Belli, R. (2015). SC: nonparametric intervals for performance data.
- Welford, B.P. (1962); West, D.H.D. (1979): the incremental running mean `PathTraceDriver` publishes; Chan, T.F. et al. (1983); Higham, N.J. (2002) §1.9 and ch. 3: the forward-error bound `driver_validate` derives for it.
- Autodesk Arnold `options.stats_file`: the benchmark log's append-per-record format; LLVM `GenerateVersionFromVCS.cmake`: the build-time VCS stamp.
- Apple (2023). `NSView.displayLink(target:selector:)`; Energy Efficiency Guide, QoS classes: the vblank callback `DisplayLink` paces on. GLFW issues #1990/#2249 and PR #2277 record why `NSOpenGLCPSwapInterval` is not used.
