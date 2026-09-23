# Component reference

[← README](../README.md)


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
| glTF loading | cgltf; per-primitive vertices baked to world-space triangles/shading data at load time, materials' textures decoded once into `ImageTexture<N>` at the channel count that slot's consumer reads (3 for base colour/normal/specular, 1 for roughness/bump; glTF's `occlusion_texture` is not read at all, AO being path-traced) — nothing GPU-resident is needed once the CPU Embree scene exists, and no map carries a channel nothing samples |
| Tangent-space normal mapping | Per-vertex tangent (glTF-supplied only), Gram-Schmidt re-orthogonalized per-ray, for surface micro-detail without extra geometry |
| Ray acceleration | Intel Embree (SIMD BVH build/traversal), CPU, built once at load — sub-linear ray-scene intersection, required before recursion is affordable |
| Primary-hit rasterizer | Watertight CPU edge-function rasterization (Pineda 1988, incremental along each row): view-frustum clip (Sutherland-Hodgman 1974, Blinn-Newell 1978 outcodes) with canonical-order intersections, fixed-point snapping at the most sub-pixel bits that keep int64 edge functions exact, top-left fill rule (D3D11.3 functional spec; Giesen 2013), row-parallel, synchronous every frame, giving instant primary-hit G-buffer AOVs (`aov.h`) decoupled from Beauty's progressive convergence |

## Materials & lighting

| Feature | Mechanism |
|---|---|
| Stochastic BSDF | EON rough-diffuse (Portsmouth, Kutz, Hill 2025), GGX microfacet specular, Walter 2007 rough dielectric transmission (delta Snell + TIR below the smooth-roughness threshold, and at `ior` 1 at every roughness), Kulla-Conty multiple-scattering compensation on both interfaces; four-lobe stochastic selection, one-sample mixture estimator — real physical response, energy-conserving at every roughness |
| Volumetric absorption | Beer-Lambert extinction (`transmissionColor` over `transmissionDepth`, Arnold/OpenPBR convention) on a single-level medium stack inside a `transmissionFactor>0` material (a constant on-surface tint at `transmissionDepth 0`) — tinted glass, thick or thin, without participating-media in-scattering ([roadmap](roadmap.md) transport #1) |
| Per-object materials | `SceneConfig::materialOverrides` (glTF node name → `materials/*.json` path) builds a per-instance settings vector, indexed by `ShadingTriangle::instanceIndex` through both render paths — different objects in one scene can carry different materials |
| Area lights | Rectangular emitters (`scene.json`'s `lights`), one-sided by default, with their own geometry in the BVH and solid-angle NEE sampling (Ureña, Fajardo & King 2013), MIS'd against BSDF sampling like the environment — the classic emissive-panel Cornell box ([roadmap](roadmap.md) transport #2), and ReSTIR's ([roadmap](roadmap.md)) prerequisite light set |
| Environment lighting | Equirect HDR map, BSDF-sampled misses + luminance-importance-sampled NEE, MIS-combined — image-based lighting, one member of `LightSet` alongside any area lights (still no punctual/directional lights, which have no hittable geometry) |
| Environment-light toggle | HUD "Environment Light" checkbox (`environment.lightEnabled` in `scene.json`, `--env-light` on `render_beauty`) — removes the environment from NEE/MIS/every miss including the background, unlike "Show/Hide Background" which only hides the camera-visible sky; lets an HDRI+area-light scene isolate the panel-only look |
| Russian roulette | Survival probability clamped from running throughput past `russianRouletteStartBounce`, reweighted by `1/p` — keeps recursion finite without biasing the estimator |
| Progressive accumulation | Each background pass re-traces at the current camera/settings and averages into the displayed result, restarting on any camera/scene change — real-time-interactive without waiting for one long render to finish |

## Debug tooling & telemetry

| Feature | Mechanism |
|---|---|
| Startup spec block | One plain-text provenance block on stdout: GPU/driver/refresh rate, host CPU topology and cache line from `sysctl`, compiler/build type/`-march`/IPO/git SHA, runtime-queried library versions, and the scene's load/BVH-build cost — confirms the actual GPU/backend before a wrong-adapter bug masquerades as a render bug, and makes every timing number attributable |
| Render telemetry (`-stats`) | 78-column terminal dashboard redrawn in place at 3 Hz via a single `write(2)`: render-thread stages with share-of-frame bars, path-trace phases, ray counts by type with Mray/s, and a `cpu total` / `frame measured` / `unaccounted` reconciliation — shows where every millisecond goes, and what it can't account for |
| Frame pacing | `DisplayLink` (`display_link.mm`): `-[NSView displayLinkWithTarget:selector:]` on a user-interactive-QoS thread wakes the render loop once per vblank of the window's own display, at swap interval 0 with a one-frame GPU fence -- NSGL's swap interval lets two swaps through per refresh on current macOS and GLFW substitutes a fixed 60 Hz `usleep` while occluded, so neither is used. A minimised window, whose link stops ticking, free-runs on the display's period grid. `render.vsync: false` (`profile.json`) skips the vblank wait and runs uncapped, still bounded to one frame in flight by the fence. The measured period is the refresh rate every consumer reports. `swap_ms` still has a tail outside the engine: NSOpenGL's flush makes a synchronous WindowServer query (`SLSFlushSurfaceWithOptionsAndIndex` -> `_CGSWindowIsOrderedIn`), and in 7 visible convergences (88k frames) 808 of the 827 1 ms samples that found the render thread blocked inside a swap over a quarter period were in it, and none found it runnable-waiting for a core; swaps over half a period were 96-98% off-CPU, unchanged with the trace and driver threads at utility QoS (13 vs 9, P = 0.52) and over-represented right after display-texture uploads (7 vs 1.3 expected). A per-frame maximum of `swap_ms` or `frame_ms` therefore measures WindowServer, not the engine |
| Frame-timing HUD | Ring buffer of recent frame times; rolling FPS/avg/min/max, GPU timer query around the post-process blit — makes blit cost measurable frame to frame |
| Memory HUD | Live RAM readout plus GPU allocation tracked at alloc/free (the path-traced display texture is the only GPU allocation left) — surfaces a memory regression immediately, not after VRAM exhaustion |
| Scene stats | Object/triangle/point counts, viewport resolution — a scene-complexity readout |
| Debug camera controls | WASD/QE fly, R reset, LMB-drag orbit around a pivot read from the path tracer's own G-buffer (world-space hit position + hit mask at its centre pixel) — interactive navigation without hand-editing camera parameters |
| Camera framing overlays | Centre crosshair, always on, drawn on the foreground overlay — a composition aid that never contaminates the AOV buffers being debugged |
| AOV selector | Dropdown across the full AOV set (`aov.h`), plus R/G/B channel-isolation hotkeys, to isolate one signal at a time |
| Live histogram | Per-channel (R/G/B) histogram of the currently displayed image — catches exposure/clipping and colour-space bugs a single still frame can hide |
| Benchmark log | Append-only JSON Lines, one record per timing run (`bench_log.h`, after Arnold's `stats_file` append mode): build-time git SHA plus the binary's Mach-O `LC_UUID`, host topology, argv, the resolved config, raw per-iteration samples, rusage, and a CRC of the output. `bench_compare` runs randomized multiple interleaved trials (Abedi & Brecht 2017) and reports the Hodges-Lehmann ratio with its exact Wilcoxon interval — makes a single-change timing claim resolvable when run-to-run drift exceeds the effect |

## Session settings (`profile.json`)

Session settings live in `assets/config/profile.json`. `render.vsync` caps the frame rate to the display's vblank (`true`) or runs uncapped (`false`); uncapped, the render thread competes with the trace workers for cores, measured at **1.58x** `pass_ms` on cornell and **1.47x** on a local 4K-textured test asset (8 cores). `render.textureBitDepth` sets the CPU storage of the scene's input images, the environment HDRI and every material texture, whose channel count is fixed per slot by its consumer (three for base colour/normal/specular and the HDRI, one for roughness/bump, and no AO map at all -- [Scene loading](#scene-loading--geometry)): `16` (the shipped default) or `32` (float) (IEEE 754 binary16: unit roundoff 2^-11, finite maximum 65504; a source texel that overflows it becomes Inf and is rejected at load rather than silently clamped). The environment's importance-sampling CDFs are built from the stored values, so sampling stays proportional to the radiance returned at either depth. Every shipped EXR is half on disk, so `16` is lossless for them: renders are bit-identical, and on that asset peak RSS drops 2103 -> 1281 MB, exactly the analytic 822 MB, and `pass_ms` is **0.978x** (half the bytes per texel fetch). `render.displayBitDepth` sets the path-traced display texture's GL storage: `16` (`GL_RGBA16F`) or `32` (`GL_RGBA32F`, twice the memory and sampling bandwidth: `present_gpu_ms` **1.44x**). The engine uploads texels already in that component type (`GL_HALF_FLOAT` or `GL_FLOAT`), so the driver converts nothing either way and `16` also transfers half the bytes -- `upload_ms` **0.549** on that asset (5.59 -> 3.06 ms, 2 rounds, no interval) against the `GL_FLOAT`-into-`GL_RGBA16F` upload it replaces.

## Benchmark tooling

`engine -bench` also takes `-bench-aovs "Beauty,Sobel,Direct Diffuse,Normal,Beauty"`, which walks that AOV sequence and times each switch into a `stage_wall_ms` column; the first entry is an unmeasured warm-up, so every switch is timed from an already-converged image. Comparability is exact: `config` carries only configured inputs, never a measured one -- the display refresh the run was paced at is a per-frame `refresh_hz` sample, since a measured double cannot satisfy an equality contract.

It prints B/A with a distribution-free confidence interval, and says "not resolved" when that interval contains 1. `bench_compare compare` and `bench_compare history --tool T` read records already in the log (unpaired, so drift is not controlled). `--metric` picks any samples column (its mean per event -- per frame, per upload or per pass, since frame count scales with run duration) or rusage field such as `user_s`.

## Testing

Every validator runs on a shared harness (`tools/check.h`): each check is registered by name and discovered into `ctest` as its own entry (`<suite>.<check>`), labelled by speed (`fast`/`slow`) and kind (`exact`/`statistical`). `ctest -L fast -L exact` is the sub-second pre-commit gate; the full suite is the pre-merge gate. Monte Carlo bands are derived from the run's own variance over independent scramble seeds at one family-wise significance level (`tools/stats.h`), not hand-picked. `render_beauty --assert-deterministic` and `--assert-converged` gate the shipping pipeline with no golden image.
