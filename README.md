# Physically based path tracer

*A CPU, unidirectional Monte Carlo path tracer with real-time progressive display: Embree-accelerated, stochastic BSDF sampling combined with environment-map NEE via MIS, converging interactively behind a thin OpenGL display/HUD layer.*

![Sample render](sample.png)

## Build

C++20, built with CMake. Currently developed against macOS only.

### Prerequisites

```
brew install cmake glfw glew glm imath openexr opencolorio embree
```

Homebrew's `embree` formula pulls in `tbb` automatically (Embree's own internal parallelism dependency) — no separate install step needed for it.

Dear ImGui is vendored as a git submodule (`third_party/imgui`) — initialise it before configuring:

```
git submodule update --init --recursive
```

### Assets

`assets/geometry/` and `assets/textures/` hold the runtime glTF/EXR assets `assets/config/scene.json` and `main.cpp` reference (e.g. `assets/geometry/broken_stump_rkswd_raw/rkswd_tier_1.gltf`, `assets/textures/republiqueHDR_2k.exr`). They're gitignored as large/generated binaries, same policy as generated `*.exr` test output — supply them locally at those paths before running; nothing fetches them automatically.

### Configure & build

```
cmake -B build
cmake --build build
```

This produces seven targets:

- `build/engine` — the path tracer
- `build/test_pattern` — EXR calibration-pattern generator (`tools/test_pattern.cpp`)
- `build/downsample` — EXR downsampling tool (`tools/downsample.cpp`)
- `build/embree_validate` — headless Embree ray-scene intersection correctness check (`tools/embree_validate.cpp`)
- `build/bsdf_validate` — headless BSDF pdf-normalization and furnace-test check (`tools/bsdf_validate.cpp`)
- `build/nee_validate` — headless NEE/MIS unbiasedness check against a brute-force reference (`tools/nee_validate.cpp`)
- `build/rasterizer_validate` — headless CPU-rasterizer-vs-Embree G-buffer correctness check (`tools/rasterizer_validate.cpp`)

`-Wall -Wextra -Werror` gates every target. If `clang-tidy` is installed, it also runs on every compile of `engine`'s own sources (see `.clang-tidy`); if `cppcheck` is installed, `cmake --build build --target cppcheck` runs it over `src/`. Both are skipped, not required, if not installed.

`cmake -B build -DENGINE_SANITIZE=ON` builds with AddressSanitizer + UndefinedBehaviorSanitizer instead (off by default) — worth running after touching the glTF/JSON/EXR loading paths.

### Run

```
./build/engine
```

## 1. Pipeline

glTF geometry/materials load once at startup into a CPU-resident scene: per-vertex shading data (`ShadingTriangle`) and world-space triangles feed an Intel Embree scene (`EmbreeAccel`); materials keep only CPU `HdrImage` textures, sampled per-ray. An equirectangular HDR environment map loads alongside it, with its own luminance-based importance-sampling CDF for NEE.

Every frame, on any camera/scene-state change, `PathTraceDriver` hands a fresh request to a background thread pool (one worker per hardware core, row-parallel, dynamic scheduling), which restarts progressive accumulation:

- Camera ray generation (pinhole) → Embree ray-scene intersection (`rtcIntersect1`/`rtcOccluded1`)
- BSDF evaluation/sampling (Heitz 2018 GGX VNDF specular, cosine-weighted diffuse, Walter 2007 rough dielectric transmission with exact Fresnel and TIR — falling back to a Snell delta lobe below the smooth-roughness threshold — Kulla-Conty multiple-scattering compensation on both the reflective and transmissive interface, Schlick conductor Fresnel) → next-event estimation against the environment map, MIS-combined with BSDF sampling (power heuristic, Veach 1997)
- Recursive bounce loop with Russian roulette, Chiang/Li/Burley 2019 shadow-terminator-corrected secondary-ray origins
- Radiance + a full G-buffer/transport-component AOV set accumulated per pass, published as a snapshot the render thread reads lock-free

Independently, on the same trigger, a synchronous CPU rasterizer (`rasterizer.cpp`) computes the 15 primary-hit-only AOVs (§3) directly on the render thread every frame — edge-function rasterization (Pineda 1988) with a near-plane Sutherland-Hodgman clip, sharing `gbuffer_shading.h`'s material sampling with the path tracer but no Embree, BSDF, or recursion. This gives those AOVs instant, glitch-free feedback during camera movement, decoupled from Beauty's own progressive convergence; the path-traced request above only restarts when the selected AOV actually needs light-transport data.

The render thread blits whichever AOV is selected through OCIO's display transform (exposure/tone-mapping) and the debug HUD, converging visibly over subsequent passes rather than blocking on a single long render. No GPU rasterization anywhere in this path — OpenGL exists only for the window, the post-process/OCIO blit, and ImGui; the primary-hit rasterizer above is CPU-only.

## 2. Component reference

| Feature | Mechanism | Role / why it matters |
|---|---|---|
| Camera / lens | Position/yaw/pitch, film-back + focal length → derived vertical FOV; pinhole primary rays derived directly from this basis | Geometric ground truth the ray-intersection/BSDF math is measured against |
| Photographic exposure | EV100 from aperture/shutter/ISO, applied as a relative-stops delta against profile.json's default triple (Filament/Frostbite EV100 formula) | Familiar photographic controls for adjusting display brightness; not an absolute photometric quantity, since the scene isn't calibrated to real-world radiance |
| OpenEXR linear pipeline | `HdrImage`/`loadExr`; all shading/compositing in linear light, OCIO display-encodes only at the final blit | Precondition for correct PBR colour math |
| Display transform | OCIO Display/View API (sRGB, Rec.709, Raw), cycled at runtime ('L') -- a colorimetric encode (sRGB / Rec.1886 OETF) only, no tone mapping | Scene-referred linear radiance throughout; values above 1.0 clip at the display by design, so lookdev sees clipping honestly rather than under a hidden filmic shoulder |
| GPU/system readout | Device name, driver/API version, refresh rate, RAM at startup | Confirms the actual GPU/backend before a wrong-adapter bug masquerades as a render bug |
| Frame-timing HUD | Ring buffer of recent frame times; rolling FPS/avg/min/max, GPU timer query around the post-process blit | Makes blit cost measurable frame to frame |
| Memory HUD | Live RAM readout plus GPU allocation tracked at alloc/free (the path-traced display texture is the only GPU allocation left — OCIO uses zero LUT textures) | Surfaces a memory regression immediately, not after VRAM exhaustion |
| Scene stats | Object/triangle/point counts, viewport resolution | Scene-complexity readout |
| Debug camera controls | WASD/QE fly, R reset, LMB-drag orbit around a pivot read directly from the path tracer's own G-buffer (world-space hit position + hit mask at its centre pixel) | Interactive navigation without hand-editing camera parameters between runs |
| Camera framing overlays | Centre crosshair, always on, drawn on the foreground overlay over the viewport | Composition aid that never contaminates the AOV buffers being debugged |
| AOV selector | Dropdown across the full AOV set (§3), plus R/G/B channel-isolation hotkeys | Isolates one signal at a time for debugging |
| Live histogram | Per-channel (R/G/B) histogram of the currently displayed image | Catches exposure/clipping and colour-space bugs a single still frame can hide |
| glTF loading | cgltf; per-primitive vertices baked to world-space triangles/shading data at load time, materials' textures decoded once to `HdrImage` | Standard interchange format; nothing GPU-resident is needed once the CPU Embree scene/shading data exists |
| Tangent-space normal mapping | Per-vertex tangent (glTF-supplied only), Gram-Schmidt re-orthogonalized per-ray | Surface micro-detail without extra geometry |
| Ray acceleration | Intel Embree (SIMD BVH build/traversal), CPU, built once at load | Sub-linear ray-scene intersection, required before recursion is affordable |
| Primary-hit rasterizer | CPU edge-function rasterization (Pineda 1988) + near-plane clip (Sutherland-Hodgman 1974), row-parallel, synchronous every frame | Instant primary-hit G-buffer AOVs (§3), decoupled from Beauty's progressive convergence |
| Stochastic BSDF | Lambertian diffuse, GGX microfacet specular, Walter 2007 rough dielectric transmission (delta Snell + TIR below the smooth-roughness threshold), Kulla-Conty multiple-scattering compensation on the reflective and transmissive interface alike; four-lobe stochastic selection, one-sample mixture estimator | Materials respond to light with real physical behaviour, including rough and smooth glass, and conserve energy at every roughness |
| Environment lighting | Equirect HDR map, BSDF-sampled misses + luminance-importance-sampled NEE, MIS-combined | The scene's sole light source — image-based, no punctual/area lights |
| Russian roulette | Survival probability clamped from running throughput from `russianRouletteStartBounce`, reweighted by `1/p` | Keeps recursion finite without biasing the estimator |
| Progressive accumulation | Each background pass re-traces at the current camera/settings and averages into the displayed result; any camera/scene change restarts accumulation | Real-time-interactive without waiting for a single long render to finish |

## 3. AOV reference

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
| Metallic | Material | Global metallic factor from `material.json` (`settings.metallicFactor`) — constant across the whole image; the engine has only one material config, not per-object metallic | Debug the configured global metallic value, not (yet) an authored per-material property |
| Roughness | Material | Roughness texture × a global factor from `material.json`, floored at 0.045 — the texture varies per hit, the multiplying factor does not | Debug material authoring independent of shading |
| Tangent | Material | Shading tangent basis at the primary hit | Debugs the tangent-space basis used for normal mapping |
| ObjectID | Material | Per-instance index, false-coloured (`falseColorForId`) | Isolation mask for compositing/debugging |
| AO | Material | Authored ambient-occlusion texture sample at the primary hit | Debug baked AO independent of lighting |
| Fresnel | Transport | Bare Schlick term at the primary hit's view angle from `f0` — simpler than shading's `mix(dielectric Fresnel, Schlick(f0), metallic)`, so it can disagree with what's actually rendered (e.g. reading near 1.0 from an unclamped specular texture where shading computes ~0.04 for a dielectric) | Debug grazing-angle reflectance behaviour in isolation; not a preview of the exact shading value |
| IOR | Transport | Global dielectric IOR from `material.json` (`settings.ior`), -1 on a miss — constant across the whole image, not read per-material | Isolates the raw refractive-index input driving Fresnel/transmission |
| BounceCount | Transport | Mean path termination depth across samples, per pixel | Debug Russian roulette/termination behaviour |
| DirectDiffuse | Lighting | Diffuse-bucketed radiance from a path's first (bounce-0) surface, physical (base colour included) | Isolates direct diffuse light arrival, in the same units as Beauty |
| IndirectDiffuse | Lighting | Diffuse-bucketed radiance from later bounces | Isolates indirect (bounced) diffuse contribution |
| DirectSpecular | Lighting | Specular-reflection-bucketed radiance, one bounce from camera | Isolates direct specular contribution |
| IndirectSpecular | Lighting | Specular-reflection-bucketed radiance, later bounces | Isolates indirect specular (reflections) |
| Refraction | Lighting | Radiance from any path that sampled a transmission lobe (sticky bucket) | Isolates glass/transmissive transport |
| Shadow | Lighting | Binary NEE occlusion test toward the env light at the primary hit, re-averaged across progressive passes into continuous shadow/penumbra density | Isolates direct-light visibility from material/lighting colour |

## 4. Roadmap

Ordered quick → complex; items within **Large** are a strict dependency chain (each needs the ones before it) and must land in that order. Items elsewhere have no hard blocker on one another.

### Quick

- **JSON library** — replace the hand-rolled scanner (`json_scan.cpp`) with nlohmann/json; do first, every item below depends on it.
- **scene.json as a CLI arg** — `main()` has no `argc`/`argv` (`main.cpp:864`); paths are hardcoded. Enables multi-scenario scenes.
- **Texture bit depth (16/32) via JSON** — hardcoded `GL_RGBA16F` today (`texture.cpp:45`); 32F ~doubles VRAM/buffer.
- **Code-quality audit** — `rotateAboutY` (`environment_map.cpp:13`) → `glm::rotate`; `ShadingFrame::toLocal`/`toWorld` (`bsdf.h:26`) → `glm::mat3`. (BSDF math in `bsdf.cpp` — GGX/Smith/Fresnel/VNDF — is standard domain logic, not an offload candidate.)
- **Expand terminal output (launch + loop)** — startup logs GL extensions/camera pose/model/BVH stats (`main.cpp:286-361`); no per-frame stats print during the interactive loop (`main.cpp:990`) — sample/pass/convergence stats reach only the HUD (`hud_overlay.cpp`), not stdout.

### Moderate

- **Recover the delighted DirectDiffuse/DirectSpecular view** — `bdecb41` deliberately made the five transport AOVs physical (each contribution written once, at its physical value) so they exactly partition Beauty, an identity `tools/integrator_validate.cpp:387` (`checkTransportPartition`) now asserts at 1e-4 relative; the prior delighted view (base colour divided out) was the deliberate casualty of that fix, not a regression, and the commit message names its replacement as `DirectDiffuse / Albedo`. Not Quick because Albedo is a `RasterGBuffer` field (`rasterizer.cpp:321`) and `main.cpp:773-808`'s `aovNeedsLightTransport` gating makes the rasterizer and path tracer mutually exclusive — Albedo and DirectDiffuse are never simultaneously fresh. Needs either a path-traced albedo buffer (3 more accumulator lanes) or relaxing that gating.
- **Scene-graph foundation** — three steps in strict order, the only ordered chain outside **Large**, and the prerequisite for (10) there.
  1. *Indexed geometry.* `gltf_loader.cpp:94-129` de-indexes every mesh into soup and stores positions twice — once in `Triangle` for Embree, once in `ShadingVertex.position` for shading — 184 B/tri, ~920 MB on a 5M-triangle asset. `rtcSetSharedGeometryBuffer` takes a byte stride, so Embree can read positions in place from the indexed shading vertices instead, deleting the `Triangle` array, keeping glTF's own index buffer rather than filling one with the identity sequence (`embree_accel.cpp:77-84`), and dropping the `reserve(size+1)` padding hack. ~5x smaller, and the traversal locality matters more than the footprint.
  2. *Object instancing.* `RTC_GEOMETRY_TYPE_INSTANCE` over one scene per unique mesh, replacing today's single flattened geometry (`embree_accel.cpp:65`) and finally giving `MeshInstance::transform` (`gltf_loader.h:18`, stored and never read) a consumer. `Hit` gains `geomID`/`instID`. Rays trace in object space against shared BVH leaves instead of the per-instance duplication today's flattened soup incurs. Needs (1)'s indexed layout.
  3. *Per-material factors.* `Material` holds six textures and nothing else, so `metallic_factor`, `roughness_factor`, `base_color_factor`, `ior` and `transmission_factor` are parsed by cgltf and discarded — every material in the scene shares one global set from `material.json`, and **a scene containing a metal object and a plastic object cannot be represented**. Move them onto `Material` and demote `MaterialConfig` to a global override layer. Also deletes `extrasTextureIndex` (`gltf_loader.cpp:35`), the hand-rolled substring scanner that exists only because roughness/specular/bump were hand-authored into glTF `extras`.
- **Frustum/backface culling** — skip `buildSubTriangles`'s per-frame full-scene walk (`rasterizer.cpp:153,274`) and the equivalent Embree traversal when out of view.
- **Low-discrepancy sampler upgrade** — Sobol / hash-based Owen scrambling (Burley 2020, §5), replacing randomized Halton (`sampler.cpp`).
- **Render-mode selector + adaptive tiling** — Single Sample / Progressive / Adaptive Tiling (today: the path tracer dispatches fixed 96x96 tiles and the rasterizer rows, both through `ThreadPool`; neither adapts to where the image is still noisy, and the mode is not selectable).
- **Adaptive per-pixel sample budget** — variance-driven, builds on tiling above; `samplesPerPixel` (`profile.json`) is one fixed global today, no per-pixel allocation.
- **Texture minification filtering (MIP-mapping)** — point/bilinear only today (`sampleBilinear`, `hdr_image.h:22`); grazing/distant surfaces alias. No mip chain exists to select from; needs ray differentials to pick a level per ray.
- **Packet tracing** — `EmbreeAccel` calls `rtcIntersect1`/`rtcOccluded1` exclusively (`embree_accel.cpp:116,144`), single-ray only; no `rtcIntersect4/8/16` packet API.
- **Ray reordering before shading** — the tile loop traces in raster order (`path_tracer.cpp:307-320`) with no Morton/direction coherence sort ahead of `tracePath` (`path_tracer.cpp:77`).
- **Deferred/sorted shading by material** — `tracePath` (`path_tracer.cpp:77`) evaluates the BSDF inline per ray; no material-bucketed shading pass.
- **Camera film-back preset drop-down** — `Camera::FilmBack` (`camera.h:13-16`) is one fixed `{widthMm, heightMm}` from `profile.json:5`. JSON-defined preset list (Alexa XT, IMAX, Medium Format, 5D, Leica M11, Red, 35mm, 70mm); `ImGui::Combo` in HUD, existing pattern at `hud_overlay.cpp:297`.
- **Photometric calibration** — tie radiometric output to real photometric units (lux/candela/lumen) so `ev100()` (`camera.h:54-56`) and light intensities can be checked against a light meter instead of eyeballed. Complements the Macbeth chart scene (§4 Large item 2).

### Large — strict dependency order

1. **Global illumination** — area lights + shadow rays to them. ReSTIR (Bitterli et al. 2020) follows once multiple area lights exist. Ray-traced AO replaces today's baked-texture AO AOV (§3). Caustics do not fall out of this: unidirectional path tracing structurally cannot sample specular-diffuse-specular paths regardless of light count — that needs (4).
2. **Cornell box + per-material showcase** — blocked on (1): needs an emissive panel, only IBL exists today. Showcase: mirror/rough conductor, smooth/rough dielectric (both implemented), subsurface once (3) lands. + **Macbeth chart scene** — no area-light dependency, validates albedo/colour under existing IBL (needs CLI-arg item above).
3. **Volumetric & subsurface transport** — participating media + BSSRDF/random-walk subsurface. Blocked: the transmissive multiple-scattering lobe is non-reciprocal (`f(wo→wi) ≠ f(wi→wo)`, `bsdf.cpp`), needs reworking first (`tools/bsdf_validate.cpp`'s `checkTransmissionReciprocity`).
4. **Bidirectional path tracing with MIS (caustics)** — light-subpath/eye-subpath vertex connection (Veach & Guibas 1995; Veach 1997); the transport algorithm caustics need, since unidirectional path tracing (1) cannot produce them at all. Blocked on (1) + (3)'s reciprocity fix, since connection needs BSDF agreement in both directions.
5. **Spectral upgrade** — per-wavelength transport, hero-wavelength sampling (Wilkie et al. 2014), spectral dispersion. Likely offline-only given sample-budget cost.
6. **Denoising** — needs (1)-(5) transport correctness first; denoising an incorrect image just smooths the error.
7. **GenAI diffusion channel** — img2img refinement AOV + raw latent/embedding output for HOST's cognitive pipeline. Needs (6)'s converged image.
8. **Upscaling** — spatial/temporal supersampling (neural, Xiao et al. 2020, §5, or classical).
9. **GPU ray-tracing backend** — Embree SYCL or CUDA-OptiX, to raise achievable sample budget beyond CPU Embree. Today's single-ray `rtcIntersect1`/`rtcOccluded1` calls and per-ray inline shading in `tracePath` (`path_tracer.cpp:77`) are the opposite of a wavefront/streaming GPU kernel design — this item is that rearchitecture, not just a backend swap.
10. **Production-scale scene/asset pipeline** — out-of-core streaming, distributed rendering, real multi-asset scene graph (today: one glTF + one HDRI), on the scene-graph foundation under **Moderate**. Broader materials (layered BSDF, hair, cloth) need (3).

### Low priority

1. **Motion blur** — blocked on (10)'s scene graph + Embree multi-timestep geometry.
2. **Depth of field** — thin-lens sampling in `primaryRay` + focus distance; technically unblocked today, cheaper once adaptive sampling (above) lands.
3. **Nuke-equivalent exposure/gamma control** — extend `OcioDisplayTransform` with live numeric control, beyond today's LUT cycling (`L`).

## 5. References

- Khronos Group. glTF 2.0 specification — scene/mesh/material interchange format.
- Mikkelsen, M.S. (2008). Simulation of wrinkled surfaces revisited — MikkTSpace tangent space standard; not implemented (tangent mapping uses glTF-supplied tangents only, no MikkTSpace generation).
- Goral, C.M., Torrance, K.E., Greenberg, D.P., Battaile, B. (1984). Modeling the interaction of light between diffuse surfaces. SIGGRAPH — the Cornell box, named in §4's Cornell box roadmap item, not yet implemented (no area lights exist to author its emissive panel with).
- Kajiya, J.T. (1986). The rendering equation. SIGGRAPH.
- Veach, E. (1997). Robust Monte Carlo Methods for Light Transport Simulation. PhD thesis, Stanford — multiple importance sampling, next-event estimation, and the MIS-weighted vertex connection named in §4's bidirectional-path-tracing roadmap item, not yet implemented.
- Veach, E., Guibas, L.J. (1995). Bidirectional estimators for light transport. Eurographics Rendering Workshop — light-subpath/eye-subpath vertex connection, named in §4's bidirectional-path-tracing roadmap item, not yet implemented.
- Pharr, M., Jakob, W., Humphreys, G. Physically Based Rendering: From Theory to Implementation (PBRT) — source of `FrDielectric`, the exact unpolarized dielectric Fresnel formula the BSDF uses.
- Cook, R.L., Torrance, K.E. (1982). A reflectance model for computer graphics. ACM ToG — BRDF and Fresnel foundations.
- Walter, B. et al. (2007). Microfacet models for refraction through rough surfaces — the GGX distribution, and the rough-refraction BTDF (value eq. 21, half-vector eq. 16, Jacobian eq. 17) the transmission lobe implements in PBRT-v3's radiance-transport form.
- Debevec, P. (1998). Rendering synthetic objects into real scenes — HDR image-based lighting.
- Wilkie, A. et al. (2014). Hero wavelength spectral sampling. Computer Graphics Forum — the sample-budget-bounding scheme named in §4's spectral-upgrade roadmap item, not yet implemented.
- OpenEXR / Academy Software Foundation technical documentation — linear HDR pipeline, exposure.
- Wald, I., Woop, S., Benthin, C., Johnson, G.S., Ernst, M. (2014). Embree: A Kernel Framework for Efficient CPU Ray Tracing. ACM ToG (SIGGRAPH) — the CPU ray-scene intersection kernel library (`EmbreeAccel`) backing BVH build/traversal, replacing an earlier hand-rolled binned-SAH implementation.
- Heitz, E. (2014). Understanding the Masking-Shadowing Function in Microfacet-Based BRDFs — the Smith height-correlated visibility term (`bsdf.cpp`'s `smithG1`/`smithG2`).
- Christensen, P.H., Jarosz, W. (2016). The Path to Path-Traced Movies. Foundations and Trends in Computer Graphics and Vision — production path-tracing grounding.
- Heitz, E. (2018). Sampling the GGX Distribution of Visible Normals. JCGT 7(4) — the VNDF importance-sampling routine the specular lobe uses.
- Sobel filtering — edge-detection AOV computed from Luminance (§3) — arXiv:2601.16806.
- Chiang, M.J.-Y., Li, Y., Burley, B. (2019). Taming the Shadow Terminator. JCGT 8(4) — the shading-point correction used for secondary-ray origins.
- Halton, J.H. (1960). On the efficiency of certain quasi-random sequences of points in evaluating multi-dimensional integrals; Cranley, R., Patterson, T.N.L. (1976). Randomization of number theoretic methods for multiple integration — `Sampler`'s radical inverse + per-pixel rotation.
- O'Neill, M.E. (2014). PCG: A Family of Simple Fast Space-Efficient Statistically Good Algorithms for Random Number Generation — `Sampler`'s fallback generator beyond its low-discrepancy dimension budget.
- Arvo, J., Kirk, D. (1990). Particle Transport and Image Synthesis — Russian roulette path termination.
- Gulbrandsen, O. (2014). Artist Friendly Metallic Fresnel — considered for conductor Fresnel, not implemented (Schlick approximation used; no authored edge-tint parameter exists in this asset set).
- Dupuy, J., Benyoub, A. (2023). Sampling Visible GGX Normals with Spherical Caps; Tokuyoshi, Y., Eto, K. (2023). Bounded VNDF Sampling for Smith-GGX Reflections — newer VNDF refinements surveyed, not implemented (Heitz 2018 used instead — better-established, lower risk to reproduce correctly from reference material alone).
- Heitz, E., Hanika, J., d'Eon, E., Dachsbacher, C. (2016). Multiple-scattering microfacet BSDFs with the Smith model — the source of the problem, not the solution used: it establishes the energy single-scatter GGX discards (a white conductor returned 0.31 of the light it received at roughness 1.0), but the implemented compensation is Kulla & Conty's cheaper directional-albedo form below rather than this paper's stochastic microsurface evaluation.
- Kulla, C., Conty, A. (2017). Revisiting Physically Based Shading at Imageworks. SIGGRAPH course — the multiple-scattering energy compensation the BSDF implements: a directional-albedo table drives a compensation lobe returning exactly the deficit `smithG2` masks away, on the reflective and transmissive interface alike, with its own cosine sampling strategy on the transmit side.
- Burley, B. (2020). Practical Hash-based Owen Scrambling. JCGT 9(4) — named in §4's low-discrepancy-sampler-upgrade roadmap item (Sobol + Owen scrambling), not yet implemented (depends on precomputed direction-number tables; randomized Halton used instead).
- Schüßler, V., Heitz, E., Hanika, J., Dachsbacher, C. (2017). Microfacet-based normal mapping for robust Monte Carlo path tracing — considered for normal-map robustness, not implemented (a simpler geometric-normal-consistency rejection is used instead: a normal-map-induced light-leak sample is absorbed rather than reconstructed via the full two-facet microsurface model).
- Jensen, H.W., Marschner, S.R., Levoy, M., Hanrahan, P. (2001). A Practical Model for Subsurface Light Transport. SIGGRAPH; Christensen, P.H., Burley, B. (2015). Approximate Reflectance Profiles for Efficient Subsurface Scattering — BSSRDF and its practical diffusion-profile approximation, named in §4's volumetric-and-subsurface roadmap item, not yet implemented.
- Novák, J., Georgiev, I., Hanika, J., Jarosz, W. (2018). Monte Carlo Methods for Volumetric Light Transport Simulation. Computer Graphics Forum (EG STAR) — participating-media survey, named in §4's volumetric-and-subsurface roadmap item, not yet implemented.
- Cook, R.L., Porter, T., Carpenter, L. (1984). Distributed Ray Tracing. SIGGRAPH — the stochastic-sampling origin of depth of field and motion blur, named in §4's depth-of-field and motion-blur roadmap items, not yet implemented (primary rays are pinhole and instantaneous; the camera's aperture and shutter drive exposure only).
- Williams, L. (1983). Pyramidal Parametrics. SIGGRAPH — MIP-mapping, named in §4's texture-minification-filtering roadmap item, not yet implemented (textures are point/bilinear-sampled per ray only).
- Zwicker, M. et al. (2015). Recent Advances in Adaptive Sampling and Reconstruction for Monte Carlo Rendering. Computer Graphics Forum (EG STAR) — denoising/reconstruction survey, named in §4's denoising roadmap item, not yet implemented.
- Belcour, L. (2018). Efficient Rendering of Layered Materials using an Atomic Decomposition with Statistical Operators. ACM ToG — layered-BSDF approach, named in §4's production-scale roadmap item's broader-material-coverage note, not yet implemented (materials are single-layer metallic-roughness only).
- Bitterli, B., Wyman, C., Pharr, M., Shirley, P., Lefohn, A., Jarosz, W. (2020). Spatiotemporal reservoir resampling for real-time ray tracing with dynamic direct lighting (ReSTIR). SIGGRAPH — named in §4's global-illumination roadmap item, not yet implemented (NEE currently samples the environment map only, with no area lights to resample across yet).
- Miller, G. (1994). Efficient algorithms for local and global accessibility shading. SIGGRAPH — ambient occlusion, named in §4's global-illumination roadmap item's ray-traced-AO note, not yet implemented (the AO AOV currently samples a baked texture, §3).
- Xiao, L., Nouri, S., Chapman, M., Fix, A., Lanman, D., Kaplanyan, A. (2020). Neural supersampling for real-time rendering. SIGGRAPH — upscaling, named in §4's upscaling roadmap item, not yet implemented.
- Ho, J., Jain, A., Abbeel, P. (2020). Denoising diffusion probabilistic models. NeurIPS; Rombach, R., Blattmann, A., Lorenz, D., Esser, P., Ommer, B. (2022). High-resolution image synthesis with latent diffusion models. CVPR — diffusion-model foundations, named in §4's genAI-diffusion-channel roadmap item, not yet implemented.
- Pineda, J. (1988). A parallel algorithm for polygon rasterization. SIGGRAPH — the edge-function incremental rasterization technique behind the primary-hit rasterizer (§1, §2, `rasterizer.cpp`).
- Sutherland, I.E., Hodgman, G.W. (1974). Reentrant polygon clipping. CACM — the near-plane polygon clip the primary-hit rasterizer's triangle setup uses (`rasterizer.cpp`).
