# Physically based path tracer

*A CPU, unidirectional Monte Carlo path tracer with real-time progressive display: Embree-accelerated, stochastic BSDF sampling combined with environment-map NEE via MIS, converging interactively behind a thin OpenGL display/HUD layer.*

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
- BSDF evaluation/sampling (Heitz 2018 GGX VNDF specular, cosine-weighted diffuse, exact dielectric Fresnel + Snell transmission with TIR, Schlick conductor Fresnel) → next-event estimation against the environment map, MIS-combined with BSDF sampling (power heuristic, Veach 1997)
- Recursive bounce loop with Russian roulette, Chiang/Li/Burley 2019 shadow-terminator-corrected secondary-ray origins
- Radiance + a full G-buffer/transport-component AOV set accumulated per pass, published as a snapshot the render thread reads lock-free

Independently, on the same trigger, a synchronous CPU rasterizer (`rasterizer.cpp`) computes the 15 primary-hit-only AOVs (§3) directly on the render thread every frame — edge-function rasterization (Pineda 1988) with a near-plane Sutherland-Hodgman clip, sharing `gbuffer_shading.h`'s material sampling with the path tracer but no Embree, BSDF, or recursion. This gives those AOVs instant, glitch-free feedback during camera movement, decoupled from Beauty's own progressive convergence; the path-traced request above only restarts when the selected AOV actually needs light-transport data.

The render thread blits whichever AOV is selected through OCIO's display transform (exposure/tone-mapping) and the debug HUD, converging visibly over subsequent passes rather than blocking on a single long render. No GPU rasterization anywhere in this path — OpenGL exists only for the window, the post-process/OCIO blit, and ImGui; the primary-hit rasterizer above is CPU-only.

## 2. Component reference

| Feature | Mechanism | Role / why it matters |
|---|---|---|
| Camera / lens | Position/yaw/pitch, film-back + focal length → derived vertical FOV; pinhole primary rays derived directly from this basis | Geometric ground truth the ray-intersection/BSDF math is measured against |
| Photographic exposure | EV100 from aperture/shutter/ISO (Filament/Frostbite calibration) | Physically meaningful brightness control, independent of arbitrary scene scaling |
| OpenEXR linear pipeline | `HdrImage`/`loadExr`; all shading/compositing in linear light, OCIO display-encodes only at the final blit | Precondition for correct PBR colour math |
| Tone-mapping / display transform | OCIO Display/View API (sRGB, Rec.709, Raw), cycled at runtime ('L') | Compresses unbounded HDR radiance into a displayable range without clipping |
| GPU/system readout | Device name, driver/API version, refresh rate, RAM at startup | Confirms the actual GPU/backend before a wrong-adapter bug masquerades as a render bug |
| Frame-timing HUD | Ring buffer of recent frame times; rolling FPS/avg/min/max, GPU timer query around the post-process blit | Makes blit cost measurable frame to frame |
| Memory HUD | Live RAM readout plus GPU allocation tracked at alloc/free (the path-traced display texture is the only GPU allocation left — OCIO uses zero LUT textures) | Surfaces a memory regression immediately, not after VRAM exhaustion |
| Scene stats | Object/triangle/point counts, viewport resolution | Scene-complexity readout |
| Debug camera controls | WASD/QE fly, R reset, LMB-drag orbit around a pivot read directly from the path tracer's own G-buffer (world-space hit position + hit mask at its centre pixel) | Interactive navigation without hand-editing camera parameters between runs |
| Camera framing overlays | Letterbox mask, centre crosshair ('K'), 3×3 rule-of-thirds grid, drawn over the viewport | Composition aids that never contaminate the AOV buffers being debugged |
| AOV selector | Dropdown across the full AOV set (§3), plus R/G/B channel-isolation hotkeys | Isolates one signal at a time for debugging |
| Live histogram | Per-channel (R/G/B) histogram of the currently displayed image | Catches exposure/clipping and colour-space bugs a single still frame can hide |
| glTF loading | cgltf; per-primitive vertices baked to world-space triangles/shading data at load time, materials' textures decoded once to `HdrImage` | Standard interchange format; nothing GPU-resident is needed once the CPU Embree scene/shading data exists |
| Tangent-space normal mapping | Per-vertex tangent (glTF-supplied only), Gram-Schmidt re-orthogonalized per-ray | Surface micro-detail without extra geometry |
| Ray acceleration | Intel Embree (SIMD BVH build/traversal), CPU, built once at load | Sub-linear ray-scene intersection, required before recursion is affordable |
| Primary-hit rasterizer | CPU edge-function rasterization (Pineda 1988) + near-plane clip (Sutherland-Hodgman 1974), row-parallel, synchronous every frame | Instant primary-hit G-buffer AOVs (§3), decoupled from Beauty's progressive convergence |
| Stochastic BSDF | Lambertian diffuse, GGX microfacet specular, smooth (delta) dielectric transmission via Snell + TIR; three-lobe stochastic selection, one-sample mixture estimator | Materials respond to light with real physical behaviour, including glass |
| Environment lighting | Equirect HDR map, BSDF-sampled misses + luminance-importance-sampled NEE, MIS-combined | The scene's sole light source — image-based, no punctual/area lights |
| Russian roulette | Survival probability clamped from running throughput from `russianRouletteStartBounce`, reweighted by `1/p` | Keeps recursion finite without biasing the estimator |
| Progressive accumulation | Each background pass re-traces at the current camera/settings and averages into the displayed result; any camera/scene change restarts accumulation | Real-time-interactive without waiting for a single long render to finish |

## 3. AOV reference

Every AOV below is computed by the path tracer each pass, except: the 15 primary-hit-only AOVs (Alpha, Depth, WorldPos, UV, Normal, GeomNormal, Albedo, Metallic, Roughness, Tangent, ObjectID, Fresnel, IOR, AO, Wireframe), which come from the synchronous CPU rasterizer (§1, §2) instead, refreshed every frame; and HSV/Luminance/Sobel/Gabor, GPU post-filters of the Beauty image (shared `PostProcessPass`, run once per selection, not per-pass).

| AOV | Category | Mechanism | Role / why it matters |
|---|---|---|---|
| Beauty | Utility | Final accumulated radiance, post tone-mapping | The primary output |
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
| Metallic | Material | Metallic factor (scalar only) | Debug material authoring independent of shading |
| Roughness | Material | Roughness texture × factor, floored at 0.045 | Debug material authoring independent of shading |
| Tangent | Material | Shading tangent basis at the primary hit | Debugs the tangent-space basis used for normal mapping |
| ObjectID | Material | Per-instance index, false-coloured (`falseColorForId`) | Isolation mask for compositing/debugging |
| AO | Material | Authored ambient-occlusion texture sample at the primary hit | Debug baked AO independent of lighting |
| Shadow | Utility | Binary NEE occlusion test toward the env light at the primary hit, re-averaged across progressive passes into continuous shadow/penumbra density | Isolates direct-light visibility from material/lighting colour |
| Wireframe | Utility | Screen-space line rasterization (Pineda 1988), z-tested against the scene's own depth: white mesh-triangle edges, yellow scene-bounding-box edges (drawn on top, so yellow wins) | Visualizes triangle density/topology and sanity-checks scene extent/framing in one combined view |
| Fresnel | Transport | Schlick term at the primary hit's view angle | Debug grazing-angle reflectance behaviour in isolation |
| IOR | Transport | Per-material dielectric IOR at the primary hit, -1 on a miss | Isolates the raw refractive-index input driving Fresnel/transmission |
| BounceCount | Transport | Mean path termination depth across samples, per pixel | Debug Russian roulette/termination behaviour |
| DirectDiffuse | Lighting | Diffuse-bucketed radiance from a path's first (bounce-0) surface, delighted (no base colour) | Isolates direct diffuse light arrival from the object's own texture |
| IndirectDiffuse | Lighting | Diffuse-bucketed radiance from later bounces | Isolates indirect (bounced) diffuse contribution |
| DirectSpecular | Lighting | Specular-reflection-bucketed radiance, one bounce from camera | Isolates direct specular contribution |
| IndirectSpecular | Lighting | Specular-reflection-bucketed radiance, later bounces | Isolates indirect specular (reflections) |
| Refraction | Transport | Radiance from any path that sampled a transmission lobe (sticky bucket) | Isolates glass/transmissive transport |

## 4. Roadmap

Unimplemented, in dependency order — each item lands on top of a feature-complete version of everything above it, not before it, since it either multiplies the cost of the existing transport/sampling or assumes it's already correct.

0. **Global illumination** — Area lights (emissive geometry, sampled by solid angle) and shadow/visibility rays to them; caustics (specular-diffuse-specular paths), emergent once area lights exist alongside the existing recursion/refraction. NEE, MIS, and importance-sampled lighting are already implemented, but only against the environment map — extending them to area lights is what's left. ReSTIR (spatiotemporal reservoir resampling, Bitterli et al. 2020) follows directly once multiple area lights exist, since uniform/single-light NEE selection degrades past a handful of emitters. Ray-traced ambient occlusion — a short-range hemispherical visibility integral reusing the same NEE cosine-hemisphere sampling this item already needs, just capped to a short max ray distance instead of a full light query — also belongs here, replacing today's AO AOV (§3), which only samples a baked texture rather than tracing anything.
1. **Volumetric & subsurface transport** — Participating media (fog/smoke/atmosphere: extinction/scattering coefficients, phase functions, ray-marched or null-collision free-flight sampling) and subsurface scattering (translucent materials: a BSSRDF or random-walk diffusion approximation) are both additional transport modes layered on the existing surface path tracer — extending the same NEE/MIS machinery to volume-embedded and beneath-the-surface light paths rather than replacing it.
2. **Multi-scattering microfacet energy compensation** — The specular lobe is single-scatter GGX only (Heitz, Hanika, d'Eon, Dachsbacher 2016, §5); at high roughness this loses energy compared to a real rough conductor/dielectric, visible as an over-dark specular response. A compensation term or explicit multi-bounce microfacet simulation closes the gap without changing the transport algorithm.
3. **Spectral upgrade** — Spectral light transport (per-wavelength radiance instead of RGB), hero-wavelength sampling (Wilkie et al. 2014) to keep the sample budget from exploding, spectral materials, spectral dispersion (Cauchy/Sellmeier index of refraction). Likely reserved for offline validation renders given the sample-budget cost, with the current RGB path kept for interactive use.
4. **Real-time integration** — Adaptive per-pixel sample budget driven by a variance/convergence estimate: today every pass traces every pixel at a fixed `samplesPerPixel`; progressive accumulation already delivers "real-time, converges over time" without it, so this is a refinement (spend the budget where noise remains, not everywhere uniformly), not a gap. Upscaling — spatial or temporal supersampling (neural, Xiao et al. 2020, §5, or classical) to reach a target display resolution at a fraction of the traced-ray cost — is the same real-time-quality trade in a different axis (perceived resolution instead of perceived sample count) and belongs alongside it. Also: texture minification filtering (MIP-mapping, Williams 1983, §5 — textures are currently point/bilinear-sampled per ray with no pre-filtering, so a grazing-angle or distant surface can alias); a GPU ray-tracing backend (Embree's own SYCL/GPU path, or CUDA-OptiX) for scene traversal/BSDF evaluation to raise the achievable sample budget beyond what CPU Embree delivers, which would also need a non-OpenGL-timer-query GPU timing mechanism and revisit whether GPU-resident/compressed textures are worth reintroducing.
5. **Chromatic aberration** — A per-pixel radial RGB channel offset, toggled on/off as a `PostProcessPass` filter over the Beauty AOV (the same GPU post-filter path HSV/Sobel/Gabor already use, §3) rather than a new render pass. Dispersion strength is derived from the camera's existing lens properties (`focalLengthMm()`, `aperture()`, §2) rather than an author-facing parameter, since a longer focal length / wider aperture already implies stronger real-lens CA. A display-side lens artifact rather than a change to scene radiance, so it has no ordering dependency on the transport items above it.
6. **Denoising** — A reconstruction filter (edge-aware spatial/temporal, or a learned kernel-predicting network, Zwicker et al. 2015, §5) to reach a clean image at a fraction of the fully-converged sample count. Sequenced after the transport/GI items above: it's a variance-reduction layer on top of a correct, unbiased estimator, not a substitute for one — denoising an incorrect image just produces a smooth incorrect image.
7. **GenAI diffusion channel** — Two related outputs built on the same diffusion model, sequenced after denoising/real-time since both want a reasonably converged image as input: a generative-refinement pass (an img2img diffusion model, Ho et al. 2020 / Rombach et al. 2022, §5, run over the converged Beauty AOV) exposed as its own selectable AOV alongside the existing HSV/Sobel/Gabor post-filters (§3); and a raw latent/embedding output — the model's internal feature representation of the frame, exposed as a data buffer rather than a viewable image, for downstream consumption by HOST's cognitive-architecture/embodied-agent pipeline rather than the HUD viewer.
8. **Production-scale scene/asset pipeline** — Out-of-core geometry streaming and a real multi-object/instanced scene graph (today: one configured glTF asset and one HDRI, loaded whole into memory at startup, with no scene graph beyond that single file's own node hierarchy) needed before scenes larger than a single asset are practical; deterministic distributed rendering across machines for feature-length throughput. Broader material coverage — layered BSDFs, hair/fur, cloth — belongs here too, alongside whatever transport (items 0–2 above) those material types need to look correct.

## 5. References

- Khronos Group. glTF 2.0 specification — scene/mesh/material interchange format.
- Mikkelsen, M.S. (2008). Simulation of wrinkled surfaces revisited — MikkTSpace tangent space standard; not implemented (tangent mapping uses glTF-supplied tangents only, no MikkTSpace generation).
- Kajiya, J.T. (1986). The rendering equation. SIGGRAPH.
- Veach, E. (1997). Robust Monte Carlo Methods for Light Transport Simulation. PhD thesis, Stanford — multiple importance sampling, next-event estimation.
- Pharr, M., Jakob, W., Humphreys, G. Physically Based Rendering: From Theory to Implementation (PBRT) — source of `FrDielectric`, the exact unpolarized dielectric Fresnel formula the BSDF uses.
- Cook, R.L., Torrance, K.E. (1982). A reflectance model for computer graphics. ACM ToG — BRDF and Fresnel foundations.
- Walter, B. et al. (2007). Microfacet models for refraction through rough surfaces — GGX BSDF.
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
- Heitz, E., Hanika, J., d'Eon, E., Dachsbacher, C. (2016). Multiple-scattering microfacet BSDFs with the Smith model — named in §4's multi-scattering-compensation roadmap item, not yet implemented (the specular lobe is single-scatter only).
- Burley, B. (2020). Practical Hash-based Owen Scrambling. JCGT 9(4) — considered for the sampler (Sobol + Owen scrambling), not implemented (depends on precomputed direction-number tables; randomized Halton used instead).
- Schüßler, V., Heitz, E., Hanika, J., Dachsbacher, C. (2017). Microfacet-based normal mapping for robust Monte Carlo path tracing — considered for normal-map robustness, not implemented (a simpler geometric-normal-consistency rejection is used instead: a normal-map-induced light-leak sample is absorbed rather than reconstructed via the full two-facet microsurface model).
- Jensen, H.W., Marschner, S.R., Levoy, M., Hanrahan, P. (2001). A Practical Model for Subsurface Light Transport. SIGGRAPH; Christensen, P.H., Burley, B. (2015). Approximate Reflectance Profiles for Efficient Subsurface Scattering — BSSRDF and its practical diffusion-profile approximation, named in §4's volumetric-and-subsurface roadmap item, not yet implemented.
- Novák, J., Georgiev, I., Hanika, J., Jarosz, W. (2018). Monte Carlo Methods for Volumetric Light Transport Simulation. Computer Graphics Forum (EG STAR) — participating-media survey, named in §4's volumetric-and-subsurface roadmap item, not yet implemented.
- Cook, R.L., Porter, T., Carpenter, L. (1984). Distributed Ray Tracing. SIGGRAPH — the stochastic-sampling origin of motion blur (and depth of field), named in §4's motion-blur roadmap item, not yet implemented.
- Williams, L. (1983). Pyramidal Parametrics. SIGGRAPH — MIP-mapping, named in §4's real-time-integration roadmap item as the texture-minification filter not yet implemented (textures are point/bilinear-sampled per ray only).
- Zwicker, M. et al. (2015). Recent Advances in Adaptive Sampling and Reconstruction for Monte Carlo Rendering. Computer Graphics Forum (EG STAR) — denoising/reconstruction survey, named in §4's denoising roadmap item, not yet implemented.
- Belcour, L. (2018). Efficient Rendering of Layered Materials using an Atomic Decomposition with Statistical Operators. ACM ToG — layered-BSDF approach, named in §4's production-scale roadmap item's broader-material-coverage note, not yet implemented (materials are single-layer metallic-roughness only).
- Bitterli, B., Wyman, C., Pharr, M., Shirley, P., Lefohn, A., Jarosz, W. (2020). Spatiotemporal reservoir resampling for real-time ray tracing with dynamic direct lighting (ReSTIR). SIGGRAPH — named in §4's global-illumination roadmap item, not yet implemented (NEE currently samples the environment map only, with no area lights to resample across yet).
- Miller, G. (1994). Efficient algorithms for local and global accessibility shading. SIGGRAPH — ambient occlusion, named in §4's global-illumination roadmap item's ray-traced-AO note, not yet implemented (the AO AOV currently samples a baked texture, §3).
- Xiao, L., Nouri, S., Chapman, M., Fix, A., Lanman, D., Kaplanyan, A. (2020). Neural supersampling for real-time rendering. SIGGRAPH — upscaling, named in §4's real-time-integration roadmap item, not yet implemented.
- Ho, J., Jain, A., Abbeel, P. (2020). Denoising diffusion probabilistic models. NeurIPS; Rombach, R., Blattmann, A., Lorenz, D., Esser, P., Ommer, B. (2022). High-resolution image synthesis with latent diffusion models. CVPR — diffusion-model foundations, named in §4's genAI-diffusion-channel roadmap item, not yet implemented.
- Pineda, J. (1988). A parallel algorithm for polygon rasterization. SIGGRAPH — the edge-function incremental rasterization technique behind the primary-hit rasterizer (§1, §2, `rasterizer.cpp`).
- Sutherland, I.E., Hodgman, G.W. (1974). Reentrant polygon clipping. CACM — the near-plane polygon clip the primary-hit rasterizer's triangle setup uses (`rasterizer.cpp`).
