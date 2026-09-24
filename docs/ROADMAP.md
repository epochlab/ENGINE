# Roadmap

## Large

#### Transport: strict dependency order

1. **Volumetric & subsurface transport**: participating media (in-scattering, phase functions) plus BSSRDF/random-walk subsurface. Beer-Lambert extinction already ships for tinted glass (`mediumSigmaA`) — a subset, not this item. What blocks the rest is structural: the transmissive multi-scatter lobe is not per-direction reciprocal, which is inherent to the formulation rather than a bug (`checkTransmissionReciprocity` is deliberately scoped to single scatter).
2. **Global illumination**: area lights and shadow rays to them ship — `LightSet` generalises NEE to a uniformly-selected light set, and `QuadLight` is solid-angle sampled (Ureña/Fajardo/King 2013), injected into the Embree scene so it occludes and is BSDF-hittable. Still needed: disk/sphere lights, emissive-mesh lights, power-weighted selection, and ReSTIR (Bitterli et al. 2020), which needs many lights before resampling is worth anything. Caustics do **not** fall out of this — see (4).
3. **GPU ray-tracing backend**: Embree SYCL or CUDA-OptiX. This is a rearchitecture, not a backend swap: today's single-ray `rtcIntersect1`/`rtcOccluded1` and per-ray inline shading in `tracePath` are the opposite of a wavefront kernel. It subsumes packet tracing, ray reordering and deferred material-sorted shading — all three are GPU-side wins and none is worth doing on CPU.
4. **Bidirectional path tracing with MIS (caustics)**: light/eye subpath vertex connection (Veach & Guibas 1995). Unidirectional path tracing structurally cannot sample specular-diffuse-specular paths at any light count. Blocked on (2) and on (1)'s reciprocity, since connection needs BSDF agreement in both directions.
5. **Spectral upgrade**: per-wavelength transport and hero-wavelength sampling (Wilkie et al. 2014). Dispersion ships in RGB form (one hero *channel* per path); two parts remain.
   - (a) A stochastic per-path wavelength draw, replacing `kRgbWavelengthsNm`'s one fixed wavelength per channel. "No transport change" holds only if lambda is drawn proportional to the channel sensitivity `S_c(lambda)`; anything else leaves an `S_c/p` factor in the throughput and puts the dispersive unbiasedness gate at risk. The open problem: the Rec.709 `r` sensitivity has a negative lobe — unavoidable with real primaries, and not a density that can be sampled.
   - (b) Radiance is still RGB, so a true spectral integrator also needs spectral upsampling of textures and environment maps (Jakob & Hanika 2019). Likely offline-only.
6. **Denoising**: needs (1)–(5) first — denoising an incorrect image smooths the error.
7. **Upscaling**: spatial/temporal supersampling, neural (Xiao et al. 2020) or classical.
8. **GenAI diffusion channel**: img2img refinement AOV plus raw latent output for HOST's cognitive pipeline. Needs (6)'s converged image.

#### HOST integration

The consumer is `notes/agent.md`'s retina sensor, feeding `notes/neural.md`'s cognitive loop. Transport #8 names the same consumer at the far end of the transport spine — a richer tier of this interface, not a separate one.

1. ~~**Headless `pathtracer_core` library**~~ — **done**. One static library with no GL/GLFW/imgui, linked by every executable; the ~18 hand-maintained per-target source lists are gone, and `render_beauty` no longer drags in glfw and imgui. Verified by a bit-identical `render_beauty` EXR and PNG before and after, and by `bench_compare` resolving no timing change.
2. **Retina sensor and synchronous step**: `agent.md` §2's point field — N rays from a position and heading, one sample each, no jitter, packing the active tier's colour channels plus depth and normal per point. Still open, and it is now the *only* open piece of the sensor path: (1) and (3) below ship a full-frame camera rendering the same AOVs, which is the simplification HOST is currently built on. What (2) adds is the bounded-FOV point field itself, sized 32²–128², in place of a rectangular image. `agent.md` §3's tiers 1–5 are all reachable today — tiers 1–2 (Luminance, Sobel) became so when those filters gained CPU implementations, having previously existed only as GL shaders.
3. ~~**Python binding**~~ — **done**. `python/pathtracer` over a plain C ABI (`include/pathtracer/api/pathtracer_c.h`) driven by `ctypes`, exposing scene load, all 28 AOVs, resolution, sample count, seed and the full camera. No third-party C++ dependency and no Python headers at build time, and `ctypes.CDLL` releases the GIL per call. Output buffers are numpy-allocated and handed down as pointers, so nothing is freed across the boundary and `torch.from_numpy` is zero-copy. Python's Beauty is asserted bit-identical to `render_beauty --out-exr` at the same seed. Still open: sharing one BVH and one thread pool across N vectorized environments — today each `Renderer` owns a pool sized to `hardware_concurrency()`, which oversubscribes by N.
4. **Optic flow AOV**: per-pixel screen-space motion vectors. `neural.md` §3 gives the Retina stage direction-selective ganglion cells for motion, and today the only motion signal is whatever the 2–4 frame buffer leaves a downstream CNN to infer — the renderer already knows the exact answer. Reprojection of the existing WorldPos AOV through a stored prior-frame view-projection, so no new intersection work. The caveat that parked this inverts here: flow is camera-only until a scene graph exists, which is *complete* for a static scene with a moving agent, exactly the HOST case. Also a shared prerequisite for transport #6 and #7, both of whose temporal variants need motion vectors, and it extends the packed signal (2) defines and (3) exposes, so it lands after them.

Offline export is not one of these items and ships with them: `render_beauty --out-exr` writes any of the 28 AOVs to disk, which covers dataset generation and debugging.

## Next

**Colour** — the CIE 1931 module ships (`cie.h`: 2° observer, D65 at 1 nm, the RP 177 matrix, checked by `colour_validate`), compiled into `metal_fit` and `colour_validate` only.
- **Out-of-gamut measured metals**: `metal_fit` rejects gold — its CIE-projected normal-incidence red is 1.038 in linear Rec.709, so no Gulbrandsen `r < 1` represents it and clamping would silently change hue and edge tint. Cu/Ag/Ni/Fe/Ti all fit. Needs a decision: a stated gamut mapping, or a wider rendering space (ACEScg) with the OCIO scene space to match.

**Validation**
- **Occlusion-sensitive curved transmissive test**: reverting either curvature-scaled epsilon (`transmissionOffsetEpsilon`) leaves the whole suite green — measured deltas 2e-4 and 4e-3, not a tessellation artefact. Needs rough glass against an opaque backdrop, which has no closed form, so it needs a converged-reference or two-estimator invariant rather than an analytic one.
- **Grazing-angle transmission**: `transmissive_sphere_energy` scores a sphere at `mu` 1 only. Binning the disc radially reads 0.9450 at 90% of the silhouette radius and 0.9020 at 95% at roughness 0.2, against 1.0000 at the centre — non-monotonic in roughness, absent by 0.7. Instrument first (a radial band assertion), then the loss it exposes. Gates authoring rough glass; no shipped material reaches it.
- **Histogram coverage**: `histogram.cpp` is FBO/PBO-bound with no CPU-reachable binning function. Needs the bin arithmetic extracted first.
- **Band calibration**: derived bands are mutation-verified but have no standing false-rejection-rate measurement over many seeds.
- **`-bench` logs the interactive accumulation on fast scenes**: stage 0 assumes the throwaway 0.1-scale accumulation is still running at the settle promotion. Should complete only on a settled full-scale generation.
- **`-bench` refuses fast passes under vsync**: one `PassRecord` slot per frame, so two passes inside one 16.7 ms frame fail the contiguity check and nothing is logged. The driver should append to a queue the render thread drains. Both `-bench` gaps only ever reproduced on a local 4K asset that no longer exists, so neither is currently observable on `cornell.json`.

**Performance** — each item justified by the benchmark log ([components](../README.md#benchmark-tooling)).
- **Display-texture upload rebuilds the whole image every published pass** (measured via the benchmark log, [components](../README.md#benchmark-tooling)). `ensurePathTraceDisplayTexture` (`main.cpp`) re-uploads the full RGBA32F buffer whenever the cache key changes, firing once per completed pass. That is 37.7 MB at 2048x1152, measured at **3.6-7.4 ms, median 4.6 ms** on the render thread over the 128 uploads of one `pathtracer -bench` convergence. Most of it is the driver's float-to-half conversion, not the transfer (46% on cornell, 68% on the stump). `displayBitDepth: 32` measured `upload_ms` **0.54x [0.50, 0.83]** of 16 on cornell and **0.32x [0.31, 0.33]** on the stump (`results/wave7`), so a 16-bit display copy only pays off if the conversion leaves the render thread. Remaining options, none tried: convert to half on the trace workers and upload `GL_HALF_FLOAT`, a PBO so the copy leaves the render thread, or uploading only the tiles the pass actually rewrote. Decide each with `bench_compare run --metric upload_ms` over `pathtracer -bench`.
- **Scene textures carry more channels than they use**: every map is stored RGBA whatever it encodes, so roughness and bump (read as `.r` only, `gbuffer_shading.cpp`) hold 4x their payload, and `aoTexture` is loaded and held but never read now that AO is path-traced (`material.h`) — 268 MB at float32 on the stump's 4K map, more than `textureBitDepth: 16` saves on it. Channel count is the next step of the data type `textureBitDepth` started, followed by the framebuffers (`HdrImage` AOVs, still float RGBA).
- **Blue-noise sample matrix beyond d = 1**: the dither mask is a scalar void-and-cluster array, so a pixel's d-dimensional shift is one value replicated along the torus diagonal and relative shifts lie on a line. Georgiev & Fajardo Sec. 3 anneals a true d-vector matrix; adopting it changes the baked table and its lookup, not the sampler.

**Hygiene** — after `bsdf.cpp` churn settles.
- **Refactor the over-length functions**: `readability-function-size` fires on 16, measured once the header filter was fixed (2026-09-24) -- the earlier count of 11 was taken while headers and several translation units went unchecked. `tracePath` (320 lines) and `computeLobeProbabilities` (141) are the extremes; the rest are `renderPathTraced`, `sampleBsdf`, `driverLoop`, `buildSphericalRectangle`, `loadProfileConfig`, `readExrRgba`, `drawStageRows`, `HeadlessRenderer::open`/`render`, and five in `main.cpp` (`main`, `initializeApp`, `presentFrame`, `renderFrame`, `updateHud`). These are hot-path functions and splitting them risks the image, so this needs its own bit-identical before/after verification. Skip `tracePath`/`renderPathTraced` if Large #3 is near. Sweep the two trivial idiom fixes in with it (`rotateAboutY` → `glm::rotate`, `ShadingFrame::toLocal`/`toWorld` → `glm::mat3`).
- **Documentation pass**: the build/target inventory, the static-analysis gates and third-party provenance now ship in the README and `third_party/README.md` (2026-09-24). Still open: an architecture reference for the module layout, and write-ups of the techniques in use.
- **Memory efficiency pass**: only monitoring exists today (Memory HUD), no active reduction initiative. Candidates once profiled: `gltf_loader.cpp`'s de-indexed mesh soup, which stores every position twice (once in `Triangle` for Embree, once in `ShadingVertex` for shading), and texture bit depth (Performance, above).

**BSDF model**
- **Smith-exact transmissive multiple scattering** (distribution only, no energy error): the transmitted compensation is a Kulla-Conty shape — right in total, only approximately right in direction. Measured against a Heitz et al. 2016 stochastic Smith random walk as total-variation distance: at `eta` 1.5 Kulla-Conty 0.052 beats single scatter 0.061 and Turquin 0.081, so the shipped model is best where shipped glass lives; at 1.33 they tie; below that it loses badly (0.076 against 0.021 at `eta` 1.01), because the multiply-scattered share collapses toward `-wo`, which a `(1-Escape)cos` row cannot express at any scaling. This supersedes the `eta = 1` discontinuity concern: the tabulated deficit does not tend to zero either side (0.20865 at 0.999, 0.21014 at 1.001), but the total is 1 on both sides so no furnace row sees it, and the far side is a shape the sweep shows is wrong anyway. Also here: the transmission `G2` is the reflection-form height-correlated term where Heitz 2014 gives a Beta function; the shipped form overstates first-order transmission by up to 80%, and adopting it alone makes the distribution worse (mean TV 0.066 → 0.098) because the energy it correctly removes moves into the approximate compensation shape. It lands with that shape, not before it.

## Parked (low value, or needs a use-case first)

One line each; the detail behind a closed measurement lives in the CHANGELOG entry named beside it.

**Camera and sampling**
- **Depth of field**: thin-lens sampling in `primaryRay` plus a focus distance. Unblocked today; cheaper after adaptive sampling.
- **Motion blur**: blocked on a scene graph/animation foundation that does not exist, plus Embree multi-timestep geometry.
- **Adaptive per-pixel sample budget**: variance-driven; `samplesPerPixel` is one fixed global today.
- **Render-mode selector**: Single Sample / Progressive, selectable from `profile.json`.
- **Fisheye lens**: equidistant/equisolid-angle/orthographic/stereographic projection families in `primaryRay` (Kannala & Brandt 2006, [references](../README.md#references)).
- **Physical camera filters**: CPL/polarising filters, which need polarised transport (Chandrasekhar 1960, [references](../README.md#references)).

**Geometry and texture**
- **Hierarchical frustum culling**: reject per instance against `instanceBounds` before `buildSubTriangles` walks the whole scene.
- **Texture minification filtering (MIP-mapping)**: point/bilinear only today; needs a mip chain and ray differentials to pick a level.
- **High-frequency binary noise texture/material**: the stress asset that makes the aliasing above visible before and after.

**Colour**
- **Kelvin light temperature**: author a light by colour temperature via the Planckian locus, instead of a hand-picked RGB triple. Wave 2 consumer.
- **Photometric calibration**: tie radiometric output to lux/candela/lumen through the CIE ȳ curve, so `ev100()` can be checked against a light meter. Wave 2 consumer.
- **Macbeth ColorChecker scene**: the reference chart both items above are validated against. No such scene exists, and `cornell.json` is now the only one in the repo.

**AOVs and display**
- **Spherical Harmonics AOV**: no SH infrastructure exists anywhere; needs a use-case writeup before it can be sized.
- **Bloom PostFX**: bright-pass, blur and additive composite in the existing `PostProcessPass`/OCIO blit, as chromatic aberration already does.
- **Contact sheet export**: every AOV at once, as a single displayed sheet. The headless half of this now ships -- `HeadlessRenderer` runs both producers plus the filters in one call, so `render()` already returns any subset of the 28 together -- leaving only the viewer side, where `aovNeedsLightTransport` still parks one producer while the other is shown.
- **Cache the post-filter AOVs across unchanged frames**: Gabor/Sobel/HSV/Luminance re-run every displayed frame even when Beauty has not changed.
- **Depth's auto-range maximum is a serial per-texel render-thread scan**: fix by publishing the range from `RasterGBuffer`, as `OverRangeStats` already is. Off the Beauty path entirely.

**Tooling and assets**
- **PNG capture tool hardening**: `render_beauty`'s `writePng` is RGB-only and its scanline filter is hardcoded to None despite a comment claiming otherwise.
- **Example images of pathtracer technology (gallery)**: small demo scenes, each isolating one feature, rendered and embedded in this README.
