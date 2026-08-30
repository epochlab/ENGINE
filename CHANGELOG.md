# Changelog

## Phase 0 — Foundation

### A — Scaffolding
- chore: CMake project scaffold (C++20, `engine` + `gen_test_pattern` targets)
- chore: Pitchfork dir layout (`include/engine/{platform,gfx,scene}`, `src/`, `assets/`, `tools/`)
- chore: stub sources for all planned modules (window, gl_debug, shader_program, mesh, texture, hdr_framebuffer, post_process_pass, ocio_display_transform, camera)
- deps: `glm`, `opencolorio` via Homebrew

### B — Window & GL context
- feat: `Window` (GLFW, GL 4.1 core fwd-compat, RAII, move-only, resize-callback hook)
- feat: `gl_debug` — `checkError`, `GL_CALL` macro, `khrDebugAvailable`
- feat: main loop — glfwInit/GLEW init → poll/clear/swap → glfwTerminate

### C — Camera
- feat: `Camera` — position/yaw/pitch, film back + focal length → derived vertical FOV, `viewMatrix`/`projectionMatrix`
- note: aperture/shutter/ISO/exposure deferred to Stage F (first real consumer is the OCIO exposure uniform)

### D — HDR FBO + polygon
- feat: `Mesh` (RAII VAO/VBO/EBO, `createQuad`), `ShaderProgram` (`loadFromFiles`/`loadFromSource` → `optional`), `Texture` (`GL_RGBA16F` upload, placeholder checkerboard)
- feat: `HdrFramebuffer` (RGBA16F color + depth renderbuffer, completeness check, `resize`/`bind`)
- feat: `PostProcessPass` — attribute-less-VAO fullscreen-triangle blit
- feat: `quad.vert`/`.frag`, `fullscreen_triangle.vert`, `passthrough.frag` (placeholder, superseded by Stage F's OCIO shader)
- feat: render loop now draws checkerboard quad into HDR FBO, blits to screen; `Window`'s resize callback wired to `HdrFramebuffer::resize`
- note: unencoded checkpoint (expected washed out until Stage F)

### E — Test EXR
- feat: `gen_test_pattern` — writes 700x100 calibration EXR (black/18%grey/white/R/G/B/ramp)
- feat: `Texture::createFromExr` — `RgbaInputFile` load, half→float, exception boundary → `optional`
- feat: quad now displays `test_pattern.exr`
- note: `*.exr` now gitignored — generated assets are local-only, not committed
- fix: `Texture::createFromExr` now flips row order on load — EXR row 0 is the image top, but `glTexImage2D` row 0 is texture `v=0` (bottom); previously every EXR-loaded texture rendered upside down, invisible on the row-uniform `test_pattern.exr`, confirmed on a real HDRI

### F — OCIO viewer LUT + exposure
- feat: `OcioDisplayTransform` — sRGB/Rec.1886-Rec.709 viewer LUTs via OCIO's real Display/View API (`Un-tone-mapped` view, no filmic tone-mapping), built once at startup, no LUT textures
- feat: `Camera` gains `aperture`/`shutterSeconds`/`iso`, `ev100()`, `exposure()` (Filament/Frostbite EV100 model)
- feat: `Window::setKeyCallback`; debug `L` key cycles sRGB → Rec709 → Raw (unencoded passthrough)
- fix: corrected an earlier design assumption — OCIO 2.5.2 has no raw `CURVE - LINEAR_to_sRGB` builtin; empirically verified the correct construction against the installed library
- note: exposure uniform left neutral (EV=0) rather than seeded from `Camera::exposure()` — the calibration pattern isn't scene-referred radiance

### G — Verify
- feat: `logColorCheck` — `glReadPixels` numeric check (black/grey/white/ramp) vs hand-computed bytes, logs at startup and on every LUT toggle
- chore: removed `Texture::createPlaceholderCheckerboard` and `assets/shaders/passthrough.frag` (zero remaining callers/loads)
- verified: sRGB grey=118, Rec709 grey=125, Raw grey=46 (all match hand-computed expectations); ramp monotonic; resize correct, no retina 2x bug

**Phase 0 — Foundation complete.**

## Phase 1 — Debug HUD & system feedback

### A — Vendor ImGui
- deps: Dear ImGui v1.92.9 (`third_party/imgui` git submodule, not `docking`) — first non-Homebrew dependency; vendored as source since Homebrew has no `find_package`-consumable ImGui+backends
- chore: CMake wiring — ImGui core + GLFW/OpenGL3 backend sources added to `engine`, `IMGUI_IMPL_OPENGL_LOADER_GLEW` (reuses GLEW's resolved GL pointers instead of ImGui's own loader), configure-time guard for an uninitialized submodule

### B — ImGui bring-up
- feat: `Window::nativeHandle()` accessor; ImGui's GLFW backend chains to `Window`'s already-installed key callback with zero changes to `window.cpp`
- feat: `engine::debug::HudOverlay` — owns the ImGui context + GLFW/OpenGL3 backend lifetime, composites after the OCIO tonemap pass (never into the linear HDR FBO)
- fix: stale "Phase 1's WASD/QE/R debug camera" comment in `window.h` → Phase 3, matching the README renumbering

### C — GPU/system readout
- feat: `engine::debug::system_info::queryGpuInfo()` — `GL_VENDOR`/`GL_RENDERER`/`GL_VERSION` + monitor refresh rate, queried once at startup
- verified: Apple M1 / 4.1 Metal - 90.5

### D — Frame-timing HUD
- feat: `engine::debug::FrameStats` — 120-entry ring buffer, fps/avg/min/max
- feat: `engine::debug::GpuTimer` — double-buffered `GL_TIME_ELAPSED` query, non-blocking read; `gpuTimerQueryAvailable()` runtime check follows `khrDebugAvailable()`'s don't-assume precedent
- feat: `Mesh::triangleCount()`; geometry + post-process passes each wrapped in a `GpuTimer` in `main.cpp`
- verified: ~118-120 FPS (caps at monitor refresh), sparkline live, geom/post ms and Mtri/Mpix per s all plausible

### E — Memory HUD
- feat: `engine::debug::memory_tracker` — `residentSetBytes()` (`mach_task_basic_info`), one atomic GPU-byte counter (`trackGpuAlloc`/`trackGpuFree`/`gpuAllocatedBytes()`)
- feat: byte-tracking hooks in `Texture`, `Mesh`, `HdrFramebuffer`; move ctor/assign now exchange the tracked byte count alongside the GL handle, else a move silently double-frees the *accounting* (not the GL object)
- verified: GPU alloc MB hand-computed vs HUD readout matches exactly (2048x1152 HDR FBO + EXR texture ≈ 27.5 MB); resizing tracks proportionally with no upward drift

### F — Styling pass
- feat: borderless/pinned-top-left HUD panel, cyan section headers, dark near-opaque background, no `imgui.ini` written

### G — Verify
- verified: visual match against reference layout; `L`-key LUT toggle regression-checked (sRGB→Rec709→Raw still correct post-ImGui-integration)

**Phase 1 — Debug HUD & system feedback complete.**

## Phase 2 — Geometry, textures & basic material

### A — glTF loading
- deps: vendor cgltf (single-header C99 parser, jkuhlmann/cgltf v1.15, MIT) — two plain files, not a submodule; one-shot asset load, not a hot path
- feat: `Mesh`'s Vertex layout gains normal/tangent (vec4, `.w` = bitangent handedness), attribute locations 2/3
- feat: `Texture::createFromFloatPixels` — mipmaps, `GL_REPEAT`, trilinear filtering; `createFromExr`'s row flip dropped (EXR's row-0-top already matches glTF's v=0-top)
- feat: `engine::scene::loadGltf()` — position/normal/uv/tangent via cgltf's own accessor helpers, recursive node-transform accumulation, all 6 material textures resolved via `Texture::createFromExr`; roughness/bump/specular (no standard glTF slot) read back from the material's `extras` JSON via a small targeted string scan
- fix: `extrasTextureIndex`'s integer parse hardened (`from_chars`, not `atoi` — `atoi` can't distinguish "parsed 0" from "failed to parse")
- verified: the real 2.55M-vertex/5M-triangle/6-EXR test asset loads in ~1.1s (Release build)

### B — First real draw
- feat: the loaded stump model replaces the placeholder quad; new `pbr.vert`/`pbr.frag` (unlit base-color only for now), per-instance `uModel` + a CPU-computed `uNormalMatrix`
- chore: removes `Mesh::createQuad()`, `quad.vert`/`.frag`, the test-pattern texture load, `logColorCheck()` — all zero remaining callers
- fix: depth testing scoped to just the scene draw (enabled/disabled per frame) — the post-process present pass draws at a fixed NDC z, so leaving depth test on for it would depend on undefined leftover depth-buffer contents
- fix: vsync re-enabled — an uncapped CPU submitting draw calls for this 5M-triangle scene faster than the GPU could drain them reproduced a genuine GPU driver hang on the test machine
- verified: 50+s continuous runtime, ~35-37 FPS, ~24-28ms real GPU geometry time/frame, no hangs; numeric pixel-sampling of screenshots confirms a coherent tree-stump silhouette renders in the expected viewport region

### C — Culling & material completeness
- feat: `Mesh` computes a model-space AABB once at construction; `frustumIntersectsAabb` (Gribb/Hartmann plane extraction) added as infrastructure
- feat: frustum culling wired into the render loop, reusing the per-instance world-space AABB already computed for the World Position AOV
- fix: previously-dead `Material::baseColorFactor` now actually multiplied into shading
- perf: per-instance world-space AABBs precomputed once after load instead of every frame

### D — AOV selector & shading
- feat: AOV debug dropdown (Beauty, Albedo, Normal, GeomNormal, Roughness, UV, WorldPos, Tangent, Metallic, ObjectID, AO) + independent R/G/B channel isolation; non-Beauty AOVs force OCIO's Raw passthrough, restoring the user's LUT choice on Beauty
- feat: active LUT shown in the debug HUD
- feat: tangent-space normal mapping (bump-derived detail normal blended in) + basic Lambertian/Fresnel shading (F0 = mix(specular, baseColor, metallic)) against one fixed test light
- feat: authored AO texture multiplied into Beauty shading — a practical simplification (no ambient term exists yet to occlude), not physically exact
- chore: default asset switched to tier1 LOD (36.5k triangles) for faster shader iteration

### E — Verify
- chore: final Phase 2 cleanup pass — removed a dead `<cstdlib>` include and comments that had drifted out of sync with the code they described
- docs: README corrected to match Phase 2's actual implementation (AOV selector moved from Phase 3, normal-mapping description corrected, AOV table's AO/Metallic/Roughness rows corrected)
- build: default to Release when `CMAKE_BUILD_TYPE` is unset — Debug measured ~40x slower on this project's CPU-bound glTF load (47s+ vs ~1.1s)

**Phase 2 — Geometry, textures & basic material complete.**

## Phase 3 — Scene controls

### A — Data-driven scene setup
- feat: minimal JSON scanner (`json_scan.cpp`) + `scene.json`/`profile.json` loaders — scene/camera setup now data-driven instead of hardcoded in `main.cpp`

### B — Debug camera & histogram
- feat: `DebugCameraController` — WASD/QE fly, R reset, LMB-drag orbit around a depth-sampled pivot (`HdrFramebuffer::sampleDepth`)
- feat: `engine::debug::Histogram` — double-buffered PBO readback of the current AOV, live RGB channel display
- feat: scene stats readout (draw calls, triangle counts, viewport resolution); camera & lens readout, debug camera controls, and viewport/scene stats wired into the HUD
- chore: fly speed tuned from 3.0 to 1.0 m/s for precise navigation at this scene's scale

### C — Remaining utility AOVs
- feat: Alpha, Depth (planar camera-space Z, Arnold/RenderMan/OpenEXR convention), Luminance, and HSV AOVs added
- feat: Sobel/Gabor edge-detection AOVs — new two-pass `edge_filter.frag` (fixed 3×3 Sobel kernel / 4-orientation Gabor bank) consuming the Luminance AOV, reusing the existing generic `PostProcessPass`; Gabor kernel weights precomputed once on the CPU
- refactor: Beauty's shading math factored into `shadeBeauty()`, shared across Beauty/Luminance/HSV/Sobel/Gabor without duplicating it in branches that don't need it
- chore: all 17 AOV indices renumbered to match the README's §3 AOV reference table order

### D — System readout
- feat: GPU refresh rate (`glfwGetVideoMode`) and system RAM (`sysctl`/`host_statistics64`, resampled on the existing 250ms throttle) added to the debug HUD

**Phase 3 — Scene controls complete.**

## Phase 4 — Direct lighting & acceleration

- feat: BVH — binned-SAH, CPU-built, one-time (`bvh.h/.cpp`); Phase 5 infrastructure only, not yet wired into rendering, validated by `tools/bvh_validate.cpp` against a brute-force reference
- feat: punctual lighting — point/directional light list (replacing the single fixed test light), analytic Cook-Torrance GGX (D, Smith height-correlated visibility, Schlick Fresnel)
- feat: IBL — SH-9 diffuse irradiance (Ramamoorthi & Hanrahan 2001, no texture) + GGX-prefiltered specular cubemap (Karis 2013 split-sum) + analytic DFG polynomial approximation (Lazarov 2013, no baked LUT) + Turquin 2019 multiplicative multi-scatter energy compensation
- fix: two real energy-conservation bugs caught by `tools/furnace_test.cpp` (Monte Carlo furnace test) — a missing diffuse/specular energy split, and compensation overshoot at high roughness/near-white F0 (now clamped to the DFG approximation's measured accuracy)
- feat: screen-space AO dropped from scope (a baked AO pass already exists); AO now modulates only the new ambient/IBL term
- feat: HUD sky background toggle (raw equirect) and a 0–359 degree HDR rotation control — diffuse and specular both rotate via the query direction rather than re-baking, so sky/diffuse/specular stay in sync at zero extra bake cost
- feat: IBL AOV shows lighting only (diffuse albedo excluded; specular keeps its real F0)
- feat: `scene.json`'s single `light` object replaced by a `lights` array (empty is a valid IBL-only scene); `json_scan` gains array-of-objects parsing
- refactor: debug HUD panel reordered (Histogram/AOV moved above Camera), Sky/HDR rotation controls moved into their own HDRI section
- docs: README updated throughout (phase table, component table, AOV table, open questions, references)

**Phase 4 — Direct lighting & acceleration complete.**

## Phase 5 — Materials & recursive transport

### A — Plumbing
- feat: `ShadingTriangle`/`ShadingVertex` (`shading_scene.h/.cpp`) — per-triangle world-space normal/uv/tangent/position, parallel-indexed to `Bvh`'s triangle list; built in `gltf_loader.cpp` from data `Mesh` would otherwise discard once uploaded to the GPU
- feat: `Bvh::Hit` gains barycentric u/v (Moller-Trumbore convention, w=1-u-v on v0)
- feat: `EnvironmentMap` retains the equirect `HdrImage` the rasterizer's IBL bake discards after startup, for path-traced miss-ray lookups; direction-to-UV mapping matches `sky.frag` exactly
- feat: `MaterialTexture` — every material texture now retained on both GPU (rasterizer) and CPU (`HdrImage`, path tracer's per-sample UV lookups); `HdrImage::sampleBilinear` added
- feat: `Material::ior`/`transmissionFactor` (`KHR_materials_ior`/`KHR_materials_transmission`)
- feat: `SceneConfig` gains `samplesPerPixel`/`maxBounces`/`russianRouletteStartBounce`
- chore: `Texture::createFromExr` removed — dead code once `MaterialTexture` decodes an EXR once and reuses it for both the GPU upload and the CPU copy

### B — Sampler + BSDF core
- feat: `Sampler` — randomized Halton (radical inverse + per-pixel Cranley-Patterson rotation) for the first 32 dimensions, PCG32 (O'Neill 2014) beyond that
- feat: stochastic metallic-roughness BSDF (`bsdf.h/.cpp`) — exact dielectric Fresnel (PBRT's `FrDielectric`) + Snell refraction with TIR, Schlick conductor Fresnel, Heitz 2018 GGX VNDF importance sampling, three-way stochastic lobe selection (rough specular reflection / Lambertian diffuse / smooth delta transmission), combined via a one-sample mixture estimator
- feat: `tools/bsdf_validate.cpp` — pdf-normalization and furnace-test checks, including colored conductors and the transmissive dielectric's exiting/TIR side
- fix: three real energy-conservation bugs caught during development — diffuse lobe missing its (1-F) factor; transmission's `transmissionFactor` cancelling out of its own throughput ratio; diffuse lobe's pdf incorrectly gated on the same term as its value (starved the MIS mixture denominator specifically for colored, non-white conductors)
- fix: opaque materials (`transmissionFactor=0`) no longer trigger transmission/TIR logic when grazing-angle normal mapping pushes the local view direction to the "wrong" side of the shading normal
- note: Sobol+Owen scrambling, Tokuyoshi & Eto 2023's bounded VNDF sampling, and Heitz et al. 2016's stochastic multi-scatter random walk were the original plan but are deliberately not implemented — each depends on precise constants/derivations not safely reproducible from memory alone; the better-established alternatives above are used instead, with the originals kept as documented future upgrades (README §5)

### C — Integrator
- feat: `Camera::primaryRay` — pinhole ray generation sharing `viewMatrix()`'s forward/right/up basis
- feat: `renderPathTraced` (`path_tracer.h/.cpp`) — BVH intersect, material/shading resolution (tangent-space normal mapping only, no bump-detail blend), BSDF sampling, geometric-normal-consistency rejection (normal-map light-leak mitigation, in place of Schüßler et al. 2017's full two-facet reconstruction), Russian roulette from `russianRouletteStartBounce`, Chiang/Li/Burley 2019 shadow-terminator-corrected ray origins, environment radiance on a miss; row-parallel `std::thread`, one thread per hardware core, dynamic scheduling via an atomic row counter
- note: no NEE this phase (Phase 7 scope) — punctual lights have no hittable geometry, so path-traced radiance comes from the environment map only

### D — Engine integration
- feat: on-demand "Path Traced" HUD section (Enable checkbox, Beauty/IOR/BounceCount AOV combo distinct from the rasterizer's own, Render button, status readout)
- feat: `presentFrame` blits the cached path-traced result through the existing OCIO/post-process path when active; rasterizer keeps running underneath regardless — deliberate simplification, this is an on-demand feature, not real-time
- feat: `Texture::id()` public accessor, needed for the path-traced display texture to reuse `PostProcessPass::draw`'s raw-GL-id present path

### E — Verify
- verified: `bvh_validate`/`furnace_test`/`bsdf_validate` all pass; visual smoke test in the running app via a temporary auto-trigger hook (reverted before commit) — IOR AOV shows a correct stump silhouette against black with the LUT correctly forced to Raw; Beauty AOV shows a coherent, plausible image
- docs: README updated (phase table, component reference, AOV reference, references)

## Rasterizer removal — path tracer as sole renderer

Follow-on to the path tracer becoming the continuously-converging primary renderer (NEE/MIS against the environment map, full G-buffer/transport-component AOV set) — the GPU rasterizer it ran alongside as a fallback was by then fully redundant and is removed.

- feat: `presentFrame` shows the path-traced result exclusively, with a black clear until the first pass publishes; no rasterizer fallback branch
- feat: orbit-pick pivot resolved from the path tracer's own G-buffer (world-space hit position + hit mask at its centre pixel), replacing a GL depth-buffer readback + view/projection unprojection
- refactor: `Camera::viewMatrix`/`projectionMatrix`, `Window`'s resize callback, and `HdrFramebuffer`'s GL depth/colour attachments removed — nothing traces a raster depth or colour buffer anymore
- refactor: `Material`'s textures are CPU-only (`HdrImage`); GPU texture upload, `MeshInstance`'s GPU `Mesh`, and the bump-map detail-normal texture (rasterizer-only consumer) are gone
- refactor: punctual lights (`Light`, `kMaxLights`, `SceneConfig::lights`) removed — the rasterizer was their only consumer; the path tracer has always lit purely from the environment map
- refactor: `SceneStats` drops draw-call/culled counts (no per-instance culling without a raster draw loop); HUD drops the "Draw calls"/"Mtri/s" readouts and the GPU timer's geometry-pass split
- chore: deleted `pbr.vert/frag`, `sky.frag`, `equirect_to_cubemap.frag`, `prefilter_specular.frag`; `cubemap_texture`, `env_prefilter_pass`, `mesh`, `hdr_framebuffer`, `sh_irradiance`, `frustum` (header+source); `tools/furnace_test.cpp` and its CMake target (validated `pbr.frag`'s analytic BRDF/SH-IBL, both deleted; `bsdf_validate` already covers the path tracer's own BSDF)
- docs: README rewritten to describe the shipped CPU path tracer + thin OpenGL display/HUD layer directly, replacing the phase-by-phase dual-renderer build history

**Phase 5 — Materials & recursive transport complete.**

## HUD polish, new debug AOVs & config split

- feat: HDRI Exposure slider, per-AOV pixel probe (native units, on-screen composited value for Beauty/post-filter AOVs), configurable progressive-accumulation max-samples cap
- feat: Shadow, Wireframe, Bounding Box AOVs — binary NEE occlusion, barycentric edge distance, ray-vs-AABB slab test
- feat: `material.json`/`profile.json` split — renderer/material defaults moved out of `scene.json`; configurable texture/HDRI asset paths; Bump texture wired up (previously loaded, unused)
- fix: `maxBounces=0` now renders direct lighting only; pixel probe reads the true on-screen pixel; specular lobe isolated from diffuse in NEE's DirectSpecular/IndirectSpecular routing
- chore: comments consolidated to single-line, technical-only style; default ISO 400, `maxBounces` 0→1, max-samples cap enabled by default

## AOV correctness fixes

- fix: specular lobe's throughput isolated from diffuse admixture in DirectSpecular/IndirectSpecular AOV routing
- fix: Depth AOV auto-ranges display exposure to the actual max depth visible in-frame, replacing a `farClip`-based normalization that read real scenes as black; Depth/BounceCount no longer clamp to white; HUD pixel-probe units corrected
- fix: Shadow AOV re-averages across progressive passes (`PathTraceGBuffer` → `PathTraceDynamic`) instead of freezing as binary speckle from pass 1; confirmed HDR exposure correctly has no effect on it (occlusion is exposure-invariant)

## Synchronous CPU rasterizer for primary-hit AOVs

- feat: `rasterizer.h/.cpp` — row-parallel edge-function rasterization (Pineda 1988) with a near-plane Sutherland-Hodgman clip, computing the 15 primary-hit-only G-buffer AOVs synchronously every frame instead of via the async path tracer's convergence loop; `gbuffer_shading.h/.cpp` factors the material/shading sampling shared with the path tracer out of `path_tracer.cpp`
- feat: Wireframe and BoundingBox merged into one combined AOV — real screen-space line rasterization z-tested against scene depth (white mesh edges, yellow bounding-box edges drawn on top), replacing the old per-pixel analytic distance tests
- feat: path-traced accumulation only restarts when the selected AOV actually needs light-transport data, avoiding wasted CPU on a raster-only AOV during camera movement
- chore: `tools/rasterizer_validate.cpp` cross-checks the rasterizer against a fresh Embree oracle, including box-edge occlusion and wireframe coverage
- chore: debug camera profile reframed (`filmBack` matched to window aspect, camera pulled back for a wider view), `maxSamples` settled at 4
- docs: README updated (pipeline, component reference, AOV table, roadmap item 0 retired now implemented)

## Light-transport correctness — MIS truncation, NEE ordering, sampling

- fix: the terminal BSDF-sampled ray is now traced for its environment miss — the loop previously sampled a direction at the final bounce and built a ray it never intersected, while NEE at that vertex stayed MIS-weighted down against a counterpart that never fired, losing `bsdfPdf^2/(bsdfPdf^2+lightPdf^2)` of its direct lighting (approaching 100% where `bsdfPdf >> lightPdf`, and applying to every second surface vertex at `maxBounces=1`); the loop now runs one iteration past `maxBounces`, guarded so it can only collect a miss, never shade
- fix: a failed `sampleBsdf` no longer skips that vertex's NEE — the two are independent estimators of independent directions, and `nullopt` (below-horizon VNDF reflection, underflowed mixture pdf, transmission lobe with no mass) says nothing about the BSDF's value toward the light; also stops the Shadow AOV reporting "occluded" where sampling merely failed
- fix: bounce-0 NEE writes both lobe buckets deterministically instead of routing by whichever lobe the continuation ray drew — that routing wrote one bucket per sample, thinning DirectDiffuse/DirectSpecular in expectation by each lobe's selection probability (a Fresnel- and view-angle-dependent factor clamped to [0.05,0.95]), so an evenly lit flat wall read as a Fresnel gradient
- fix: NEE samples environment radiance nearest-texel (`EnvironmentMap::sampleDirectionNearest`, sharing `equirectTexelOf` with `pdf()`), matching the piecewise-constant cell its pdf is derived from; bilinear there bled a bright texel's energy into neighbours whose density is correctly low, spiking `f/pdf` into fireflies at exactly the small bright features the luminance CDF exists to find. The miss path keeps bilinear, where the env pdf enters only a bounded MIS weight
- fix: Russian roulette survival ceiling 0.95 → 1.0 — a path carrying full throughput was killed 5% of the time and the survivors reweighted, unbiased but strictly variance-increasing for no compensating saving
- note: the four validation tools pass, but none can detect the energy loss the first two fixes remove — every furnace and pdf assertion is one-sided. A two-sided furnace test, a transmission round-trip test and an env-map pdf-consistency test follow separately

## Validation suite — integrator coverage and two-sided energy bounds

- feat: `tools/integrator_validate.cpp` — first test to exercise `renderPathTraced`/`tracePath` at all. One unoccluded quad under a uniform L0=1 environment has no indirect light, so `maxBounces=0` and `maxBounces=1` must agree and both must equal the analytic single-scatter integral; Russian roulette must not change the answer. Catches the MIS truncation fixed in the previous commit, which read 0.17 against a correct 0.97 at `maxBounces=0` — the "direct lighting only" mode was producing roughly one sixth of the correct radiance
- feat: two-sided white furnace test — a white non-absorbing surface under uniform radiance must return exactly 1.0, so a LOWER bound is what detects energy loss; the previous suite asserted only `Lo <= bound` everywhere and was structurally blind to it. Floors are a measured regression baseline, not a correctness target: single-scatter GGX retains 1.00/0.92/0.31 at roughness 0.05/0.5/1.0 (white conductor, normal incidence), quantifying the multiple-scattering compensation that is still outstanding
- feat: transmission round-trip test — asserts the non-symmetric `eta^2` radiance-compression factors cancel over enter-then-exit, the invariant `bsdf.cpp`'s own comment claims and nothing tested. Pairs the exiting leg at the Snell-refracted angle, not the incident one, which would put it past the critical angle
- feat: env-map pdf consistency test — `pdf(importanceSampleDirection(u).direction)` must equal that sample's own pdf, on a structured map under non-zero rotation. Untested before, and a mismatch silently corrupts every MIS weight in the renderer
- fix: the pdf-normalization check's `ndotV<0` rows integrated the +z hemisphere while `pdfBsdf` mirrors `wi` into `wo`'s hemisphere, so they measured exactly 0 and passed `<= 1.0` vacuously — the exiting side was never tested. Now integrates the hemisphere the density actually occupies
- fix: corrected a comment claiming `Material`/`MeshInstance` need a live GL context; both are plain data, and that claim was why the suite had no integrator-level test
- chore: `enable_testing()` + `add_test` — all five tools now run under `ctest --test-dir build`; previously they were buildable but nothing invoked them
- note: the pdf check stays upper-bound-only by design. VNDF reflection sampling is not normalized over the hemisphere (below-horizon samples are discarded), so the true integral is the horizon-clipped mass, which has no closed form — the white furnace test measures it instead

## Energy conservation — multiple scattering and BSDF reciprocity

- feat: Kulla-Conty multiple-scattering compensation (Kulla & Conty 2017) — single-scatter GGX discards the energy `smithG2` masks away and never returns it, so a white conductor reflected 0.31 of the light it received at roughness 1.0. A compensation lobe `Fms*(1-E(mu_o))*(1-E(mu_i))/(pi*(1-Eavg))` returns exactly that deficit, driven by a 32x32 directional-albedo table built once at startup (a few ms) by deterministic stratified quadrature of the VNDF weight identity `G2/G1`
- feat: reciprocal diffuse/specular coupling — the diffuse substrate now receives `1 - coatAlbedo` on the way in *and* out, replacing the one-sided `1 - F(mu_o)`. Restores Helmholtz reciprocity (`f(wo->wi) == f(wi->wo)`), required by every bidirectional transport algorithm on the roadmap, and fixes a second energy loss: at roughness 1.0 / `ndotV` 0.4 the macro Fresnel is 0.129 while the coat actually reflects 0.030, and that 10% went to neither lobe
- feat: the specular lobe-selection probability is scaled by `E(mu_o, roughness)`, so VNDF sampling takes the single-scatter share and cosine sampling takes the (cosine-shaped) multiple-scatter share. Without it a rough white metal, whose Fresnel pins `specularProb` to the 0.95 clamp, would sample 69% of its own reflectance only 5% of the time. No new sampling lobe, so the one-sample mixture estimator is unchanged and still unbiased
- feat: the albedo table stores the Schlick-basis split `Ess(f0) = f0*a + b` so one table serves any `f0`, with the single-scatter term rescaled by exact-dielectric/Schlick Fresnel at each direction — without that rescale a smooth white dielectric *creates* ~1.4% energy at grazing, since Schlick under-predicts exact Fresnel there and the substrate is handed the difference
- feat: white furnace floors are now a correctness target, not a regression baseline — 1.0 +/- 0.02 on both bounds, across 28 cells. Half sit deliberately off the table's grid, where the measured value is `E_true + (1 - E_interpolated)` and so tests interpolation error directly, which is why the table needs no public accessor
- feat: `checkReciprocity` — an equality to float precision, not a statistical bound, since every term is symmetric by construction after the coupling change. Fails at 96 of 144 sampled geometries on the parent commit, worst case 0.306 vs 0.179
- note: measured white-furnace energy is now within +/-0.9% of unity at every roughness, angle and material tested, from a prior range of 0.31-1.00. The residual is Monte Carlo noise plus table interpolation
- note: `BsdfSample::rawThroughputWeight` reports a cosine-drawn multiple-scattering sample as diffuse, so the delighted Direct/Indirect Specular AOVs under-count it. Beauty is exact either way — `throughputWeight` is the physical value — and the delighted split is removed entirely when the transport buckets become a true partition
- note: transmission keeps its one-sided `(1-F)*t` energy fraction and its delta lobe; rough transmission and its own multiple-scattering compensation are a separate change

## Rough transmission — Walter 2007 lobe and transmissive energy compensation

- feat: rough specular transmission (Walter et al. 2007, PBRT-v3's radiance-transport form) replacing a delta lobe at every roughness — `alpha` was computed in the transmission branch and never referenced, so `roughness` silently did nothing on any transmissive material while the reflection lobe on the *same* interface was rough GGX. Frosted glass is now authorable
- feat: below `alpha < 1e-3` transmission stays the original delta lobe, matching PBRT's `EffectivelySmooth`. The existing `kMinAlpha` floor sits inside that region, so smooth glass keeps the exact, noise-free Snell path rather than becoming a stochastic estimate of the same thing — verified by the transmission round-trip test producing byte-identical output. At roughness 0.05 the new rough path reproduces the delta path to 0.2%
- feat: the albedo table gains reflected/transmitted escape channels indexed by (roughness, mu, eta), built with **exact** dielectric Fresnel. A transmissive interface loses energy on a different curve from a reflective one (0.559 vs 0.307 at roughness 1.0) because the below-horizon reflections that drive `E` down are the *valid* side for refraction, so the existing table could not describe it. One axis in `log(eta)` covers entering and exiting, since the two are reciprocals; the VNDF samples are shared with the existing build loop, so the third axis costs only the refraction
- feat: multiple-scattering compensation for the transmissive interface, deficit `1 - (R + T)` redistributed reciprocally across both hemispheres and blended against the opaque compensation by `transmissionFactor*(1-metallic)`, so an opaque material keeps exactly the measured behaviour it already had
- fix: `coatFresnelRatio` hardcoded the entering orientation, so total internal reflection was invisible to the escape budget on the exiting side. Schlick's basis cannot express TIR at all — exact Fresnel is 1.0 inside the cone where Schlick reads ~0.1 — which is why the new channels use exact Fresnel rather than a rescaled Schlick split
- fix: the deficit ratio floored only its denominator, breaking the numerator/denominator cancellation as roughness fell and turning a vanishing lobe into a huge one (a round trip reading 1.54 where it should read 1.02); now guarded rather than clamped
- note: energy balance is within ~1% to roughness 0.4 but falls to 0.68 at roughness 1.0 on the TIR-heavy exiting side. The multiple-scattering term sits after `evaluateTransmissionLobe`'s geometric rejections, and those directions have no transmission sampling density — moving it earlier would place energy that can never be sampled, trading a shortfall for bias. Closing it requires giving the multiple-scattering lobe its own cosine sampling strategy on the transmissive side, which is the next change. `checkFurnace` remains upper-bound-only for transmissive materials, so this is not yet gated
- note: the integrator still treats every transmission sample as a delta for MIS purposes (`lastSampleWasTransmission`), and NEE does not yet sample the back side of a transmissive surface. Both follow with the sampling-strategy change

## Transmissive energy conservation — multiple-scattering sampling and back-side NEE

- feat: the transmit-side multiple-scattering lobe gains its own cosine sampling strategy over the far hemisphere, carved out of the transmit selection mass in proportion to the energy each carries. That is what lets the compensation move outside `evaluateTransmissionLobe`'s half-vector rejections and be delivered over the whole hemisphere rather than the refraction-reachable cone alone — energy balance goes from 0.68 to 1.0 at roughness 1.0 exiting. Both strategies gate on the same deficit threshold, so no selection mass ever reaches a lobe of zero value and smooth glass is untouched
- fix: the rough transmission single-scatter value omitted `transmissionFactor`, so a `transmission=0.5` dielectric refracted at full strength on top of a diffuse substrate already scaled by `(1-transmissionFactor)` — 1.42 of the energy it received. Both that factor and `(1-metallic)` now come from `transmitWeight`, which also drops `transmissionFactor` on the exiting side to match `transmitProb` and `transmitPhysicalValue`: inside the medium there is no substrate to withhold anything, so a ray must reflect internally or exit
- fix: a rough transmission sample is MIS-eligible and no longer bypasses the power heuristic. `lastSampleWasTransmission` becomes `lastSampleWasDelta`, derived from `pdfBsdf` returning exactly 0 rather than from the lobe type, which removes the special case entirely
- fix: NEE now samples both sides of a transmissive vertex. The `geoCos > 0 && shadingCos > 0` guard restricted light samples to `wo`'s side, so rough glass lit from behind got BSDF sampling alone — and, paired with the delta MIS bypass, double-counted the overlap where NEE did reach the far side of an exiting vertex. Bounce-0 far-side NEE writes the Refraction bucket
- test: `checkTransmissiveEnergyBalance` — `sampleBsdf` with transmitted draws converted back to the energy domain, where 1.0 is correct at every roughness, side and `transmissionFactor`. `checkFurnace` could see neither defect: it is upper-bound-only, and the entering side's `eta^2` compresses 1.42 to 0.95, under its 1.0 bound. Measured 1.0 within 0.7% across 64 cells
- test: `checkTransmissiveSlab` — a white non-absorbing slab in a uniform environment must be invisible. Two quads with opposing normals are the only configuration in the suite that reaches a transmissive *exiting* vertex; a single quad reads as entering at every hit. Measures 1.22 at roughness 1.0 on the parent commit against 0.99 after
- test: `checkTransmissionReciprocity` — the eta^2-corrected invariant `f_t(wo->wi)*eta_wi^2 == f_t(wi->wo)*eta_wo^2`, which is what holds across a refracting interface in place of Helmholtz symmetry. Catches a misplaced `etaR^2`, a flipped `denom` orientation or an un-flipped `ht`: O(1) errors the furnace and round-trip tests cannot see, since they assert totals in which the two sides' errors cancel. Inverting the correction fails all 360 pairs by exactly `ior^4`. Worst honest discrepancy 6.4e-3 at roughness 0.05 against a 1e-2 bound — `dD/D ~ 4/alpha^2` amplifies a few-ULP `ht` difference at these roughnesses, so `checkReciprocity`'s 1e-4 is not transferable. `wi` is constructed by refracting `wo` through the macro normal rather than sampled, since the lobe is a fraction of a degree wide and an arbitrary far-side direction would pass the check vacuously; the non-zero-pair count is asserted for that reason
- note: scoped to single scatter. `multiScatterShape` evaluates `escapeWi` at the wo-side `eta` for every `wi`, transmitted ones in the other medium included — which is precisely what makes its hemisphere integral come out at `(1 - escapeWo)` — but under the swap both `etaSq` and that `eta` flip. Isolated with no new accessor by testing only below `kMinDeficit`, where the lobe switches itself off. Recorded against README §4's volumetric-and-subsurface item: it blocks bidirectional transport through rough glass, not the unidirectional integrator
- docs: README corrected where this stack invalidated it — rough transmission and four-lobe selection in §1/§2, the multi-scattering-compensation roadmap item retired, Walter 2007 and Heitz et al. 2016 re-annotated as implemented/superseded, and Kulla & Conty 2017 added as the compensation actually used
- docs: new §4 roadmap item for reference scenes and material validation — a Cornell box (Goral et al. 1984) the renderer is measured against rather than merely rendered in, carrying the per-material showcase. `tools/` validates the BSDF and integrator analytically and nothing validates a full scene, so bucket misattribution, light leaks at shared edges and terminator artifacts are visible by eye only. Blocked on area lights (item 0): the environment map is the only light source, so an emissive panel cannot be authored
- docs: new §4 roadmap item for lens and shutter sampling — depth of field and motion blur, both Cook et al. 1984 primary-ray dimensions rather than post-process filters. `Camera` already carries `aperture()`, `focalLengthMm()` and `shutterSeconds()`, but they feed EV100 exposure only, so f/1.4 brightens without shallowing depth of field and the shutter never reaches the sampler. Fixes a dangling §5 citation: Cook 1984 was annotated "named in §4's motion-blur roadmap item" and no such item existed

## Interactive performance — rasterizer and display path

- test: `tools/raster_bench.cpp` — a timing harness for `renderRasterGBuffer`, the synchronous per-frame render-thread work behind the 15 primary-hit AOVs. Committed rather than thrown away, because every change in this section is a performance claim and the previous workstream's figures are unreproducible without rebuilding its benchmark from prose. Reports best-of-N (run-to-run spread here is +/-10%, wide enough to hide a single change); deliberately outside `add_test`, since a benchmark's pass/fail is a human reading a number
- test: the harness scene is synthetic and sweeps the two axes the rasterizer's cost is actually a function of, neither adjustable in a fixed asset: `--triangles` moves the sub-triangle array across the L2 boundary, and `--layers` sets depth complexity, which is what a depth prepass trades against. Per-layer triangle radius is derived from that layer's frustum cross-section, so raising `--triangles` shrinks triangles instead of piling up overdraw and the two axes stay independent
- note: baseline at the shipped 2048x1152 framebuffer, best-of-five: **135 ms per rasterization** at the scene's 20,561 triangles — and **136 ms at a single triangle**. The cost at the shipped triangle count is entirely fixed per-frame overhead, not scan conversion; triangle count first registers between 20K and 200K (401 ms), which is where the 84-byte-per-sub-triangle array crosses this machine's 12 MB L2. Any estimate of the row scan's cost that assumed the array streams from DRAM does not describe the shipped scene
- feat: `renderScale` and `interactiveRenderScale` in `profile.json` — the path tracer and rasterizer render at a fraction of the framebuffer and the display blit upscales, which it already did for free (`glViewport` targets the framebuffer, the display texture samples `GL_LINEAR`). A 1024x576 window is a 2048x1152 framebuffer on a Retina display, so the renderer was tracing 2.36M paths per pass for a window implying 590K
- feat: the renderer drops to `interactiveRenderScale` on any input change and promotes back to `renderScale` a quarter-second after the last one, the standard progressive-renderer trade of resolution for latency while the camera moves. The promotion needs no separate code path — it changes the trigger, and a changed trigger is already what dispatches
- fix: the trigger state is split into inputs and scale, because the scale is derived from whether the inputs changed. As one struct the promotion to full resolution would read as fresh interaction on the following frame, re-arming the timer it had just satisfied and pinning the renderer at the interactive scale permanently
- note: measured 151 -> 38.6 ms per rasterization at 2048x1152 vs 1024x576, best-of-five — 3.9x against an exact 4x pixel ratio. The path tracer takes the same 4x on paths per pass. Setting both scales equal reproduces the previous behaviour exactly, so the mechanism is inert if unwanted
- feat: the rasterizer runs only when one of its own 15 AOVs is selected. It previously ran unconditionally on every trigger change — a full-screen shade of 15 images on the render thread, on nearly every frame of camera interaction, producing nothing anybody was looking at whenever Beauty was displayed, which is the default. The two AOV sets partition the enum exactly (12 light-transport, 15 rasterizer-backed, `AovId::Count` 27), so the complement of the existing predicate is an exact gate
- feat: the orbit pivot comes from a single Embree ray down the view centre rather than the centre texel of the G-buffer, which is what had forced that rasterization to be unconditional — two million pixels shaded to read one. Also the more accurate instrument: no fill rule, no z-precision, no dependence on the resolution the G-buffer happened to be rendered at
- feat: `aov` joins the trigger state, so switching between two rasterizer-backed AOVs (Normal to Albedo changes neither the camera nor the light-transport predicate) still re-rasterizes
- perf: the G-buffer is allocated once and rendered into in place, each row cleared by the worker about to overwrite it instead of 15 sequential full-image memsets beforehand — 566 MB of allocate-and-zero per call at 2048x1152. Measured 151 -> 125 ms, best-of-five: real, but again an order of magnitude below what the byte count suggests, consistent with the previous section's finding
- fix: `RasterGBuffer` carries a `generation` counter, because reusing the buffer in place leaves its address constant and the display texture cached on exactly that pointer identity — it would have uploaded the first render and then never updated again. Verified separately that a reused, deliberately dirtied buffer produces bit-identical output to a fresh one across all 15 AOVs, which is the failure mode the per-row clear introduces
- perf: `renderRow` visits a per-row bucket of the sub-triangles whose bounding box covers that row, in flat CSR form, instead of scanning the whole sub-triangle array once per scanline. The cost removed is memory bandwidth rather than comparisons: `RasterSubTriangle` is 84 bytes and the scan is linear, so every row streamed the entire array — invisible while that array fits in cache, dominant once it does not
- perf: `buildSubTriangles` clips and projects in parallel over chunks of the triangle list rather than on one thread, each chunk appending to its own vector and the chunks concatenated in order afterwards. That order is what makes the result identical to the sequential build, since the z-test is first-writer-wins at exactly equal depth — verified by hashing all 15 AOVs across three camera poses on a scene seeded with coplanar triangles at equal depth, with and without the change
- note: measured at 2048x1152, best-of-five, by triangle count — 20,561 (the shipped scene): 123.5 -> 114.8 ms; 200K: 423 -> 142; 1M: 2158 -> 284; 5M (the `rkswd_tier_0` tier that also ships): **11,446 -> 604 ms**, 18.9x. The review's "~90 GB/frame" premise was analytical and does not describe the shipped scene at all — at 20,561 triangles the array is 1.7 MB and stays in L2, which is why that case gains 7% rather than an order of magnitude. It describes tier_0 exactly, where a single rasterization took 11.4 seconds
- note: CSR memory is O(sum of row spans) — peak RSS at 5M triangles rises 2461 -> 2894 MB. A scene of few very large triangles would need more of it than the sub-triangle array itself, the opposite regime from the one it exists for
- perf: visibility is resolved for the whole row before anything is shaded, so a pixel is shaded exactly once instead of once per depth-record improvement. The single-pass form shaded on every improvement and threw the result away on the next one, paying 8 bilinear fetches, a shading frame and 15 texel writes per discarded surface
- perf: the shading pass walks the row in x order rather than in triangle order, so the 15 AOV writes advance linearly through each image instead of scattering across it — worth ~9% on its own at zero overdraw, where the prepass itself saves nothing
- fix: `raster_bench` emits its depth shells furthest-first. Nearest-first, every deeper shell was rejected on arrival by the z-test, so a pixel accumulated one depth record however high `--layers` went and the flag measured the best case for the code it exists to stress — it reported a 2% regression where there is a 6.5x gain
- note: measured at 2048x1152, 20,561 triangles, minimum of four best-of-five runs, by overdraw — 1 layer: 113 -> 104 ms; 2: 281 -> 127; 4: 593 -> 141; 8: 1164 -> 178, 6.5x. Single-pass cost is linear in overdraw, prepass cost is not; the residual growth is coverage and depth testing, which no prepass removes. This is the worst case by construction, an unsorted scene averaging the harmonic number of records (~2.7 at 8 shells) and a front-to-back one none
- note: the winner index is 4 bytes per pixel alongside the z-buffer, 9.4 MB at 2048x1152. Both passes derive barycentrics from one shared `coverPixel`, so identical source expressions guarantee identical floating-point contraction and the shading pass recomputes bit-for-bit what the depth pass chose on — verified by the same 15-AOV hash across three poses, unchanged at `69c421dfeedf667e`
- perf: channel isolation is a shader uniform rather than a value baked into the uploaded texels. Pressing R/G/B previously re-copied the whole image on the CPU and recreated the GL texture to display data the shader was about to read anyway; it is now a `glUniform1i` against the texture already resident
- feat: `uChannelView` added to the three OCIO display programs (generated source, alongside `uExposure`) and to `edge_filter.frag`. In the edge filter it applies inside `sampleLuminance`, where isolating to grey and then Rec.709-dotting returns that channel unchanged because the weights sum to 1 -- the previous behaviour reproduced exactly, not approximated. `hsv_display.frag` already had its own, applied deliberately to HSV output rather than to source RGB
- fix: `channelViewToBake` drops out of the display-texture cache key, since the texture no longer depends on it. This is also what frees `flipRowsForDisplay` of everything but the flip
- perf: the display texture is uploaded in place rather than destroyed and recreated every frame. `Texture::upload` reallocates storage only when the render resolution actually changes and is otherwise a `glTexSubImage2D`, which matters more now that a resize is routine (the interactive/settled scale switch) rather than window-only
- perf: no mip chain on the display texture. The only consumer is a 1:1 fullscreen blit sampling LOD 0 exclusively, so the chain was regenerated on every upload and never read; min filter drops to `GL_LINEAR`
- perf: the row-order flip moved into `fullscreen_triangle.vert` as `vUv = vec2(p.x, 1.0 - p.y)`, deleting `flipRowsForDisplay` and its full-image scratch copy -- the HdrImage now uploads directly. All five programs sharing that vertex shader sample the same display texture, so the convention change is total; `edge_filter.frag` is invariant under it (Sobel takes `length(gx,gy)`, and reflection permutes the 0/45/90/135 Gabor bank whose max over `abs` is unchanged)
- note: the HUD's GPU memory readout drops the phantom `+1/3` it added for a mip chain that no longer exists, so the display texture now reports what is actually allocated
- perf: `Camera::primaryRay` gains an overload taking a prebuilt `ViewBasis`, and `renderPathTraced` hoists that basis out of its per-sample loop. The aspect-taking overload rebuilds it -- two sin, two cos, an atan, a tan, two normalize, two cross -- on every primary ray, for a value constant across the whole pass. Measured 332 -> 310 ms per pass at 1024x576 (best-of-5, min of three runs); `rasterizer.cpp` already hoisted it this way
- perf: `PathTraceDriver::setSuspended` parks the driver whenever the selected AOV is one it does not produce. Previously, selecting a rasterizer-backed AOV left it accumulating passes of an image no longer on screen, on every core, competing with the rasterizer the render thread runs synchronously
- refactor: `RowThreadPool` renamed to `ThreadPool` (and its file with it). It dispatches an index, not a row -- the path tracer has dispatched tiles through it since the buffer-pipeline work -- so the name named only its first caller
- note: the two thread pools were left separate. With the AOV gate and suspension above, the rasterizer and the driver are never both dispatching, so merging them under a shared dispatch mutex would serialize two things that no longer overlap and add a lock for no measured gain; parked workers cost stack memory and no CPU
- note: `RTC_SCENE_FLAG_ROBUST` was evaluated and rejected. Embree's watertight traversal measured ~24% slower per pass (348 -> 433 ms, single-variable A/B), not the few percent expected, which is too much for an artifact class the renderer already mitigates through ray epsilons, shadow-terminator origins and geometric-normal leak rejection
- note: moving the pixel probe off `glReadPixels` was evaluated and rejected. Reading the CPU image would report scene-referred float radiance, but the probe's purpose is the value actually on screen, so the display-encoded framebuffer read is the correct source and stays

## Colour and display — EXR precision, exposure-aware AOVs, output dither

- fix: `loadExr` reads via `Imf::InputFile` with an explicit float `FrameBuffer`, replacing `Imf::RgbaInputFile`'s half decode. Half saturates at 65504 and silently turns a legitimate above-ceiling source value into `inf`, which then propagated through `EnvironmentMap`'s importance-sampling CDFs (`rowSum`/`marginalCdf_`) and corrupted NEE sampling for the whole run with no error anywhere. Every returned image is now scanned and rejected outright if any texel is non-finite
- feat: `loadExr` also checks the EXR's `chromaticities` header attribute against Rec.709 (compared via a default-constructed `Imf::Chromaticities`, itself Rec.709 by OpenEXR's own convention) and warns, non-fatally, on a mismatch -- a linear-but-wrong-gamut asset (ACEScg/P3 HDRIs are the usual culprit) previously rendered with systematically wrong saturation/hue and nothing caught it
- feat: `hsv_display.frag`/`edge_filter.frag` gain a `uExposure` uniform, applied at the single point each shader samples `uHdrColor`, so HSV/Luminance/Sobel/Gabor respond to the exposure slider like Beauty does. Previously these four AOVs displayed at unity gain regardless of exposure while Beauty alone responded
- refactor: `Camera::ev100()` gains a static overload taking `(aperture, shutterSeconds, iso)` directly, and is now the single definition of the formula -- `DebugCameraController::relativeExposureEv()` calls it twice (current vs. profile.json default) instead of inlining the `log2` arithmetic a third time. `Camera::exposure()` deleted entirely: unused outside a debug log line, and its `/1.2` belonged to an absolute photometric model the engine never adopted (the scene isn't calibrated to real-world radiance)
- feat: a HUD readout next to the histogram reports the fraction of texels clipping at the display encode and the peak value as a multiple of display range (e.g. "3.2%, peak 47.8x"), computed from the pre-display-transform float `HdrImage` at the same capture cadence as the existing GPU histogram. The histogram alone bins the post-display-transform, post-8-bit-clamp framebuffer and cannot distinguish "just over 1.0" from "100x over" -- both saturate its bin 255 identically
- feat: triangular-PDF output dither added to all three display fragment shaders (both OCIO LUTs and Raw), just before the final 8-bit quantization -- a screen-space hash subtracted against itself at an offset, static per pixel since this targets a converged (Monte-Carlo-noise-free) image rather than motion
- docs: README §2's tone-mapping and photographic-exposure rows corrected to match what the code does -- a colorimetric-only display encode with no tonal compression (values above 1.0 clip by design), and exposure as a relative-stops delta against the profile default rather than an absolute photometric quantity
