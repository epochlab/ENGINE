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
