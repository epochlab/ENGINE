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
