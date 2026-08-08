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
