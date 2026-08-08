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
