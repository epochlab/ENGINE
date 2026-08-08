# Phase 0 — Foundation: task list

Ordered checklist for implementation. Full rationale: `~/.claude/plans/construct-a-refined-plan-curried-stearns.md`

## A — Scaffolding
- [x] `brew install glm opencolorio`
- [x] dirs: `assets/{shaders,textures}`, `include/engine/{platform,gfx,scene}`, `src/{platform,gfx,scene}`, `tools/`
- [x] `.gitignore`: `build/`, `.DS_Store`, `compile_commands.json`
- [x] `CMakeLists.txt`: C++20, `engine` + `gen_test_pattern` targets, `find_package(OpenGL glfw3 GLEW glm Imath OpenEXR OpenColorIO)`, `target_include_directories(engine PRIVATE include)`
- [x] verify target names (`OpenColorIO::OpenColorIO`, `OpenEXR::OpenEXR`, `Imath::Imath`) post-install
- [x] `cmake -S . -B build` configures clean

## B — Window & GL context
- [x] `include/engine/platform/window.h` + `src/platform/window.cpp`: `Window` (GL4.1 core fwd-compat hints, `shouldClose`/`pollEvents`/`swapBuffers`/`framebufferSize`)
- [x] `glfwInit`/`glfwTerminate` bracket `Window` in `main.cpp`
- [x] GLEW init: `glewExperimental=GL_TRUE`, drain init `glGetError`, `glfwSwapInterval(1)`
- [x] `include/engine/gfx/gl_debug.h` + `src/gfx/gl_debug.cpp`: `checkError`, `GL_CALL` macro, `khrDebugAvailable`
- [x] render loop: poll → clear → swap (bind HDR FBO / draw quad / post-process blit land in Stage D/F)

## C — Camera
- [x] `include/engine/scene/camera.h`: doc convention (RH, +Y up, −Z forward)
- [x] `include/engine/scene/camera.h` + `src/scene/camera.cpp`: position+yaw/pitch, film back + focal length → derived vertical FOV, `viewMatrix`, `projectionMatrix(aspect)`

## D — HDR FBO + polygon
- [x] `include/engine/gfx/mesh.h` + `src/gfx/mesh.cpp`: `Vertex{pos,uv}`, `createQuad`, RAII VAO/VBO/EBO
- [x] `include/engine/gfx/shader_program.h` + `src/gfx/shader_program.cpp`: `loadFromFiles`→optional, `loadFromSource`, `use`, `uniformLocation`
- [x] `assets/shaders/quad.vert`/`quad.frag` (`#version 410 core`, unlit passthrough)
- [x] `include/engine/gfx/texture.h` + `src/gfx/texture.cpp`: `createFromFloatPixels`→`GL_RGBA16F`, `createPlaceholderCheckerboard`
- [x] `include/engine/gfx/hdr_framebuffer.h` + `src/gfx/hdr_framebuffer.cpp`: RGBA16F color tex + depth renderbuffer, completeness check, `resize`/`bind`/`colorTexture`
- [x] `include/engine/gfx/post_process_pass.h` + `src/gfx/post_process_pass.cpp`: attribute-less-VAO fullscreen-triangle draw
- [x] `assets/shaders/fullscreen_triangle.vert` + placeholder passthrough frag
- [x] checkpoint: checkerboard quad visible (unencoded, expected washed out) — confirmed via screenshot, correctly bounded/scaled quad, no viewer LUT yet (Stage F)

## E — Test EXR
- [x] `tools/gen_test_pattern.cpp` (no `engine/` dependency) → `assets/textures/test_pattern.exr` (black / 18%grey / white / R,G,B / ramp — 700x100, 100px column-uniform stripes)
- [x] run once, kept local (`*.exr` now gitignored, not committed)
- [x] EXR loader: `Texture::createFromExr` — `RgbaInputFile` → validate `dataWindow` → `createFromFloatPixels`
- [x] swap quad texture: checkerboard → `test_pattern.exr`

## F — OCIO viewer LUT + exposure
- [x] confirm `BuiltinTransform` names (sRGB, Rec.709/1886) against installed OCIO 2.5.2 — corrected an earlier wrong assumption: no plain `CURVE - LINEAR_to_sRGB` style exists; uses the real Display/View API (`getProcessor(scene, display, "Un-tone-mapped", FORWARD)`) against OCIO's bundled `cg-config-v1.0.0_aces-v1.3_ocio-v2.1`, empirically verified (0.18→0.4614 sRGB / 0.4894 Rec.1886, 0→0, 1→1, zero LUT textures)
- [x] extend `Camera` with `aperture`/`shutterSeconds`/`iso`, `ev100()`, `exposure()` (standard photographic EV100 model)
- [x] `include/engine/gfx/ocio_display_transform.h` + `src/gfx/ocio_display_transform.cpp`: build both `{sRGB,Rec709}` `GpuShaderDesc`/shaders once (`GLSL_4_0`), splice into frag template; zero LUT textures needed (verified)
- [x] exposure uniform `pow(2,ev)` before OCIO call — left at neutral EV=0 for now rather than seeded from `Camera::exposure()`: the test pattern's values are calibration constants, not real scene-referred radiance, so a physical f/2.8 exposure crushed them to near-black; `exposure()`/`ev100()` still exercised via the startup log, gain a real render-path consumer once scene-referred content exists (Phase 2+)
- [x] interface: `setActiveLut`, `setExposureEv`, `activeShader`, `bind`
- [x] swap render-loop shader: placeholder → `ocioTransform.activeShader()`
- [x] confirm `GLFW_SRGB_CAPABLE`/`GL_FRAMEBUFFER_SRGB` never enabled
- [x] debug key (`L`) cycles `setActiveLut` — sRGB → Rec709 → Raw (genuine no-display-encode passthrough, for direct encoded-vs-unencoded comparison) → sRGB

## G — Verify
- [ ] visual: grey mid-grey, white pure white, ramp smooth
- [ ] numeric: `glReadPixels` at known coords vs hand-computed sRGB bytes (18% grey ≈124/255)
- [ ] sRGB vs Rec709 differ minutely on toggle
- [ ] resize correct (framebuffer size, no retina 2x bug)
- [ ] remove/mark-unused placeholder checkerboard + passthrough shader

## Deferred
Tone-mapping · debug camera/HUD/AOV selector (Phase 1) · glTF/textures beyond test pattern (Phase 2)
