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
- [ ] `include/engine/scene/camera.h`: doc convention (RH, +Y up, −Z forward)
- [ ] `include/engine/scene/camera.h` + `src/scene/camera.cpp`: position+yaw/pitch, `viewMatrix`, `projectionMatrix(aspect)`

## D — HDR FBO + polygon
- [ ] `include/engine/gfx/mesh.h` + `src/gfx/mesh.cpp`: `Vertex{pos,uv}`, `createQuad`, RAII VAO/VBO/EBO
- [ ] `include/engine/gfx/shader_program.h` + `src/gfx/shader_program.cpp`: `loadFromFiles`→optional, `loadFromSource`, `use`, `uniformLocation`
- [ ] `assets/shaders/quad.vert`/`quad.frag` (`#version 410 core`, unlit passthrough)
- [ ] `include/engine/gfx/texture.h` + `src/gfx/texture.cpp`: `createFromFloatPixels`→`GL_RGBA16F`, `createPlaceholderCheckerboard`
- [ ] `include/engine/gfx/hdr_framebuffer.h` + `src/gfx/hdr_framebuffer.cpp`: RGBA16F color tex + depth renderbuffer, completeness check, `resize`/`bind`/`colorTexture`
- [ ] `include/engine/gfx/post_process_pass.h` + `src/gfx/post_process_pass.cpp`: attribute-less-VAO fullscreen-triangle draw
- [ ] `assets/shaders/fullscreen_triangle.vert` + placeholder passthrough frag
- [ ] checkpoint: checkerboard quad visible (unencoded, expected washed out)

## E — Test EXR
- [ ] `tools/gen_test_pattern.cpp` (no `engine/` dependency) → `assets/textures/test_pattern.exr` (black / 18%grey / white / R,G,B / ramp)
- [ ] run once, commit `.exr`
- [ ] EXR loader: `RgbaInputFile` → validate `dataWindow` → `createFromFloatPixels`
- [ ] swap quad texture: checkerboard → `test_pattern.exr`

## F — OCIO viewer LUT + exposure
- [ ] confirm `BuiltinTransform` names (sRGB, Rec.709/1886) against installed OCIO 2.5.2
- [ ] `include/engine/gfx/ocio_display_transform.h` + `src/gfx/ocio_display_transform.cpp`: build both `{sRGB,Rec709}` `GpuShaderDesc`/shaders once (`GLSL_4_0`), splice into frag template, upload LUTs if any
- [ ] exposure uniform `pow(2,ev)` before OCIO call
- [ ] interface: `setActiveLut`, `setExposureEv`, `activeShader`, `bind`
- [ ] swap render-loop shader: placeholder → `ocioTransform.activeShader()`
- [ ] confirm `GLFW_SRGB_CAPABLE`/`GL_FRAMEBUFFER_SRGB` never enabled
- [ ] debug key (`L`) toggles `setActiveLut`

## G — Verify
- [ ] visual: grey mid-grey, white pure white, ramp smooth
- [ ] numeric: `glReadPixels` at known coords vs hand-computed sRGB bytes (18% grey ≈124/255)
- [ ] sRGB vs Rec709 differ minutely on toggle
- [ ] resize correct (framebuffer size, no retina 2x bug)
- [ ] remove/mark-unused placeholder checkerboard + passthrough shader

## Deferred
Tone-mapping · debug camera/HUD/AOV selector (Phase 1) · glTF/textures beyond test pattern (Phase 2)
