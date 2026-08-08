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
