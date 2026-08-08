# Phase 1 — Debug HUD & system feedback: task list

Ordered checklist for implementation. Full rationale: `~/.claude/plans/i-have-defered-the-squishy-pebble.md`

## A — Vendor ImGui
- [x] `git submodule add` Dear ImGui (tagged stable release, not `docking`) at `third_party/imgui` (v1.92.9)
- [x] `CMakeLists.txt`: add ImGui core (`imgui.cpp`, `imgui_draw.cpp`, `imgui_tables.cpp`, `imgui_widgets.cpp`) + `backends/imgui_impl_glfw.cpp` + `backends/imgui_impl_opengl3.cpp` to `engine`'s sources; add `third_party/imgui`(`/backends`) include dirs; define `IMGUI_IMPL_OPENGL_LOADER_GLEW`
- [x] configure-time guard: fail with a clear message if `third_party/imgui/imgui.cpp` is missing (submodule not initialized)
- [x] verify: clean build, no engine-side ImGui usage yet

## B — ImGui bring-up
- [x] `Window::nativeHandle()` accessor (`include/engine/platform/window.h`); fix stale "Phase 1's WASD/QE/R" comment → Phase 3
- [x] `engine::debug::HudOverlay` (`include/engine/debug/hud_overlay.h` + `src/debug/hud_overlay.cpp`): context/backend init+shutdown, `beginFrame()`/`render()`
- [x] wire into `main.cpp`: construct after `window`, `beginFrame()` after `pollEvents()`, `render()` after the post-process blit and before `swapBuffers()`, empty panel
- [x] verify: borderless empty panel top-left; `L` key LUT toggle still works; resize doesn't break it

## C — GPU/system readout
- [x] `engine::debug::system_info` — `queryGpuInfo()` (`GL_VENDOR`/`GL_RENDERER`/`GL_VERSION` + monitor refresh rate), queried once at startup
- [x] HUD panel GPU section
- [x] verify: matches actual hardware string (Apple M1 / 4.1 Metal - 90.5)

## D — Frame-timing HUD
- [x] `engine::debug::FrameStats` — 120-entry ring buffer, `tick()`, `fps()`/`avgMs()`/`minMs()`/`maxMs()`
- [x] `engine::debug::GpuTimer` — double-buffered `GL_TIME_ELAPSED`, `gpuTimerQueryAvailable()`
- [x] `Mesh::triangleCount()` getter
- [x] wrap geometry pass + post-process pass in `main.cpp` with a `GpuTimer` each
- [x] HUD Frame section: fps/avg, sparkline, min/max, geom/post ms, Mtri/s + Mpix/s, vsync cap line
- [x] verify: fps caps at monitor refresh (~118-120 FPS), sparkline moves, geom/post ms both nonzero and plausible

## E — Memory HUD
- [x] `engine::debug::memory_tracker` — `residentSetBytes()` (`mach_task_basic_info`), `trackGpuAlloc`/`trackGpuFree`, `gpuAllocatedBytes()` (one atomic counter)
- [x] byte-tracking hooks + move-op fixes in `Texture`, `Mesh`, `HdrFramebuffer` (`resize()`'s untrack/track pairing must be exact)
- [x] HUD Memory section
- [x] verify: GPU alloc MB matches a hand-computed figure (2048x1152 HDR FBO + EXR texture ≈ 27.5 MB, confirmed exactly); repeated resizing tracks proportionally (10.6 → 42.7 → 13.9 MB), no upward drift

## F — Styling pass
- [x] borderless/pinned-top-left window flags, cyan `TextColored` section headers, `IniFilename = nullptr`
- [x] verify: visual match against reference layout

## G — Verify
- [x] visual: matches reference (GPU/Frame/Memory sections, no chrome, top-left)
- [x] numeric: GPU alloc bytes hand-computed vs HUD readout; fps caps correctly; RAM MB plausible (~94 MB)
- [x] regression: `L` key still cycles sRGB→Rec709→Raw correctly; window resize doesn't misplace the panel or leak tracked bytes

## Deferred
Scene/viewport stats · camera & lens readout · debug camera controls (WASD/QE/R) · camera framing overlays · AOV selector · live histogram — all Phase 3, need real scene geometry (Phase 2) to be meaningful

**Phase 1 complete.**
