# PBR Pathtracer

*A CPU, unidirectional Monte Carlo path tracer with real-time progressive display: Embree-accelerated, stochastic BSDF sampling combined with environment-map NEE via MIS, behind a thin OpenGL display/HUD layer.*

![Sample render](sample.png)


## Documentation

| | |
|---|---|
| [Pipeline](#pipeline) | How a frame is produced, below |
| [Component reference](docs/components.md) | Every subsystem, session settings, benchmark tooling |
| [Material library](docs/materials.md) | Shipped presets and every `MaterialConfig` field |
| [AOV reference](docs/aovs.md) | All 27 debug outputs, by category |
| [Roadmap](docs/roadmap.md) | Large, Next and Parked work |
| [References](docs/references.md) | The literature each technique implements |

## Build

C++20, built with CMake. Currently developed against macOS only.

```
brew install cmake glfw glew glm imath openexr opencolorio embree
git submodule update --init --recursive
cmake -B build
cmake --build build
```

## Run

```
./build/engine [-scene path/to/scene.json] [-stats] [-bench log.jsonl]
```

Defaults to `assets/scenes/cornell.json` if `-scene` is omitted — currently the only scene shipped. Session settings (vsync, texture and display bit depth, camera, sampling) live in `assets/config/profile.json`; each is documented, with its measured cost, under [Session settings](docs/components.md#session-settings-profilejson).

## Benchmark

Timing runs append one JSON Lines record each to a local log: `engine -bench PATH`, `render_beauty --bench-log PATH`, `raster_bench --bench-log PATH`. A performance claim is made with a randomized interleaved A/B of two builds, not by comparing two runs by eye:

```
./build/bench_compare run --a buildA/render_beauty --b buildB/render_beauty --rounds 12 --log renders/bench.jsonl \
    -- --scene scenes/cornell.json --out renders/ab.png --passes 32 --width 640 --height 360 --bench-log renders/bench.jsonl
```

It prints B/A with a distribution-free confidence interval, and says "not resolved" when that interval contains 1. See [Benchmark tooling](docs/components.md#benchmark-tooling) for the rest.

## Pipeline

**Startup.** glTF geometry/materials load once into a CPU-resident scene: per-vertex shading data (`ShadingTriangle`) and world-space triangles feed an Embree scene (`EmbreeAccel`); materials keep only CPU `HdrImage` textures, sampled per-ray. An equirectangular HDR environment map loads alongside it, with its own luminance-importance-sampling CDF for NEE. Area lights (`scene.json`'s `lights`) are injected into the same Embree scene at load (`appendQuadLights`), so they occlude and are camera/BSDF-hittable with no second intersection path.

**Per frame**, on any camera/scene-state change, two paths run independently off the same trigger:

- **Path tracer** — `PathTraceDriver` hands a fresh request to a background thread pool (one worker/core, row-parallel, dynamic scheduling), restarting progressive accumulation:
  1. Camera ray generation (pinhole) → Embree intersection (`rtcIntersect1`/`rtcOccluded1`)
  2. BSDF eval/sampling, combining five techniques:
     - Heitz 2018 GGX VNDF specular
     - EON rough-diffuse (Portsmouth/Kutz/Hill 2025)
     - Walter 2007 rough dielectric transmission with exact Fresnel/TIR, falling back to a Snell delta lobe below the smooth-roughness threshold and at an index-matched interface
     - Kulla-Conty multiple-scattering compensation on both interfaces
     - exact complex-IOR conductor Fresnel via Gulbrandsen 2014's reflectivity/edge-tint parameterisation
  3. NEE against a `LightSet` (environment map, optionally excluded via the HUD's "Environment Light" checkbox, plus any area lights — uniform selection, quads importance-sampled by solid angle via Ureña/Fajardo/King 2013's spherical-rectangle parametrisation), MIS-combined with BSDF sampling (power heuristic, Veach 1997)
  4. Recursive bounce loop with Russian roulette, Chiang/Li/Burley 2019 shadow-terminator-corrected secondary-ray origins, Beer-Lambert extinction inside transmissive media
  5. Radiance + full G-buffer/transport-component AOV set accumulated per pass, published lock-free for the render thread

- **Rasterizer** — a synchronous CPU pass (`rasterizer.cpp`) computes the 13 primary-hit-only AOVs (`aov.h`) every frame on the render thread: watertight edge-function rasterization (Pineda 1988) -- view-frustum Sutherland-Hodgman clipping, vertices snapped to a fixed-point grid whose precision is derived per frame for exact int64 edge functions, and the top-left fill rule, so triangles sharing an edge cover every pixel centre on it exactly once -- sharing `gbuffer_shading.h`'s material sampling with the path tracer but no Embree/BSDF/recursion. This gives those AOVs instant, glitch-free feedback during camera movement, decoupled from Beauty's own progressive convergence — the path-traced request above only restarts when the selected AOV needs light-transport data.

**Display.** The render thread blits whichever AOV is selected through OCIO's display transform (exposure/tone-mapping) and the debug HUD, converging over subsequent passes rather than blocking on one long render. No GPU rasterization anywhere: OpenGL exists only for the window, the post-process/OCIO blit, and ImGui; the primary-hit rasterizer is CPU-only.
