# Real-time PBR render engine

*Design overview: a physically based render engine, built progressively from geometric/radiometric correctness toward full spectral path tracing*

## Build

C++20, built with CMake. Currently developed against macOS only.

### Prerequisites

```
brew install cmake glfw glew glm imath openexr opencolorio
```

Dear ImGui is vendored as a git submodule (`third_party/imgui`) — initialise it before configuring:

```
git submodule update --init --recursive
```

### Configure & build

```
cmake -B build
cmake --build build
```

This produces two targets:

- `build/engine` — the render engine
- `build/test_pattern` — EXR calibration-pattern generator (`tools/test_pattern.cpp`)

### Run

```
./build/engine
```

## 1. Full pipeline overview

The target feature set is a full spectral, physically based, GPU path tracer with unbiased Monte Carlo global illumination — normally an offline technique. Rather than resolve the real-time/unbiased tension up front with a denoiser or biased shortcut, the build starts with a simple direct-lighting ray tracer and layers in real-time adaptation, global illumination, and spectral transport as separate, later phases (§4 records what's still open here).

- Environment geometry (glTF, UV textures, normal maps) → Camera (thin lens, Euclidean transforms) → Acceleration structure (BVH)
- → Direct lighting (NEE, area lights, shadows) → Materials (BSDF/PBR, Fresnel)
- → Recursive path tracing (Russian roulette, throughput accumulation) → Global illumination (indirect/multiple scattering, MIS)
- → Spectral transport (wavelength sampling, dispersion) → Real-time integration (adaptive sampling, accumulation)
- → Linear-to-display (exposure, tone-mapping) → Framebuffer (RGB-D)

A debug HUD (Phase 1) is cross-cutting instrumentation, not a pipeline stage — it reads live state (frame timing, memory, camera, whichever AOV buffer exists) from every phase rather than sitting in the chain above.

Nine build phases carry this pipeline from a bare camera to a spectral path tracer:

| Phase | Name | Adds |
|---|---|---|
| 0 | Foundation | OpenGL renderer, camera, Euclidean space, OCIO, OpenEXR linear pipeline, exposure, tone-mapping |
| 1 | Debug HUD & system feedback | GPU/system readout, frame-timing HUD, memory HUD |
| 2 | Geometry, textures & basic material | glTF mesh loading, UV texture mapping, linear EXR textures, tangent-space normal mapping, basic metallic-roughness PBR shading, basic AOV debug dropdown (no histogram) |
| 3 | Scene controls | Scene/viewport stats, camera & lens readout, debug camera controls (WASD/QE/R fly + LMB-drag orbit around a depth-sampled pivot), camera framing overlays, live histogram on the Phase 2 AOV selector |
| 4 | Direct lighting & acceleration | BVH, punctual direct lighting (analytic Cook-Torrance GGX), prefiltered IBL (SH-9 diffuse irradiance, prefiltered specular mip chain, analytic split-sum BRDF approximation — no baked LUT), multi-scatter energy compensation, analytic energy conservation — no secondary rays |
| 5 | Materials & recursive transport | BSDF/PBR, BRDF importance sampling, Fresnel, recursive tracing, Russian roulette, throughput accumulation, radiance estimator |
| 6 | Real-time integration | Adaptive sampling, progressive accumulation, backend migration (OpenGL → Vulkan/CUDA-OptiX) |
| 7 | Global illumination | Unbiased Monte Carlo GI, MIS, caustics, area lights, shadows, next-event estimation, importance-sampled IBL |
| 8 | Spectral upgrade | Spectral light transport, wavelength sampling, spectral materials, spectral dispersion |

Phases 0–6 deliberately contain no global illumination — they establish a correct, direct-lighting-only ray tracer through physically based materials, made real-time via adaptive sampling and progressive accumulation, before any indirect complexity is added. Phase 5's BRDF-importance-sampled recursive tracing deliberately has no NEE counterpart to pair against — that's intentional, not an oversight: NEE returns in Phase 7, MIS-combined with BSDF sampling via the balance heuristic (§2's Multiple importance sampling row), once area lights and visibility rays exist to sample.

## 2. Component reference

| Feature | Mechanism | Role / why it matters | Phase |
|---|---|---|---|
| OpenGL camera / Euclidean space | View/projection matrices over a fixed right-handed coordinate convention | Geometric ground truth everything else is measured against | 0 |
| Thin lens model | Pinhole first; lens radius + focal distance sample depth-of-field rays | Camera-realistic defocus, added once pinhole geometry is verified | 0 |
| OpenEXR linear pipeline | Float framebuffer, scene-referred values, no implicit gamma | Keeps lighting math in physically meaningful units end to end | 0 |
| Correct exposure | Photographic exposure value (aperture/shutter/ISO) mapping scene-referred → display-referred | Stops arbitrary brightness scaling from masking or exaggerating lighting bugs | 0 |
| Linear workflow | All shading/compositing in linear light; gamma/OETF only at final display encode | Precondition for correct PBR colour math | 0 |
| Tone-mapping | Filmic/ACES-style operator, applied last | Compresses unbounded HDR radiance into a displayable range without clipping | 0 |
| GPU/system readout | Query device name and driver/API version at startup | Confirms the actual GPU/backend at startup, before a wrong-adapter bug masquerades as a render bug | 1 |
| Frame-timing HUD | Ring buffer of recent frame times; rolling FPS/avg/min/max, GPU timer queries split by pass (geom/post), derived Mtri/s and Mpix/s | Makes "maximum performance" measurable frame to frame, not just felt | 1 |
| Memory HUD | Live RAM readout plus GPU allocation tracked at alloc/free (meshes, FBOs, textures) | Surfaces a memory regression (e.g. an untracked texture upload) immediately, rather than after VRAM exhaustion | 1 |
| Scene/viewport stats | Object count, draw calls issued vs. culled, triangle/point counts, per-mesh breakdown, viewport resolution | Early-warning signal for culling bugs or draw-call bloat once real scenes (Phase 2 onward) land | 3 |
| Camera & lens readout | Live position/rotation, filmback size, focal length, near/far clip mirrored from the Phase 0 camera state | Inspect and tune camera/lens parameters without recompiling | 3 |
| Debug camera controls | WASD strafes/moves the Phase 0 camera in the view plane, QE moves it vertically, R resets to a default pose; LMB drag tumbles the camera in orbit around a pivot set by an LMB click's depth-sampled hit point at screen centre | Interactive navigation while debugging, without hand-editing camera parameters between runs; orbit adds inspection of a specific point without manual pivot bookkeeping | 3 |
| Camera framing overlays | Aspect-ratio letterbox mask, centre crosshair, 3×3 rule-of-thirds grid — drawn over the viewport, not baked into the framebuffer | Composition/framing aids while lining up a shot; overlay-only so they never contaminate the AOV buffers being debugged | 3 |
| AOV selector | Dropdown switching the displayed buffer across the full AOV set (§3), plus R/G/B channel-isolation hotkeys | Isolates one signal at a time for debugging; availability is phase-gated per §3 | 2 |
| Live histogram | Per-channel (R/G/B) histogram of the currently selected AOV, updated every frame | Catches exposure/clipping and colour-space bugs (e.g. an sRGB texture read as linear) that a single still frame can hide | 3 |
| glTF mesh loading | Indexed triangle meshes, interleaved vertex buffers (position/normal/UV/tangent), uploaded once to GPU-resident buffers | Standard interchange format for scene geometry; interleaving keeps vertex fetch cache-coherent and avoids per-frame re-upload | 2 |
| UV texture mapping | Per-vertex UV, bilinear/trilinear sampled against mipmapped textures | Surface detail without paying for it in geometry | 2 |
| Linear EXR textures | All texture maps stored as linear-light OpenEXR; GPU-uploaded as compressed HDR (BC6H where supported) or half-float, with full mip chains | Keeps texture data in the same linear space as the framebuffer/lighting math, with VRAM and texture-cache footprint bounded from the start | 2 |
| Tangent-space normal mapping | Per-vertex tangent (glTF-supplied only — fails clearly if absent, no MikkTSpace generation yet); 3-channel normal map blended with a bump-map-derived detail normal (UV-space central-difference height gradient) | Surface micro-detail without extra geometry; the bump blend adds fine detail beyond the base normal map's resolution | 2 |
| Basic metallic-roughness PBR | Base colour/metallic/roughness (glTF slots) plus bump and specular maps (project-specific, beyond glTF's standard channels); Lambertian diffuse + Schlick-Fresnel specular (F0 = mix(specular, base colour, metallic)) against one fixed test light — no importance sampling; the authored AO map is multiplied in as a blanket occlusion term | Validates geometry/texture/material data end-to-end against real assets before the full BSDF stack lands in Phase 5 | 2 |
| BVH acceleration | Bounding volume hierarchy over scene primitives (SAH-built) | Makes ray-scene intersection sub-linear; required before recursion is affordable | 4 |
| Punctual direct lighting | Direct analytic evaluation of point/directional lights (Cook-Torrance GGX: normal distribution, Smith height-correlated visibility, Schlick Fresnel) — no visibility rays, no NEE. Shares its BRDF with IBL specular below, so direct and ambient specular stay consistent | Physically-scaled direct lighting before the visibility-ray stack (area lights, shadows, NEE) exists | 4 |
| IBL (prefiltered) | SH-9 diffuse irradiance (Ramamoorthi & Hanrahan 2001, no texture) + GGX-prefiltered specular mip chain (Karis 2013) + analytic split-sum BRDF approximation (Lazarov 2013 polynomial fit — no baked LUT texture/bake pass), sampled directly at shading time, no runtime importance sampling | Real-time-affordable image-based lighting without per-frame Monte Carlo integration | 4 |
| Multi-scatter energy compensation | Multiplicative compensation factor (Turquin 2019) reusing the split-sum DFG (scale, bias) output — no extra texture/pass — applied to both punctual and IBL specular | Single-scatter split-sum/analytic GGX loses energy at high roughness (visibly darkens rough conductors); compensates without a second multi-scatter BRDF model | 4 |
| Energy conservation | Furnace test (`tools/furnace_test.cpp`): Monte Carlo hemisphere integration of the full BRDF under uniform incident radiance, swept over roughness/metallic | A surface must never reflect more energy than it received — checked as soon as direct lighting first exercises a material, before recursive/GI complexity could mask a non-conserving BRDF | 4 |
| Area lights | Emissive geometry sampled by solid angle | Physically grounded light sources instead of point/directional hacks | 7 |
| Shadows / soft shadows | Visibility rays to sampled light points; softness from light area and sample count | Direct lighting must be occluded correctly once secondary/visibility rays exist | 7 |
| Next-event estimation | Explicit light sampling at each shading point, rather than relying on a bounce to find it | Removes most of the variance from direct lighting | 7 |
| IBL (importance-sampled) | Environment map treated as an area light at infinity, importance-sampled by luminance at runtime | Ground-truth IBL correctness once visibility/secondary rays exist; supersedes the Phase 4 prefiltered approximation rather than extending it | 7 |
| BSDF / PBR materials | Physically based reflectance models — diffuse, microfacet specular, dielectric/conductor | Materials respond to light with real physical behaviour | 5 |
| BRDF sampling + importance sampling | Sample directions proportional to the BRDF's contribution (e.g. GGX importance sampling) | Cuts variance sharply versus uniform hemisphere sampling | 5 |
| Fresnel effects | Fresnel term (Schlick approximation or full dielectric/conductor equations) | Correct reflectance/transmission split from grazing to normal incidence | 5 |
| Recursive tracing | Path extended bounce to bounce until termination | Mechanism by which transport of arbitrary depth is computed | 5 |
| Russian roulette termination | Probabilistically terminate low-throughput paths, reweighting survivors | Keeps recursion finite without biasing the estimator | 5 |
| Path throughput accumulation | Running product of BSDF × cosine / pdf carried along the path | The quantity that turns a traced path into a radiance contribution | 5 |
| Radiance estimation formulation | Monte Carlo estimator of the rendering equation (Kajiya 1986) | The formal contract every sampling strategy above must satisfy | 5 |
| Adaptive sampling | Per-pixel sample count driven by a variance/convergence estimate | Spends the sample budget where the image is still noisy, not uniformly | 6 |
| Monte Carlo unbiased GI (indirect/multiple scattering) | Recursive tracing continued past the first bounce, no shortcuts | Light bouncing off multiple surfaces before reaching the eye | 7 |
| Multiple importance sampling (MIS) | Weighted combination of NEE and BSDF-sampling estimators (balance heuristic) | Low variance of both strategies without either one's blind spots | 7 |
| Caustics | Emergent from specular-diffuse-specular paths once recursion + refraction exist | A correctness test for the transport code, not a separate mechanism | 7 |
| Spectral light transport | Radiance carried per-wavelength rather than as RGB triples | RGB is a perceptual approximation; the rendering equation integrates over wavelength | 8 |
| Wavelength sampling | Hero-wavelength sampling: one wavelength per path, others carried as MIS candidates | Makes spectral transport tractable without an N× sample explosion | 8 |
| Spectral materials | Reflectance/transmission defined as spectral curves, not RGB constants | Physically correct colour reproduction, and a precondition for dispersion | 8 |
| Spectral dispersion | Wavelength-dependent index of refraction (Cauchy/Sellmeier) | Prism / chromatic-aberration effects, only meaningful once transport is spectral | 8 |

## 3. AOV reference

Arbitrary output variables exposed via the AOV selector (§2, Phase 2), beyond the composited beauty pass. Availability is phase-gated — an AOV appears only once the phase producing its underlying data lands. AO has two independent sources, not a single supersession chain: AO (baked), from an authored texture, lands earliest at Phase 2 and stays — it also becomes the modulating term for Phase 4's ambient/IBL contribution, its first physically meaningful role now that an ambient term exists to occlude — and is explicitly superseded, not merely joined, by AO (ray-traced) once visibility rays exist at Phase 7 (no screen-space AO stage exists in this pipeline: a baked AO pass already covers the real-time case). See the AO rows below.

| AOV | Category | Mechanism | Role / why it matters | Phase |
|---|---|---|---|---|
| Beauty (RGB) | Utility | Final composited radiance, post tone-mapping | The primary output; every other AOV isolates one contributor to it | 0 |
| Alpha | Utility | Ray/coverage hit test | Compositing over other elements | 0 |
| Depth (Z) | Utility | Camera-space hit distance | Depth-based compositing/debugging; also the channel `agent.md`'s Retina stage requires | 0 |
| HSV | Utility | Colour-space transform of the beauty pass | Isolates hue/saturation shifts a pure RGB view can hide | 1 |
| Luminance | Utility | Photometric luminance (Rec. 709/2020 channel weights) of the beauty pass | Isolates perceived brightness from colour — catches exposure/contrast issues a per-channel RGB view can hide | 1 |
| Sobel / edge | Utility | 3×3 Sobel gradient kernel (Gx/Gy magnitude) applied to the Luminance AOV | Cheap edge/gradient signal, computed once as an AOV rather than duplicated per-consumer; backs the agent retina's Tier 2 fidelity level (`agent.md` §3) | 1 |
| Gabor | Utility | 4-orientation Gabor kernel bank (0/45/90/135°) applied to the Luminance AOV, max response across orientations | Directional edge/texture response Sobel's isotropic magnitude can't distinguish | 1 |
| World position (P) | Utility | Ray-hit world-space coordinate | Debugging geometry/UV placement independent of shading | 2 |
| UV coordinates | Utility | Per-vertex UV interpolated at the hit point | Visualizes the texture-space mapping directly — catches a bad unwrap by eye | 2 |
| Normals (shading) | Material | Interpolated, normal-mapped surface normal | The normal actually used in shading, isolated from lighting | 2 |
| Geometric normal | Material | Interpolated normal before normal-map perturbation | Separates a bad normal map from a bad base mesh | 2 |
| Albedo / base colour | Material | glTF base-colour texture sample | Isolates texture data from lighting | 2 |
| Metallic | Material | Metallic factor (scalar only — no metallic texture in this asset set) | Debug material authoring independent of shading | 2 |
| Roughness | Material | Roughness texture (project-specific slot, not glTF's combined metallicRoughness channel) × roughness factor | Debug material authoring independent of shading | 2 |
| Tangent | Material | Per-vertex tangent basis (glTF-supplied only, no MikkTSpace generation yet) | Debugs the tangent-space basis used for normal mapping directly | 2 |
| Object ID / Material ID | Material | Flat per-object or per-material index, false-coloured | Isolation masks for compositing/debugging | 2 |
| AO (baked) | Material | Authored ambient-occlusion texture sample | Debug baked AO independent of the screen-space/ray-traced approximations; also multiplied into Beauty as a blanket occlusion term (not physically exact — no ambient/indirect term exists yet to modulate) | 2 |
| Direct diffuse | Lighting | NEE-sampled direct lighting, diffuse term only | Isolates direct diffuse from specular and indirect | 7 |
| Direct specular | Lighting | NEE-sampled direct lighting, specular term only | Isolates direct specular contribution | 7 |
| Shadow / occlusion mask | Lighting | Per-light visibility-ray result | Debug shadow correctness independent of shading | 7 |
| IBL / ambient (RGB) | Lighting | SH-9 diffuse irradiance + prefiltered-specular-cubemap contribution, AO-modulated, no direct lights; diffuse albedo is excluded (shown as if white) so this reads as light arriving, not the object's colour under ambient light — specular keeps the real F0, a physical reflectance property rather than albedo | Isolates the Phase 4 ambient term from direct lighting; the visual check that AO's re-scoped role and the SH/prefiltered-cubemap result look physically plausible | 4 |
| AO (ray-traced) | Lighting | Ray-traced, short-range cosine-weighted hemisphere occlusion, light-independent | Ground-truth occlusion once BVH/visibility rays exist; supersedes the Phase 2 baked AO's ambient-occlusion role rather than extending it | 7 |
| Index of refraction (IOR) | Transport | Per-material IOR (dielectric η, or complex η+ik for conductors) evaluated at the hit point | Isolates the raw refractive-index input driving Fresnel/transmission, independent of the reflectance curve it produces; becomes wavelength-dependent once Phase 8's spectral dispersion lands | 5 |
| Fresnel/reflectance term | Transport | Schlick Fresnel evaluated at the hit point's actual view angle | Debug grazing-angle reflectance behaviour in isolation | 4 |
| Bounce-count heatmap | Transport | Path depth at termination, per pixel | Debug Russian roulette/termination behaviour | 5 |
| Variance/noise heatmap | Real-time | Per-pixel estimator variance | Shows the adaptive sampler where the image hasn't converged | 6 |
| Sample-count heatmap | Real-time | Per-pixel accumulated sample count | Shows where the adaptive sampler is spending budget | 6 |
| Indirect diffuse | Global illumination | Recursive-bounce contribution, diffuse term | Isolates indirect diffuse from direct lighting | 7 |
| Indirect specular | Global illumination | Recursive-bounce contribution, specular term | Isolates indirect specular (reflections/caustics) from direct lighting | 7 |
| Per-wavelength radiance slice | Spectral | Single-wavelength radiance from hero-wavelength sampling | Debug spectral transport/dispersion per wavelength | 8 |

## 4. Open questions

| Section | Open question |
|---|---|
| Real-time vs. unbiased GI | Phase 6 lands real-time adaptive sampling/progressive accumulation before GI exists; once GI lands in Phase 7, does it stay within that same progressive-accumulation infrastructure (unbiased, converges over ticks) or does GI's added noise force a move to denoised real-time GI (biased, stable every frame)? Deferred by design — not re-litigated here. |
| Backend migration | At what phase does compute move from OpenGL to Vulkan/CUDA-OptiX for hardware-accelerated BVH/RT? Likely Phase 6/7, as real-time integration then GI demand it, but not committed. |
| HUD GPU timing | Phase 1's per-pass GPU timing uses OpenGL timer query objects; the Phase 6 backend migration to Vulkan/CUDA-OptiX needs an equivalent timestamp mechanism, not yet decided. |
| Spectral rendering cost | Hero-wavelength sampling multiplies the sample budget — is full spectral rendering reserved for offline validation renders, with an RGB approximation used elsewhere? |
| Texture/mesh compression | BC6H vs ASTC for compressed linear HDR textures, and whether Draco/meshopt mesh compression is worth the decode cost. |
| IOR/Fresnel/bounce-count AOV availability | Resolved: Fresnel/reflectance is exposed at Phase 4 (Schlick Fresnel is already evaluated for both punctual and IBL specular once GGX lands this phase). IOR and Bounce-count heatmap move to Phase 5 — no per-material IOR field exists yet (Phase 4 only has the F0 baked into `uSpecular`/`uBaseColorFactor`), and there is no recursion/bounce count to visualize until Phase 5's recursive tracing. |

## 5. References

- Khronos Group. glTF 2.0 specification — scene/mesh/material interchange format.
- Mikkelsen, M.S. (2008). Simulation of wrinkled surfaces revisited — MikkTSpace tangent space standard.
- Kajiya, J.T. (1986). The rendering equation. SIGGRAPH.
- Veach, E. (1997). Robust Monte Carlo Methods for Light Transport Simulation. PhD thesis, Stanford — multiple importance sampling, next-event estimation, bidirectional path tracing.
- Pharr, M., Jakob, W., Humphreys, G. Physically Based Rendering: From Theory to Implementation (PBRT).
- Cook, R.L., Torrance, K.E. (1982). A reflectance model for computer graphics. ACM ToG — BRDF and Fresnel foundations.
- Walter, B. et al. (2007). Microfacet models for refraction through rough surfaces — GGX BSDF.
- Wilkie, A. et al. (2014). Hero wavelength spectral sampling. Computer Graphics Forum.
- Kulla, C., Conty, A. (2017). Revisiting physically based shading at Imageworks — production spectral rendering.
- Debevec, P. (1998). Rendering synthetic objects into real scenes — HDR image-based lighting.
- OpenEXR / Academy Software Foundation technical documentation — linear HDR pipeline, exposure.
- Karis, B. (2013). Real Shading in Unreal Engine 4 — practical real-time PBR, split-sum IBL, and importance sampling.
- Wald, I. (2007). On fast Construction of SAH-based Bounding Volume Hierarchies — the BVH build strategy behind Phase 4's `Bvh` (binned SAH, CPU, one-time, static scene).
- Ramamoorthi, R., Hanrahan, P. (2001). An Efficient Representation for Irradiance Environment Maps — the SH-9 diffuse IBL projection (`sh_irradiance.h`).
- Heitz, E. (2014). Understanding the Masking-Shadowing Function in Microfacet-Based BRDFs — the Smith height-correlated visibility term used in Phase 4's Cook-Torrance GGX.
- Lazarov, D. (2013). Getting More Physically Based Reflectance — the analytic split-sum DFG polynomial approximation used in place of a baked BRDF LUT.
- Turquin, E. (2019). Practical multiple scattering compensation for microfacet models — the multiplicative energy-compensation factor applied to both punctual and IBL specular.
- Bitterli, B. et al. (2020). Spatiotemporal reservoir resampling for real-time ray tracing (ReSTIR) — forward reference for the real-time GI open question.
- Sobel filtering — edge-detection AOV computed from Luminance (§3) — arXiv:2601.16806.
