# Derivations

Long-form reasoning behind implementation choices that a source comment cannot hold in one line
(`notes/architect.md` §7). Each entry is referenced from the code it justifies. Nothing here is
required to read the code; it is here so the argument survives the comment budget.

See also [References](README.md#references) for the literature each technique implements, and
[ROADMAP](ROADMAP.md) for what is deliberately not implemented.

## Contents

| | |
|---|---|
| [Sobol padding and net quality](#sobol-padding-and-net-quality) | `include/pathtracer/scene/sampler.h` |
| [Blue-noise shift in output space](#blue-noise-shift-in-output-space) | `src/scene/sampler.cpp` |
| [Albedo table quadrature](#albedo-table-quadrature) | `tools/albedo_table.cpp` |
| [Albedo table layout](#albedo-table-layout) | `src/scene/bsdf.cpp` |
| [Multiple-scattering lobe sampling](#multiple-scattering-lobe-sampling) | `src/scene/bsdf.cpp` |
| [Conductor Fresnel and F_avg](#conductor-fresnel-and-f_avg) | `tools/bsdf_validate.cpp` |
| [Hemisphere integration estimator](#hemisphere-integration-estimator) | `tools/bsdf_validate.cpp` |
| [Dispersion channel commitment](#dispersion-channel-commitment) | `src/scene/path_tracer.cpp` |
| [Path-traced render contract](#path-traced-render-contract) | `include/pathtracer/scene/path_tracer.h` |
| [Retrace trigger state](#retrace-trigger-state) | `src/main.cpp` |

## Sobol padding and net quality

`include/pathtracer/scene/sampler.h`, `src/scene/sampler.cpp`

Per-pixel-per-sample sampler: Owen-scrambled, shuffled Sobol (Sobol 1967; Joe & Kuo 2008
direction numbers; Burley 2020 hash-based scrambling, shuffling and padding), blue-noise
dithered across the image (Georgiev & Fajardo 2016).

**Padding.** Every draw is an independently randomized copy of the *same* low Sobol dimensions
rather than a slice of one high-dimensional sequence — "padding", Burley 2020 §5, and what
Cycles ships. Each draw gets its own Owen scramble and its own Owen-shuffled sample index, so
consecutive draws are uncorrelated while each one individually keeps the best stratification
the sequence offers.

This matters because Sobol's 2D projections degrade as the dimension pair climbs. Measured
here, dimensions (0,1) are a perfect (0,7,2)-net while (62,63) is only a (4,7,2)-net — sixteen
points per cell instead of one — so a deep path drawing dimensions 50+ for its last bounces
would sample them worse than white noise. Padding means a 12-bounce path's last bounce is
stratified exactly as well as its first, and it removes any dimension budget: there is no depth
past which the sampler degrades.

**Pixels share the sequence.** Pixels do not get their own randomization. They all draw the one
sequence, and what separates them is a toroidal shift read from a blue-noise mask tiled over the
image (Cranley & Patterson 1976; Georgiev & Fajardo 2016). Giving each pixel its own scramble
instead is the white-noise special case of the same construction: it leaves neighbouring pixels'
errors independent, the arrangement the eye and every subsequent filter handle worst.
Correlating them moves error out of the low frequencies without reducing it, so this is a
perceptual change, not a convergence one — the RMSE curve sits where it did.

The shift is a property of (pixel, dither channel), fixed across the accumulation:

- It must **not** vary with the sample index. That would be a fresh random rotation per sample
  and would destroy the stratification the sequence exists to provide, exactly as a per-sample
  `scrambleSeed` would.
- It must vary **per channel**. One shared shift puts a pixel's whole sample vector on the
  diagonal of the d-torus, and a neighbourhood of pixels then integrates the path integrand
  along a line rather than over the torus, leaving a slowly varying low-frequency residual.
  Measured at 199× white noise in the lowest octave before each channel was given its own
  translation of the mask.

**Constructor argument roles.** These are not interchangeable, and swapping them silently
degrades the sampler to white noise:

| Argument | Role |
|---|---|
| `sampleIndex` | *Advances* per accumulated sample. Selects the point along the sequence; what makes a pixel's accumulated point set stratified rather than N independent points. |
| `scrambleSeed` | *Fixed* across an accumulation, fresh per render. Randomizes the sequence (Owen 1995) so different pixels and renders decorrelate without disturbing stratification. |
| `sampleCount` | Total this image will accumulate. Bounds the sequence drawn from (Burley 2020 §5.2; Cycles' `shuffled_index_mask`). A performance parameter, not a correctness one — overstating costs speed, understating would alias two sample indices onto one sequence point, so the sampler floors the bound at the index it is given. Pass 0 when unbounded: that selects the full 32-bit sequence, correct and simply slowest. |

Averaging N independently randomized copies of a *single* point is plain Monte Carlo no matter
which sequence produced the point, so a `scrambleSeed` that varies per sample — or a
`sampleIndex` that does not advance — forfeits the entire benefit. `sampler_validate`'s
pass-direction check is the gate on exactly that mistake.

## BSDF lobe selection

`include/pathtracer/scene/bsdf.h`, `src/scene/bsdf.cpp` — `sampleBsdf`

Four strategies: rough specular reflection, diffuse, refraction, and transmit-side multiple
scattering. One is chosen per sample by a Fresnel- and energy-derived probability.

**Diffuse and specular share a mixture estimator.** Both lobes are evaluated at whichever `wi`
was drawn, not just the lobe that was sampled. A rough surface's lobes overlap, so evaluating
only the sampled lobe would under-count every direction the other lobe also reaches.

**The specular selection probability is scaled by E(mu_o, roughness)**, the GGX directional
albedo. VNDF sampling then takes the single-scattering share and the cosine strategy takes the
multiple-scattering share, which is cosine-shaped. Without the scale the two strategies compete
for the same energy and the split stops being a partition.

**Transmission** is Walter et al. 2007 rough refraction about a VNDF-sampled microfacet normal,
falling back to a pure-Snell delta lobe below the smooth-roughness threshold — matching PBRT's
`EffectivelySmooth`, so smooth glass stays exact and noise-free. TIR is folded into the specular
probability. The boundary is a single non-nested dielectric.

**Transmit-side multiple scattering is a fourth strategy**, not a share of refraction sampling,
and is cosine-distributed over the far hemisphere. Refraction sampling reaches only directions
some microfacet can refract into, whereas the multiple-scattering lobe spans the whole
hemisphere, so no reweighting of the refraction strategy can cover it.

## Over-range readout binning

`include/pathtracer/scene/path_tracer.h` — `overRangeBin`, `OverRangeStats`

**Bracket.** `kOverRangeEvRadius = 16` stops either side of unity is the range the readout is
exact over. The display exposure control reaches ±13.95 EV (the HUD's f/1–f/22 aperture,
1/4000–1 s shutter and ISO 50–6400 sliders against `profile.json`'s defaults), so 16 — the next
binade bound above that — puts every threshold those controls can produce strictly inside.
Thresholds outside are clamped, which is what makes the count exact everywhere inside: a value
beyond either end folds into that end's bin, where it is unambiguously above or below every
interior threshold.

**Resolution.** `kOverRangeSubBinBits = 10` splits each binade into 1024 bins. Chosen by
measurement: the readout's only error is the occupancy of the bin holding the threshold, which
on cornell at 2048×1152 falls 2.6 → 0.65 → 0.18 → 0.047 percentage points as the bits go
4 → 6 → 8 → 10. 0.047 pp is below half the 0.1 pp the HUD's `%.1f%%` can display, so the digit
on screen is the digit an exact per-texel count would print.

**Why a shift and not a log2.** Positive IEEE-754 floats are monotone under integer comparison
of their bit patterns, so right-shifting those bits *is* a monotone binning of the whole
positive range. Edges are exactly representable floats, widths are one binade over
2^`kOverRangeSubBinBits` (uniform in stops to within the mantissa's 1.4× spread across a
binade), and it costs a shift and a subtract.

The clamp brackets the range and also keeps the index valid for every bit pattern: negatives
and −0 fold into bin 0, where they can never exceed a positive threshold; infinities and NaNs
fold into the top bin, where they read as over-range — the truthful answer for a texel that
cannot be displayed at all.

**Exposure is factored out.** Exposure is display-stage only, so it is not in `main.cpp`'s
`PathTraceInputState` and moving it never retraces. A fixed threshold evaluated on the driver
thread would therefore freeze on a converged image while the slider moves. Instead the
displayed peak is `exposure * rawPeak` and the displayed fraction is `aboveBin` read at
`1/exposure`, so both stay live under a drag for O(1) render-thread work rather than a pass
over every texel.

`aboveBin` is a complementary CDF rather than the histogram it is folded from, so the reader
indexes instead of summing: `aboveBin[b]` is the number of texels whose bin is ≥ b, hence
`aboveBin[0]` is the texel count and `aboveBin[kOverRangeBinCount]` is 0.

## Transport AOV bucketing

`include/pathtracer/scene/path_tracer.h` — `PathTraceResult`

The five transport buckets (direct/indirect × diffuse/specular, plus refraction) are a
decomposition of beauty, not a rescaling: every value written is the same radiance that went
into beauty, attributed rather than reweighted.

**The five buckets plus background sum to beauty exactly.** Background is the bounce-0 miss —
the camera seeing the environment with no surface interaction — and is deliberately unbucketed,
as production renderers also keep it out of the surface-transport AOVs. It is the only radiance
beauty carries that the five do not. `tools/integrator_validate.cpp` asserts the identity per
pixel with `showSky` off, which zeroes the background term.

**Bucketing rule.** A path is bucketed once, by the lobe sampled at its first (bounce 0) surface
interaction, independent of how many further bounces it takes. The one exception: any
transmission-lobe sample, at bounce 0 or later, stickily overrides the bucket to Refraction from
that point on.

Direct vs indirect is not tracked separately. It falls out of which bounce the
radiance-contributing event lands on — bounce 0's NEE and bounce 1's BSDF-sampled miss are the
two halves of the same one-vertex path. Bounce 0's NEE contribution is the one place a single
sample writes several buckets, split by the lobe that actually carried the light
(`bsdf.h`'s `BsdfEval`) rather than by the lobe the continuation ray drew.

**Not an LPE.** Bucketing by first event describes a debug viewer's decomposition, not an
arbitrary path expression. A diffuse bounce off a red wall onto a specular surface still reads
as diffuse transport, and later-bounce surfaces' colours legitimately tint the indirect buckets.

## Path-traced render contract

`include/pathtracer/scene/path_tracer.h` — `renderPathTraced`

Blocking, multithreaded (one thread per hardware core, dynamically scheduled square tiles)
unidirectional path trace: BSDF-sampled recursive bounces with next-event estimation against
`lights`, MIS power heuristic against BSDF sampling, Russian roulette from
`russianRouletteStartBounce`.

**Filtering.** Samples are reconstructed through a Blackman-Harris filter of 1.5 px radius
rather than accumulated per pixel, so each sample contributes to several pixels and every output
image is a weighted mean over the filter's support. All 10 images share one weight, which is
what keeps the transport buckets an exact partition of beauty *through* filtering.

**Parameters whose contract is not obvious:**

| Parameter | Contract |
|---|---|
| `showSky` | Gates only the primary ray's own miss — the camera seeing the background directly. Indirect bounces and NEE always sample real light radiance, so hiding the background does not unlight the scene. |
| `instanceLightIndex` | Parallel to `instances`; −1 for ordinary geometry, else the index into `lights`' quads. A hit on an emitter instance is Le, not a BSDF vertex (see `tracePath`). |
| `out` | Caller-owned, **must** already be sized width × height (see `makePathTraceResult`). By reference so a progressive driver allocates once rather than 10 fresh images per pass. Every pixel of every image is written, so no pre-clear is needed and a reused buffer carries nothing over. |
| `generation` / `requestedGeneration` | Cooperative cancellation for `PathTraceDriver`'s async use. Each worker checks `generation.load() != requestedGeneration` once per tile — the same polling idiom as `ThreadPool`'s index-stealing atomic — and returns early if superseded, leaving unwritten tiles as the previous pass left them. Safe because a cancelled pass is discarded whole and the next pass rewrites every tile. A synchronous caller passes an atomic already holding `requestedGeneration`. |
| `stats` | Ray/tile counters for this pass, caller-owned and reused. `reset()` before, read after (see `render_stats.h` for why the reads need no explicit fence). |
| `threadPool` | Caller-owned and reused; `PathTraceDriver` keeps one for its lifetime, avoiding OS thread creation and join per pass. |
| `scrambleSeed` / `sampleBase` | The sampler's two independent inputs, **not** interchangeable. `sampleBase` is the count already accumulated, so pass N supplies N and the sampler advances along its Sobol sequence. `scrambleSeed` randomizes that sequence and must stay fixed for every pass of one accumulation, varying only on restart. A `scrambleSeed` that changes per pass, or a `sampleBase` stuck at 0, degrades the sampler to plain Monte Carlo — the defect this pair replaced, and what `sampler_validate`'s pass-direction check guards. See [Sobol padding and net quality](#sobol-padding-and-net-quality). |

## Radially averaged power spectrum

`include/pathtracer/debug/power_spectrum.h`

Total power per octave of spatial frequency for a scalar field. Index 0 is the highest octave
(Nyquist/2 to Nyquist), index `kSpectrumBands-1` the lowest.

It exists because two separate questions in this repo are questions about how energy distributes
over spatial frequency rather than how much of it there is: whether the baked dither mask really
is blue noise (`sampler_validate`), and whether a sampling change rearranged a render's error
without changing its magnitude (`render_beauty --error-spectrum`). Both are invisible to any
single scalar, and neither is answerable by eye.

**Construction.** The mean is removed first: DC is a bias in the field, not an arrangement of it,
and would otherwise swamp the lowest band. Frequencies are normalised per axis (cycles/pixel,
Nyquist 0.5) so a non-square field bins correctly. The transform is a separable DFT, rows then
columns — O(n³) rather than a direct 2D transform's O(n⁴), which is milliseconds at a mask's
128² and seconds at render resolution. Deliberately not an FFT: these run behind a flag or in a
validator, never per frame, and a radix-2 implementation would constrain the caller's dimensions
to powers of two for no gain.

**The white-noise null is analytic, not measured.** White noise has a flat spectrum, so the
expected band distribution is just each band's share of the DFT lattice points. Bands are far
from equal in width, so a raw share is not interpretable without it.

A sampled null would cost a seed, carry its own sampling noise, and — the failure actually hit
here — can be correlated with the field under test: shuffling a 16384-element array with
`mt19937(1)` reproduces the exact permutation `bluenoise_mask.cpp` uses to seed the mask, which
inflated the null eightfold and silently loosened the gate built on it.

## Flat C ABI

`include/pathtracer/api/pathtracer_c.h`

A C ABI rather than a Python extension module, on purpose: it adds no third-party dependency,
needs no Python headers to build, and one dylib serves every interpreter and every version of
it. `ctypes` also drops the GIL around each foreign call by construction, which is what a render
taking milliseconds to seconds needs.

Two rules hold across the whole header:

- **Threading.** Every function is safe to call on distinct `PtRenderer` instances from distinct
  threads. None is safe to call concurrently on the *same* instance — one renderer owns one
  thread pool and one set of reused buffers.
- **Ownership.** Every output buffer is allocated by the caller. Nothing returns memory that
  must be handed back for freeing, so there is no ownership protocol to get wrong across the
  boundary.
