# Derivations

Long-form reasoning behind implementation choices that a source comment cannot hold in one line
(`notes/architect.md` §7). Each entry is referenced from the code it justifies. Nothing here is
required to read the code; it is here so the argument survives the comment budget.

See also [References](../README.md#references) for the literature each technique implements, and
[ROADMAP](ROADMAP.md) for what is deliberately not implemented.

## Contents

| | |
|---|---|
| [Sobol padding and net quality](#sobol-padding-and-net-quality) | `include/pathtracer/scene/sampler.h`, `src/scene/sampler.cpp` |
| [BSDF lobe selection](#bsdf-lobe-selection) | `include/pathtracer/scene/bsdf.h`, `src/scene/bsdf.cpp` — `sampleBsdf` |
| [Over-range readout binning](#over-range-readout-binning) | `include/pathtracer/scene/path_tracer.h` — `overRangeBin`, `OverRangeStats` |
| [Transport AOV bucketing](#transport-aov-bucketing) | `include/pathtracer/scene/path_tracer.h` — `PathTraceResult` |
| [Path-traced render contract](#path-traced-render-contract) | `include/pathtracer/scene/path_tracer.h` — `renderPathTraced` |
| [Radially averaged power spectrum](#radially-averaged-power-spectrum) | `include/pathtracer/debug/power_spectrum.h` |
| [Flat C ABI](#flat-c-abi) | `include/pathtracer/api/pathtracer_c.h` |
| [GGX numerical forms](#ggx-numerical-forms) | `src/scene/bsdf.cpp` — `distributionGGX`, `smithRadical`, `smithVisibility` |
| [Kulla-Conty energy tables](#kulla-conty-energy-tables) | `src/scene/bsdf.cpp` (`albedo_table.inc`), baked by `tools/albedo_table.cpp` |
| [Multiple-scattering lobe sampling](#multiple-scattering-lobe-sampling) | `src/scene/bsdf.cpp` — `sampleMsReflect`, `sampleMsTransmit`, `msReflectPdf`, `msTransmitPdf` |
| [Average Fresnel quadrature](#average-fresnel-quadrature) | `src/scene/bsdf.cpp` — `conductorFresnelAvg`, `dielectricFresnelAvg` |
| [Dielectric coat coupling](#dielectric-coat-coupling) | `src/scene/bsdf.cpp` — `coatAlbedo`, `multiScatterTint` |
| [Cauchy dispersion](#cauchy-dispersion) | `src/scene/bsdf.cpp` — `cauchyIor` |
| [EON albedo inversion](#eon-albedo-inversion) | `src/scene/bsdf.cpp` — `eonAlbedoInversion` (Portsmouth, Kutz & Hill 2025, JCGT 14(1), App. A) |
| [Smooth-transmission threshold](#smooth-transmission-threshold) | `src/scene/bsdf.cpp` |
| [Conductor Fresnel](#conductor-fresnel) | `src/scene/bsdf.cpp` — `conductorIorFromReflectivity`, `fresnelConductorChannel` |
| [Retrace trigger state](#retrace-trigger-state) | `src/main.cpp` — `PathTraceInputState`, `PathTraceTriggerState`, `RasterTriggerState` |
| [Dispersion channel commitment](#dispersion-channel-commitment) | `src/scene/path_tracer.cpp` — `tracePath` |
| [Transmission ray offset](#transmission-ray-offset) | `src/scene/path_tracer.cpp` |
| [Rasterizer buffer reuse](#rasterizer-buffer-reuse) | `include/pathtracer/scene/rasterizer.h` |
| [Static analysis policy](#static-analysis-policy) | `.clang-tidy` |
| [Check discovery](#check-discovery) | `cmake/PathtracerChecks.cmake` |
| [Build flag policy](#build-flag-policy) | `CMakeLists.txt` |
| [Rasterizer benchmark design](#rasterizer-benchmark-design) | `tools/raster_bench.cpp` |
| [Running-mean forward error](#running-mean-forward-error) | `tools/driver_validate.cpp` — `running_mean_matches_batch_mean` |
| [NEE and MIS validation](#nee-and-mis-validation) | `tools/nee_validate.cpp` |
| [Headless beauty render](#headless-beauty-render) | `tools/render_beauty.cpp` |
| [Albedo table bake](#albedo-table-bake) | `tools/albedo_table.cpp` → `src/scene/albedo_table.inc` |
| [Integrator validation](#integrator-validation) | `tools/integrator_validate.cpp` |
| [BSDF validation](#bsdf-validation) | `tools/bsdf_validate.cpp` |

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

### Implementation notes

**Direction vectors** are derived at static init rather than checked in, matching
`path_tracer.cpp`'s `buildFilterTable()`: it keeps the committed data to the published seed rows
and puts the recurrence in the source where it can be checked against the paper. Published V is
1-indexed with V[i] scaled by 2^(32−i); this table is 0-indexed by bit position, so index b
holds published V[b+1].

**Owen scrambling** (Owen 1995) is a random permutation of each node of the value's binary digit
tree, preserving the sequence's net properties exactly while decorrelating it. Burley (2020,
JCGT 9(4)) showed a cheap integer hash reproduces that structure without building the tree,
since in bit-reversed order a hash's carry propagation touches exactly the ancestors of each
digit.

The constants are Vegdahl's improved Laine-Karras hash as shipped by Cycles, not Burley's
original: the same construction with measurably better tree-permutation quality. Transcribed
from that source, never hand-derived — these are permutation constants, and a mistyped digit
degrades the scramble in ways no image inspection would catch.

**Dimension 1 is not looped.** Its direction vectors are V[b] = 1 << (31 − b), so XOR-ing the
ones the index's set bits select is *by definition* that index's bit reversal — an identity, not
an approximation, and one RBIT rather than a loop iteration per set bit. Measured over `next2D`:
62 → 48 ns/draw from this alone, and 14 ns once the index mask bounds dimension 2's loop.

**The index mask** bounds the sequence to the length actually drawn from. Without it the per-set
shuffle spreads a small sample index across all 32 bits and `sobolPoint`'s loop runs once per
set bit — about 16 iterations rather than at most log2(sampleCount). Measured at 128 samples:
62 ns/draw unmasked, 14 ns masked.

Its floor at `sampleIndex + 1` is a correctness guard, not a tuning knob: a caller understating
`sampleCount` would otherwise mask two distinct sample indices onto the same sequence point,
drawing one point twice and biasing the estimate with nothing to signal it.

**Per-set index shuffling** (Burley 2020 §5.2) is what decorrelates sets from one another;
without it every set would visit the sequence in the same order. Owen-scrambling the index
permutes its digit tree, so a power-of-two prefix stays a stratified point set rather than
becoming an arbitrary subset.

### The blue-noise shift

The shift is computed in the sampler's own 24-bit output space, so applying it is one add and
one mask and the wraparound *is* the toroidal wrap. MN = 2^14 divides 2^24 exactly, so the
mask's (rank + 0.5)/MN lands on rank·1024 + 512 with no rounding at all. That exactness is what
lets `sampler_validate` invert the shift and keep asserting the net properties at zero
tolerance.

For d = 1 the paper's matrix "is identical to a dither mask", and the mask here is Ulichney's
void-and-cluster array (1993), generated by `tools/bluenoise_mask.cpp`.

**Channel offsets come from R2**, the 2D low-discrepancy sequence of Roberts (2018): point c is
frac(c·(1/phi2, 1/phi2²)) where phi2 = 1.324717957244746 is the plastic number, the real root of
x³ = x + 1. The constants are 2^32/phi2 and 2^32/phi2², so the fixed-point multiply wraps to the
same frac().

A hash was tried first and is not sufficient: hashed offsets are free to land close together,
and two channels within the mask's correlation radius are correlated — measured at |r| = 0.14
between channels 8 and 23, which `sampler_validate`'s `checkChannelsAreDecorrelated` now fails
on. R2 spreads them by construction: the minimum toroidal separation over the 146 channels a
12-bounce path reaches is 8.1 px, where the sigma = 1.5 filter is ~1e-6.

Translating one mask per channel makes the components mutually decorrelated — a blue-noise
mask's autocorrelation is near-delta, so two different lags are effectively independent — while
each component stays exactly the same blue-noise field in screen space. That is a cheap stand-in
for the paper's §3 annealed d-vector matrix, which remains the principled upgrade.

The tile wraps by masking rather than modulo: `kMaskSize` is a power of two, so this is also
correct for the negative pixel coordinates a filter footprint can reach past the image edge.

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

**The pdf must not be gated on `kd`.** `sampleBsdf` selects the diffuse lobe with probability
`lobes.diffuse`, which `computeLobeProbabilities` derives deterministically from params and wo.
The pdf side of the MIS mixture must therefore match that selection density whatever value the
lobe itself carries. Selection mass may depend on `kd`, but only by moving to another strategy
of the same mixture.

`kd` carries the wo-side (1−F)/(1−Favg) coupling and `evaluateDiffuseLobe` applies the matching
wi-side (1−F), so the lobe is reciprocal (A4) while its directional albedo still integrates to
(1−F(mu_o)) — the same total energy as the old one-sided form, correctly distributed.

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

The three components partition `eval.total()` exactly, so the split writes the same energy
`radiance` just took, only attributed. `common` carries throughput as a factor shared by `radiance`
and all three accumulators, so the partition holds whatever throughput is — including the two
exactly-zero channels a dispersive path is masked to, which the hero-channel block can set before
this point at bounce 0 (**measured: 132461 of 2000000 bounce-0 NEE splits reach here non-unit**).

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

## GGX numerical forms

`src/scene/bsdf.cpp` — `distributionGGX`, `smithRadical`, `smithVisibility`

**D in the cancellation-free form** (Filament 4.4.2, Google 2018). The textbook denominator
`ndotH^2*(alpha^2-1)+1` subtracts two near-equal numbers wherever the half-vector is near the
normal, which at low roughness is the entire lobe. Measured: 20% of D lost at the peak at
alpha = 4e-4, and 3.3e-4 at alpha = 1e-2.

`nh` is in the local shading frame (N = +z), so `nh.x^2 + nh.y^2` *is* sin²(theta_h). Every
term of the sum is then non-negative and the peak value 1/(pi*alpha²) is exact.

**The denominator needs no floor.** d = alpha²cos² + sin² is minimised at alpha², and callers
enforce alpha ≥ `kMinAlpha`, so `kPi*d*d >= 8e-14` — twenty-four orders above float32
underflow. A floor here is not merely unnecessary but harmful: the one that used to stand here
engaged for every roughness below 0.0867 and suppressed D by 124340× at 0.02 and 81.5× at 0.05.

**`smithRadical`** is cos·sqrt(1 + alpha²tan²), the GGX Smith Lambda's radical scaled by the
cosine: 1 + Lambda(c) = (c + radical)/(2c). Unlike tan it is finite at c = 0.

**`smithVisibility`** is G2/(4cos_o·cos_i) for the height-correlated Smith G2 (Heitz 2014;
Filament's `V_SmithGGXCorrelated`). The cosines multiply rather than divide, so it is exact to
the silhouette and vanishes only where both cosines do.

## Kulla-Conty energy tables

`src/scene/bsdf.cpp` (`albedo_table.inc`), baked by `tools/albedo_table.cpp`

Kulla & Conty 2017, "Revisiting Physically Based Shading at Imageworks". The `.inc` defines
`kAlbedoRoughnessRes`, `kAlbedoMuRes`, `kMsReflectMuRes`, `kTransmitRoughnessRes`,
`kTransmitMuRes`, `kEtaRes`, `kEtaMin` and `kEtaMax` alongside the arrays, so the grid the
lookups index is the grid the generator wrote and the two cannot drift apart.

**`kAlbedoA`/`kAlbedoB`** are the directional albedo of the single-scattering GGX lobe with
Fresnel forced to 1 — the fraction of energy the height-correlated Smith G2 lets through, so
1−E is exactly what multiple scattering must return. Split by Schlick's form
F(c) = f0(1 − (1−c)⁵) + (1−c)⁵ so one table serves any f0 (the standard environment-BRDF
split): Ess(mu, f0) = f0·a + b, which at f0 = 1 collapses to a + b = E, the Fresnel-free albedo
the multiple-scattering lobe needs. Indexed `[roughnessIndex][muIndex]`, mu uniform in
sqrt(mu).

**`kEscapeReflect`/`kEscapeTransmit`** are the escaping fraction of a dielectric interface,
split into reflected and transmitted shares, indexed `[roughnessIndex][muIndex][etaIndex]`,
mu uniform in sqrt(mu). They use *exact* dielectric Fresnel rather than the Schlick split:
inside the total-internal-reflection cone exact Fresnel is 1.0 while Schlick reads ~0.1, so no
rescale of a Schlick-basis number can stand in for it and the escape budget would under-count
the reflected share by the whole TIR cone. That third axis is why they keep their own grid.

**Baked offline, not at startup.** Building it at startup is what used to bound its accuracy:
the grid and the quadrature were sized by load latency rather than by what the energy tests
need. See [Albedo table quadrature](#albedo-table-quadrature) for the rule and its residual.

**The mu axis is uniform in sqrt(mu)** because E climbs from 1 at grazing to its plateau over
mu ~ alpha (0.31 at mu = 1, alpha = 1) — a layer a uniform mu axis spans with under one cell at
low roughness.

The escape term is `R + T`, unweighted by `transmissionFactor`: energy the interface refracts but
`transmissionFactor` withholds from the transmit lobe enters the diffuse substrate instead (what
`diffuseKd`'s `(1-transmissionFactor)` does) and escapes from there. Either way it leaves, so the
microfacet deficit the compensation must return is the same. Weighting the term by `transmitWeight`
instead over-reported the deficit at partial transmission — **measured Lo = 2.64 against a bound of
2.25**.

## Multiple-scattering lobe sampling

`src/scene/bsdf.cpp` — `sampleMsReflect`, `sampleMsTransmit`, `msReflectPdf`, `msTransmitPdf`

**Why not cosine sampling.** The reflect lobe's value is
fms·(1−E(mu_o))·(1−E(mu_i))/(pi·(1−Eavg)), so the density making f·cos/pdf independent of wi is
(1−E(mu_i))·cos/(pi·(1−Eavg)). Cosine sampling instead pays the ratio (1−E(mu_i))/(1−Eavg) as
weight variance. Measured relative variance 0.029 at roughness 1, but **+1.66 at 0.25 and
+17.3 at 0.126**, where 1−Eavg and 1−E(mu) are both small and their quotient is not, with
weights reaching 92×. The win is at *low* roughness, not high.

Cosine was the standard practical choice (Kulla & Conty 2017). The table holds the
zero-variance shape instead, as a piecewise-linear density over mu with its exact prefix
integrals. The transmit side does the same: its cosine-sampling weight variance was 25 at
roughness 0.13.

**One interpolant for value, density and sampler** (Dupuy & Jakob 2018). The row blend is the
albedo lookups' own, so the density the sampler inverts and the density the pdf evaluates are
the same function of roughness — which is what keeps the estimator unbiased rather than merely
close.

**The sampling grid is uniform in mu, not sqrt(mu).** It reads the sampling shape, whose grid
stays uniform so the inversion keeps one step width. The albedo table's warp does not reach it:
the generator already applied that warp when it built each node from E.

**Stable quadratic root.** Inverting a piecewise-linear density on an edge-aligned grid means a
binary search of the prefix integrals for the segment, then the positive root of its quadratic.
Written as `2c/(q + sqrt(q*q + 2*dq*c))` rather than the textbook `(-q + sqrt(...))/dq`. The
two are algebraically identical, but this form stays finite as a segment flattens (dq → 0,
where it reduces to c/q) and at mu = 0, where q is exactly 0 and it reduces to sqrt(2c/dq).

Both fetches must see the same normalised density, so a caller holding an unnormalised table
scales both by its total rather than passing raw entries: the 1e-9 floor is a density-scale
quantity, not a free epsilon.

**The transmit table is stored unnormalised** and the blend divided by its own blended total,
rather than each row being normalised at bake time. Integration is linear, so a blend of exact
prefix integrals *is* the exact prefix integral of the blended density — and this is the
interpolation of raw deficits `escapeAlbedo` itself performs. Blending four already-normalised
rows would not commute with it.

It also makes a numerically dead row harmless: it contributes its own near-zero weight to the
blend instead of a unit-mass shape of amplified noise, so no row needs a bake-time abort or a
substituted fallback.

**Two rows per orientation:** the reciprocal etaT/etaI for the transmitted share, whose wi has
crossed into the far medium, and the forward etaI/etaT for the transmissive reflected share in
`evaluateSpecularLobe`, whose wi has not.

## Average Fresnel quadrature

`src/scene/bsdf.cpp` — `conductorFresnelAvg`, `dielectricFresnelAvg`

Cosine-weighted average Fresnel, 2·∫₀¹ F(mu)·mu dmu — the normalisation both the
multiple-scattering tint and the reciprocal diffuse coupling need. One 3-node rule
Σ wᵢF(muᵢ) serves both interfaces, each evaluated over the same Fresnel its own single scatter
evaluates, so an average and the term it compensates always describe one interface.

**Fit.** Nodes and weights by equality-constrained least squares against 128-point
Gauss-Legendre over the whole clamped Gulbrandsen domain (r ∈ [1e-4, 0.9999] × g ∈ [0, 1]):
max absolute error 4.0e-4, RMS 1.2e-4, measured in float32 through the shipped inversion.
Karis is 216× worse there (max 0.086, at r = 0.255, g = 1).

The weights sum to exactly 1.0F in float32, so each rule is near-exact wherever its F is
constant in mu — the r→1 mirror `checkWhiteFurnaceTwoSided` runs on, where the conductor lands
within 4e-8 — and is a convex combination of F values, so it cannot leave [0, 1] at all.
`multiScatterTint`'s 1/(1−Favg) survives the endpoint regardless: its f·f·a numerator goes to
zero on the same approach.

**A rational fit over (r, g) was measured and rejected.** At r = 0.99 the inverted n collapses
from 39.8 to 0.005 across the last tenth of g — a boundary layer no low-order form in that
chart holds; 49 terms reached only 9e-3. Sampling the function's own values sidesteps the
chart, and (n, k) is what F_avg actually depends on.

**The dielectric reuses the conductor's nodes, unrefitted.** Measured worst 5.5e-5 over
ior ∈ [1.05, 3.0] (at 1.0575) and 4.5e-5 over [1.1, 3.0], against 6.5e-3 for the two-constant
rational fit it replaces (at 1.17) and 2.5e-2 for Karis' Schlick mean (at 1.1).

Uniformity is the point, not just magnitude: the rational fit was worse than the Schlick mean
below ior ≈ 1.42, which is why `checkCoatFresnelAvg` could only sweep 1.5–1.8. The rule beats
both everywhere except ior ≤ 1.0046 and a 4.4e-4-wide sliver at 1.6518 where the fit's own
error crosses zero and any fit is momentarily exact — the rule is still within 3.0e-6 there.

**Exactly +0 at ior 1, structurally.** etaI/etaT is exactly 1, so `cos2Transmitted` returns
mu·mu, sqrt(mu·mu) is exactly mu, and both polarisations are an exact (c−c)/(c+c).
`checkIndexMatchedCoat` requires that zero at tolerance exactly 0, and it comes from the same
function the single scatter calls, collapsing for the same reason.

**Known limitation, bounded and measured.** As ior → 1 the reflectance becomes a boundary layer
— F(0) = 1 for every ior > 1, collapsing over a width ~sqrt(ior−1) — which three fixed nodes
cannot resolve. Worst 5.9e-4 at ior 1.0057, where truth is itself 1.8e-3. It reaches
`coatAlbedo` scaled by d(coatAlbedo)/dF_avg ≈ 0.054, so ~3.2e-5 on the coupling, and no shipped
material is in that band.

**The exact closed form (d'Eon & Irving 2011) was rejected.** It would re-derive the interface
in a second place, which is the desynchronisation `fresnel_dielectric.h` exists to prevent, one
level up. Its accuracy is unobservable, since the only instrument that can read F_avg is itself
table-limited at ~1e-4, and its log((ior−1)/(ior+1)) and 1/(ior⁴−1) terms cancel
catastrophically in exactly the ior → 1 band where the quadrature is weakest.

## Dielectric coat coupling

`src/scene/bsdf.cpp` — `coatAlbedo`, `multiScatterTint`

**The coat reflects its total directional albedo, not F(mu_o).** Single scatter plus its own
multiple-scattering lobe. At roughness 1 and mu 0.4 the two differ by 4× (0.030 vs 0.129), and
coupling the diffuse substrate to F(mu_o) hands that difference to neither lobe — measured as a
**10% energy loss** before this was used.

**`fresnelRatio`** rescales the single-scatter term by exact-dielectric ÷ Schlick Fresnel at
this direction. The table is built on Schlick's basis so one table serves any f0, but the
specular lobe evaluates exact `fresnelDielectric`, and Schlick under-predicts it at grazing,
leaving the substrate too much energy — ~1.4% at smooth grazing angles.

The rescale makes the two agree exactly in the smooth limit, where the coat albedo *is* the
Fresnel term, and approximately as roughness widens the lobe away from the macro angle. It also
collapses correctly at ior = 1, where exact Fresnel is identically zero but Schlick's (1−c)⁵
tail is not.

**`fresnelAvg` is the coat's own cosine mean**, `dielectricFresnelAvg(ior)`, for the same
reason: the coat reflects by exact `fresnelDielectric`, so Karis' Schlick mean of `coatF0`
describes the wrong function here too (−0.0061 against −4.8e-6 at ior 1.5) and, unlike the
quadrature rule over that same Fresnel, does not collapse to 0 at ior = 1.

**`multiScatterTint`** is the share of the (1−E) energy surviving repeated bounces on the
microsurface, each attenuated by Favg. It equals 1 for a perfect reflector (Favg = 1), so a
white conductor conserves exactly.

**It takes etaI/etaT, not a bare ior.** On the exiting side `fresnelDielectric` returns exactly
1.0 past the critical angle, and Schlick cannot express total internal reflection at all.
Hardcoding the entering orientation under-reported the reflected share of an exiting ray by up
to the whole TIR cone, which the compensation then tried to hand back as multiple scattering.

`bsdf_validate`'s `coat_fresnel_average` recovers `fresnelAvg` by inverting this coupling, which is unimodal in it: ternary search locates the extremum and bisection then runs on the
monotone side. That side is monotone *increasing*, which is not obvious — `F_avg` raises
`coatAvg` faster than it raises `coatAlbedo`.

## Cauchy dispersion

`src/scene/bsdf.cpp` — `cauchyIor`

Two-term Cauchy n(lambda) = A + B/lambda², with (A, B) inverted from the authored (n_d, V_d) —
the construction Khronos `KHR_materials_dispersion` specifies normatively. Two terms is the
right order: the material supplies exactly two numbers, so a Sellmeier form would have to
invent its remaining coefficients.

B follows from the Abbe definition applied to the Cauchy form,
n_F − n_C = B(lambda_F⁻² − lambda_C⁻²), and A from pinning n(lambda_d) = n_d. Written in that
general form rather than the spec's composite one, which pre-multiplies
1/(lambda_F⁻² − lambda_C⁻²) into a literal 523655 and hides both Fraunhofer lines inside it.

The Fraunhofer d, F and C lines are the three wavelengths the Abbe number is defined at and the
only ones (ior, abbe) actually pins — physical constants of the definition, not tuning.

`abbe <= 0` is the documented off switch (Arnold's `transmission_dispersion_abbe` and OpenPBR's
dispersion scale use the same convention) and keeps 1/abbe off the hot path for every
non-dispersive material. No clamp on the result: n_d = 1 already gives B = 0 exactly, so an
index-matched medium is non-dispersive out of the algebra rather than by special case.

## EON albedo inversion

`src/scene/bsdf.cpp` — `eonAlbedoInversion` (Portsmouth, Kutz & Hill 2025, JCGT 14(1), App. A)

The rho achieving a desired *observed* albedo, so `diffuseColour` means what OpenPBR says
`base_color` means: "the observed reflection color (viewed at normal incidence under uniform
illumination) in areas where the Fresnel reflection is negligible". OpenPBR declares that
meaning but then sets rho = C directly, which does not deliver it at high `diffuseRoughness`.

Eq. 29 gives the normalised FON albedos at normal incidence, E_F(N) = 1/(1+c₁r) and
⟨E_F⟩ = (1+c₂r)/(1+c₁r). Setting eq. 28's E_EON(N) = C yields the quadratic
a·rho² + b·rho − C = 0 with eq. 31's a = ⟨E_F⟩ − E_F(N) and b = E_F(N) + C(1−⟨E_F⟩), both
non-negative over the whole domain.

**Eq. 30 states the unstable root.** It gives (−b + sqrt(b² + 4ac))/(2a); a is proportional to
r, so as r → 0 a vanishing denominator divides a difference of near-equal quantities. The
paper's remedy is a Taylor form switched in below some roughness. The conjugate-multiplied root
used here is algebraically identical, needs no threshold, and cancels nothing.

So the whole domain is one branch-free expression: r = 0 gives a = 0, b = 1 and so rho = C —
the Lambertian identity out of the algebra rather than special-cased — and C = 1 gives rho = 1
exactly, leaving the white furnace untouched.

## Smooth-transmission threshold

`src/scene/bsdf.cpp`

Below the threshold the GGX transmission lobe is treated as a delta (PBRT's
`TrowbridgeReitzDistribution::EffectivelySmooth`). `kMinAlpha` (roughness 0.02) sits inside
this region, so smooth glass keeps the exact, noise-free Snell path rather than becoming a
stochastic estimate of the same thing.

**ior == 1 is a delta at every roughness**, not a rough interface. `refractAbout` returns −wo
about every microfacet normal, so `evaluateTransmissionLobe`'s half-vector
normalize(wo + etaR·wi) normalises the zero vector and every guard below it becomes a NaN
comparison. Measured NaN throughput on 7783 of 7783 transmission draws at roughness 0.1, and
6666 of 7783 at roughness 1.0.

## Conductor Fresnel

`src/scene/bsdf.cpp` — `conductorIorFromReflectivity`, `fresnelConductorChannel`

Gulbrandsen 2014, "Artist Friendly Metallic Fresnel", JCGT 3(4), ported from the paper's
Appendix A listing. Replaces Schlick on the metal path, which is monotone in cos by
construction ((1−c)⁵ ≥ 0) and therefore forces every conductor to exactly white at grazing,
unable to express the reflectance dip real metals have. Parameterised by reflectivity r
(`params.f0`) and edgetint g (`params.edgeTint`), inverted to a complex IOR (n, k).

### Clamping the inversion

Reflectivity is clamped, not asserted: f0 arrives from `resolveBsdfParams` as an unbounded
texture product (baseColor × diffuseColour), so both ends are reachable from an asset.

**Upper end.** At r = 1, nMax is infinite and g = 1 evaluates 0·inf = NaN. The listing clamps
to 0.99, which costs 1% of normal-incidence reflectance at f0 = 1 — enough to move the white
furnace test onto its tolerance edge. 0.9999 is safe here because of the k² form below:
measured in float32, n and k land within 1.4e-8 of their double values and the furnace rows
shift by under 2e-4.

**Lower end.** r = 0 inverts to n = 1, k = 0 — an index-matched interface, where the Fresnel is
0/0 at cosTheta = 0 exactly, and reachable from any black texel on a metal. At the 1e-4 floor
k = 0.02, F tends to 1 at grazing and nothing is degenerate; the floor costs a pure-black metal
1e-4 of normal-incidence reflectance.

### The factored k²

Paper eq. 12 for n (a linear blend in g between the two ends of eq. 11's range) and eq. 2 for
k. k² is evaluated as (nMax − n)(n − nLow) rather than the listing's
((n+1)²r − (n−1)²)/(1−r).

The two are the same expression — those factors are eq. 3's interval endpoints, i.e. eq. 2's
own roots — and agree to 4e-12 relative in double, which `bsdf_validate` asserts against the
literal form.

The literal form is what forces the listing's 0.99 clamp: it subtracts two large near-equal
numbers, and in float32 at r = 0.9999, g = 0 it returns k² = −1.28e6 where the true value is
exactly 0. The factored form returns exactly 0 there, since nMax − n is exactly zero at g = 0.

### Exact unpolarized reflectance

Born & Wolf, in the standard real-arithmetic form: two hardware sqrts, no complex division, no
`<complex>` in this translation unit.

**Not** the paper's Appendix A rs/rp, which is the large-|eta| approximation (PBRT-v2's
`FrCond`) and deviates by up to **0.094 absolute around r ≈ 0.25** — mid reflectivity, where
real metals such as `chrome.json`'s measured chromium (r ≈ 0.55) sit. `bsdf_validate` compares
this against the complex-arithmetic definition; the two match to 2.2e-12 over the clamped
(r, g, cosTheta) domain.

Deliberately free of clamps, unlike the inversion. No denominator can vanish once r is floored
away from 0: a2b2 − t0 > 0 follows from a2b2 ≥ |t0| alone, and a2b2 itself is positive either
because k ≥ 0.02 (as g drives n toward nMin) or because n = nMax > 1 forces t0 = n² − s2 > 0 at
g = 0, the one input where k is exactly 0. A max() here could not prevent a fault, only hide
one — which is exactly what one did.

**a² = (a2b2 + t0)/2, taken through whichever of its two algebraically equal forms is a sum.**
Direct when t0 ≥ 0. When t0 < 0 the direct form subtracts two near-equal magnitudes: at f0 = 1,
n is ~5e-5 and it must resolve 2.5e-9 out of two numbers near 1.0, which in float32 collapses a
to zero and pins F at exactly 1.0.

That failure is invisible to the obvious test: Schlick's own answer at f0 = 1 is also exactly
1.0, so the white furnace's conductor rows came back byte-identical and the fault read as "the
change is inert". The identity (a2b2 + t0)(a2b2 − t0) = 4n²k² turns that branch into a division
by a sum.

## Retrace trigger state

`src/main.cpp` — `PathTraceInputState`, `PathTraceTriggerState`, `RasterTriggerState`

A snapshot of every input `renderPathTraced`'s result actually depends on, compared frame to
frame to decide whether to hand `PathTraceDriver` a fresh request.

**The selected AOV is deliberately not in it.** One pass writes all of `PathTraceResult`'s
images, so which one is displayed cannot change what the driver has to compute. Keying on it
restarted a converged accumulation on every AOV switch — including between two of its own
lanes, and including into the four GPU post-filters, which only ever re-read beauty.

**Camera geometry is factored out** into a shared sub-struct: it is the whole of what
`renderRasterGBuffer`'s output depends on that can change (`rasterizer.h` takes no environment
argument) and the leading part of what `renderPathTraced`'s does. Factored rather than
duplicated so the two producers compare the same fields.

**The resolution is not in it.** `profile.json`'s `render.width`/`render.height` are fixed for
the session and the window is an independent viewport, so the traced size varies only through
`renderScale`, which the trigger already carries. Comparing two values that cannot differ would
be work that can never fire.

**Inputs and render scale are two structs, not one.** `renderScale` is *derived* from whether
the inputs changed. Folding it in would make the settle-time promotion to full resolution read
as fresh interaction on the very next frame, restarting the accumulation it had just earned.

**The rasterizer keeps its own trigger state** because the two producers refresh independently:
an environment change must retrace without re-rasterizing a G-buffer that does not depend on
it, and selecting a rasterizer AOV must not restart a path-trace accumulation.

**`pathTraceDriver` is a `unique_ptr`, not a by-value `optional`.** `PathTraceDriver` holds
reference members and an owned `jthread`/`mutex`, so it is neither copyable nor movable. A
by-value `optional<T>` member would propagate that non-movability to `AppResources` itself
(`optional<T>`'s move constructor exists only when T's does), breaking the
return-by-value/RVO pattern every other member relies on. A `unique_ptr`'s move transfers
ownership of the pointee's address without touching the reference members, so `AppResources`
stays movable and `PathTraceDriver` is never relocated once constructed — which its
constructor precondition requires.

`kSettleSeconds` is the quiet period before the renderer promotes itself back to full
`renderScale` — long enough that the momentary gaps between mouse-drag events during an orbit
do not each trigger a full-resolution restart, short enough to feel immediate when the camera
actually stops.

## Dispersion channel commitment

`src/scene/path_tracer.cpp` — `tracePath`

A path commits to one RGB channel on first reaching a dispersive interface, *before* any BSDF
work at that vertex, because the whole interaction is wavelength dependent — Fresnel and the
lobe probabilities as much as the refraction direction. Every later vertex then stays on that
wavelength, medium included.

Committing here rather than at path start, as a spectral hero-wavelength renderer must
(Wilkie et al. 2014), is strictly cheaper: a path that never meets dispersive glass keeps full
RGB and pays nothing.

**One-sample channel estimator** with probabilities proportional to the throughput carried so
far (OpenPBR implementation note, arXiv:2512.23696). The surviving channel takes T_c/p_c, which
is sum(T) for every c, so the estimator is unbiased *and* the path's magnitude — hence Russian
roulette's continuation probability — no longer depends on which channel was drawn.

**The `heroChannel` guard is an optimisation, not a correctness gate.** Masking leaves two
channels exactly zero, so a throughput-weighted redraw at a later dispersive vertex can only
return the same channel (measured: 400k redraws, zero changed). What it buys is one fewer
sampler dimension per later crossing. Under uniform 1/3 selection it *would* be load-bearing.

`sum == 0` is reachable, not impossible: `rrMinProb` floors the roulette, so a zero-throughput
path survives rather than being killed. It carries no energy to any channel, so there is
nothing to commit and the draw is skipped.

## Transmission ray offset

`src/scene/path_tracer.cpp`

Reflection and diffuse continuation rays stay close to the geometric normal's hemisphere, where
`kRayEpsilon`, a floating-point-scale constant, has always been sufficient.

Transmission bends sharply away from it. On curved geometry approximated by flat facets, the
shading-normal-derived refraction direction can clip a *neighbouring* facet a small but
non-zero distance away — a genuine geometric intersection, not floating-point noise. Measured
on a 500-triangle sphere: self-intersections at ~1e-3, an order of magnitude below a facet's
own edge length.

**Scaled by curvature, not raw facet size.** The mechanism is the smooth shading normal
diverging from the flat facet's true geometric normal across the facet, so the fix must vanish
wherever that divergence does. sin(angle) between each pair of vertex normals — the cross
product of two unit vectors — is exactly zero on any planar patch, whatever the facet's
absolute size, and grows with tessellation coarseness on genuinely curved geometry.

Scaling raw edge length alone, an earlier and broken version of this fix, has no such zero: it
blew up on a flat 2000-unit slab quad, pushing the continuation ray origin far past the
geometry it needed to traverse. `integrator_validate` caught it.

## Rasterizer buffer reuse

`include/pathtracer/scene/rasterizer.h`

`out` is owned by the caller and reused across calls, exactly as `threadPool` is. Its 14 images
are reallocated only when `width`/`height` change.

**Cleared per row inside the parallel loop, not up front.** These are the same bytes the earlier
per-call `makeImage` zeroing touched, but written in parallel, in the row that is about to be
overwritten, rather than as 14 sequential full-image memsets beforehand. At 2048x1152 that
allocate-and-zero was 566 MB per call.

**`instanceBounds`** is one world-space AABB per instance (`computeInstanceBounds`,
`shading_scene.h`), parallel to `instances`. It is computed once at load rather than per frame:
the geometry is static and its positions are already world-space.

## Static analysis policy

`.clang-tidy`

RAII and pointer discipline (`cppcoreguidelines-owning-memory`, the `pro-type-*` cast checks), the
~60-line "one function, one screen" rule (`readability-function-size`, `LineThreshold: 60`), and
general bug-pattern detection (`bugprone-*`).

**Vendored sources are exempted twice.** Their translation units go through the `vendored_cgltf`
and `vendored_imgui` OBJECT libraries, which never set `CXX_CLANG_TIDY` — that property is
target-level only in CMake, so there is no per-source override — and their headers go through
`ExcludeHeaderFilterRegex`.

**Three checks are disabled as false-positive noise** for this codebase's conventions, not because
the underlying concern is invalid elsewhere:

- `cppcoreguidelines-pro-type-union-access` fires on glm's internal union-based vector and matrix
  component access (`.x`/`.y`/`.z`). That is ordinary, well-tested library usage, not a union-safety
  hazard.
- `cppcoreguidelines-pro-type-vararg` fires on every `ImGui::Text`/`TextColored` call, ImGui's
  whole rendering API being printf-style vararg.
- `bugprone-exception-escape` flags `main()` simply for calling anything that is not `noexcept`
  (`std::string` and `std::vector` allocation, and so on) — true of nearly every C++ program's
  `main()`, not a specific defect.

## Check discovery

`cmake/PathtracerChecks.cmake`

Registers one ctest entry per check in a validator built on `tools/check.h`, by asking the binary
itself what it contains. CMake cannot run a target at configure time, so this uses the same
POST_BUILD mechanism as CMake's own `gtest_discover_tests`: the binary writes a CMake fragment and
`TEST_INCLUDE_FILES` pulls it into the test set. The consequence worth having is that adding a
check adds a ctest entry with no CMake edit at all.

**`PATHTRACER_TEST_THREADS`** keeps `ctest -j` from oversubscribing. Every render-driving check
would otherwise build a pool sized to `hardware_concurrency()`, so N concurrent checks would spawn
N*cores threads on cores. The binary emits a matching `PROCESSORS` property so ctest's own job pool
accounts for what each test will actually use.

## Build flag policy

`CMakeLists.txt`

`pathtracer_target_defaults` gates NATIVE, IPO and TIDY behind opt-in flags rather than applying
them everywhere.

**NATIVE and IPO are opt-in because two codegen tools must be reproducible.** `albedo_table` and
`metal_fit_core` write committed source (`albedo_table.inc`, and the conductor fits a material JSON
carries), so their output has to reproduce bit-for-bit on any machine. `-march=native` and FMA
contraction under IPO would both break that. Those two link no native library for the same reason —
only `glm::glm` — because `pathtracer_core` is itself built `-march=native`, which is exactly what
they must avoid. The bake runs in seconds either way.

`bluenoise_mask` writes a committed artifact too (`blue_noise_mask.inc`) but is **not** in that
group: it takes `NATIVE IPO` and links `pathtracer_core`, because it is convolution-bound and only
ever run by hand.

**TIDY is opt-in** because only the three shipping targets are gated; the tools are covered by
cppcheck and ctest instead.

**Everything else does take NATIVE and IPO**, including the validators, so that they exercise the
exact codegen that ships rather than a differently-optimized build of the same math.

### pathtracer_core membership

Every source that does not touch GL, GLFW or imgui. The rule is exactly "does this file include a
GL, GLFW or imgui header" — nothing in it can open a window, which is what lets a foreign runtime
(`tools/`, and the C ABI in `src/api/`) link it.

`cie.cpp` is deliberately absent: it belongs to `metal_fit_core`, built without `-march=native` and
IPO so its committed output reproduces exactly, and having it in both would be a duplicate symbol.

## Rasterizer benchmark design

`tools/raster_bench.cpp`

Timing harness for `renderRasterGBuffer`, the synchronous per-frame render-thread work behind the 14
primary-hit AOVs. Deliberately **not** an `add_test`: a benchmark is not a correctness gate, and the
rasterizer's gate is `rasterizer_validate`.

**Synthetic, not asset-driven**, so the two variables the cost is actually a function of are
independently controllable. `--triangles` sweeps the sub-triangle array past cache — the shipped
scene is 20561 triangles, 1.81 MB and resident, while the 5M-triangle tier is 440 MB and is not.
`--layers` sweeps depth complexity, which is what a depth prepass is a function of. Neither is
adjustable in a fixed asset.

**Shell construction.** Each of `layers` screen-filling shells holds an equal share of the triangle
budget, and per-shell triangle radius is derived from that shell's frustum cross-section, so every
shell covers the screen exactly once whatever the triangle count. Raising `--triangles` therefore
shrinks triangles rather than piling up overdraw, keeping the two axes independent.

**Submission order is the measurement.** Shells are emitted furthest first, so every shell improves
the depth record and a pixel accumulates `layers` of them — the quantity a depth prepass trades
against. Order, not shell count, decides this: nearest-first would have every deeper shell rejected
on arrival by the z-test, leaving one record per pixel however high `--layers` goes. This is
therefore the worst case. An unsorted real scene averages the harmonic number of records (~2.7 at 8
shells), and a front-to-back one none at all.

**Best-of-N, not the mean.** Run-to-run spread on this hardware is ±10%, wide enough to hide a
single change. `--bench-log` records the raw frames; `bench_compare run` makes the A/B.

## Running-mean forward error

`tools/driver_validate.cpp` — `running_mean_matches_batch_mean`

The driver publishes a *running* mean (Welford 1962; West 1979),

    m_k = m_{k-1} + (x_k - m_{k-1})/k,

a different rounding sequence from the batch mean `sum(x)/n`, so the two differ by a deterministic
forward error with no probability in it. Its float32 step, FMA-contracted or not, is

    m^_k = fl(m^_{k-1} + fl(fl(x_k - m^_{k-1}) * fl(1/k))),

and in Higham's theta/gamma calculus (*Accuracy and Stability of Numerical Algorithms*, 2nd ed.,
Lemmas 3.1 and 3.3) its error `e_k = m^_k - m_k` obeys, exactly,

    e_k = (1 - 1/k)(1 + d) e_{k-1} + (1 + d) theta_3 (x_k - m^_{k-1})/k + d m_k,
    |d| <= u,  |theta_3| <= gamma_3,  e_1 = 0,

so

    |e_k| <= E_k = (1 - 1/k)(1 + u) E_{k-1} + gamma_4 (|x_k - m_{k-1}| + E_{k-1})/k + u|m_k|

— the updating-mean analysis of Chan, Golub & LeVeque (1983) carried to a rigorous per-element
bound. The oracle evaluates `E_n` per float from the exact data plus `u|m_n|` for its own rounding;
evaluating it in double perturbs it by ~2^-29 of itself.

A running sum published without its division, a stale previous mean, a wrong `1/n`, or a missing
entry in the nine-image accumulate loop all miss this bound by orders of magnitude.

## NEE and MIS validation

`tools/nee_validate.cpp`

Standalone correctness check on `path_tracer.cpp`'s NEE + MIS combination (Veach & Guibas 1995
power heuristic), first against `EnvironmentMap`'s importance sampling and then against `LightSet`'s
rectangular-emitter sampling. It exercises the estimator `tracePath` uses: NEE via
`importanceSampleDirection` + `evaluateBsdf`/`pdfBsdf`, the BSDF-sampled environment hit via
`sampleBsdf` + `EnvironmentMap::pdf`, combined by the power heuristic.

**No scene resources.** No geometry, BVH or GL context is needed — `integrator_validate.cpp` covers
the full `renderPathTraced` path including `Material`/`MeshInstance` construction, and both are
plain data. The quad-light checks need no BVH either: `directionHitsQuad` is a closed-form
ray/rectangle test.

**The reference.** A flat unoccluded surface under a uniform-radiance (`L0 = 1`) environment has
exactly `Lo(wo) = integral over the hemisphere of evaluateBsdf(wo,wi) * cos(wi) dwi`, computed here
by independent uniform-hemisphere Monte Carlo — the same convention as `furnace_test.cpp` and
`bsdf_validate.cpp`. The MIS-combined estimator mirrors `tracePath`'s single-bounce case (no
occlusion, so NEE's shadow ray and the BSDF-sampled continuation both always reach the environment)
and must converge to it; a mismatch means double- or under-counting in the MIS weighting.

**Opaque only** (`transmissionFactor = 0`). The delta transmission lobe has no continuous pdf, so it
is excluded from `evaluateBsdf`/NEE and always takes MIS weight 1.0 on the BSDF-sampled side
(`path_tracer.cpp`). There is no double-counting question there — only in the two continuous lobes.

**Why the agreement band is a difference of estimators.** BOTH sides are noisy, which the old "5% of
`max(reference, 0.1)`" band got wrong twice over: it modelled the reference as exact, and its 0.1
floor stood in for the fact that a relative band is undefined as the denominator approaches zero.
It is now the difference of two independent estimators, whose variances add (Welch 1947). The
reference is drawn with `std::mt19937` and is genuinely iid, so its replicates are iid trivially;
the MIS side is `Sampler`-driven and is replicated over independent scramble seeds, which is what
makes ITS replicate means iid (see `stats.h` `kReplicates`). The total sample budget is unchanged —
the same count, redistributed across replicates.

**The Float16 CDF discriminator.** The CDFs must be built from the values the map returns, not the
source they were rounded from, or the sampling density stops being proportional to the radiance it
weights. The check uses a background of 1.0 and one texel at `1 + 2^-11`, the exact binary16
midpoint above 1.0: round-to-nearest-even stores it as 1.0, so at `Float16` the pdf ratio must sit
nearer the stored luminance ratio (1) than the source ratio (`1 + 2^-11`), and at `Float32` the
reverse. Discriminating by nearest hypothesis needs no tolerance.

## Headless beauty render

`tools/render_beauty.cpp`

Headless beauty render for before/after comparison across a code change. Loads a scene exactly as
`main.cpp` does, accumulates N path-traced passes, and writes one path-traced AOV (`--aov`, Beauty
by default) as an 8-bit PNG through the same display encoding the viewer shows it under.

It exists because the renderer is a GLFW application: comparing two revisions otherwise means two
manual screenshots, which cannot be pixel-differenced and cannot be trusted to share a camera.
Everything here is deterministic — fixed camera from `profile.json`, a fixed scramble seed for the
whole render with the sample index advancing per pass, no interaction — so two runs over unchanged
code produce a byte-identical file, which is what makes a non-zero diff meaningful. Same
standalone-CLI convention as the validate tools: no test framework, non-zero exit on failure.

**Reporting flags.** `--compare` takes a previously written PNG and reports max/RMS channel
deviation against the render just produced, so "did this change the picture, and where" is answered
numerically rather than by eye. `--out-exr`/`--compare-exr` are its linear-light companions, for
measuring convergence rather than inspecting an image: the PNG path is display-transformed and
8-bit, so it compresses highlights and clamps everything above display range, and a sampling
change's effect on exactly those bright high-variance regions reads as zero. `--bench-log` appends
the timing run to the benchmark log (`bench_log.h`) for `bench_compare`.

**The scramble seed is exposed** because it is one realization of the randomization, not a property
of the sampler: two seeds give two independent error images with the same expected RMSE. A claim
that two samplers converge equally well needs the spread across seeds to say what "equally" means,
and a reference sharing a seed with the render measured against it also shares that render's exact
samples, cancelling part of the error being measured.

**Determinism gate** (`--assert-deterministic`). Exact, and the only gate here needing no statistics
at all: the same seed must produce the same floats, because every input to the render is fixed. A
failure means the renderer's output depends on something that is not its inputs — thread scheduling,
an uninitialised read, or sampler state surviving a pass — and until that is ruled out no other
image comparison in this suite means anything. Nothing verified the header's own determinism claim
until this flag existed.

**Convergence gate** (`--assert-converged`). Two independent randomizations of an unbiased estimator
must agree within their own measured error, so this needs no reference image and no tuned threshold.
Per-pixel error is estimated by SPLITTING each render into R independent sub-renders and taking the
variance across them — the buffer-variance estimator standard in the sampling/denoising literature
(Zwicker et al. 2015 STAR; Rousselle et al. 2011). It is the only honest construction here: the
renderer's samples are one Owen-scrambled Sobol set, so no closed-form `sqrt(N)` error applies, and
splitting by PASS PARITY would not fix it either — consecutive passes are stratified against each
other, so parity halves are correlated and their spread understates the true error. Independent
SCRAMBLE SEEDS are what make the sub-renders genuinely independent, and the two families draw from
disjoint seed ranges so no sub-render is shared.

`R = 8` rather than 2 for a reason that is easy to miss: two buffers give the variance ONE degree of
freedom, and a chi-square with 1 dof is so heavy-tailed that the standardised statistic is
Cauchy-like and no normal quantile applies to it. Eight gives seven, and a Student-t that means what
it says. Welch degrees of freedom are bounded below by `R-1` for equal-sized samples, so using that
is conservative — it can only widen the threshold, never narrow it into a false failure. The
threshold is Sidak-corrected over the pixels actually examined, so it follows the resolution instead
of being restated for it.

**Error spectrum** (`--error-spectrum`). Reports how `--compare-exr`'s error distributes over
spatial frequency rather than only how large it is, as each octave band's share of total power. A
sampling change that rearranges error without reducing it is invisible to RMSE by construction, so
RMSE alone can neither confirm nor refute one — and rearrangement is exactly what a blue-noise
sampler claims to do. Normalised by the total so two renders with different error magnitudes are
still comparable as distributions. Bands are far from equal in width, so a raw share says nothing on
its own; the share white noise would put in each band is printed alongside, so a band reads directly
as blue (below 1.0) or red (above). Luminance rather than per-channel throughout: a scalar field is
what has a spectrum, and error visibility is a luminance effect.

**Two error metrics.** Absolute RMSE is dominated by the brightest pixels, so it tracks the
highlights a sampling change moves most; relative MSE (Rousselle et al. 2011, the standard metric in
the denoising/sampling literature) divides by the reference's own intensity, so a dim corner's noise
counts as much as a bright one's. The epsilon is the conventional guard against dividing by a black
pixel, not a tuned parameter. A signed mean is reported beside RMS to give the direction of an
energy change, not just its magnitude: a near-zero mean against a non-zero RMS means light moved
rather than appeared or vanished.

**Timing.** Per-pass wall clock, so a change's traversal cost is measured rather than argued. Only
the trace is timed; the accumulation inside `HeadlessRenderer` is O(pixels) and identical across
revisions. The **mean** is the figure to compare — unlike `raster_bench`'s single-threaded frames, a
pass's minimum is set by how the tile queue happened to drain and varies ~12% run to run, where the
mean holds to ~1%. Best and worst are reported alongside so a run disturbed by other load is visible
rather than silently folded in. Pass 0 carries the pool spin-up and first-touch faults and is
counted like any other: discarding it would change the image, and it biases both sides of an A/B
equally. A rasterizer-backed AOV traces no rays and runs no passes, so no per-pass distribution is
reported for it.

**Display encoding.** `encodeForDisplay` takes the scene-referred image to display-referred 8-bit
matching the viewer's pipeline exactly: exposure multiply, the display curve, then dither and
quantize. Its `applyDisplayTransform` flag mirrors `presentFrame`'s `isBeauty ? userLut : Raw` —
only Beauty is scene-referred radiance, and putting a data AOV like AO or Shadow through a display
curve would distort values that are already display-ready. `Raw` is the OCIO-free branch, exactly
what `buildRawFragmentSource` does. The dither is reproduced rather than skipped so the output
matches what the viewer shows; being a pure function of `uv` it is identical across runs and cancels
in a before/after difference. Depth is auto-ranged to the buffer's own maximum, as the viewer does:
its raw metres exceed the 8-bit `[0,1]` range and would quantize to solid white, and
`Camera::farClip` is a conservative ray `tMax` bound rather than a proxy for the scene's real depth
extent, so normalizing by it reads as near-black. This overrides `--exposure` for Depth, again as
the viewer does — the AOV has no photographic exposure.

**Accumulation is factored out** so the gates can render the same scene several times: the scene,
BVH, lights, camera and thread pool are built once by `HeadlessRenderer` and shared, so a gate costs
renders and nothing else. Each pass advances the sampler's sequence index rather than re-randomizing
it, so the accumulated samples stratify against each other exactly as they do in the viewer.

## Albedo table bake

`tools/albedo_table.cpp` → `src/scene/albedo_table.inc`

Offline generator for the Kulla-Conty energy tables `bsdf.cpp` bakes in ("Revisiting Physically
Based Shading at Imageworks", SIGGRAPH 2017 Course Notes). Same standalone-CLI convention as the
other tools, and grouped with `bluenoise_mask`/`gltf_tangent` rather than the validate tools: it
produces a committed artifact, it does not check one, so it stays out of the ctest loop.

This used to run at every process start (`const AlbedoTable kAlbedo = buildAlbedoTable()`), which
sized the grid and the quadrature by startup latency rather than by the accuracy the energy tests
need. Offline that bound is gone, so the reflect side is resolved to the point where E is an
instrument rather than a floor — `bsdf_validate`'s energy tests pin `F_avg` against a 2.0e-4 signal
and the old table's own error was ~1.5e-3, where this reports **1.1e-6 on the cosine-weighted means
and 3.0e-5 on the directional grid**. Determinism is why this target is built with neither
`-march=native` nor IPO, unlike every other tool here: the output is committed source, so it must
reproduce bit-for-bit on any machine, and FMA contraction is free to differ. The bake runs in
seconds, so there is nothing to buy back.

### Reflect-side resolutions

Three resolutions rather than one square grid, each sized by what `checkAlbedoTableInterpolation`
measures on that axis (`tools/bsdf_validate.cpp`). The bilinear read of the stored grid is a second
error source beside the quadrature's, and it is the one that dominates: at 128x128 uniform it was
4.2e-2 in the first mu cell against a 3.0e-5 quadrature residual.

**Roughness 256.** That axis' error is a smooth single-signed hump, not a boundary layer, so only
resolution touches it. At 128 it measured 1.1e-3, and it enters every shade as a bias rather than as
noise a render averages out. Quartering it puts the axis under the quadrature's own residual, which
is the stopping criterion — past that the stored values are the limit and further rows buy nothing.

**mu 256, uniform in sqrt(mu)** (`reflectMu`). The warp is what the grazing layer needs: a uniform
mu axis puts that whole layer inside one cell for roughness below ~0.1, where the error saturates at
the layer's full amplitude (measured 4.2e-2), and the layer's width is ~alpha in mu and so
~sqrt(alpha) in the warped axis — which is what lets one fixed warp serve every roughness row rather
than an alpha-dependent one. Node 0 is mu = 0 itself, not a nudge off it: `E(0, alpha) = 1` exactly
for every alpha and `smithG2OverCosO` holds to that limit, so the column that used to be the table's
worst is now its sharpest, and `buildReflect` asserts the identity on every row.

**The doubling is what the warp COSTS**, measured rather than assumed. A warp moves cells, it does
not add them: cell width in mu becomes `2 sqrt(mu)/(res-1)`, so at mu = 0.4 the cells are 1.27x a
uniform axis' and the error there rose with them. At 128 that turned `checkCoatFresnelAvg`'s worst
row from 5.1e-5 into 8.7e-5 — a regression on the very instrument this table is read by, whose worst
rows all sit at mu 0.4. 256 buys the working band back at 0.63x the original uniform width and
quarters the grazing layer at the same time.

**An alpha-dependent warp was measured and rejected**, not assumed away: Smith G1 as the axis
coordinate resolves each row's own layer exactly, but at alpha 0.25 it compresses mu in [0.3, 1]
into 12% of the axis, for 5.3e-4 against a uniform axis' 1.2e-5 in exactly the band that matters —
two extra `sqrt` in a hot lookup to buy a 40x regression where E is not flat.

### Transmit-side resolutions

Sized by the energy closure it buys: linear interpolation in mu (the steep grazing rise) and in eta
(curvature through the TIR onset) sets the per-vertex error once the quadrature is converged.
Measured on a white ior-1.5 interface: 32 mu nodes lose 2% at mu 0.02, and 32 eta nodes lose 9e-4
midway between eta nodes against 1e-4 on them; 64 x 64 closes to within 3e-4. Roughness keeps 32
nodes, where node and midpoint already agree to that level. Uniform in mu, 64 nodes still left 1e-2
at mu 0.01 at roughness 0.15, where E climbs over mu ~ alpha, so the escape tables' mu axis is
uniform in sqrt(mu) (`escapeMu`) — E climbs from its grazing limit over mu ~ alpha (G1 ~ 2mu/alpha
below it), which a uniform mu grid spans with under two cells at roughness 0.15. `bsdf.cpp`'s
`escapeAlbedo` indexes by sqrt(mu) to match, and node 0 is mu = 0 itself.

### Smith G2 over cosO

Height-correlated G2 divided by cosO, `2 cosI/(cosI s(cosO) + cosO s(cosI))` (`bsdf.cpp`'s
`smithVisibility` times `4 cosI`): no cosine divides, so it holds to cosO = 0, where it is `2/alpha`.
Both sides use it, and the reflect side now MUST. The lambda form it replaced there carries a
`max(cos^2, 1e-8)` clamp, harmless while the reflect grid started at mu = 1e-3 (cos^2 = 1e-6, clear
of it) and not once `reflectMu`'s node 1 is mu = 6.2e-5: the clamp would silently substitute a
different cosine on the grazing rows with no diagnostic at all. It also supplies the exact grazing
limit the warp's node 0 needs, which a form that divides by cosO cannot.

### Reflect side: exact-domain Gauss-Legendre, not Monte Carlo

The quantity is the directional albedo of the single-scattering GGX lobe with Fresnel forced to 1,
the fraction of energy G2 lets through, so `1-E` is exactly what the multiple-scattering lobe must
return. It depends on nothing but `(mu, alpha)`: Fresnel, metallic, baseColor and lobe-selection
probabilities are all applied by the caller.

Sampling it (VNDF draws, discarding `wi.z <= 0`) puts a jump discontinuity — the horizon — inside
the integration domain, which caps any quadrature at first order in the sample count no matter how
smooth the rest of the integrand is. That, not the sample budget, is what held the old table at
~1.5e-3. The fix is to integrate over a domain whose boundary IS the horizon, and both halves of
that are available in closed form:

1. **Measure.** GGX NDF sampling (Walter et al. 2007, "Microfacet Models for Refraction through
   Rough Surfaces", eq. 35) is `tan(theta_h) = alpha*tan(psi)` with `u = sin^2(psi)` uniform, and
   its density is exactly `D(h)*cos(h)`, so `D(h) cos(theta_h) dw_h = du dphi / (2*pi) = sin(psi)
   cos(psi) dpsi dphi / pi`. Changing variable to psi absorbs the peak D would otherwise have — at
   alpha = 4e-4 that peak is 4e-4 radians wide and no fixed grid in theta_h could resolve it — and
   leaves an integrand that is analytic in psi.
2. **Domain.** With `wo = (sin tv, 0, cos tv)` and h at `(theta_h, phi)`, both cosines collapse to a
   single harmonic: `wo.h = R cos(theta_h - d)` and `wi.z = R cos(2 theta_h - d)`, where
   `R = hypot(sin tv cos phi, cos tv)` and `d = atan2(sin tv cos phi, cos tv)`. So the horizon clip
   `wi.z > 0` is exactly `theta_h < (d + pi/2)/2`, one bound per phi, and `wo.h > 0` holds
   throughout it. Nothing is discarded, because nothing invalid is ever evaluated.

The integrand that remains — `(wo.h/cos theta_h) * G2 * sin psi cos psi` — is analytic on the closed
interval (G2 vanishes linearly at the upper limit, where `wi.z` does), so Gauss-Legendre converges
geometrically and the node count is a measured choice, reported against a doubled rule on every run:
at 96 the mean channels agree to 1.1e-6 and the directional ones to 3.0e-5, that worst case confined
to the mu = 0 column every consumer weights by cos.

Split by Schlick's form `F(c) = f0*(1 - (1-c)^5) + (1-c)^5` so one table serves any f0 (the standard
environment-BRDF split): `Ess(mu, f0) = f0*a + b`, and with `f0 = 1` that collapses to `a + b = E`,
the Fresnel-free albedo above.

**phi panelling.** phi is even about 0, so half the circle is integrated and the result doubled. Two
panels meet at pi/2, where `cos(phi)` changes sign: `d(phi)` sweeps the full `-pi/2..pi/2` of its
range within `|cos phi| < mu` there, a boundary layer that narrows with mu and would otherwise be
missed entirely by the grazing rows. As a panel endpoint it is resolved instead, Gauss-Legendre
placing its outermost node `O(1/n^2)` from the edge.

**mu = 0 is never 0/0**, by two separate arguments. On the first phi panel `delta` is `+pi/2`, so
`wiZ` vanishes only at `psiMax`, which Gauss-Legendre's strictly interior nodes never reach. On the
second `delta` is `-pi/2` and `psiMax` is exactly 0, so every node sits at `psi = 0` and the weight
carries `psiMax` and `sin(psi)` as exact zero factors — the panel contributes nothing, which is
correct: no facet reflects a grazing `wo` above the horizon on that side.

Gauss-Legendre nodes and weights are mapped to `[0,1]` by Newton iteration on `P_n` through Bonnet's
recurrence (Press et al., *Numerical Recipes* 3rd ed., sec. 4.6.1). Weights sum to 1, so a node
array doubles as the `[0,1]` average.

### Transmit side: the same measure, panelled at the interface's boundaries

Its energy curve is not the reflect side's: the below-horizon reflections that drive E down are the
valid side for refraction (measured 0.559 combined vs 0.307 reflect-only at roughness 1.0). It
depends on eta (G2 uses the refracted `|wt.z|`, and TIR gates validity), so the Schlick split cannot
factor it out; one axis in `log(eta)` covers entering and exiting, since the two are reciprocals.

Same measure as `reflectAlbedo`, `D(h)cos(theta_h) dw = sin(psi)cos(psi) dpsi dphi / pi` with
`tan(theta_h) = alpha*tan(psi)`, which flattens the peak and resolves the GGX slope tail; a
stratified VNDF midpoint rule lumped that tail into its last stratum and converged first order
(3.6e-3 at roughness 0.19, mu 1, eta 1.56). Same closed forms too, so per phi the visibility bound,
the reflection horizon and the TIR onset `wo.h = sqrt(1 - 1/eta^2)` are all exact `theta_h`
breakpoints, the TIR circle's tangency `R = sqrt(1 - 1/eta^2)` is an exact phi breakpoint, and each
panel between them is integrated on its own. That matters most at TIR, where `1-F` has a square-root
singularity no fixed rule resolves (3.9e-3 residual at a doubled rule without the split); the one
boundary left unaligned, `wt.z = 0`, is a kink where G2 vanishes linearly.

The VNDF weight `G1*(wo.h)*D/mu` divided by G1 per escaping path leaves `(wo.h)/(mu*cos(theta_h)) *
G2` in this measure. Fresnel and the TIR predicate are the shipped `fresnel_dielectric.h`, so the
table is baked against the interface it is shaded against — with exact dielectric Fresnel, since
inside the TIR cone Fresnel is 1.0 where Schlick reads ~0.1 and no Schlick-basis rescale can stand
in for it. A facet reflects with probability F and refracts with `1-F`, so the two shares are
Fresnel-weighted complements of one throughput, never independent quantities.

### Multiple-scattering sampling shapes

**Reflected lobe: the exact `(1-E)cos` sampler.** That lobe's value is
`fms*(1-E(mu_o))*(1-E(mu_i))/(pi*(1-Eavg))`, so its own zero-variance density is
`(1-E(mu_i))*cos / (pi*(1-Eavg))`, and cosine sampling pays the ratio `(1-E(mu_i))/(1-Eavg)` as
weight variance. Measured off the table rather than assumed, and the answer inverts the intuition:
the ratio is harmless at high roughness (relative variance 0.029 at roughness 1, 0.042 at 0.5) and
severe at low, where `1-Eavg` and `1-E(mu)` are both small and their quotient is not — **+1.66 at
roughness 0.25 and +17.3 at 0.126, with weights reaching 92x**. `1-E` is strictly positive at every
roughness the table reaches (measured minimum 1.4e-7, at roughness 0), so that is a bake-time
assertion and not a shading-time guard.

Stored as a piecewise-linear density over mu with its exact prefix integrals beside it, NOT as the
mu-at-quantile inverse CDF the roadmap names. A sampler and a pdf must agree on the density itself;
a quantile table defines it only implicitly, leaving the pdf to reconstruct it by differencing and
biasing the estimator by whatever the two then disagree about. With the density stored, `bsdf.cpp`
inverts it exactly (one quadratic per segment) and evaluates the same interpolant for its pdf, so
the pair is consistent by construction at any resolution and the remaining approximation is only how
closely the interpolant tracks `(1-E)*mu` — variance, never bias. Uniform in mu, unlike the albedo
table's sqrt(mu) axis, so `bsdf.cpp`'s piecewise-linear inversion keeps one step width; each node
reads E through that warp exactly as the shading does, so the shape is still built from the values
the shading actually sees. This is also why the sampling grid is its own constant rather than
`kAlbedoMuRes`: a later change to the albedo warp's resolution must not silently resize a sampling
density.

**Transmissive lobes: the escape-deficit shape**, the far-hemisphere twin, one axis wider because
the escape it is built from is eta-dependent. `bsdf.cpp` reads it as both value and density: each
transmissive share is its energy times this normalised `(1-Escape(mu_i))*cos` density divided by
cos, so the density is the zero-variance one and the share integrates to its energy exactly —
cosine sampling paid relative variance 25 at roughness 0.13. Unlike the reflect shape this is stored
UNNORMALISED: `bsdf.cpp` blends four rows over (roughness, eta) and divides by the blended total,
which reproduces the raw-deficit interpolation `escapeAlbedo` itself performs, where a blend of
per-row-normalised shapes would not commute with it. Unnormalised storage is also what removes the
degenerate row — a row whose deficit is numerically zero carries near-zero weight into the blend
rather than a unit-mass shape of amplified noise — so this needs neither the reflect side's
bake-time abort nor a substituted fallback. `bsdf.cpp` derives each share's value from this same
density, so value and pdf share one support by construction.

### Output

The cosine-weighted mean is a Gauss-Legendre integral over mu in its own right, not a trapezoid over
the stored columns: `Eavg` is the denominator of the multiple-scattering normalisation, so its error
enters every compensated shade directly rather than being smoothed by interpolation. Rows are
independent and each writes only its own slice of the table, so threading one roughness row per
worker is a pure speedup with no effect on the values, and the result is identical to the serial
order — the determinism the committed artifact needs survives the threading.

`%.9g` is `FLT_DECIMAL_DIG` digits, which round-trips float32 exactly, so the committed values are
the computed ones. It drops the point on a whole number though ("1"), and "1F" is not a literal, so
one is restored where needed.

## Integrator validation

`tools/integrator_validate.cpp`

Standalone correctness check for `path_tracer.cpp`'s integrator (`renderPathTraced`/`tracePath`),
distinct from `bsdf_validate` and `nee_validate`, which test the BSDF and the light sampling
underneath it. `Material` and `MeshInstance` are plain data — six `ImageTexture` members and a mat4
— so no GL context is needed.

**Reference configuration:** one large unoccluded quad under a uniform-radiance (`L0 = 1`)
environment. Nothing else is in the scene, so every ray leaving the surface reaches the environment
and the answer is analytic. Two invariants follow, catching different integrator bugs than a
BSDF-level furnace test can:

1. **Depth invariance.** With no indirect light, `maxBounces=0` and `maxBounces=1` must produce the
   same image. A depth cap dropping the terminal BSDF-sampled contribution shows up here and nowhere
   else. It is exact up to Monte Carlo noise, so it gets the tighter bound.
2. **Absolute agreement** with the analytic reference, which no self-consistency check between two
   renderer settings can give on its own. Looser, the uniform-hemisphere reference being the noisier
   of the two estimators.

Russian roulette is exercised as a third case: it reweights by `1/p` on survival, so an RR-enabled
render must return the same answer as an RR-disabled one, within noise.

**Scene scaffolding.** The quad half-extent is large enough that every primary ray in the narrow
test FOV lands on it, so no pixel sees the environment directly. The FOV is narrow (200mm on a 36x24
gate, ~6.9 degrees vertical) so every pixel's view direction is within a fraction of a degree of the
quad normal, letting one analytic value stand for the whole block. The sphere scene exists for its
CURVATURE: every other scene here is flat quads, whose coplanar vertex normals make
`transmissionOffsetEpsilon`'s curvature factor exactly zero, so they cannot exercise it at all — and
the tessellation sets that offset epsilon (max edge length times the sine of the vertex-normal
divergence, both across the diagonal), so changing the resolution changes what is being tested. One
`ThreadPool` serves the whole binary, sized from the runner's `--threads` so concurrent ctest jobs
do not oversubscribe the machine; eleven checks each constructing their own spun up and tore down
`hardware_concurrency()` workers eleven times per run for no isolation gained. Each check keeps its
own `ok` accumulator and per-row stderr diagnostics — those carry the scene, the row and the
measured value, which is what makes a failure diagnosable — and reports one assertion.

### Transmission

A white, non-absorbing dielectric slab in a uniform `L0=1` environment is invisible: every photon
entering the front face leaves somewhere, so the block reads exactly the environment behind it. It
is the only case in the suite reaching a transmissive exiting vertex, gating the far-side NEE guard
against the miss branch's MIS weight. Held to the BSDF-only walk (`fixtures::slabWalkLo`) rather
than to 1.0, so it measures what the integrator adds and nothing else. Both sides are replicated
over independent scramble seeds and compared as the difference of two estimators (Welch), the whole
image averaged, so the band is the run's own interval rather than a hand-set tolerance — the
centre-4x4, single-seed reading this replaced carried ~3% standard error against a hand-set 0.03
tolerance, so what read as a real excess was within its own noise. The sphere is the curved
counterpart on the same invariant: it traps far more light than a slab, since past the critical
angle every internal hit totally internally reflects, so paths ring around the inside and the bounce
budget matters. Roughness 0.02 is below `bsdf.cpp`'s smooth threshold and takes the delta
transmission path; the rest take the Walter lobe, where far-side NEE is live.

**Dispersive sphere.** The same invariant with the dielectric dispersive: a white non-absorbing
sphere is invisible whatever its index, and a wavelength-varying index is still, at each wavelength,
an index. That makes it the gate on the one-sample channel estimator (`path_tracer.cpp`'s
hero-channel block). Each channel is an independent estimate of the same 1.0, so a missing or
mis-scaled `1/p` reads ~1/3 in every channel, and a hero channel re-drawn instead of sticky loses
another factor on each further interface crossing — neither of which any analytic BSDF test can see,
since the estimator lives in the integrator and not in the BSDF. Read per channel rather than
through `renderCentre`'s max, which would hide a per-channel loss behind whichever channel read
highest. Real catalogue pairs, and N-BK7 is the material `assets/materials/glass.json` ships: the
suite asserts the invariant for the glass the renderer actually renders.

**Beer-Lambert absorption** asserts the documented contract rather than restating its formula:
`transmissionDepth` is the distance at which transmittance reaches `transmissionColor`, so that
identity is the specification. `ior 1.0` is what makes it exact rather than approximate, and it is a
physically real configuration — an index-matched pure absorber — not a test-only contrivance: the
ray travels a known straight chord. Per-channel colour, distinct in every channel, so a swapped or
luminance-collapsed `sigmaA` passes a grey test and fails this one. The band is relative, since the
squared row's green channel is 0.0625 and an absolute band would be vacuous there; the flat rows are
exact to 3e-4 relative, and the sphere row reads systematically high from two named biases that both
shorten its path — the tessellated chord is shorter than the true one, and the offset epsilon trims
more. The dispersive row crosses the hero channel with the already-per-channel `sigmaA` and isolates
the channel estimator from the refraction geometry: at `ior 1.0` the Cauchy B coefficient is exactly
0 (`bsdf.cpp`'s `cauchyIor`), so `n(lambda) == 1` at every wavelength and the interface stays exactly
index-matched — path, traversal distance and expected reading all unchanged — while hero-channel
selection still fires, since it keys on `abbe` and `transmissionFactor` alone. What it asserts is
that the two mechanisms do not interact: the surviving channel is attenuated by its OWN `sigma_a`
and the two masked-out channels stay exactly zero.

**On-surface tint** is the other half of the convention: `transmissionDepth == 0` means there is no
interior medium at all, and `transmissionColor` is then the tint applied once per crossing.
Complementary to Beer-Lambert rather than a restatement — that one authors a colour at depth > 0 and
requires exactly `transmissionColor` at that distance; this one requires distance not to matter at
all. A flat slab and a sphere, whose traversal distances differ by construction. Tight and relative:
with no absorption and no refraction there is no distance-dependent bias, so both rows carry only the
estimator's own noise.

**Rough transmission tint** covers the rough lobe's half of that convention, which the check above
cannot reach: it authors roughness 0.02 on both scenes (alpha 4e-4, below `kSmoothAlpha`), so every
reading there comes from `sampleBsdf`'s delta branch. Measured, not assumed: tinting only that branch
and reverting `evaluateTransmissionLobe` and `transmitMultiScatter` fails `bsdf_validate` 54 times
and leaves this binary entirely green. It runs at `ior 1.5` — `makeSettings`' own — and not the 1.0
the delta rows force, because at `ior 1` the transmission lobe IS the delta branch at every roughness
(`transmissionIsRough`; PBRT-v4's `eta == 1 || EffectivelySmooth()`), so an index-matched rough row
would re-measure the path already covered.

That costs the closed form — `colour^2` holds only where every path crosses exactly two interfaces —
so the scene supplies the invariant instead of an oracle. ONE interface, not a slab: a rough slab at
`ior 1.5` internally reflects, giving paths 2, 4, 6 … crossings and a polynomial in the tint rather
than a line. Black environment, one one-sided light behind that interface, so a reflected path sees
nothing at all: there is no untinted term anywhere in the reading and every photon in it crossed
exactly once. `Lo` is therefore exactly linear in `transmissionColor` through the origin, and Exact
rather than Statistical: `computeLobeProbabilities` never reads `transmissionTint` and Russian
roulette is off, so the three renders of a row draw the identical sampler sequence and select the
identical lobe at every vertex. Each path's contribution carries the tint exactly once, so the
relation holds path by path, noise and all, and the only residual is float multiplication not
distributing over the accumulation sum — **measured worst 1.37e-06 relative over both rows, and
identical at 1, 2, 4 and 8 threads**, so it is non-distributivity and not a reduction-order race.
7x headroom on that, against faults that are percent-scale. The zero row is what makes the linearity
worth asserting: linearity alone cannot distinguish transmitted energy that is never tinted from
reflected energy, since both are constant in the tint, and here both are zero. The two roughness
rows sit at the ends of the far-hemisphere split: at `ior 1.5` and mu 0.8, `msTransmit` carries
2.6e-5 of the selection mass at roughness 0.15, where single-scatter refraction dominates, and 0.0263
at 0.6, where the multiple-scattering lobe is drawn in earnest.

**Sphere tessellation** sets the offset epsilon (max edge length times the sine of the vertex-normal
divergence, both across the quad's diagonal), which backs each ray origin that much into the medium
and so shortens every measured in-medium segment: at 64x32 on a unit sphere that is **1.93e-2**.
Coarse enough that the curvature terms are firmly non-zero, fine enough that the analytic chord stays
accurate; the detection strength this costs was measured, not assumed, by re-running the checks at
24x12 (**528 triangles**, cornell's own tessellation), where they behave the same.

The sphere row's two biases are quantified. The offset epsilon backs the origin 1.93e-2 into the
medium, and the probed block's outermost ray has an impact parameter of 0.075 rather than 0, a chord
of **1.9944** rather than 2.0. Measured **0.89/1.82/0.35%** across the three channels, which divided
by each channel's own `sigma_a` give the same **0.025 path deficit** — one shortened path, matching
the predicted **0.019 + 0.006**, and not a per-channel error. The worst row is therefore 1.8%
against the band.

### Per-instance materials

Nothing asserted that `ShadingTriangle::instanceIndex` resolves into the right `perInstanceSettings`
entry. Two coplanar quads, one instance each, identical `Material`s, distinguished only by a
per-instance diffuse colour — red left of x=0, blue right. Probed columns stay 2px clear of the seam,
wider than the 1.5px reconstruction filter, so neither block contains a pixel any sample from the
other side could splat into. Swapping the vector must swap the picture, so each block is compared
against the other assignment's opposite block.

The resolution side (`resolvePerInstanceSettings`, mapping each `materialOverrides` key — a glTF
node name — onto the instance carrying it) was uncovered: all five binaries exit 0 on a build where
it is broken. It is compared against `loadMaterialConfig`'s own reading of the same file rather than
against literals copied out of it, so editing the asset cannot silently invalidate the check. The
base is sentinel values deliberately unlike `glass.json` in every field: nothing is rendered, so they
need only be distinguishable. Two are load-bearing rather than cosmetic — `abbe`, because at the
commit that introduced it `glass.json` did not yet declare one and it loaded as the 0.0 default,
which is also `makeSettings`' value; and `edgeTint`, which `glass.json` omits so it loads as the
`[1,1,1]` default, also `PathTraceSettings`' own. Without sentinels the two agree and the non-vacuity
gate fires. That gate is the assumption every assertion rests on: the base must differ from the
override in all 12 fields, so each field's copy is independently observable.

### Transport AOV partition

The five transport buckets are a partition of beauty, not a set of related-looking images: with the
background term zeroed (`showSky` off), `DirectDiffuse + IndirectDiffuse + DirectSpecular +
IndirectSpecular + Refraction` must equal Beauty at every pixel, to float error. Every radiance
contribution `tracePath` adds is written to exactly one bucket at its own physical value, so any gap
means a contribution was bucketed twice, dropped, or rescaled — exactly what the previous delighted
buckets did by construction, stripping `baseColor` at bounce 0 only, leaving direct and indirect in
different units and neither summing to anything. The slab rows carry the load: a single quad reaches
only the Direct buckets, while the slab's internal reflections populate Indirect and Refraction and
exercise the transmissive exiting vertex. The corner scene is required too — a flat quad's
continuation ray always escapes at bounce 1 (Direct) and the slab's every multi-vertex path is
transmission-sticky (Refraction), so without it the two Indirect buckets are identically zero in
every case and the identity guards nothing about them. The dispersive row exists because the hero
channel is the one mechanism writing a DIFFERENT value into different channels of the same
throughput — it zeroes two of them. The partition is a per-channel identity, so a mask applied to
beauty but not to the bucket accumulators breaks it in exactly the two zeroed channels, invisible to
every furnace in the suite: a furnace measures the total that arrives, and this asks where it was
filed.

### Quad lights

The receiver is an exactly Lambertian surface: `ior=1.0` makes `fresnelDielectric` identically zero
(the same device the transmission checks use for an unbent, unreflected ray), `metallicFactor=0` and
`transmissionFactor=0` keep only the diffuse lobe live, and `diffuseRoughness=0` is EON's own
documented Lambertian limit. What survives is `f(wo,wi) = baseColor/pi` exactly, constant in
direction — the one BRDF shape Lambert's polygon irradiance formula can be compared against in
closed form, since it integrates `Le*cos` alone and has no way to fold in an angle-dependent BSDF
term.

**The reference** is Lambert's formula (1760) / Baum, Rushmeier & Winget 1989's closed-form polygon
form factor: the irradiance a uniform-radiance `L` planar convex polygon light produces at a
Lambertian receiver point `p` with normal `n`. It is genuinely independent of `light.cpp`'s
spherical-rectangle solid-angle formula (Girard's theorem on the polygon's INTERNAL vertex angles,
for importance-sampling density) — this instead sums, over the polygon's EDGES, the great-circle
angle each edge subtends as seen from `p`, weighted by how much that edge's rotation axis aligns
with the receiver normal: the PROJECTED solid angle directly, which is exactly what a Lambertian
receiver integrates (`E = L * projected solid angle`). `cross(r1, r0)`, not `(r0, r1)`: the sign
convention that makes the sum positive when `n` points toward the polygon depends on the traversal
direction, fixed empirically against the renderer (which independently gets the physically-correct
sign from `quadRadianceToward`'s own front-face test) and matching Baum/Rushmeier/Winget's vertex
ordering.

**Light placement.** The light is offset in x rather than centred above the receiver: the camera
sits at `(0,0,5)` looking straight down the `x=0,y=0` column, so a light straddling that column would
put its own non-emitting back face directly in the camera's primary ray, reading black for a reason
unrelated to anything being tested. Offsetting clears the camera's narrow view cone (radius ~0.2 at
this depth, ~0.23 for the grazing row's near edge at `x=0.5`) while staying close enough for a
strong, easily-measured irradiance.

Three checks share that geometry. **Irradiance:** the receiver's centre-pixel `Lo` must equal
`(baseColor/pi) * E` exactly, with no Monte Carlo anywhere in the reference; several light
positions/sizes, since a single on-axis case cannot distinguish a mis-scaled solid angle from a
mis-scaled radiance. **One-sided:** the SAME position, `edge0`/`edge1` swapped so the emitting face
points UP away from the receiver, which still has geometric line of sight (`nearSide` is true, NEE
still fires and evaluates the BSDF/occlusion exactly as the irradiance case does), so this isolates
`quadRadianceToward`'s front/back-face test from the near/far-side gating a light merely behind
opaque geometry would exercise instead. Must read exactly 0. **Occlusion:** the front-face-down
light with an opaque wall between it and the receiver, also exactly 0. Paired with the irradiance
check, this brackets `kShadowDistanceEpsilon` from both sides — too small and the light's own front
face self-occludes, absent and the blocker stops working. The wall is sized to the light's own
footprint as projected from the receiver (the light spans `x[0.5,1.5] y[-0.5,0.5]` at `z=1.5`; at
the wall's `z=1.0` that projects by similar triangles to `x[0.333,1.0] y[-0.333,0.333]`, and the
wall is oversized around that with margin) rather than `kQuadExtent`, which would also swallow the
camera's own sightline and read the wall's lit topside instead of testing occlusion.

**Curved receiver.** Those three share a FLAT receiver, where `shadowTerminatorOffset` is a no-op and
the shadow ray leaves from `shading.position + geoNormal*kRayEpsilon`, one part in `1e4` of the
distance to the light and so comfortably inside the `1e-3` relative back-off. On curved geometry the
Chiang/Li/Burley origin is pulled onto the vertex tangent planes, a displacement of order `e^2/(2R)`
for edge length `e` on radius `R`, which on ordinary tessellation is the SAME order as the back-off:
`cornell.json`'s spheres (`e = 0.026`, `R = 0.15`) put it at `~2e-3` against a back-off of `6e-4`.
The ray then crosses the light's plane BEYOND `tMax`, and the emitter's own front face — which is in
the BVH — registers as its own occluder. No choice of `kShadowDistanceEpsilon` fixes this, because
the offset does not scale with the distance to the light.

The construction is therefore pbrt's `SpawnRayTo` (PBR 6.8.6) rather than a back-off along `wi`: the
sampled point is reconstructed as `shading.position + wi*distance` and the ray is re-formed FROM the
offset origin TO that point, so the back-off is relative to the distance actually travelled whatever
the light's orientation. A back-off along `wi` alone is not enough — the offset origin's ray is a
parallel shift, and it meets the light's plane at `distance - dot(delta, n_light)/dot(wi, n_light)`,
not at `distance - dot(delta, wi)`. The environment keeps the direction as drawn: its point is at
infinity, and `FLT_MAX` is already effectively unbounded.

`quad_light_visibility_on_curved_receiver` locks it, as a hard zero rather than a tolerance: nothing
lies between a CONVEX receiver and a light it faces, so the Shadow AOV over a sphere lit by an
unobstructed quad must read exactly 0 at every tessellation. Two rows, `64x32` and `32x16`, because
the offset scales with `e^2` — both read 0.874 and 0.987 against the old construction.

**Inverse square.** Quadratic falloff is IMPLICIT in this renderer — NEE divides by
`pdf = selectionPdf/solidAngle`, so the light's subtended solid angle enters as a multiplier and
shrinks as `A*cos(theta_l)/d^2` with distance. There is deliberately no explicit `1/d^2` term
anywhere: that is needed only under uniform-AREA sampling, where the geometry term
`G = cos(theta_r)*cos(theta_l)/d^2` converts the measure. Nothing asserted the limit, so this does.
The statement tested is the law itself in the form that isolates it: for a small light,
`E * d^2 / (L * A * cos(theta_r) * cos(theta_l)) -> 1` as `d` grows, measured off the RENDERED
irradiance (`Lo * pi`, the receiver being the exact-Lambertian floor above) so it constrains the
renderer rather than the reference. The x offset is held FIXED across the sweep, so the true
probe-to-light distance is `sqrt(kOffsetX^2 + z^2)` rather than `z`, and is computed as such. The
far row's 1e-2 band is comfortably above the MC noise on the ratio and far below the near-field
deviation demanded, so the two cannot be confused. The near-field row is the conditioning guard, not
decoration: a light 30x wider at half the nearest distance subtends nearly the receiver's whole
hemisphere, where irradiance flattens instead of following `1/d^2`, and the renderer must follow the
exact Lambert polygon form there. Without it, a renderer that had hardcoded a `1/d^2` point light
would pass the far-field rows while being wrong everywhere a real area light differs from a point.

### Ambient occlusion

**Closed-form cosine-weighted obscurance for the corner scene.** A receiver on the infinite floor at
perpendicular distance `d` from the wall plane, occlusion range `D`, `c = d/D`. Malley's method makes
the sampled direction's tangential projection uniform on the unit disk (PBR 4th ed. 13.6.3), and a
ray reaches the wall plane at `t = d/a` for disk coordinate `a`, so it is in range exactly when
`a >= c` and carries `x = t/D = c/a` there. Integrating the deficit `1 - rho = (1 - c/a)^2` over the
disk's chord length `2*sqrt(1-a^2)` gives

    I(c) = int_c^1 (1-c/a)^2 * 2*sqrt(1-a^2) da,

elementary from `int 2*sqrt(1-a^2)`, `int sqrt(1-a^2)/a` and `int sqrt(1-a^2)/a^2`, and
`AO = 1 - I/pi`. Rotating the tangent frame maps a uniform disk onto itself, so this does not depend
on how `buildShadingFrame` orients it. `c -> 0` approaches the half-space limit 0.5, unchanged from
the binary form; `c >= 1` puts the whole wall beyond `D`. Evaluated in double because the three terms
are O(1) while their sum vanishes as `(32*sqrt(2)/105)*(1-c)^(7/2)` — that high-order tangency IS the
`rho'(1) = 0` condition, and it is severe cancellation to evaluate.

**Wall distance** is ten times `makeCornerScene`'s default, for two quantitative reasons. `AO(c)` is
nonlinear, so the measured block's spatial spread in `d` biases its mean by about
`AO''(c)*Var(d/D)/2`: the frame spans `|x| <= 0.3` at the floor, which at `d=10` holds that bias to
3.3e-5 at the worst row (`c=0.50`), 2% of its tolerance, where at `d=1` it would be 100x larger and
twice the tolerance there. And the `c>1` row needs `c` above 1 at every measured pixel, not just at
the centre: at `d=10` the nearest is `c=1.067`, at `d=1` it is below 1. Rays reach at most
`D = d/c <= 200` here, well inside the wall's `kQuadExtent` reach, so no ray escapes past its edges.

**Sample count and tolerance.** One `rho` draw per sample, so the standard error falls only as
`1/sqrt(N)`. The count puts the 6-sigma tolerance at least 8x below the gap to every curve the sweep
must exclude: 10-23x for a linear `rho`, 26-116x for the old hard cutoff, and 8x at the narrowest for
uniform-hemisphere sampling of the same `rho`, whose margin is what sets the count. The tolerance
itself is the standard error on the mean at ~6 sigma, the same device as `nee_validate`'s
quad-solid-angle tolerance: taken from the estimator's own statistics rather than picked by hand. The
per-sample `rho` is no longer a Bernoulli draw but any value in `[0,1]`, where Bhatia-Davis bounds
`Var <= (M-mu)(mu-m) = p(1-p)`, so the expression is a conservative bound on the standard error
rather than the exact one. `N` counts only the block's own samples; the 1.5px reconstruction filter
mixes a one-pixel halo in, which can only raise the contributing count. At `p = 1` it is exactly
zero, which is correct rather than degenerate — every sample is then deterministically unoccluded,
and the AO lane and the filter-weight lane accumulate the identical sequence of weights, so the
quotient at write-out is bit-exactly 1.0.

Every pixel is read, not `centreMean`'s 4x4 block: AO is a visibility query about the plane's
constant +Z normal and does not depend on the view direction, so the spread `centreMean` exists to
limit costs nothing here, and reading the whole frame gives the same `N` for a sixteenth of the
traced paths. `maxBounces=0`, because AO is written at bounce 0 before any BSDF work, and the
Lambertian material for the same reason — AO is a pure visibility query and reads no material at all.

**The sweep, not a point.** Three wrong integrators pass any single row: uniform-hemisphere sampling
of the same `rho`, a linear `rho`, and the hard cutoff this replaced. Rows are chosen so the analytic
value clears all three by the margins the sample count is sized for. The unoccluded plane is the row
an inverted polarity fails outright. Separately, `aoMaxDistance` is bracketed from both sides on one
fixed geometry, so the only thing changing between those two rows is the bound: at `c=0.50` the wall
is inside range and darkens the plate by 11 tolerances; at `c=1.1` it is outside and every ray
escapes, so the lane must read exactly 1.0 — exact rather than approximate because even the
frame-edge sample nearest the wall, at the `x=0.3` where the film clips, still sits at `c=1.067`. A
build that ignores `aoMaxDistance` reads the unbounded half-space value 0.5 in both rows. The inside
row sits at `c=0.50` and not just under the bound as it did for the hard cutoff: `rho'(1) = 0` makes
the deficit vanish as `(1-c)^(7/2)`, so `c=0.90` now reads 0.99995, within 0.6 tolerances of
unoccluded and unresolvable at any practical N. That is the discontinuity being gone, not a lost
test.

## BSDF validation

`tools/bsdf_validate.cpp`

Upper-bound and identity checks on `scene::bsdf`. The measurements each tolerance and design choice
rests on are recorded here, since they do not fit the one-line comment budget.

**White furnace.** Single-scatter GGX loses the energy Smith G2 masks away (Heitz, Hanika, d'Eon &
Dachsbacher 2016): a white conductor at roughness 1.0 measured **0.307**, under a third of the light
received. Kulla-Conty multiple-scattering compensation plus the directional-albedo diffuse coupling
return it, which is what makes 1.0 a correctness target rather than a regression baseline — both
bounds share one tolerance and a shortfall is a bug.

**Index-matched coat.** The invariance bound is exactly zero, an algebraic guarantee from
`x*1.0F == x` rather than a measured run. The Karis revert breaks it by a measured **7.8e-4
relative**, worst at roughness 0.92 with `mu_o = mu_i = 1`, about **8000 ULP** at the diffuse value's
magnitude. The second bound is a float32-against-double residual on the same closed form, ~15
operations deep, **measured worst 2.03e-7** at the grazing tail — **4.9x under**, thin on purpose. It
is not the instrument but the backstop that stops a roughness-INDEPENDENT corruption (a pinned
`diffuseKd`, a lost `1/(1-coatAvg)` normalisation, a channel swap) from passing as bit-identical,
which the invariance assertion alone cannot see.

**Coat Fresnel average.** Truth is `2*int F(mu)*mu dmu` by the same `cosineAverageFresnel`
`average_fresnel` uses, **~1e-13 accurate**, so the tolerance is not set by the reference. Nor is it
set by `dielectricFresnelAvg`, whose own error at these iors is **3.3e-6 to 2.2e-5** since it became
a quadrature rule over the same `fresnelDielectric` the coat reflects by. What it spends is the
albedo table and this inversion: **measured worst 3.5e-5 at ior 1.33**, so 6e-5 is ~1.7x headroom.

Deliberately NOT measured as `diffuse(ior)/diffuse(ior=1)`, the tempting form since the index-matched
check proves the denominator is exactly `evaluateEon`: that identity holds only for a `fresnelAvg`
that collapses at index match, so the very revert this must catch (Karis' mean returns 1/21 at
ior=1) would corrupt the denominator too. Measured that way the Karis revert reads **0.0435** rather
than its actual **0.0857** — still a failure, but one reported as the wrong cause.

This check has inverted its purpose over time. It was the instrument for `F_avg`; it is now an
instrument for `src/scene/albedo_table.inc`, and a regeneration of that table that loses accuracy
trips here first. `F_avg` enters `coatAlbedo` only through `multiScatterTint(F, Eavg)*(1-E(mu))`,
whose derivative in F is ~0.054, so an error `e` in E recovers as `e/0.054` in `F_avg` — at the old
32x32 startup table's ~1.5e-3 that is **0.028, 140x this tolerance**, which is why the check could
not exist before the bake moved offline.

**Albedo-table interpolation.** The generator prints the quadrature's own residual on every bake
(**3.0e-5**), but the shipped grid is then read bilinearly, and that is a separate error no code
produced: it reached the tree as comments carrying numbers from an ad-hoc measurement nothing
reproduces. Re-measuring showed both misattributed — the recorded "7.1e-3 first bin" is really
**4.4e-2**, and the "3.2e-5 away from it" is the ROUGHNESS axis, not the mu axis, which is ~1e-5
there. Nothing else in the suite resolves this: the two-sided white furnace bounds it only to 2%,
**500-600x** looser than what these rows need, and pays for that bound with Monte Carlo noise no node
count removes.

Every bound is the measured worst plus **~1.7x**, the convention `average_fresnel` already uses, so
each axis trips on its own row rather than under a combined figure. Where the worsts sit is itself
the result: all three directional ones are on the lowest-alpha rows (roughness <= 0.022, at the
`kMinAlpha` floor or just off it) at **mu <= 4.4e-3**, where the lobe is a near-mirror and the layer
is narrower than a cell however the axis is warped. That region is sub-degree grazing on a mirror and
every integral consuming E weights it by cos; what a shading path at ordinary angles sees is the
control row's **3.0e-5**, the stored values' own quadrature residual and not an interpolation error.

**Dielectric Fresnel monotonicity.** Strict, with no epsilon: unpolarized external reflection is
monotone in theta for every n > 1, and the smallest real step here is `cos 1.0 -> 0.9` at **1.9e-2
relative**, five orders above float32's own precision. Strictness is what makes a Fresnel pinned to
its normal-incidence value fail rather than round into a pass. The index-matched rows run to cos
1e-5 because below **2.44e-4 (2^-12)** the old form rounded `1.0F-mu*mu` to exactly `1.0F`, reported
total internal reflection at an interface with no critical angle and returned `1.0F`: **measured
4.8e+11** in the lobe at roughness 0.02, cos 1e-5, since `D/(4*mu*mu)` multiplies it by **~5e11**
there.

**Transmission reciprocity** catches a misplaced `etaR^2`, a flipped denominator orientation or an
un-flipped `ht` — O(1) errors (a stray `eta^2` is **2.25x or 0.44x at ior 1.5**) invisible to the
furnace and round-trip tests, which assert only totals and in which the two sides' errors cancel. Its
band is not `reciprocity`'s 1e-4: D is sharply peaked at these alphas (**2.5e-3 to 1e-2**) and the
two queries build `ht` from differently scaled sums, so a few-ULP direction difference is amplified
by `dD/D ~ 4/alpha^2` off the peak. **Worst measured 6.4e-3 at roughness 0.05, 2.1e-3 at 0.10**; full
discriminating power against the O(1) structural errors survives at 1e-2.

It is SINGLE SCATTER ONLY, permanently — not a symptom of a fixable bug. `transmitMultiScatter` is
`(1-escapeWo)` at wo's eta times the escape-deficit density at the reciprocal eta, each normalised
within its own orientation, which makes its total exact but cannot make it reciprocal: the swap
exchanges which orientation each factor is read at, inherent to transmissive multiple scattering
rather than an implementation gap (ROADMAP transport #1). **Roughness 0.40 fails hard, up to a 5x
forward/reverse mismatch**, for exactly this reason — do not chase it by widening the sweep.

**Transmission round trip.** Not tight to 1.0: each side's furnace value also contains that
interface's reflected lobe, so the product carries a Fresnel cross-term that grows toward grazing
(**measured 1.027 at normal incidence, 1.058 at 60 degrees**). The band still discriminates strongly
— factors that compounded rather than cancelled would land near **ior^2 = 2.25**, and ones that
under-cancelled near **1/2.25 = 0.44**. The exit angle is not the entry angle, so reusing `thetaI`
would put the exit past the critical angle (**cos ~0.745 at ior 1.5**), where the interface totally
internally reflects and no round trip exists at all.

**TIR predicate agreement: the honest detection bound.** Angular resolution is set by the finest
offset in the sweep — a divergence that moves the critical cosine by less than 1e-3 falls between
rows and is not detectable here. Measured by mutation rather than claimed: shifting the smooth
branch's threshold to `cos^2ThetaT < 0.05` (critical cosine **+0.021** at ior 1.33) raises **5
rows**, while `< 0.002` (**+0.0009**) raises none. That is the bound on this check, not a claim of
exactness. The draw count matters for the same reason: just outside the cone the transmitted share
is already ~26% (**measured 1068/4096 at ior 1.33**), so a zero count over 4096 draws is a
structural absence rather than a sampling accident.

**Chi-square panel count.** Even, for Simpson, per axis per bin; measured, not guessed: the peaked
refraction lobe at roughness 0.2 is mis-integrated badly enough to report **p = 1e-78** on correct
code at 48 panels, still fails at 64, passes from 96, and the p-value stops moving past 256.
