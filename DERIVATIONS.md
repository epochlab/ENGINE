# Derivations

Long-form reasoning behind implementation choices that a source comment cannot hold in one line
(`notes/architect.md` §7). Each entry is referenced from the code it justifies. Nothing here is
required to read the code; it is here so the argument survives the comment budget.

See also [References](README.md#references) for the literature each technique implements, and
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

**Camera and framebuffer geometry are factored out** into a shared sub-struct: it is the whole
of what `renderRasterGBuffer`'s output depends on (`rasterizer.h` takes no environment
argument) and the leading part of what `renderPathTraced`'s does. Factored rather than
duplicated so the two producers compare the same fields.

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
