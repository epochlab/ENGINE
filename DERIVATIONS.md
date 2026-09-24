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
