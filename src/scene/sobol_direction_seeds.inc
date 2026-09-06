// Sobol direction-number seeds, transcribed from Joe & Kuo (2008), "Constructing Sobol Sequences with Better
// Two-Dimensional Projections", SIAM J. Sci. Comput. 30(5), 2635-2654 -- direction-number file new-joe-kuo-6.21201,
// https://web.maths.unsw.edu.au/~fkuo/sobol/ (the de-facto standard table, also used by PBRT-v4, SciPy and TensorFlow
// Probability). The file's row for dimension d = 2 reads "2  1  0  1", i.e. degree s = 1, coefficients a = 0, and the
// single initial direction number m_1 = 1; that row is reproduced verbatim below.
// Fields per row: {degree s, bit-encoded polynomial coefficients a, {initial direction numbers m_1..m_s}}.
//
// Only dimension 2 appears here, and only one row is needed, because the sampler pads: every draw reuses Sobol
// dimensions 1 and 2 under a fresh scramble and index shuffle rather than walking into higher dimensions (see
// sampler.h). Dimension 1 needs no seeds at all -- its direction vectors are the identity, the van der Corput sequence
// in base 2 -- and is generated directly in sampler.cpp. A jointly-stratified next3D/next4D would need dimensions 3 and
// 4, whose rows ("3  2  1  1 3" and "4  3  1  1 3 1") are the next two lines of the same published file.
// The recurrence that turns these seeds into 32-bit direction vectors is in sampler.cpp, where it can be checked
// against Joe & Kuo's own reference implementation rather than against a generator's output.

{1, 0, {1}},  // d = 2
