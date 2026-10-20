# Changelog

## v1.0.1 — 2026-10-20

- Added `INTEGRATION.md` (voice lifecycle, audio callback, signal mapping,
  threading, sample-rate/oversampling, self-oscillation, VCA routing).
- Added `docs/MODEL.md` (state equations, numerical method, parameter
  provenance and limits).
- Added `seed()` to `Filter`/`Chip`; a deterministic zero state stays at the
  unstable equilibrium above Q onset, so seed on note-on.
- Added `Makefile`, `build.bat` and convenience build targets.
- Added a seed-validation test; direct build and UBSan remain green.

## v1.0.0 — 2026-10-20

- First public source-only release: 4-state nonlinear VCF (2-pole/4-pole,
  Q/Ga, FC), dual OTA VCA/mixer (EXPO, linear controls, MIX OUT), `Chip`
  convenience wrapper.
- Implicit trapezoidal filter solve with damped Newton iteration.
- Self-contained tests, filter/full-chip CSV examples, CMake, README,
  VALIDATION and MIT license.
