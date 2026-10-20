# SSM2045 full-chip C++ validation

Date: 2026-10-20. Windows x64, LLVM-MinGW 20261006 (clang 23.1.3), C++20,
`-O2 -Wall -Wextra -static`. The reference is the corrected ngspice-47
behavioural model used during development.

## Build and tests

```sh
c++ -std=c++20 -O2 -Wall -Wextra -Iinclude src/chip.cpp tests/chip_tests.cpp -o chip_tests
./chip_tests
```

Result:

```text
PASS: SSM2045 full chip (VCF + VCA); implicit 4-state filter, Q/Ga, FC law, VCA/EXPO, reset, stability, validation
```

UBSan (`-O1 -g -fsanitize=undefined -fno-sanitize-recover=all`): same `PASS`.

Test coverage:

- 2-pole/4-pole frequency response at 48, 96 and 192 kHz against the analytic
  bilinear H(s) (100 Hz, 500 Hz, 1 kHz, 2 kHz);
- DC polarity (pin 1 inverted, pin 8 non-inverted);
- FC inversion and 500 µs smoothing;
- Q current, zero-current stability, bounded self-oscillation at 180 µA,
  bounded high-Q (1 mA);
- stability sweep 20 Hz..20 kHz, Q 0..1 mA, 50 ms random input;
- reset equivalence;
- VCA centre gm at 500 µA, EXPO 60 dB, offset, linear-control helper;
- full `Chip` path;
- rejection of invalid rates, capacitors, Q, FC, VCA controls and NaN/inf
  inputs.

## C++ filter vs corrected ngspice

Scenario: FC=655.2 Hz, Q=0, 192 kHz, 75 mV peak (150 mVpp) at 150 Hz, 120 ms.
Reference: corrected `tb_thd.cir` data (`V1`/`V8`) from the corrected
ngspice package. Comparison after 50 ms:

| Tap | max error | RMS error | reference Vpp | C++ Vpp |
|---|---:|---:|---:|---:|
| 2-pole | 38 µV | 10 µV | 0.14477 V | 0.14479 V |
| 4-pole | 42 µV | 16 µV | 0.14578 V | 0.14582 V |

The C++ initial state is zero; the ngspice data starts from its OP, so the
first milliseconds differ by the offset settling. After 50 ms the waveforms
are within 0.03 % of the reference.

Self-oscillation, FC=655.2 Hz, Q=161 µA, 192 kHz, impulse seed:

| | C++ | corrected ngspice |
|---|---:|---:|
| frequency | 862.6 Hz | 857.2 Hz |
| amplitude | 0.1123 Vpp | 0.1127 Vpp |

## VCA

At V14=0, I16=I15=500 µA, 20 mV input:

- centre gm = 4.085 mS (datasheet 4085 µmho);
- MIX OUT offset = 7 µA (1.4 mV across 200 Ω);
- EXPO ±250 mV crossfade ratio ≈940 (≈−59.5 dB; datasheet −60 dB).

The `Chip` test also exercises the combined path; the `chip_demo` example
routes 4-pole into VCA1 and 2-pole into VCA2.

## Performance

One voice, 192 kHz, 1 s of 440 Hz sine, no I/O, `-O2`, Windows x64:

```text
samples=192000 seconds=0.074192 realtime_x=13.48
```

≈13.5× real-time on the test host. Profile on the target; 8–16 voices are
expected to fit on a modern desktop, but the Newton solve cost is
signal-dependent.

## Limits / claims

- The component is behavioural and calibrated, not a transistor-level model
  and not hardware-measured.
- The Q threshold is ≈160 µA in this calibration; datasheet 150 µA typ,
  120..185 window.
- The high FC end is conservative (≈13.9 kHz −3 dB at +90 mV in the SPICE
  reference, versus the sheet's "about 20 kHz").
- The VCA is voltage-input; external 910 Ω/load/input bias is the caller's
  responsibility. Control currents above 500 µA per pin are rejected; the
  datasheet's ≤500 µA total must be enforced by the caller.
- No noise/thermal block is included in this component. The optional offsets
  are not a noise model.
- The C++ solver is trapezoidal with a damped Newton iteration; it is not a
  transistor-level SPICE solver. At 48 kHz the usable FC range is about
  10.8 kHz; use 96/192 kHz for the full range.
