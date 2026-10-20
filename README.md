# SSM2045 full chip — C++20 VCF + VCA/mixer

Dependency-free C++20 component for the **SSM2045 Music Voicing System**:

- 2-pole / 4-pole nonlinear lowpass VCF with FC pin law and Q/Ga feedback;
- dual OTA VCA cells, EXPO crossfade, linear control inputs and MIX OUT;
- no emulator headers, no dynamic allocation, no global state.

It is the **signal-path** part of the chip (VCF + VCA). Noise, thermal warm-up,
the external Q driver and the digital control network are outside this
component. The model is behavioural and calibrated to the published sheet and
the corrected ngspice reference; it is **not** a transistor-level netlist and
**not** a hardware-calibrated clone.

## Quick start

```cpp
#include "ssm2045/chip.h"

ssm2045::FilterConfig cfg;
cfg.sampleRate = 192000.0;     // recommended for a 20 kHz FC range
cfg.fcSmoothingSeconds = 0.0;  // or 500e-6 to model the external RC
ssm2045::Chip chip(cfg);

chip.setCutoffHz(1000.0);      // or chip.setFcPinVolts(pin5Volts)
chip.setQCurrent(0.0);         // 0..~160 uA; self-oscillation above onset
chip.setLinearControlCurrents(500e-6, 500e-6);
chip.setExpoVolts(0.0);

for (;;) {
    auto out = chip.process(audioIn, vcaIn1, vcaIn2);
    // out.filter.twoPole  (pin 1, inverted)
    // out.filter.fourPole (pin 8, non-inverted)
    // out.vca.current     (MIX OUT current, A)
    // out.vca.volts       (current * mixLoadOhms)
}
```

The VCA inputs are the chip's pins 11/12 and are **independent** of the filter
outputs; route `twoPole`/`fourPole` to them as the board does. `Chip::process`
is a convenience that advances both blocks in the same sample.

## API

### Filter

| Call | Meaning |
|---|---|
| `Filter(FilterConfig)` | Construct; validates all parameters |
| `setSampleRate(hz)` | Sets the rate and resets the state |
| `reset()` | Clears the four integrator states; snaps FC smoothing |
| `setCutoffHz(hz)` | Inverts the expo law; throws if the second section is at Nyquist |
| `setFcPinVolts(v)` | Sets the pin-5 control voltage with optional smoothing |
| `setFcPinVoltsImmediate(v)` | No smoothing (tests/offline) |
| `setQCurrent(a)` | Q/Ga control current; non-negative |
| `process(x)` | One sample; returns `twoPole`, `fourPole` |
| `cutoffHz()` | Current first-section natural cutoff |
| `fcPinVolts()` / `fcTargetVolts()` | Smoothed / target pin voltage |

`FilterConfig` defaults are the corrected values: C1=10 nF, C2=3.3 nF,
C3=10 nF, C4=680 pF, gm0=72.188 µS, Vtef=27.844 mV, IOSC=150 µA,
W1=2.10216, W8=3.10216, VL_GA=1.2 V, VL_G1=0.145 V, VL_G2=0.26 V,
VL_T=7.0 V. `offsets=true` includes the typical VOSG/IOSN terms; set it false
for a DC-clean test path. `nonlinear=false` replaces every tanh by its linear
argument (debug only).

### VCA / mixer

| Call | Meaning |
|---|---|
| `setLinearControlCurrents(i16, i15)` | Pin 16/15 control currents, 0..500 µA each |
| `setLinearControlVoltages(v16, v15, Rseries=10k)` | Figure 2 convenience: I=(V−0.3)/(R+130), clamped |
| `setExpoVolts(v14)` | EXPO control, nominally ±250 mV |
| `process(in1, in2)` | Returns MIX OUT current, volts and `weight1` |

`VcaConfig` defaults: gain 8.17 A/V per A, VL_VCA=0.211 V, WMIN=1e-3,
VT=25.85 mV, load 200 Ω, IOSVCA=7 µA. With 500 µA on both controls and
EXPO=0, each cell is at −6 dB (4085 µmho centre); at ±250 mV the crossfade is
about −59 dB.

### Chip

`Chip(FilterConfig, VcaConfig)`; forwards the filter/VCA setters; `process`
returns `{filter, vca}`; `filter()`/`vca()` expose the sub-blocks. A
`Chip::reset()` resets the filter (the VCA is memoryless).

## Build

Direct:

```sh
c++ -std=c++20 -O2 -Wall -Wextra -Iinclude src/chip.cpp tests/chip_tests.cpp -o chip_tests
./chip_tests
```

CMake:

```sh
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

Targets: `ssm2045_chip` (static library), `ssm2045_chip_tests`,
`ssm2045_chip_render` (filter CSV), `ssm2045_chip_demo` (full VCF+VCA CSV).
The library is C++20, no third-party dependency.

## Sample rate and CPU

The core is a 4-state nonlinear trapezoidal (implicit) solve with a damped 4×4
Newton iteration. It runs at the host sample rate; there is no oversampling.

- 192 kHz: recommended, covers the full 20 kHz FC range (second section
  ≈2.203× the first).
- 96 kHz: FC up to ≈21.7 kHz before the second section reaches Nyquist.
- 48 kHz: FC up to ≈10.8 kHz. `setCutoffHz` throws above the limit; do not
  silently alias.

Measured on the Windows x64 test host (LLVM-MinGW clang, `-O2`, one core):
**≈13.5× real-time** at 192 kHz for one voice. A 16-voice instance is expected
to fit in real time on a modern desktop, but profile on the target.

## Validation

`chip_tests` passes; UBSan build passes. The filter output matches the corrected
ngspice reference after 50 ms to:

- 2-pole: **38 µV max / 10 µV RMS**
- 4-pole: **42 µV max / 16 µV RMS**

for 150 mVpp at 150 Hz, FC≈655 Hz, 192 kHz. Self-oscillation at IQ=161 µA:
**862.6 Hz / 0.1123 Vpp** vs ngspice **857.2 Hz / 0.1127 Vpp**. VCA at 500 µA:
**4.085 mS** centre, ≈−59 dB crossfade, 7 µA offset. Full details, test
coverage and commands are in [VALIDATION.md](VALIDATION.md).

The test suite covers:

- 2-pole and 4-pole response at 48/96/192 kHz against the analytic H(s);
- DC polarity: pin 1 inverted, pin 8 non-inverted;
- FC inversion and 500 µs smoothing;
- Q current, bounded self-oscillation, high-Q stability;
- a 20 Hz..20 kHz / 0..1 mA stability sweep;
- reset, VCA centre gm, EXPO 60 dB, linear-control helper, offsets;
- full `Chip` path and rejection of invalid inputs/parameters.

## Model notes and limits

The filter equations are in `include/ssm2045/chip.h`; the implementation solves

```text
C1*x1' = gm*VL_G1*tanh((u + y1 - x1 - ga + VOSG1)/VL_G1) + IOSN1
C2*x2' = -gm*VL_G2*tanh((x1 - VOSG2)/VL_G2) + IOSN2
y1     = VL_T*tanh(x2/VL_T)/10
C3*x3' = gm*VL_G1*tanh((y1 + y2 - x3 + VOSG3)/VL_G1) + IOSN3
C4*x4' = -gm*VL_G2*tanh((x3 - VOSG4)/VL_G2) + IOSN4
y2     = VL_T*tanh(x4/VL_T)/10
ga     = (IQ/IOSC)*VL_GA*tanh((W1*y1 + W8*y2)/VL_GA)
```

with `gm = gm0*exp(-(V5 - Vbias)/Vtef)`. The VCA uses
`Iout = K*2*w1*I16*VL*tanh(V11/VL) - ...` with the EXPO split
`w1 = WMIN+(1-2WMIN)*0.5*(1-tanh(V14/(2VT)))`.

Datasheet anchors: the capacitor set, H(s), −19.3 mV/oct, 150 µA nominal Q
(120..185 window), 4085 µmho VCA centre, ±250 mV EXPO, ±50 mV output offset,
6 dB in-band loss at oscillation, 0.1 %/1.0 % THD points. Calibrated/assumed:
the Ga tap weights and soft limits, the Q(IQ) fit, the VCA VL/WMIN, the offsets
and the exact FC bias. The internal Ga netlist is not published.

Remaining limits:

- Behavioural, not transistor-level; no device mismatch, noise or thermal
  drift in this component. Offset terms are optional but not a noise model.
- The Q threshold is ≈160 µA in this calibration (datasheet 150 typ, 120..185
  window); the FC high end is conservative (≈13.9 kHz at +90 mV in the SPICE
  reference, not 20 kHz).
- The VCA is voltage-input; add the external 910 Ω/load and input bias as the
  board requires. Control currents above 500 µA per pin are rejected; the
  datasheet also recommends ≤500 µA total, which the caller must enforce.
- No hardware SSM2045 was measured. Do not call this a calibrated hardware
  clone; use it as a stable, documented musical model and validate in the
  target mix.

## Files

```text
include/ssm2045/chip.h      public API and equations
src/chip.cpp                filter, VCA/mixer, chip
tests/chip_tests.cpp        self-contained test suite
examples/render.cpp         filter CSV (sine/impulse)
examples/chip_demo.cpp      full VCF+VCA CSV demo
CMakeLists.txt
VALIDATION.md
LICENSE                     MIT
```

## License

MIT; see [LICENSE](LICENSE). The SSM2045 name is used descriptively; this is an
independent behavioural model, not an official Solid State Micro Technology
product.
