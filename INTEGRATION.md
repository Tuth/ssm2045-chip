# Integrating the SSM2045 C++ component

This guide is for the host/emulator integrator (or an integrating coding agent).
The package is deliberately small: one public header, one source file, no
third-party dependency.

## 1. What to integrate

| Block | Class | State |
|---|---|---|
| VCF (2-pole/4-pole) | `ssm2045::Filter` | 4 integrator states + FC smoothing |
| VCA/mixer | `ssm2045::VcaMixer` | memoryless controls |
| Both | `ssm2045::Chip` | one filter + one VCA |

Use `Chip` when you want one object per voice; use `Filter`/`VcaMixer`
separately if the host already has its own routing or wants to share a VCA.

## 2. Add the source

### Copy

Copy:

```text
include/ssm2045/chip.h
src/chip.cpp
```

and compile `src/chip.cpp` into the host target. C++20 is required.

### CMake subdirectory

```cmake
add_subdirectory(third_party/ssm2045-chip)
target_link_libraries(your_audio_target PRIVATE ssm2045_chip)
```

The library exports its include directory publicly.

### FetchContent

```cmake
include(FetchContent)
FetchContent_Declare(
  ssm2045_chip
  GIT_REPOSITORY https://github.com/Tuth/ssm2045-chip.git
  GIT_TAG        v1.0.1
)
FetchContent_MakeAvailable(ssm2045_chip)
target_link_libraries(your_audio_target PRIVATE ssm2045_chip)
```

Pin a release tag, not `main`, for a reproducible integration.

## 3. Minimal per-voice wrapper

```cpp
#include "ssm2045/chip.h"
#include <algorithm>
#include <cmath>

struct Voice {
    ssm2045::Chip chip;
    bool active = false;

    void prepare(double sampleRate) {
        ssm2045::FilterConfig cfg;
        cfg.sampleRate = sampleRate;
        cfg.fcSmoothingSeconds = 500e-6; // external FC RC; 0 for snap
        chip = ssm2045::Chip(cfg);
    }

    void noteOn(double fcPinVolts, double qCurrent, double i16, double i15,
                double expoVolts) {
        chip.reset();
        chip.setFcPinVoltsImmediate(fcPinVolts);
        chip.setQCurrent(qCurrent);
        chip.setLinearControlCurrents(i16, i15);
        chip.setExpoVolts(expoVolts);
        // The model is deterministic: at exactly zero it would stay at the
        // unstable equilibrium above Q onset. A -60 dB seed starts it like
        // device noise would.
        chip.seed(1e-3);
        active = true;
    }

    float render(float in) {
        auto f = chip.filter().process(in);
        auto vca = chip.vca().process(f.fourPole, f.twoPole);
        return float(f.fourPole + vca.volts);
    }
};
```

`Chip::process(in, vcaIn1, vcaIn2)` is a convenience that advances both blocks
with **external** VCA inputs. It does not automatically route the filter taps.
If the board routes pin 8 into VCA1 and pin 1 into VCA2, the explicit
`chip.filter().process` + `chip.vca().process` form above is clearer.

## 4. Signal mapping

| Chip pin / block | API | Notes |
|---|---|---|
| 5, FC | `setFcPinVolts(v)` or `setCutoffHz(hz)` | `setFcPinVolts` expects the actual control voltage; internal 0.982 mV bias is handled. Default smoothing 500 µs. |
| 17, Q | `setQCurrent(a)` | 0..~160 µA useful; self-oscillation above onset. External Q driver not included. |
| 2, SIG IN | `process(in)` | Voltage-domain input; the 900 Ω input load is external. |
| 1, 2-pole out | `Outputs::twoPole` | Inverted relative to pin 8. |
| 8, 4-pole out | `Outputs::fourPole` | Non-inverted. |
| 11/12, VCA in | `process(in1, in2)` / `VcaMixer::process` | Voltage inputs; 910 Ω network external. VCA1 is inverting, VCA2 non-inverting. |
| 16/15, LIN | `setLinearControlCurrents(i16,i15)` | 0..500 µA each; datasheet recommends ≤500 µA total. |
| 16/15 via resistor | `setLinearControlVoltages(v16,v15,R)` | Figure 2 helper: `I=(V−0.3)/(R+130)`, clamped. |
| 14, EXPO | `setExpoVolts(v14)` | 0 V = both cells at −6 dB; ±250 mV ≈ −60 dB crossfade. |
| 13, MIX OUT | `VcaOutputs::current` / `volts` | Current into the external load; `mixLoadOhms` default 200 Ω. |

Do **not** feed a normalized 0..1 FC value directly. The model wants pin-5
volts (or a real cutoff in Hz). Apply the EII/board control network first; do
not duplicate an unknown exponential mapping.

## 5. Audio callback pattern

```cpp
void processBlock(float* out, const float* in, int frames,
                  const ControlSnapshot& ctl) {
    // Control updates happen once per block on the audio thread.
    for (auto& v : voices) {
        if (!v.active) continue;
        v.chip.setFcPinVolts(ctl.fcPinVolts);   // smoothed internally
        v.chip.setQCurrent(ctl.qCurrent);
        v.chip.setLinearControlCurrents(ctl.i16, ctl.i15);
        v.chip.setExpoVolts(ctl.expoVolts);
    }

    for (int n = 0; n < frames; ++n) {
        float mix = 0.0f;
        for (auto& v : voices) {
            if (!v.active) continue;
            const auto f = v.chip.filter().process(in[n]);
            const auto vca = v.chip.vca().process(f.fourPole, f.twoPole);
            mix += float(f.fourPole + vca.volts); // choose the intended sum
        }
        out[n] = mix;
    }
}
```

- Prepare voices and allocate before the callback. `process` allocates nothing.
- Do not call `setSampleRate` from the audio thread; it resets the filter.
- Call `reset()`/`seed()` on note-on, not per block.
- `setCutoffHz` can throw when the second section reaches Nyquist; validate or
  clamp on the control side, outside the audio callback.
- `process` throws on non-finite input. Keep the host signal finite; if a
  voice can receive NaN, check at the voice boundary and reset that voice.

## 6. Threading and parameter changes

The model has **no internal locks or atomics**. Each `Filter`/`VcaMixer` is
independent and has no global state, so one instance per voice is safe across
voices/threads as long as a single thread owns a given instance.

For UI → audio parameter changes:

1. write plain values/atomics in the UI thread;
2. snapshot them once per audio block;
3. call the setters on the audio thread.

Do not call setters on one thread while `process` runs on another for the same
instance. If the host must do that, protect the instance with its own lock
(not ideal) or double-buffer the whole voice.

## 7. Sample rate and oversampling

The filter runs at the host sample rate. Its second section is ≈2.203× the
first-section cutoff, so the usable first-section FC is limited by Nyquist:

| Host rate | Usable first-section FC (approx.) |
|---:|---:|
| 48 kHz | 10.8 kHz |
| 96 kHz | 21.7 kHz |
| 192 kHz | 43 kHz (full 20 kHz FC comfortably) |

`setCutoffHz` throws above the limit. For a 48 kHz host that must reach
20 kHz, run the component at 192 kHz (4× oversampling), then decimate with the
host's existing anti-alias path. Do not silently alias; a clamped FC is
usually worse than an explicit range limit.

The default FC smoothing is 500 µs, matching the external RC. Set it to 0 if
the host already smooths the FCV.

## 8. Gain staging, offsets and DC

- Nominal input level is 150 mVpp; the 2-pole/4-pole taps are roughly unity
  at DC (2-pole inverted, 4-pole non-inverted).
- The filter output is soft-limited by the `VL_T` tanh to about ±0.7 V; the VCA
  output depends on its control current and load and can be larger. Check the
  host gain structure.
- `FilterConfig::offsets` is true by default to match the SPICE model
  (pin 1 ≈ −15 mV, pin 8 ≈ 0 mV at V5=0). Set it false for a DC-clean test
  path; AC-couple in the host if the offsets are unwanted.
- VCA offset is `VcaConfig::iosVca` (7 µA default → 1.4 mV into 200 Ω).
- The VCA input linear range is `VL_VCA=0.211 V`; larger signals compress
  through the tanh. Do not drive it with a hot ±1 V signal and expect a clean
  gain.

## 9. Self-oscillation and Q

At `IQ=0` the filter is damped. Near the datasheet's 150 µA window it starts
to ring, and above onset it self-oscillates. The model is deterministic:
`reset()` leaves the state at exactly zero, which is an unstable equilibrium
above onset. Call `seed()` (default 1e-3) on note-on, or inject a tiny noise
sample, whenever the Q current can be above onset. Without a seed the voice
can remain mathematically silent.

The model's own onset is ≈160 µA in this calibration (datasheet 150 µA typ,
120..185 window). The Q current is an input; the external Q driver, its
temperature drift and the exact pin-17 network are the host's responsibility.

## 10. VCA/mixer details

- With 500 µA on both controls and EXPO=0, each cell is at −6 dB
  (≈4.085 mS). The centre is not unity gain; the crossfade gives 0 dB at the
  endpoints.
- EXPO sign: positive `V14` favours VCA2, negative favours VCA1 (the model
  follows the datasheet's Figure 6b orientation). Verify against the host's
  control polarity.
- `setLinearControlVoltages` models the Figure 2 external resistor to the
  internal 0.3 V/130 Ω node. If the host already has a current-control
  network, use `setLinearControlCurrents` and skip the helper.
- `mixLoadOhms` is fixed at construction. If the board uses 910 Ω or a virtual
  ground, construct the `VcaMixer`/`Chip` with a matching `VcaConfig`.
- The VCA inputs are voltage inputs; the 910 Ω input network and any input
  bias are external.

## 11. Validation in the host

```sh
c++ -std=c++20 -O2 -Wall -Wextra -Iinclude src/chip.cpp tests/chip_tests.cpp -o chip_tests
./chip_tests
```

Expected: `PASS: SSM2045 full chip (VCF + VCA); ...`.

For a host-level A/B, compare:

- 2-pole/4-pole response at 48/96/192 kHz;
- zero-Q decay and Q-onset/self-oscillation;
- VCA centre gm and EXPO endpoints;
- full `Chip` path with the board's routing.

The reference measurements and limitations are in [VALIDATION.md](VALIDATION.md).

## 12. Integration checklist

- [ ] One `Chip`/`Filter` per voice; no shared instance.
- [ ] Prepared at the host rate before the audio callback.
- [ ] FC mapped to pin-5 volts (or a real Hz value), not normalized 0..1.
- [ ] Q mapped to pin-17 current; Q driver external.
- [ ] `seed()` on note-on if self-oscillation is possible.
- [ ] VCA inputs routed from the intended filter taps.
- [ ] VCA control currents/EXPO mapped with the correct polarity and range.
- [ ] DC offsets handled/AC-coupled as the host requires.
- [ ] Sample-rate limit respected or oversampling added.
- [ ] Parameter updates snapshotted per block; no cross-thread setters.
- [ ] No allocations, no exceptions, no NaN in the audio callback.
- [ ] Tests and a host A/B pass.

## 13. Known limits

The component is behavioural and calibrated, not transistor-level or
hardware-measured. It has no noise, thermal warm-up, device mismatch, Q-driver
or digital control. The Q threshold, FC high-end and VCA control limits are
documented in the README and VALIDATION; do not quote it as a calibrated
hardware clone.
