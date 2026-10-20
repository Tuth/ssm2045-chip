# SSM2045 model notes

This is the technical reference for the C++ implementation. It is a
behavioural model calibrated to the published SSM2045 sheet and the corrected
ngspice macro-model; it is not a transistor-level netlist.

## 1. Filter state equations

The filter core has four integrator states `x1..x4` (C1..C4 voltages):

```text
C1*x1' = gm*VL_G1*tanh((u + y1 - x1 - ga + VOSG1)/VL_G1) + IOSN1
C2*x2' = -gm*VL_G2*tanh((x1 - VOSG2)/VL_G2) + IOSN2
y1     = VL_T*tanh(x2/VL_T)/10
C3*x3' = gm*VL_G1*tanh((y1 + y2 - x3 + VOSG3)/VL_G1) + IOSN3
C4*x4' = -gm*VL_G2*tanh((x3 - VOSG4)/VL_G2) + IOSN4
y2     = VL_T*tanh(x4/VL_T)/10
ga     = (IQ/IOSC)*VL_GA*tanh((W1*y1 + W8*y2)/VL_GA)
```

`y1` is the 2-pole output (pin 1, inverted relative to `y2`) and `y2` is the
4-pole output (pin 8, non-inverted). The `/10` is the internal 8.1 kΩ:900 Ω
divider; the `10` in the capacitor equations is the effective C2/C4 scaling.

With `offsets=false`, all `VOSG*`/`IOSN*` terms are zero. With
`nonlinear=false`, `sat(v,scale)` is replaced by `v` (debug only).

## 2. Expo converter and FC

```text
gm = gm0 * exp(-(V5 - Vbias) / Vtef)
```

- `gm0 = 72.188 µS`, `Vtef = 27.844 mV`, `Vbias = 0.000982 V` (1 µA across the
  external 1 kΩ).
- With C1=10 nF, C2=3.3 nF, the natural first-section FC at V5=0 is
  `gm/(2π*sqrt(C1*10*C2)) = 632.45 Hz`.
- The sheet's sensitivity is −19.3 mV/octave; `setCutoffHz` inverts the law.
- `setFcPinVolts` expects the actual pin-5 control voltage. The 0.982 mV bias
  is internal; do not add it twice.
- `setCutoffHz` throws if the second section (`fc * 2.203`) reaches
  `0.49 * sampleRate`.

The second-section natural frequency follows from C3/C4:

```text
fc2/fc1 = sqrt((C1*10*C2)/(C3*10*C4)) = 2.20294
```

## 3. Q / Ga

`IQ` is the pin-17 current, `IOSC=150 µA`. The Ga amplifier injects a weighted
sum of the two taps into the first stage. The small-signal, low-amplitude loop
is calibrated so that the datasheet's oscillation window is reproduced;
`IQ=0` is damped, `IQ≈160 µA` starts self-oscillation in this calibration.
The exact internal Ga netlist is not published; the tap weights and soft limits
are the calibrated part.

`VL_GA=1.2 V` limits the Ga output. At large amplitude the loop settles into a
bounded, sine-like oscillation; the 4-pole output is about 0.11 Vpp at
`IQ=161 µA`, FC≈655 Hz. The datasheet does not specify an oscillation
amplitude (1 Vpp is the output maximum, not the self-oscillation level).

The deterministic model has an unstable equilibrium at exactly zero above Q
onset. Use `Filter::seed()` (default 1e-3) on note-on/self-oscillation to start
it, like device noise would.

## 4. VCA / mixer

```text
w1 = WMIN + (1 - 2*WMIN) * 0.5 * (1 - tanh(V14/(2*VT)))
w2 = 1 - w1
Iv1 = 2*w1*I16
Iv2 = 2*w2*I15
Iout = -K*Iv1*VL_VCA*tanh(V11/VL_VCA)
       + K*Iv2*VL_VCA*tanh(V12/VL_VCA)
       + IOSVCA
Vout = Iout * Rload
```

Defaults: `K=8.17 A/V per A`, `VL_VCA=0.211 V`, `WMIN=1e-3`, `VT=25.85 mV`,
`Rload=200 Ω`, `IOSVCA=7 µA`, `Vmin=0.3 V`, internal 130 Ω.

At `I16=I15=500 µA`, `V14=0`, each cell is at −6 dB; the centre gm is
`K*500 µA = 4.085 mS`. At `V14=±250 mV` the crossfade is about −60 dB.
`setLinearControlVoltages` models the Figure 2 resistor:
`I=(V−0.3)/(R+130)`, clamped to 0..500 µA.

## 5. Numerical method

The analog equations are integrated with the trapezoidal rule:

```text
x[n+1] = x[n] + T/2 * (f(x[n], u[n]) + f(x[n+1], u[n+1]))
```

The 4×4 implicit system is solved with a damped Newton iteration (up to 12
iterations, 6 line-search halvings, residual < 1e-10 V). The Jacobian is
analytic. If Newton fails, a single explicit step is used and the state is
guarded against NaN; normal operation converges. There is no unit-delay
feedback path.

This is a zero-delay implicit solve, but it is not the same code as the
original `ssm2045_zdf` SVF kernel; it is the corrected behavioural topology
from the ngspice package.

## 6. Datasheet anchors vs calibration

**From the sheet:** C1=10 nF, C2=3.3 nF, C3=10 nF, C4=680 pF; H(s) form;
−19.3 mV/oct; 150 µA nominal Q (120..185); 4085 µmho VCA centre; ±250 mV EXPO;
±50 mV output offset; 6 dB in-band loss at oscillation; 0.1 %/1.0 % THD points;
150 mVpp input; 1 Vpp/3 Vpp output maximum.

**Calibrated/assumed:** Ga weights `W1=2.10216`, `W8=3.10216`, `VL_GA=1.2 V`;
stage limits `VL_G1=0.145 V`, `VL_G2=0.26 V`, `VL_T=7.0 V`; the Q(IQ) fit;
the VCA `VL_VCA/WMIN`; offsets; the exact FC bias. The internal Ga schematic is
not in the sheet.

## 7. Output levels and limits

- `y1/y2 = VL_T*tanh(x/VL_T)/10`, so the filter taps are soft-limited to
  about ±0.7 V.
- `Filter::process` throws on NaN/inf input.
- `setQCurrent` rejects negative values; high Q is bounded by the tanh stages.
- `setCutoffHz` rejects values whose second section reaches Nyquist.
- VCA control currents above 500 µA per pin are rejected. The datasheet's
  ≤500 µA total recommendation is the caller's responsibility.
- No noise, thermal drift, device mismatch, Q driver or digital control is in
  this component. Offsets are optional, not a noise model.

## 8. Validation reference

The C++ output matches the corrected ngspice-47 filter to ≈38 µV max
(2-pole) and ≈42 µV max (4-pole) after 50 ms for 150 mVpp at 150 Hz;
self-oscillation is 862.6 Hz/0.1123 Vpp vs 857.2 Hz/0.1127 Vpp. VCA centre
4.085 mS, crossfade ≈−59 dB, offset 7 µA. See [VALIDATION.md](../VALIDATION.md).
