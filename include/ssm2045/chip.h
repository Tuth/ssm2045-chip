#pragma once

namespace ssm2045 {

// Capacitors of the two cascaded 2-pole sections (datasheet Figure 2 values).
struct Capacitors {
    double c1 = 10e-9;
    double c2 = 3.3e-9;
    double c3 = 10e-9;
    double c4 = 680e-12;
};

struct FilterConfig {
    double sampleRate = 192000.0;
    Capacitors caps{};

    // Expo converter: gm = gm0 * exp(-(fcPin - fcBias) / vtef).
    // gm0 corresponds to fc = 632.45 Hz with the Figure 2 capacitors.
    double gm0 = 72.188e-6;
    double vtef = 27.844e-3;
    double fcBiasVolts = 0.000982; // 1 uA bias across the external 1 kOhm

    // Ga / Q feedback.
    double qOscillationCurrent = 150e-6;
    double w1 = 2.10216;
    double w8 = 3.10216;
    double vlGa = 1.2;

    // OTA / output soft limits.
    double vlG1 = 0.145;
    double vlG2 = 0.26;
    double vlT = 7.0;

    // Typical offsets (optional, enabled by default to match the SPICE model).
    double vosg1 = 4e-3;
    double vosg2 = -11e-3;
    double vosg3 = 6e-3;
    double vosg4 = -9e-3;
    double iosn1 = 12e-9;
    double iosn2 = -9e-9;
    double iosn3 = 15e-9;
    double iosn4 = -11e-9;
    bool offsets = true;

    // If false, every tanh is replaced by its linear argument (test/debug).
    bool nonlinear = true;

    // Optional one-pole smoothing of the FC control voltage; 0 disables it.
    double fcSmoothingSeconds = 500e-6;
};

struct Outputs {
    double twoPole = 0.0;  // pin 1, inverted relative to the 4-pole tap
    double fourPole = 0.0; // pin 8, non-inverted
};

// 4-state nonlinear SSM2045 filter core.
//
// The analog equations are the corrected behavioural model:
//
//   C1*x1' = gm*VL_G1*tanh((u + y1 - x1 - ga + VOSG1)/VL_G1) + IOSN1
//   C2*x2' = -gm*VL_G2*tanh((x1 - VOSG2)/VL_G2) + IOSN2
//   y1     = VL_T*tanh(x2/VL_T)/10
//   C3*x3' = gm*VL_G1*tanh((y1 + y2 - x3 + VOSG3)/VL_G1) + IOSN3
//   C4*x4' = -gm*VL_G2*tanh((x3 - VOSG4)/VL_G2) + IOSN4
//   y2     = VL_T*tanh(x4/VL_T)/10
//   ga     = (IQ/IOSC)*VL_GA*tanh((W1*y1 + W8*y2)/VL_GA)
//
// Trapezoidal integration is solved implicitly with a damped 4x4 Newton
// iteration; there is no unit-delay feedback.
class Filter {
public:
    explicit Filter(const FilterConfig& config = {});

    void reset();
    // Perturbs the integrator state so a Q current above onset can start
    // oscillating. The real chip starts from device noise; a deterministic
    // model at exactly zero would stay at the unstable equilibrium.
    void seed(double amplitude = 1e-3);
    void setSampleRate(double sampleRate);
    double sampleRate() const { return config_.sampleRate; }

    void setFcPinVolts(double volts);
    void setFcPinVoltsImmediate(double volts);
    void setCutoffHz(double hz);

    void setQCurrent(double amps);
    double qCurrent() const { return qCurrent_; }

    double fcPinVolts() const { return fcPin_; }
    double fcTargetVolts() const { return fcTarget_; }
    double cutoffHz() const;

    // Throws std::invalid_argument on a non-finite input.
    Outputs process(double input);

private:
    void validate() const;
    void updateSmoothingCoefficient();
    double gmNow() const;

    void deriv(const double x[4], double u, double gm, double aq, double f[4]) const;
    void jacobian(const double x[4], double u, double gm, double aq,
                  double j[4][4]) const;
    void solveSample(double u, double gm, double aq);

    FilterConfig config_;
    double x_[4]{};
    double f_[4]{};
    double fcPin_ = 0.0;
    double fcTarget_ = 0.0;
    double qCurrent_ = 0.0;
    double smoothing_ = 1.0;
};

struct VcaConfig {
    double vcaGain = 8.17;        // A/V per A of control current (datasheet)
    double vlVca = 0.211;         // VCA input linear range (calibrated)
    double wmin = 1e-3;           // -60 dB EXPO floor
    double vt = 25.85e-3;         // thermal voltage at ~300 K
    double mixLoadOhms = 200.0;   // external I/V load at MIX OUT
    double iosVca = 7e-6;         // MIX OUT offset current
    double controlReferenceVolts = 0.3;
    double controlInternalOhms = 130.0;
    double maxLinearCurrent = 500e-6;
};

struct VcaOutputs {
    double current = 0.0; // MIX OUT current (A); negative for VCA1 input
    double volts = 0.0;   // current * mixLoadOhms
    double weight1 = 0.5; // EXPO split for VCA1; weight2 = 1 - weight1
};

// Two OTA VCA cells and the EXPO crossfade / MIX OUT stage. Memoryless.
class VcaMixer {
public:
    explicit VcaMixer(const VcaConfig& config = {});

    void setLinearControlCurrents(double i16, double i15);
    // Convenience for the Figure 2 circuit: I = (V - 0.3) / (R + 130 ohm),
    // clamped to 0..maxLinearCurrent.
    void setLinearControlVoltages(double v16, double v15, double seriesOhms = 10e3);
    void setExpoVolts(double volts);

    double expoVolts() const { return expo_; }
    double linearCurrent1() const { return i16_; }
    double linearCurrent2() const { return i15_; }

    VcaOutputs process(double in1, double in2) const;

private:
    void validate() const;

    VcaConfig config_;
    double i16_ = 0.0;
    double i15_ = 0.0;
    double expo_ = 0.0;
};

struct ChipOutputs {
    Outputs filter;
    VcaOutputs vca;
};

// Full VCF + VCA/mixer signal path. The filter and VCA inputs are separate
// pins, exactly as on the chip; route the 2-pole/4-pole outputs externally.
class Chip {
public:
    explicit Chip(const FilterConfig& filter = {}, const VcaConfig& vca = {});

    void reset();
    void setSampleRate(double sampleRate);
    void seed(double amplitude = 1e-3) { filter_.seed(amplitude); }

    void setFcPinVolts(double volts) { filter_.setFcPinVolts(volts); }
    void setFcPinVoltsImmediate(double volts) { filter_.setFcPinVoltsImmediate(volts); }
    void setCutoffHz(double hz) { filter_.setCutoffHz(hz); }
    void setQCurrent(double amps) { filter_.setQCurrent(amps); }

    void setLinearControlCurrents(double i16, double i15) { vca_.setLinearControlCurrents(i16, i15); }
    void setLinearControlVoltages(double v16, double v15, double seriesOhms = 10e3) {
        vca_.setLinearControlVoltages(v16, v15, seriesOhms);
    }
    void setExpoVolts(double volts) { vca_.setExpoVolts(volts); }

    ChipOutputs process(double audioIn, double vcaIn1, double vcaIn2);

    Filter& filter() { return filter_; }
    const Filter& filter() const { return filter_; }
    VcaMixer& vca() { return vca_; }
    const VcaMixer& vca() const { return vca_; }

private:
    Filter filter_;
    VcaMixer vca_;
};

} // namespace ssm2045
