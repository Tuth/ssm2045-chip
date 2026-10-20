#include "ssm2045/chip.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace ssm2045 {
namespace {

constexpr double kTwoPi = 6.283185307179586476925286766559;

double sat(double v, double scale, bool nonlinear) {
    if (!nonlinear) return v;
    return scale * std::tanh(v / scale);
}

double dsat(double v, double scale, bool nonlinear) {
    if (!nonlinear) return 1.0;
    const double t = std::tanh(v / scale);
    return 1.0 - t * t;
}

bool solve4(double a[4][4], double b[4], double x[4]) {
    int p[4] = {0, 1, 2, 3};
    for (int k = 0; k < 4; ++k) {
        int pivot = k;
        double best = std::abs(a[k][k]);
        for (int r = k + 1; r < 4; ++r) {
            const double v = std::abs(a[r][k]);
            if (v > best) { best = v; pivot = r; }
        }
        if (!std::isfinite(best) || best < 1e-18) return false;
        if (pivot != k) {
            for (int c = 0; c < 4; ++c) std::swap(a[k][c], a[pivot][c]);
            std::swap(b[k], b[pivot]);
            std::swap(p[k], p[pivot]);
        }
        const double d = a[k][k];
        for (int r = k + 1; r < 4; ++r) {
            const double m = a[r][k] / d;
            for (int c = k; c < 4; ++c) a[r][c] -= m * a[k][c];
            b[r] -= m * b[k];
        }
    }
    for (int k = 3; k >= 0; --k) {
        double s = b[k];
        for (int c = k + 1; c < 4; ++c) s -= a[k][c] * x[c];
        if (!std::isfinite(a[k][k]) || std::abs(a[k][k]) < 1e-18) return false;
        x[k] = s / a[k][k];
        if (!std::isfinite(x[k])) return false;
    }
    return true;
}

} // namespace

// ---------------------------------------------------------------------------
// Filter
// ---------------------------------------------------------------------------

Filter::Filter(const FilterConfig& config) : config_(config) {
    validate();
    updateSmoothingCoefficient();
    fcTarget_ = 0.0;
    fcPin_ = 0.0;
    reset();
}

void Filter::validate() const {
    const auto finite = [](double v) { return std::isfinite(v); };
    if (!finite(config_.sampleRate) || config_.sampleRate <= 0.0)
        throw std::invalid_argument("SSM2045: sample rate must be positive");
    if (!(config_.caps.c1 > 0.0 && config_.caps.c2 > 0.0 &&
          config_.caps.c3 > 0.0 && config_.caps.c4 > 0.0))
        throw std::invalid_argument("SSM2045: all capacitors must be positive");
    if (!finite(config_.gm0) || config_.gm0 <= 0.0)
        throw std::invalid_argument("SSM2045: gm0 must be positive");
    if (!finite(config_.vtef) || config_.vtef <= 0.0)
        throw std::invalid_argument("SSM2045: vtef must be positive");
    if (!finite(config_.fcBiasVolts))
        throw std::invalid_argument("SSM2045: invalid FC bias");
    if (!finite(config_.qOscillationCurrent) || config_.qOscillationCurrent <= 0.0)
        throw std::invalid_argument("SSM2045: Q oscillation current must be positive");
    if (!finite(config_.w1) || !finite(config_.w8) ||
        !finite(config_.vlGa) || config_.vlGa <= 0.0 ||
        !finite(config_.vlG1) || config_.vlG1 <= 0.0 ||
        !finite(config_.vlG2) || config_.vlG2 <= 0.0 ||
        !finite(config_.vlT) || config_.vlT <= 0.0)
        throw std::invalid_argument("SSM2045: invalid Ga/soft-limit parameter");
    if (!finite(config_.fcSmoothingSeconds) || config_.fcSmoothingSeconds < 0.0)
        throw std::invalid_argument("SSM2045: invalid FC smoothing time");
    const double offsets[] = {config_.vosg1, config_.vosg2, config_.vosg3,
                              config_.vosg4, config_.iosn1, config_.iosn2,
                              config_.iosn3, config_.iosn4};
    for (double v : offsets)
        if (!finite(v)) throw std::invalid_argument("SSM2045: invalid offset");
}

void Filter::updateSmoothingCoefficient() {
    const double tau = config_.fcSmoothingSeconds;
    if (tau <= 0.0) {
        smoothing_ = 1.0;
        return;
    }
    const double x = 1.0 / (config_.sampleRate * tau);
    smoothing_ = 1.0 - std::exp(-x);
    smoothing_ = std::clamp(smoothing_, 0.0, 1.0);
}

double Filter::gmNow() const {
    const double veff = fcPin_ - config_.fcBiasVolts;
    const double exponent = std::clamp(-veff / config_.vtef, -700.0, 700.0);
    return config_.gm0 * std::exp(exponent);
}

void Filter::reset() {
    for (double& v : x_) v = 0.0;
    for (double& v : f_) v = 0.0;
    fcPin_ = fcTarget_;
    const double gm = gmNow();
    const double aq = qCurrent_ / config_.qOscillationCurrent;
    deriv(x_, 0.0, gm, aq, f_);
}

void Filter::setSampleRate(double sampleRate) {
    if (!std::isfinite(sampleRate) || sampleRate <= 0.0)
        throw std::invalid_argument("SSM2045: sample rate must be positive");
    config_.sampleRate = sampleRate;
    updateSmoothingCoefficient();
    reset();
}

void Filter::setFcPinVolts(double volts) {
    if (!std::isfinite(volts))
        throw std::invalid_argument("SSM2045: non-finite FC control voltage");
    fcTarget_ = volts;
    if (config_.fcSmoothingSeconds <= 0.0) fcPin_ = volts;
}

void Filter::setFcPinVoltsImmediate(double volts) {
    if (!std::isfinite(volts))
        throw std::invalid_argument("SSM2045: non-finite FC control voltage");
    fcTarget_ = volts;
    fcPin_ = volts;
}

void Filter::setCutoffHz(double hz) {
    if (!std::isfinite(hz) || hz <= 0.0)
        throw std::invalid_argument("SSM2045: cutoff must be positive");
    const double ratio = std::sqrt((config_.caps.c1 * 10.0 * config_.caps.c2) /
                                   (config_.caps.c3 * 10.0 * config_.caps.c4));
    const double second = hz * ratio;
    if (second >= 0.49 * config_.sampleRate)
        throw std::invalid_argument("SSM2045: second section cutoff too close to Nyquist");
    const double root = std::sqrt(config_.caps.c1 * 10.0 * config_.caps.c2);
    const double gm = kTwoPi * hz * root;
    const double veff = -config_.vtef * std::log(gm / config_.gm0);
    setFcPinVolts(veff + config_.fcBiasVolts);
}

void Filter::setQCurrent(double amps) {
    if (!std::isfinite(amps) || amps < 0.0)
        throw std::invalid_argument("SSM2045: Q current must be non-negative");
    qCurrent_ = amps;
}

double Filter::cutoffHz() const {
    const double root = std::sqrt(config_.caps.c1 * 10.0 * config_.caps.c2);
    const double gm = gmNow();
    if (!std::isfinite(gm) || root <= 0.0) return 0.0;
    return gm / (kTwoPi * root);
}

void Filter::deriv(const double x[4], double u, double gm, double aq, double f[4]) const {
    const bool nl = config_.nonlinear;
    const double y1 = sat(x[1], config_.vlT, nl) / 10.0;
    const double y2 = sat(x[3], config_.vlT, nl) / 10.0;
    const double ga = aq * sat(config_.w1 * y1 + config_.w8 * y2, config_.vlGa, nl);
    const double o1 = config_.offsets ? config_.vosg1 : 0.0;
    const double o2 = config_.offsets ? config_.vosg2 : 0.0;
    const double o3 = config_.offsets ? config_.vosg3 : 0.0;
    const double o4 = config_.offsets ? config_.vosg4 : 0.0;
    const double n1 = config_.offsets ? config_.iosn1 : 0.0;
    const double n2 = config_.offsets ? config_.iosn2 : 0.0;
    const double n3 = config_.offsets ? config_.iosn3 : 0.0;
    const double n4 = config_.offsets ? config_.iosn4 : 0.0;

    const double i1 = gm * sat(u + y1 - x[0] - ga + o1, config_.vlG1, nl) + n1;
    const double i2 = -gm * sat(x[0] - o2, config_.vlG2, nl) + n2;
    const double i3 = gm * sat(y1 + y2 - x[2] + o3, config_.vlG1, nl) + n3;
    const double i4 = -gm * sat(x[2] - o4, config_.vlG2, nl) + n4;

    f[0] = i1 / config_.caps.c1;
    f[1] = i2 / config_.caps.c2;
    f[2] = i3 / config_.caps.c3;
    f[3] = i4 / config_.caps.c4;
}

void Filter::jacobian(const double x[4], double u, double gm, double aq,
                      double j[4][4]) const {
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c) j[r][c] = 0.0;

    const bool nl = config_.nonlinear;
    const double y1 = sat(x[1], config_.vlT, nl) / 10.0;
    const double y2 = sat(x[3], config_.vlT, nl) / 10.0;
    const double dy1 = dsat(x[1], config_.vlT, nl) / 10.0;
    const double dy2 = dsat(x[3], config_.vlT, nl) / 10.0;
    const double s = config_.w1 * y1 + config_.w8 * y2;
    const double dga = aq * dsat(s, config_.vlGa, nl);
    const double dga1 = dga * config_.w1;
    const double dga2 = dga * config_.w8;
    const double o1 = config_.offsets ? config_.vosg1 : 0.0;
    const double o2 = config_.offsets ? config_.vosg2 : 0.0;
    const double o3 = config_.offsets ? config_.vosg3 : 0.0;
    const double o4 = config_.offsets ? config_.vosg4 : 0.0;

    const double b1 = u + y1 - x[0] - aq * sat(config_.w1 * y1 + config_.w8 * y2,
                                               config_.vlGa, nl) + o1;
    const double b2 = x[0] - o2;
    const double b3 = y1 + y2 - x[2] + o3;
    const double b4 = x[2] - o4;
    const double g1 = gm * dsat(b1, config_.vlG1, nl);
    const double g2 = gm * dsat(b2, config_.vlG2, nl);
    const double g3 = gm * dsat(b3, config_.vlG1, nl);
    const double g4 = gm * dsat(b4, config_.vlG2, nl);

    j[0][0] = -g1 / config_.caps.c1;
    j[0][1] = g1 * dy1 * (1.0 - dga1) / config_.caps.c1;
    j[0][3] = -g1 * dga2 * dy2 / config_.caps.c1;

    j[1][0] = -g2 / config_.caps.c2;

    j[2][1] = g3 * dy1 / config_.caps.c3;
    j[2][2] = -g3 / config_.caps.c3;
    j[2][3] = g3 * dy2 / config_.caps.c3;

    j[3][2] = -g4 / config_.caps.c4;
}

void Filter::solveSample(double u, double gm, double aq) {
    const double T = 1.0 / config_.sampleRate;
    const double h = 0.5 * T;
    double xp[4], fp[4];
    for (int i = 0; i < 4; ++i) { xp[i] = x_[i]; fp[i] = f_[i]; }

    double x[4];
    for (int i = 0; i < 4; ++i) x[i] = xp[i] + T * fp[i];

    bool converged = false;
    for (int iter = 0; iter < 12; ++iter) {
        double f[4];
        deriv(x, u, gm, aq, f);
        double F[4];
        double norm = 0.0;
        for (int i = 0; i < 4; ++i) {
            F[i] = x[i] - xp[i] - h * (fp[i] + f[i]);
            norm = std::max(norm, std::abs(F[i]));
        }
        if (!std::isfinite(norm)) break;
        if (norm < 1e-10) { converged = true; break; }

        double J[4][4];
        jacobian(x, u, gm, aq, J);
        double A[4][4], b[4], dx[4];
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) A[r][c] = (r == c ? 1.0 : 0.0) - h * J[r][c];
            b[r] = -F[r];
        }
        if (!solve4(A, b, dx)) break;

        double alpha = 1.0;
        double best = norm;
        bool improved = false;
        for (int ls = 0; ls < 6; ++ls) {
            double xt[4];
            for (int i = 0; i < 4; ++i) xt[i] = x[i] + alpha * dx[i];
            double ft[4];
            deriv(xt, u, gm, aq, ft);
            double n2 = 0.0;
            for (int i = 0; i < 4; ++i) {
                const double Fi = xt[i] - xp[i] - h * (fp[i] + ft[i]);
                n2 = std::max(n2, std::abs(Fi));
            }
            if (std::isfinite(n2) && n2 < best) {
                best = n2;
                for (int i = 0; i < 4; ++i) x[i] = xt[i];
                improved = true;
                break;
            }
            alpha *= 0.5;
        }
        if (!improved) break;
        if (best < 1e-10) { converged = true; break; }
    }

    if (!converged) {
        // Robust fallback: one explicit step, but never leave the state NaN.
        double f0[4];
        deriv(xp, u, gm, aq, f0);
        for (int i = 0; i < 4; ++i) x[i] = xp[i] + T * f0[i];
    }
    for (int i = 0; i < 4; ++i) {
        if (!std::isfinite(x[i])) x[i] = xp[i];
        x_[i] = x[i];
    }
    deriv(x_, u, gm, aq, f_);
    for (double& v : f_)
        if (!std::isfinite(v)) v = 0.0;
}

Outputs Filter::process(double input) {
    if (!std::isfinite(input))
        throw std::invalid_argument("SSM2045: non-finite audio input");
    fcPin_ += smoothing_ * (fcTarget_ - fcPin_);
    const double gm = gmNow();
    const double aq = qCurrent_ / config_.qOscillationCurrent;
    solveSample(input, gm, aq);

    Outputs out;
    out.twoPole = sat(x_[1], config_.vlT, config_.nonlinear) / 10.0;
    out.fourPole = sat(x_[3], config_.vlT, config_.nonlinear) / 10.0;
    return out;
}

// ---------------------------------------------------------------------------
// VCA / mixer
// ---------------------------------------------------------------------------

VcaMixer::VcaMixer(const VcaConfig& config) : config_(config) {
    validate();
}

void VcaMixer::validate() const {
    const auto finite = [](double v) { return std::isfinite(v); };
    if (!finite(config_.vcaGain) || config_.vcaGain <= 0.0 ||
        !finite(config_.vlVca) || config_.vlVca <= 0.0 ||
        !finite(config_.wmin) || config_.wmin < 0.0 || config_.wmin >= 0.5 ||
        !finite(config_.vt) || config_.vt <= 0.0 ||
        !finite(config_.mixLoadOhms) || config_.mixLoadOhms < 0.0 ||
        !finite(config_.iosVca) ||
        !finite(config_.controlReferenceVolts) ||
        !finite(config_.controlInternalOhms) || config_.controlInternalOhms < 0.0 ||
        !finite(config_.maxLinearCurrent) || config_.maxLinearCurrent <= 0.0)
        throw std::invalid_argument("SSM2045: invalid VCA configuration");
}

void VcaMixer::setLinearControlCurrents(double i16, double i15) {
    if (!std::isfinite(i16) || !std::isfinite(i15) || i16 < 0.0 || i15 < 0.0)
        throw std::invalid_argument("SSM2045: VCA control currents must be non-negative");
    if (i16 > config_.maxLinearCurrent || i15 > config_.maxLinearCurrent)
        throw std::invalid_argument("SSM2045: VCA control current above maximum");
    i16_ = i16;
    i15_ = i15;
}

void VcaMixer::setLinearControlVoltages(double v16, double v15, double seriesOhms) {
    if (!std::isfinite(v16) || !std::isfinite(v15) ||
        !std::isfinite(seriesOhms) || seriesOhms < 0.0)
        throw std::invalid_argument("SSM2045: invalid VCA control voltage");
    const double denominator = seriesOhms + config_.controlInternalOhms;
    const auto current = [&](double v) {
        if (denominator <= 0.0) return 0.0;
        const double i = (v - config_.controlReferenceVolts) / denominator;
        return std::clamp(i, 0.0, config_.maxLinearCurrent);
    };
    setLinearControlCurrents(current(v16), current(v15));
}

void VcaMixer::setExpoVolts(double volts) {
    if (!std::isfinite(volts))
        throw std::invalid_argument("SSM2045: non-finite EXPO voltage");
    expo_ = volts;
}

VcaOutputs VcaMixer::process(double in1, double in2) const {
    if (!std::isfinite(in1) || !std::isfinite(in2))
        throw std::invalid_argument("SSM2045: non-finite VCA input");
    const double w1 = std::clamp(
        config_.wmin + (1.0 - 2.0 * config_.wmin) * 0.5 *
                           (1.0 - std::tanh(expo_ / (2.0 * config_.vt))),
        config_.wmin, 1.0 - config_.wmin);
    const double w2 = 1.0 - w1;
    const double iv1 = 2.0 * w1 * i16_;
    const double iv2 = 2.0 * w2 * i15_;
    const double current =
        -config_.vcaGain * iv1 * config_.vlVca * std::tanh(in1 / config_.vlVca) +
        config_.vcaGain * iv2 * config_.vlVca * std::tanh(in2 / config_.vlVca) +
        config_.iosVca;
    VcaOutputs out;
    out.current = current;
    out.volts = current * config_.mixLoadOhms;
    out.weight1 = w1;
    return out;
}

// ---------------------------------------------------------------------------
// Chip
// ---------------------------------------------------------------------------

Chip::Chip(const FilterConfig& filter, const VcaConfig& vca)
    : filter_(filter), vca_(vca) {}

void Chip::reset() { filter_.reset(); }

void Chip::setSampleRate(double sampleRate) { filter_.setSampleRate(sampleRate); }

ChipOutputs Chip::process(double audioIn, double vcaIn1, double vcaIn2) {
    ChipOutputs out;
    out.filter = filter_.process(audioIn);
    out.vca = vca_.process(vcaIn1, vcaIn2);
    return out;
}

} // namespace ssm2045
