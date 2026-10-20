#include "ssm2045/chip.h"

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdio>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kTwoPi = 2.0 * kPi;

void require(bool condition, const char* what) {
    if (!condition) throw std::runtime_error(std::string("SSM2045 test failed: ") + what);
}

bool close(double a, double b, double rel = 1e-6, double abs = 1e-12) {
    return std::abs(a - b) <= abs + rel * std::max(std::abs(a), std::abs(b));
}

ssm2045::FilterConfig filterConfig(double rate = 192000.0) {
    ssm2045::FilterConfig c;
    c.sampleRate = rate;
    c.fcSmoothingSeconds = 0.0;
    c.offsets = false;
    c.nonlinear = true;
    return c;
}

double alpha(double c1, double c2) { return std::sqrt(10.0 * c2 / c1); }

std::complex<double> analyticH2(double freq, double fc, double qInv, double rate) {
    const double w = (2.0 * rate) * std::tan(kPi * freq / rate);
    const double w0 = kTwoPi * fc;
    return -w0 * w0 / (w0 * w0 - w * w + std::complex<double>(0.0, qInv * w0 * w));
}

std::complex<double> measure(ssm2045::Filter& f, double freq, double amplitude,
                             double rate, double settle, int cycles, bool fourPole) {
    const int nSettle = int(settle * rate);
    for (int n = 0; n < nSettle; ++n)
        f.process(amplitude * std::sin(kTwoPi * freq * n / rate));
    const int N = std::max(2048, int(std::llround(rate * cycles / freq)));
    std::complex<double> out(0.0, 0.0), in(0.0, 0.0);
    for (int n = 0; n < N; ++n) {
        const double x = amplitude * std::sin(kTwoPi * freq * n / rate);
        const auto y = f.process(x);
        const auto rot = std::exp(std::complex<double>(0.0, -kTwoPi * freq * n / rate));
        out += (fourPole ? y.fourPole : y.twoPole) * rot;
        in += x * rot;
    }
    return out / in;
}

void testFrequencyResponse() {
    for (double rate : {48000.0, 96000.0, 192000.0}) {
        auto cfg = filterConfig(rate);
        ssm2045::Filter f(cfg);
        f.setCutoffHz(1000.0);
        require(close(f.cutoffHz(), 1000.0, 1e-9), "cutoff inversion");
        const double a1 = alpha(cfg.caps.c1, cfg.caps.c2);
        const double a2 = alpha(cfg.caps.c3, cfg.caps.c4);
        const double fc2 = 1000.0 * std::sqrt((cfg.caps.c1 * 10.0 * cfg.caps.c2) /
                                              (cfg.caps.c3 * 10.0 * cfg.caps.c4));
        for (double freq : {100.0, 500.0, 1000.0, 2000.0}) {
            auto two = f;
            const auto m2 = measure(two, freq, 0.005, rate, 0.3, 60, false);
            const auto h2 = analyticH2(freq, 1000.0, a1, rate);
            require(std::abs(m2 - h2) < 3e-2 * (1.0 + std::abs(h2)), "2-pole response");
            auto four = f;
            const auto m4 = measure(four, freq, 0.005, rate, 0.4, 60, true);
            const auto h4 = analyticH2(freq, 1000.0, a1, rate) *
                            analyticH2(freq, fc2, a2, rate);
            require(std::abs(m4 - h4) < 5e-2 * (1.0 + std::abs(h4)), "4-pole response");
        }
    }
}

void testDcPolarity() {
    auto cfg = filterConfig();
    ssm2045::Filter f(cfg);
    f.setCutoffHz(1000.0);
    ssm2045::Outputs out;
    for (int n = 0; n < 40000; ++n) out = f.process(0.1);
    require(close(out.twoPole, -0.1, 1e-4, 1e-5), "2-pole DC inversion");
    require(close(out.fourPole, 0.1, 1e-4, 1e-5), "4-pole DC polarity");
}

void testCutoffAndSmoothing() {
    auto cfg = filterConfig();
    ssm2045::Filter f(cfg);
    f.setCutoffHz(1000.0);
    require(close(f.cutoffHz(), 1000.0, 1e-9), "1 kHz cutoff");
    f.setCutoffHz(2000.0);
    require(close(f.cutoffHz(), 2000.0, 1e-9), "2 kHz cutoff");

    auto smoothCfg = filterConfig();
    smoothCfg.fcSmoothingSeconds = 500e-6;
    ssm2045::Filter s(smoothCfg);
    s.setCutoffHz(1000.0);
    for (int n = 0; n < int(0.02 * 192000.0); ++n) s.process(0.0);
    require(std::abs(s.cutoffHz() - 1000.0) < 5.0, "smoothed cutoff settles at 1 kHz");
    s.setCutoffHz(2000.0);
    require(std::abs(s.cutoffHz() - 1000.0) < 20.0, "cutoff does not jump");
    for (int n = 0; n < int(0.02 * 192000.0); ++n) s.process(0.0);
    require(std::abs(s.cutoffHz() - 2000.0) < 20.0, "cutoff approaches 2 kHz");
}

void testQAndOscillation() {
    auto cfg = filterConfig();
    ssm2045::Filter quiet(cfg);
    quiet.setQCurrent(0.0);
    quiet.setCutoffHz(1000.0);
    quiet.seed(1e-6);
    double quietPeak = 0.0;
    for (int n = 0; n < 96000; ++n) {
        quietPeak = std::max(quietPeak, std::abs(quiet.process(0.0).fourPole));
    }
    require(quietPeak < 1e-5, "zero Q current is stable");

    auto oscCfg = filterConfig();
    ssm2045::Filter osc(oscCfg);
    osc.setQCurrent(180e-6);
    osc.setCutoffHz(1000.0);
    osc.seed(1e-3);
    std::vector<double> tail;
    double peak = 0.0;
    for (int n = 0; n < int(0.6 * 192000.0); ++n) {
        const double y = osc.process(0.0).fourPole;
        peak = std::max(peak, std::abs(y));
        if (n > int(0.4 * 192000.0)) tail.push_back(y);
    }
    double rms = 0.0;
    for (double y : tail) rms += y * y;
    rms = std::sqrt(rms / std::max<std::size_t>(1, tail.size()));
    require(rms > 1e-3, "self-oscillation above onset");
    require(peak <= 0.71, "self-oscillation bounded by the output tanh");

    auto hotCfg = filterConfig();
    ssm2045::Filter hot(hotCfg);
    hot.setQCurrent(1e-3);
    hot.setCutoffHz(1000.0);
    hot.seed(1e-3);
    double hotPeak = 0.0;
    for (int n = 0; n < 48000; ++n) {
        hotPeak = std::max(hotPeak, std::abs(hot.process(0.0).fourPole));
    }
    require(std::isfinite(hotPeak) && hotPeak <= 0.71, "high Q current remains bounded");
}

void testStabilitySweep() {
    std::mt19937 rng(0x2045u);
    std::uniform_real_distribution<double> noise(-0.15, 0.15);
    for (double fc : {20.0, 100.0, 1000.0, 5000.0, 10000.0, 20000.0}) {
        for (double q : {0.0, 50e-6, 120e-6, 150e-6, 200e-6, 1e-3}) {
            auto cfg = filterConfig();
            ssm2045::Filter f(cfg);
            f.setQCurrent(q);
            f.setCutoffHz(fc);
            double peak = 0.0;
            for (int n = 0; n < int(0.05 * 192000.0); ++n) {
                const double y = f.process(noise(rng)).fourPole;
                require(std::isfinite(y), "finite stability-sweep output");
                peak = std::max(peak, std::abs(y));
            }
            require(peak <= 0.71, "bounded stability-sweep output");
        }
    }
}

void testReset() {
    auto cfg = filterConfig();
    ssm2045::Filter a(cfg), b(cfg);
    a.setCutoffHz(2000.0);
    b.setCutoffHz(2000.0);
    for (int n = 0; n < 2000; ++n) a.process(0.1 * std::sin(0.01 * n));
    a.reset();
    for (int n = 0; n < 2000; ++n) {
        const double in = n == 0 ? 0.25 : 0.0;
        const auto x = a.process(in);
        const auto y = b.process(in);
        require(close(x.twoPole, y.twoPole, 1e-10, 1e-10), "reset 2-pole");
        require(close(x.fourPole, y.fourPole, 1e-10, 1e-10), "reset 4-pole");
    }
}

void testVca() {
    ssm2045::VcaConfig cfg;
    ssm2045::VcaMixer v(cfg);
    v.setLinearControlCurrents(500e-6, 500e-6);
    v.setExpoVolts(0.0);
    const auto centre = v.process(0.02, 0.0);
    const double expected = -cfg.vcaGain * 500e-6 * cfg.vlVca *
                            std::tanh(0.02 / cfg.vlVca) + cfg.iosVca;
    require(close(centre.current, expected, 1e-9, 1e-9), "VCA centre current");
    require(close(centre.volts, centre.current * cfg.mixLoadOhms, 1e-12, 1e-15),
            "VCA load conversion");
    const double tiny = 1e-3;
    const auto small = v.process(tiny, 0.0);
    const double gm = -(small.current - cfg.iosVca) / tiny;
    require(std::abs(gm - 4.085e-3) < 2e-6, "VCA centre gm");

    v.setExpoVolts(0.25);
    const auto high = v.process(0.02, 0.0);
    v.setExpoVolts(-0.25);
    const auto low = v.process(0.02, 0.0);
    const double gainHigh = std::abs(high.current - cfg.iosVca);
    const double gainLow = std::abs(low.current - cfg.iosVca);
    const double ratio = gainLow / gainHigh;
    require(ratio > 900.0 && ratio < 1100.0, "EXPO 60 dB crossfade");

    v.setExpoVolts(0.0);
    const auto off = v.process(0.0, 0.0);
    require(close(off.current, cfg.iosVca, 1e-12, 1e-12), "VCA offset current");

    ssm2045::VcaMixer volts(cfg);
    volts.setLinearControlVoltages(5.0, 5.0, 10e3);
    require(volts.linearCurrent1() > 460e-6 && volts.linearCurrent1() < 470e-6,
            "VCA voltage-to-current helper");
}

void testChip() {
    ssm2045::FilterConfig fc;
    fc.fcSmoothingSeconds = 0.0;
    ssm2045::Chip chip(fc);
    chip.setCutoffHz(1000.0);
    chip.setQCurrent(0.0);
    chip.setLinearControlCurrents(500e-6, 500e-6);
    chip.setExpoVolts(0.0);
    auto out = chip.process(0.01, 0.01, 0.0);
    require(std::isfinite(out.filter.twoPole) && std::isfinite(out.vca.current),
            "chip combined output");
    require(std::abs(out.vca.current) > 1e-6, "chip VCA active");
}

void testValidation() {
    const auto rejects = [](auto fn, const char* what) {
        bool thrown = false;
        try { fn(); } catch (const std::invalid_argument&) { thrown = true; }
        require(thrown, what);
    };
    ssm2045::FilterConfig c;
    c.sampleRate = 0.0; rejects([&] { ssm2045::Filter f(c); }, "zero sample rate");
    c = {}; c.caps.c1 = 0.0; rejects([&] { ssm2045::Filter f(c); }, "zero C1");
    c = {}; c.qOscillationCurrent = 0.0; rejects([&] { ssm2045::Filter f(c); }, "zero Q onset");
    c = {}; c.fcSmoothingSeconds = -1.0; rejects([&] { ssm2045::Filter f(c); }, "negative smoothing");
    ssm2045::Filter f;
    rejects([&] { f.setCutoffHz(0.0); }, "zero cutoff");
    rejects([&] { f.setCutoffHz(0.49 * f.sampleRate()); }, "cutoff at Nyquist");
    rejects([&] { f.setQCurrent(-1.0); }, "negative Q current");
    rejects([&] { f.setFcPinVolts(std::numeric_limits<double>::quiet_NaN()); }, "NaN FC");
    rejects([&] { f.seed(std::numeric_limits<double>::quiet_NaN()); }, "NaN seed");
    rejects([&] { f.process(std::numeric_limits<double>::quiet_NaN()); }, "NaN input");
    rejects([&] { f.setSampleRate(-1.0); }, "negative sample rate");

    ssm2045::VcaMixer v;
    rejects([&] { v.setLinearControlCurrents(-1.0, 0.0); }, "negative VCA current");
    rejects([&] { v.setLinearControlCurrents(501e-6, 0.0); }, "VCA current over max");
    rejects([&] { v.setExpoVolts(std::numeric_limits<double>::infinity()); }, "infinite EXPO");
    rejects([&] { v.process(std::numeric_limits<double>::infinity(), 0.0); }, "infinite VCA input");
}
} // namespace

int main() try {
    testFrequencyResponse();
    testDcPolarity();
    testCutoffAndSmoothing();
    testQAndOscillation();
    testStabilitySweep();
    testReset();
    testVca();
    testChip();
    testValidation();
    std::printf("PASS: SSM2045 full chip (VCF + VCA); implicit 4-state filter, "
                "Q/Ga, FC law, VCA/EXPO, reset, stability, validation\n");
    return 0;
} catch (const std::exception& e) {
    std::fprintf(stderr, "FAIL: %s\n", e.what());
    return 1;
}
