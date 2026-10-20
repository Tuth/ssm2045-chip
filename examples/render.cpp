#include "ssm2045/chip.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {
constexpr double kTwoPi = 6.283185307179586476925286766559;

double number(const char* text, const char* what) {
    try {
        std::size_t used = 0;
        const double v = std::stod(text, &used);
        if (used != std::string(text).size() || !std::isfinite(v))
            throw std::invalid_argument("bad number");
        return v;
    } catch (...) {
        throw std::invalid_argument(std::string("invalid ") + what);
    }
}
} // namespace

int main(int argc, char** argv) try {
    if (argc != 7) {
        std::cerr << "usage: render FC_HZ Q_AMP RATE SECONDS SINE_HZ OUT.csv\n"
                     "       SINE_HZ=0 renders an impulse instead of a sine.\n";
        return 2;
    }
    const double fc = number(argv[1], "cutoff");
    const double q = number(argv[2], "Q current");
    const double rate = number(argv[3], "sample rate");
    const double seconds = number(argv[4], "duration");
    const double sine = number(argv[5], "sine frequency");
    if (q < 0.0 || seconds <= 0.0 || sine < 0.0)
        throw std::invalid_argument("Q, duration and sine frequency must be non-negative");

    ssm2045::FilterConfig cfg;
    cfg.sampleRate = rate;
    cfg.fcSmoothingSeconds = 0.0;
    ssm2045::Filter f(cfg);
    f.setCutoffHz(fc);
    f.setQCurrent(q);

    std::ofstream csv(argv[6]);
    if (!csv) throw std::runtime_error("cannot create output CSV");
    csv << "time,two_pole,four_pole\n";
    const int n = int(seconds * rate);
    for (int i = 0; i < n; ++i) {
        const double t = i / rate;
        const double in = sine > 0.0 ? 0.075 * std::sin(kTwoPi * sine * t)
                                     : (i == 0 ? 0.1 : 0.0);
        const auto y = f.process(in);
        csv << t << "," << y.twoPole << "," << y.fourPole << "\n";
    }
    if (!csv) throw std::runtime_error("write error");
    std::cerr << "wrote " << argv[6] << "\n";
    return 0;
} catch (const std::exception& e) {
    std::cerr << "FAIL: " << e.what() << "\n";
    return 1;
}
