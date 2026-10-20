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
    if (argc != 10) {
        std::cerr << "usage: chip_demo FC_HZ Q_AMP RATE SECONDS SINE_HZ "
                     "I16_UA I15_UA EXPO_MV OUT.csv\n";
        return 2;
    }
    const double fc = number(argv[1], "cutoff");
    const double q = number(argv[2], "Q current");
    const double rate = number(argv[3], "sample rate");
    const double seconds = number(argv[4], "duration");
    const double sine = number(argv[5], "sine frequency");
    const double i16 = number(argv[6], "I16") * 1e-6;
    const double i15 = number(argv[7], "I15") * 1e-6;
    const double expo = number(argv[8], "EXPO") * 1e-3;
    if (q < 0.0 || seconds <= 0.0 || sine < 0.0 || i16 < 0.0 || i15 < 0.0)
        throw std::invalid_argument("invalid negative parameter");

    ssm2045::FilterConfig cfg;
    cfg.sampleRate = rate;
    cfg.fcSmoothingSeconds = 0.0;
    ssm2045::Chip chip(cfg);
    chip.setCutoffHz(fc);
    chip.setQCurrent(q);
    chip.setLinearControlCurrents(i16, i15);
    chip.setExpoVolts(expo);

    std::ofstream csv(argv[9]);
    if (!csv) throw std::runtime_error("cannot create output CSV");
    csv << "time,two_pole,four_pole,mix_volts\n";
    const int n = int(seconds * rate);
    for (int i = 0; i < n; ++i) {
        const double t = i / rate;
        const double in = 0.075 * std::sin(kTwoPi * sine * t);
        const auto filter = chip.filter().process(in);
        // Demo routing: 4-pole into VCA1, 2-pole into VCA2.
        const auto vca = chip.vca().process(filter.fourPole, filter.twoPole);
        csv << t << "," << filter.twoPole << "," << filter.fourPole
            << "," << vca.volts << "\n";
    }
    if (!csv) throw std::runtime_error("write error");
    std::cerr << "wrote " << argv[9] << "\n";
    return 0;
} catch (const std::exception& e) {
    std::cerr << "FAIL: " << e.what() << "\n";
    return 1;
}
