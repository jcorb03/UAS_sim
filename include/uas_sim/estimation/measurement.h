#pragma once

struct UAS_measurement {
    std::optional<double> x;
    std::optional<double> y;
    std::optional<double> v;
    std::optional<double> heading;
    double time;
};
