#pragma once
#include <optional>

struct UAS_measurement {
    std::optional<double> x = std::nullopt;
    std::optional<double> y = std::nullopt;
    std::optional<double> v = std::nullopt;
    std::optional<double> heading = std::nullopt;
    double time = -1.0;
    bool valid = false;
};

