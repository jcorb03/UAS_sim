#pragma once

#include <cmath>

namespace helpers {
  template <typename T>

  double distance(T one, T two) {
    return std::sqrt((one.x - two.x) * (one.x - two.x)
      + (one.y - two.y) * (one.y - two.y));
  }

  template <typename T>

  double bearing(T target, T current) {
    double bearing = std::atan2(target.x - current.x, target.y - current.y);
    bearing = std::atan2(std::sin(bearing),
      std::cos(bearing));
    return bearing;
  }
}