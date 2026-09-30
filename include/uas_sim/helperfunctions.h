#pragma once

#include <cmath>
#include <algorithm>
#include "uas_sim/UAS_structs.h"

namespace helpers {
  template <typename T>

  double distance(T one, T two) {
    return std::sqrt((one.x - two.x) * (one.x - two.x)
      + (one.y - two.y) * (one.y - two.y));
  }

  inline double distance(const std::pair<double, double>& one,
    const std::pair<double, double>& two) {
    return std::sqrt((one.first - two.first) * (one.first - two.first)
      + (one.second - two.second) * (one.second - two.second));
  }

  template <typename T>

  double bearing(T target, T current) {
    double bearing = std::atan2(target.x - current.x, target.y - current.y);
    bearing = std::atan2(std::sin(bearing),
      std::cos(bearing));
    return bearing;
  }

  // Returns the forward intersection of a path segment and the lookahead
  // circle centred on the UAS.  If the segment does not intersect the circle,
  // the segment end is the most useful fallback target.
  inline Waypoint intercept(const Waypoint& drone, double lookahead_distance,
    const Waypoint& from, const Waypoint& to) {

    const double dx = to.x - from.x;
    const double dy = to.y - from.y;
    const double a = dx * dx + dy * dy;

    if (a == 0.0 || lookahead_distance <= 0.0) {
      return to;
    }

    const double fx = from.x - drone.x;
    const double fy = from.y - drone.y;
    const double b = 2.0 * (fx * dx + fy * dy);
    const double c = fx * fx + fy * fy
      - lookahead_distance * lookahead_distance;
    const double discriminant = b * b - 4.0 * a * c;

    if (discriminant < 0.0) {
      return to;
    }

    const double root = std::sqrt(discriminant);
    const double t1 = (-b - root) / (2.0 * a);
    const double t2 = (-b + root) / (2.0 * a);

    // Choose the furthest valid point along the segment.  This gives the
    // outward intersection when the segment begins within the lookahead
    // circle, which is the pure-pursuit target we want.
    double t = -1.0;
    if (t1 >= 0.0 && t1 <= 1.0) {
      t = t1;
    }
    if (t2 >= 0.0 && t2 <= 1.0) {
      t = std::max(t, t2);
    }

    if (t < 0.0) {
      return to;
    }

    return { from.x + t * dx, from.y + t * dy };
  }
}
