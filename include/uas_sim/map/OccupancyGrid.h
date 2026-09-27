#pragma once
#include "uas_sim/map/ObstacleMap.h"
#include "uas_sim/sensors/RangeSensor.h"
#include <cmath>

enum class OccupancyState {
  Unknown,
  Free,
  Occupied
};

struct OccupancyGrid {

  double resolution_;
  WorldBounds bounds;

  std::vector<std::vector<OccupancyState>> grid;

  OccupancyGrid(WorldBounds bounds_, double resolution)
    : resolution_(resolution),
    bounds(bounds_)
  {
    const std::size_t nx = static_cast<std::size_t>(
      std::ceil((bounds.max_x - bounds.min_x) / resolution));

    const std::size_t ny = static_cast<std::size_t>(
      std::ceil((bounds.max_y - bounds.min_y) / resolution));

    grid.resize(
      nx,
      std::vector<OccupancyState>(ny, OccupancyState::Unknown)
    );
  }

  std::pair<std::size_t, std::size_t>
    worldToGrid(const Waypoint& point) const {
    const auto n = static_cast<std::size_t>(
      (point.x - bounds.min_x) / resolution_);

    const auto m = static_cast<std::size_t>(
      (point.y - bounds.min_y) / resolution_);

    return { n,m };
  }

  void UpdateGrid(const UAS_state& state,
    const std::vector<RangeMeasurement>& measurements,
    double sensor_max_range) {
    
    Waypoint point;
    for (const RangeMeasurement& measurement : measurements) {
      
      for (double range = 0.0;
        range < measurement.range && range < sensor_max_range;
        range += resolution_) {

        point.x = state.x + range * std::sin(measurement.bearing);
        point.y = state.y + range * std::cos(measurement.bearing);

        //Nearest point
        std::pair<std::size_t, std::size_t> index = worldToGrid(point);

        grid.at(index.first).at(index.second) = OccupancyState::Free;
      }

      //Hit Targets
      if (std::isfinite(measurement.range)) {

        point.x = state.x + measurement.range * std::sin(measurement.bearing);
        point.y = state.y + measurement.range * std::cos(measurement.bearing);

        //Nearest point
        std::pair<std::size_t, std::size_t> index = worldToGrid(point);
        
        grid.at(index.first).at(index.second) = OccupancyState::Occupied;
      }
    }
  }
};