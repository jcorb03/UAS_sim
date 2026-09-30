#pragma once
#include "uas_sim/map/ObstacleMap.h"
#include "uas_sim/sensors/RangeSensor.h"
#include <cmath>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <utility>
#include <vector>




#ifndef UAS_SIM_PROJECT_DIR
#define UAS_SIM_PROJECT_DIR "."
#endif

enum class OccupancyState {
  Unknown,
  Free,
  Occupied
};


struct OccupancyCell {
  double log_odds = 0.0;
};

struct OccupancyGrid {
  double free_update = -0.4;
  double occupied_update = 0.85;
  double min_log_odds = -4.0;
  double max_log_odds = 4.0;
  double resolution_;
  WorldBounds bounds;

  std::vector<std::vector<OccupancyCell>> grid;

  OccupancyGrid(WorldBounds bounds_, double resolution);

  std::pair<std::size_t, std::size_t>
    worldToGrid(const Waypoint& point) const;

  std::pair<double, double>
    gridtoWorld(std::pair<std::size_t, std::size_t> index) const;

  bool contains(const Waypoint& point) const;

  void UpdateGrid(const UAS_state& state,
    const std::vector<RangeMeasurement>& measurements,
    double sensor_max_range);

  OccupancyState getState(std::size_t x, std::size_t y) const;

  void WriteToCsv() const;
};
