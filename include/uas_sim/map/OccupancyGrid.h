#pragma once
#include "uas_sim/map/ObstacleMap.h"
#include "uas_sim/sensors/RangeSensor.h"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <utility>
#include <vector>

// CMake sets this to the source directory.  The fallback keeps this header
// usable from Visual Studio's normal compiler and IntelliSense as well.
#ifndef UAS_SIM_PROJECT_DIR
#define UAS_SIM_PROJECT_DIR "."
#endif


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

  std::pair<double, double>
    gridtoWorld(std::pair<std::size_t, std::size_t> index) const {
    
    const auto x = 
      static_cast<double>(index.first) * resolution_ + bounds.min_x;
    const auto y = 
      static_cast<double>(index.second) * resolution_ + bounds.min_y;
    
    return { x,y };
  }

  bool contains(const Waypoint& point) const {
    return point.x >= bounds.min_x && point.x < bounds.max_x &&
      point.y >= bounds.min_y && point.y < bounds.max_y;
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

        if (!contains(point)) {
          break;
        }

        //Nearest point
        std::pair<std::size_t, std::size_t> index = worldToGrid(point);

        grid.at(index.first).at(index.second) = OccupancyState::Free;
      }

      //Hit Targets
      if (std::isfinite(measurement.range)) {

        point.x = state.x + measurement.range * std::sin(measurement.bearing);
        point.y = state.y + measurement.range * std::cos(measurement.bearing);

        if (!contains(point)) {
          continue;
        }

        //Nearest point
        std::pair<std::size_t, std::size_t> index = worldToGrid(point);
        
        grid.at(index.first).at(index.second) = OccupancyState::Occupied;
      }
    }
  }

  void WriteToCsv() const {
    const std::filesystem::path grid_path =
      std::filesystem::path{ UAS_SIM_PROJECT_DIR } / "results" / "grid.csv";
    std::ofstream grid_file(grid_path);

    grid_file << "X,Y,Status\n";

    for (std::size_t i = 0; i<grid.size(); ++i ){
      for (std::size_t j = 0; j < grid.at(0).size(); ++j) {
        std::pair<double, double> coords = gridtoWorld({ i,j });

        int status = static_cast<int>(grid.at(i).at(j));

        grid_file << coords.first << ","
          << coords.second << ","
          << status  << "\n";

      }
    }
  }
};
