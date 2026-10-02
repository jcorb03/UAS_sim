#pragma once
#include "uas_sim/map/OccupancyGrid.h"
#include "uas_sim/sensors/RangeSensor.h"
#include <cmath>
#include <limits>
#include <vector>
#include "uas_sim/helperfunctions.h"

// Used for navigation based on the OccupancyGrid

struct GridIndex {
  int i, j;

  bool operator==(const GridIndex&) const = default;
};

struct Costmap {
    
    std::vector<std::vector<double>> costs;
    WorldBounds bounds;
    double inflation_radius_ = 5.0;
    static constexpr double free_cost = 1.0;
    static constexpr double blocked_cost =
        std::numeric_limits<double>::infinity();
    double resolution_;

    explicit Costmap(const OccupancyGrid& occupancy_grid,
        double inflation_radius);

    bool isTraversable(GridIndex cell) const;
    double getCost(GridIndex cell) const;

};
