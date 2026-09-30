#include "uas_sim/map/CostMap.h"

Costmap::Costmap(const OccupancyGrid& occupancy_grid,
  double inflation_radius)
  : resolution_(occupancy_grid.resolution_),
  bounds(occupancy_grid.bounds),
  inflation_radius_(inflation_radius),
  costs(
    occupancy_grid.grid.size(),
    std::vector<double>(
      occupancy_grid.grid.at(0).size(),
      blocked_cost
    )
  )
{
  //Give free cells free cost
  for (int i = 0; i < occupancy_grid.grid.size(); ++i) {
    for (int j = 0; j < occupancy_grid.grid.front().size(); ++j) {
      if (occupancy_grid.getState(i, j) == OccupancyState::Free) {
        costs.at(i).at(j) = free_cost;
      }
    }
  }

  //Find occupied cells
  std::vector<std::pair<int, int>> occupied_cells{};
  for (int i = 0; i < occupancy_grid.grid.size(); ++i) {
    for (int j = 0; j < occupancy_grid.grid.front().size(); ++j) {
      if (occupancy_grid.getState(i, j) == OccupancyState::Occupied) {
        occupied_cells.push_back({ i,j });
      }
    }
  }
  int max_cells = std::ceil(inflation_radius / resolution_);
  
  //Give high cost to cells in inflation radius of obstacles
  const int radius_cells = static_cast<int>(
    std::ceil(inflation_radius / resolution_)
    );

  constexpr double max_inflation_cost = 100.0;

  for (const auto& obstacle : occupied_cells) {
    const int min_x = std::max(
      0, static_cast<int>(obstacle.first) - radius_cells
    );
    const int max_x = std::min(
      static_cast<int>(costs.size()) - 1,
      static_cast<int>(obstacle.first) + radius_cells
    );

    const int min_y = std::max(
      0, static_cast<int>(obstacle.second) - radius_cells
    );
    const int max_y = std::min(
      static_cast<int>(costs.front().size()) - 1,
      static_cast<int>(obstacle.second) + radius_cells
    );

    for (int x = min_x; x <= max_x; ++x) {
      for (int y = min_y; y <= max_y; ++y) {
        const double dx =
          (x - static_cast<int>(obstacle.first)) * resolution_;
        const double dy =
          (y - static_cast<int>(obstacle.second)) * resolution_;
        const double distance = std::hypot(dx, dy);

        if (distance > inflation_radius) {
          continue;
        }

        double& cost = costs.at(x).at(y);

        // Preserve blocked unknown/occupied cells.
        if (!std::isfinite(cost)) {
          continue;
        }

        const double normalised_distance =
          distance / inflation_radius;

        const double inflation_cost =
          max_inflation_cost * (1.0 - normalised_distance);

        cost = std::max(cost, free_cost + inflation_cost);
      }
    }
  }
}

bool Costmap::isTraversable(GridIndex cell) const {
  if (cell.i < 0 || cell.j < 0 ||
    cell.i >= static_cast<int>(costs.size()) ||
    cell.j >= static_cast<int>(costs.front().size())) {
    return false;
  }

  return std::isfinite(costs.at(cell.i).at(cell.j));
}

double Costmap::getCost(GridIndex cell) const {
  if (!isTraversable(cell)) {
    return blocked_cost;
  }

  return costs.at(cell.i).at(cell.j);
}


