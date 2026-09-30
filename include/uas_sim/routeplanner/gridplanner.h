#pragma once
#include "uas_sim/map/CostMap.h"

struct PlanResult {
  bool found_path = false;
  std::vector<GridIndex> path;
  double total_cost = 0.0;
};

class GridPlanner {
public:
  GridPlanner();
  PlanResult plan(const Costmap& costmap,
    GridIndex start,
    GridIndex goal) const;
};