#pragma once
#include "uas_sim/map/CostMap.h"
#include <iostream>
#include <vector>
#include <utility>

struct PlanResult {
  bool found_path = false;
  std::vector<GridIndex> path;
  double total_cost = 0.0;
};

struct Node {
  GridIndex index{};
  double g_cost = std::numeric_limits<double>::infinity(); // Cost from start to this node
  double h_cost = 0.0; // Heuristic cost from this node to goal
  double f_cost() const { return g_cost + h_cost; } // Total cost
  Node* parent = nullptr; // Pointer to parent node for path reconstruction
  bool open = true;
};

class GridPlanner {
public:
  GridPlanner();
  bool Plan(const Costmap& costmap,
    GridIndex start,
    GridIndex goal);
  PlanResult GetRoute() const;
  double Heuristic(GridIndex from, GridIndex goal,
    double resolution);

private:
  PlanResult route_;
  std::vector<std::pair<int,int>> neighbour_diffs_ = 
  { {1,1},{1,0},{1,-1},{0,1},
    {0,-1},{-1,1},{-1,0},{-1,-1} };
};