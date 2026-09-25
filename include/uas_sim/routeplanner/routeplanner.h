#pragma once
#include <vector>
#include "uas_sim/UAS_structs.h"
#include "uas_sim/map/ObstacleMap.h"
#include <random>

enum class RoutePlanningAlgos {
  SIMPLE,
  RRT_STAR
};

struct RRT_Node {
  Waypoint coords;
  int number,parent;
  double cost;
};


class RoutePlanner {
public:
  RoutePlanner(std::vector<Waypoint> checkpoints, ObstacleMap obstacle_map);
  
  void SetPlanningAlgorithm(RoutePlanningAlgos algorithm);

  std::vector<Waypoint> Plan(const Waypoint& start);

private:

  std::vector<Waypoint> plan_RRT_route(Waypoint start);

  RoutePlanningAlgos algorithm_ = RoutePlanningAlgos::SIMPLE;
  std::vector<Waypoint> checkpoints_;
  std::vector<Waypoint> route_;
  ObstacleMap obstacle_map_;

};

