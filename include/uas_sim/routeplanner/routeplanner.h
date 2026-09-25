#pragma once
#include "UAS_structs.h"

enum class RoutePlanningAlgos {
  SIMPLE,
  RRT_STAR
};

class RoutePlanner {
public:
  RoutePlanner(std::vector<Waypoints> checkpoints);
  
  SetPlanningAlgorithm(RoutePlanningAlgos algorithm);

  Initialise();

private:
  RoutePlanningAlgos algorithm_ = RoutePlanningAlgos::SIMPLE;
  std::vector<Waypoints> checkpoints_;

};