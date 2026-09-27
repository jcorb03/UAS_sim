#include <iostream>
#include "uas_sim/UAS_structs.h"
#include "uas_sim/Simulation.h"
#include "uas_sim/data/make_csv.h"
#include "uas_sim/map/ObstacleMap.h"
#include "uas_sim/routeplanner/routeplanner.h"
#include "uas_sim/courses/course1.h"
#include "uas_sim/SampleSimulations.h"

int main() {
  
  //Choose Simulation from SimulationCatalogue()

  SimulationCatalogue::ObstacleCourseNavigation();

  return 0;
}
