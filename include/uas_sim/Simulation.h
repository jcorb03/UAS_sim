#pragma once
#include "uas_sim/UAS_structs.h"
#include "uas_sim/dynamics/UAS.h"
#include "uas_sim/sensors/GPSSensor.h"
#include "uas_sim/control/Guidance.h"
#include "uas_sim/estimation/UASEstimator.h"
#include <vector>


class Simulation {
  public:
    Simulation(const UAS_operating_constraints& operating_constraints,
      const std::vector<Waypoint>& waypoints,
      UAS_state initial_state, GPSSensor gps_sensor);
    void run(double sim_time, double timestep);

private:
  // Guidance module Guidance;
  // Estimation module Estimator;
  UAS uas_;
  UAS_state estimated_state_;
  UAS_state true_state_;
  GPSSensor gps_sensor_;
  std::vector<Waypoint> waypoints_;
  UAS_operating_constraints operating_constraints_;
};