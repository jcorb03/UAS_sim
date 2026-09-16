#pragma once
#include "uas_sim/UAS_structs.h"
#include "uas_sim/dynamics/dynamics.h"
#include "uas_sim/sensors/GPSSensor.h"
#include "uas_sim/control/Guidance.h"
#include "uas_sim/estimation/UASEstimator.h"
#include "uas_sim/UAS/UAS.h"
#include <vector>
#include <iostream>



class Simulation {
  public:
    Simulation(std::vector<UAS> uas_s , SimConfig sim_config);
    bool run();
    std::vector<double> getTimeHistory() const;
    std::vector<UAS_state> getStateEstimateHistory() const;
    

private:
  std::vector<UAS> UASs_;
  double sim_time_ = 0;
  SimConfig sim_config_;
  std::vector<std::vector<Waypoint>> waypoints_ = {};
  std::vector<UAS_state> estimation_history_ = {};
  std::vector<double> time_history_ = {};
  std::vector<bool> UAS_statuses = {};
};