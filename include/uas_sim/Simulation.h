#pragma once
#include "uas_sim/UAS_structs.h"
#include "uas_sim/dynamics/UAS.h"
#include "uas_sim/sensors/GPSSensor.h"
#include "uas_sim/control/Guidance.h"
#include "uas_sim/estimation/UASEstimator.h"
#include <vector>
#include <iostream>

struct SimConfig {
  double sim_length;
  double timestep;
  double waypoint_tolerance_m;
};

class Simulation {
  public:
    Simulation(const UAS_operating_constraints& operating_constraints,
      const std::vector<Waypoint>& waypoints,
      UAS_state initial_state, GPSSensor gps_sensor, SimConfig sim_config);
    bool run();
    bool getObjective(const UAS_state& state_estimate);
    void initialiseKalman(KalmanFilterState kalman);
    std::vector<double> getTimeHistory() const;
    std::vector<UAS_state> getStateEstimateHistory() const;

private:
  GuidanceModule guidance_;
  Estimator estimator_;
  UAS uas_;
  UAS_state estimated_state_;
  GPSSensor gps_sensor_;
  UAS_measurement gps_measurement_;
  std::vector<Waypoint> waypoints_;
  UAS_operating_constraints operating_constraints_;
  double sim_time_ = 0;
  SimConfig sim_config_;
  std::vector<UAS_state> estimation_history_ = {};
  std::vector<double> time_history_ = {};
};