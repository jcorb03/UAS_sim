#pragma once
#include <vector>
#include "uas_sim/UAS_structs.h"
#include "uas_sim/dynamics/dynamics.h"
#include "uas_sim/sensors/GPSSensor.h"
#include "uas_sim/control/Guidance.h"
#include "uas_sim/estimation/UASEstimator.h"

class UAS {
public:
  UAS(const UAS_operating_constraints& operating_constraints,
    const std::vector<Waypoint>& waypoints,
    UAS_state initial_state, GPSSensor gps_sensor);

  void initialiseKalman(const KalmanFilterState& kalman);
  void setWaypointTolerance(SimConfig sim_config);
  bool getObjective();
  bool step(SimConfig sim_config, double sim_time);
  std::vector<Waypoint> getWaypoints() const;
  UAS_state getEstimatedState() const;
  
private:
  PathFollower guidance_;
  Estimator estimator_;
  Dynamics dynamics_;
  UAS_state estimated_state_;
  GPSSensor gps_sensor_;
  UAS_measurement gps_measurement_;
  std::vector<Waypoint> waypoints_{};
  UAS_operating_constraints operating_constraints_;
  std::vector<UAS_state> estimation_history_ = {};
  std::vector<double> time_history_ = {};
  double waypoint_tolerance_ = 5.0;
};
