#pragma once
#include "uas_sim/UAS_structs.h"
#include "uas_sim/dynamics/dynamics.h"
#include "uas_sim/sensors/GPSSensor.h"
#include "uas_sim/control/Guidance.h"
#include "uas_sim/estimation/UASEstimator.h"

class UAS {
public:
  UAS(const UAS_operating_constraints& operating_constraints,
    const std::vector<Waypoint>& waypoints,
    UAS_state initial_state, GPSSensor gps_sensor, Estimator estimator);

  bool getObjective(const UAS_state& state_estimate);
  void initialiseKalman(KalmanFilterState kalman);
  double getWaypointTolerance();
  std::vector<Waypoint> getWaypoints();
  


private:
  GuidanceModule guidance_;
  Estimator estimator_;
  Dynamics dynamics_;
  UAS_state estimated_state_;
  GPSSensor gps_sensor_;
  UAS_measurement gps_measurement_;
  std::vector<Waypoint> waypoints_;
  UAS_operating_constraints operating_constraints_;
  double waypoint_tolerance_m = 0;

};