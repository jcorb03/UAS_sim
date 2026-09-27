#pragma once
#include <vector>
#include "uas_sim/UAS_structs.h"
#include "uas_sim/dynamics/dynamics.h"
#include "uas_sim/sensors/GPSSensor.h"
#include "uas_sim/control/Guidance.h"
#include "uas_sim/estimation/UASEstimator.h"
#include "uas_sim\map\OccupancyGrid.h"
#include "uas_sim\routeplanner\routeplanner.h"

class UAS {
public:
  UAS(
    const UAS_operating_constraints& constraints,
    const std::vector<Waypoint>& checkpoints,
    const UAS_state& initial_state,
    const GPSSensor& gps_sensor,
    FollowerMethod follower_method,
    const RoutePlanner& route_planner,
    std::optional<RangeSensorConfig> range_sensor_config = std::nullopt,
    bool use_occupancy_grid = false,
    double occupancy_grid_resolution = 1.0
  );

  void initialiseKalman(const KalmanFilterState& kalman);
  void setWaypointTolerance(SimConfig sim_config);
  bool getObjective();
  bool step(SimConfig sim_config, double sim_time);
  std::vector<Waypoint> getWaypoints() const;
  void planRoute();
  UAS_state getEstimatedState() const;
  bool hasOccupancyGrid() const;
  OccupancyGrid getOccupancyGrid() const;
  
  
private:
  std::optional<OccupancyGrid> occupancy_grid_;
  std::optional<RangeSensor> range_sensor_;
  RoutePlanner planner_;
  ObstacleMap map_;
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
