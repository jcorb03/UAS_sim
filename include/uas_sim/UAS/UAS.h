#pragma once
#include <optional>
#include <vector>
#include "uas_sim/UAS_structs.h"
#include "uas_sim/dynamics/dynamics.h"
#include "uas_sim/sensors/GPSSensor.h"
#include "uas_sim/control/Guidance.h"
#include "uas_sim/estimation/UASEstimator.h"
#include "uas_sim/map/OccupancyGrid.h"
#include "uas_sim/routeplanner/routeplanner.h"
#include "uas_sim/routeplanner/gridplanner.h"
#include "uas_sim/map/CostMap.h"

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
  UAS_state getTrueState() const;
  bool hasOccupancyGrid() const;
  OccupancyGrid getOccupancyGrid() const;
  bool PlannedStep(SimConfig sim_config, double sim_time);
  bool FrontierStep(SimConfig sim_config, double sim_time);
  
private:
  bool updateOccupancyMap(double timestep);
  bool activePathIsTraversable() const;
  bool replanUnknownMap();
  void advanceActivePath();
  GridIndex estimatedGridIndex() const;
  Waypoint gridCellCentre(GridIndex index) const;

  // Unknown-map navigation state
  std::optional<OccupancyGrid> occupancy_grid_;
  std::optional<RangeSensor> range_sensor_;
  std::optional<Costmap> cost_grid_;
  std::optional<GridPlanner> gridplanner_;
  double range_sensor_elapsed_ = 0.0;
  double range_sensor_update_period_ = 0.0;
  double inflation_radius_ = 3.0;
  double grid_path_tolerance_ = 1.0;
  bool replan_required_ = true;

  // Known-map planning data
  RoutePlanner planner_;
  ObstacleMap map_;

  // Mission goals remain persistent; active_path_ is replaced by replanning.
  std::vector<Waypoint> mission_waypoints_{};
  std::size_t mission_goal_index_ = 0;
  std::vector<Waypoint> active_path_{};
  std::vector<GridIndex> active_grid_path_{};

  // Used in both navigation modes
  PathFollower guidance_;
  Estimator estimator_;
  Dynamics dynamics_;
  UAS_state estimated_state_;
  GPSSensor gps_sensor_;
  UAS_measurement gps_measurement_;
  UAS_operating_constraints operating_constraints_;
  double waypoint_tolerance_ = 5.0;

  //Results
  std::vector<UAS_state> estimation_history_ = {};
  std::vector<double> time_history_ = {};
};
