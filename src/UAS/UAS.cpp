#include "uas_sim/UAS/UAS.h"

UAS::UAS(
  const UAS_operating_constraints& operating_constraints,
  const std::vector<Waypoint>& checkpoints,
  const UAS_state& initial_state,
  const GPSSensor& gps_sensor,
  FollowerMethod follower_method,
  const RoutePlanner& planner,
  std::optional<RangeSensorConfig> range_sensor_config,
  bool use_occupancy_grid,
  double occupancy_grid_resolution
) :
  operating_constraints_(operating_constraints), waypoints_(checkpoints),
  estimated_state_(initial_state), gps_sensor_(gps_sensor),
  dynamics_(operating_constraints, initial_state),
  guidance_(operating_constraints_, follower_method),
  planner_(planner)
{
  if (use_occupancy_grid) {
    if (!range_sensor_config.has_value()) {
      throw std::invalid_argument(
        "Unknown-map navigation requires a range sensor configuration."
      );
    }
    if (occupancy_grid_resolution <= 0.0) {
      throw std::invalid_argument(
        "Occupancy-grid resolution must be positive."
      );
    }

    range_sensor_.emplace(*range_sensor_config);
    occupancy_grid_.emplace(
      planner_.getMap().bounds,
      occupancy_grid_resolution
    );
  }
}



void UAS::initialiseKalman(const KalmanFilterState& kalman) {
  estimator_.initialiseKalmanProperties(kalman);
}

void UAS::setWaypointTolerance(SimConfig sim_config) {
  waypoint_tolerance_ = sim_config.waypoint_tolerance_m;
}

bool UAS::getObjective() {

  while (!waypoints_.empty()) {
    Waypoint next_waypoint = waypoints_.front();
    double dx = estimated_state_.x - next_waypoint.x;
    double dy = estimated_state_.y - next_waypoint.y;
    double euclid_distance = std::sqrt(dx * dx + dy * dy);
    if (euclid_distance < waypoint_tolerance_) {
      waypoints_.erase(waypoints_.begin()); 
    } else 
    { return false; } 
  } 
  return true;
}

std::vector<Waypoint> UAS::getWaypoints() const {
  return waypoints_;
}

UAS_state UAS::getEstimatedState() const {
  return estimated_state_;
}

OccupancyGrid UAS::getOccupancyGrid() const {
  if (occupancy_grid_.has_value()) {
    occupancy_grid_->WriteToCsv();
    return occupancy_grid_.value();
  }
  else{
    throw std::invalid_argument(
      "OccupancyGrid not created"
    );
  }
}

bool UAS::hasOccupancyGrid() const {
  return occupancy_grid_.has_value();
}

void UAS::planRoute() {
  waypoints_ = planner_.Plan(Waypoint{ estimated_state_.x, estimated_state_.y });
}

bool UAS::step(SimConfig sim_config, double sim_time) {
  bool done = false;
    
    done = getObjective();
    if (done == false) {

      // Get desire velocity and heading
      UAS_command command = guidance_.GetCommand(estimated_state_, waypoints_);

      // Set demand to within operating constraints and step forward in time
      // Get true state at next timestep
      dynamics_.step(sim_config.timestep, command);

      // Get measurement based off noisy sampling of true_state
      // Won't always produce a measurement depending on update period
      bool updated = gps_sensor_.update(sim_config.timestep, dynamics_.getState());

      if (updated) {
        gps_measurement_ = gps_sensor_.getMeasurement();
      }
       
      //Estimate state
      estimator_.update(gps_measurement_, sim_time, updated, gps_sensor_.getUpdateInt(),
        operating_constraints_, command);

      estimated_state_ = estimator_.get_state_estimate();

      estimation_history_.push_back(estimated_state_);
      time_history_.push_back(sim_time + sim_config.timestep);


      //Get range measurement if we're using the occupancy grid

      if (occupancy_grid_.has_value() && range_sensor_.has_value()) {
        range_sensor_->getSurroundings(estimated_state_, planner_.getMap());
        occupancy_grid_->UpdateGrid(
          estimated_state_,
          range_sensor_->getMeasurement(),
          range_sensor_->getMaxRange()
        );
      }
      

      if (waypoints_.empty()) {
        std::cout << "Final Waypoint reached \n";
        std::cout << "Mission Complete \n";
        return true;
      }
    }

    return getObjective();
}
