#include "uas_sim/UAS/UAS.h"

UAS::UAS(const UAS_operating_constraints& operating_constraints,
  const std::vector<Waypoint>& waypoints,
  UAS_state initial_state, GPSSensor gps_sensor) :
  operating_constraints_(operating_constraints), waypoints_(waypoints),
  estimated_state_(initial_state), gps_sensor_(gps_sensor),
  dynamics_(operating_constraints, initial_state),
  guidance_(operating_constraints_, waypoints)
{
  
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

bool UAS::step(SimConfig sim_config, double sim_time) {
  bool done = false;
  
    // 
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

      if (waypoints_.empty()) {
        std::cout << "Final Waypoint reached \n";
        std::cout << "Mission Complete \n";
        return true;
      }
    }

    return getObjective();
}
