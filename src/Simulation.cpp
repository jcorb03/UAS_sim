#include "uas_sim/Simulation.h"

Simulation::Simulation(const UAS_operating_constraints& operating_constraints,
  const std::vector<Waypoint>& waypoints,
  UAS_state initial_state, GPSSensor gps_sensor, SimConfig sim_config) : operating_constraints_(operating_constraints),
  waypoints_(waypoints), gps_sensor_(gps_sensor), uas_(operating_constraints, initial_state),
  estimated_state_(initial_state), guidance_(operating_constraints), sim_config_(sim_config)
{

}

void Simulation::run() {
  bool done = false;
  while (sim_time_ < sim_config_.sim_length && done == false) {
    // 
    done = getObjective(estimated_state_);
    double last_gps_time = 0.0;
    
    if (done == false) {
      
      // Get desire velocity and heading
      UAS_command command = guidance_.getCommand(estimated_state_, waypoints_.front());

      // Set demand to within operating constraints and step forward in time
      // Get true state at next timestep
      uas_.step(sim_config_.timestep, command);
 
      // Get measurement based off noisy sampling of true_state
      // Won't always produce a measurement depending on update period
      bool updated = gps_sensor_.update(sim_config_.timestep, uas_.getState());
      
      if (updated) {
        gps_measurement_ = gps_sensor_.getMeasurement();
        last_gps_time = sim_time_;
      }
      //Estimate state
      estimator_.update(gps_measurement_, sim_time_, updated, gps_sensor_.getUpdateInt(),
        operating_constraints_,command);

      estimated_state_ = estimator_.get_state_estimate();

      sim_time_ += sim_config_.timestep;

      std::cout << "Time: " << sim_time_ << "\n";
      if (!waypoints_.empty()) {
        std::cout << "Next waypoint: { "
          << waypoints_.front().x << " , "
          << waypoints_.front().y << " }\n";
      }
      else {
        std::cout << "Mission complete\n";
      }
      std::cout << "True State: { " << uas_.getState().x <<
        " , " << uas_.getState().y << " }\n";
      std::cout << "Estimated State: { " << estimated_state_.x <<
        " , " << estimated_state_.y << " }\n\n";
    }
   
  }
}

bool Simulation::getObjective(const UAS_state& state_estimate) {

  while (!waypoints_.empty()) {

    Waypoint next_waypoint = waypoints_.front();

    double dx = state_estimate.x - next_waypoint.x;
    double dy = state_estimate.y - next_waypoint.y;

    double euclid_distance = std::sqrt(dx * dx + dy * dy);

    if (euclid_distance < sim_config_.waypoint_tolerance_m) {
      waypoints_.erase(waypoints_.begin());
    }
    else {
      return false;
    }
  }

  return true;
}

void Simulation::initialiseKalman(KalmanFilterState kalman) {
  estimator_.initialiseKalmanProperties(kalman);
}
