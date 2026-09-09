#include "uas_sim/Simulation.h"

Simulation::Simulation(const UAS_operating_constraints& operating_constraints,
  const std::vector<Waypoint>& waypoints,
  UAS_state initial_state, GPSSensor gps_sensor) : operating_constraints_(operating_constraints),
  waypoints_(waypoints), true_state_(initial_state), gps_sensor_(gps_sensor), uas_(operating_constraints, initial_state),
  estimated_state_(initial_state)
{
  
}

void Simulation::run(double sim_length, double timestep) {
  bool done = false;
  while (sim_time_ < sim_length && done == false) {
    done = getObjective(estimated_state_);

    if (done == false) {
      // Get Guidance Command - outputs desired state

      // Dynamics + Control Step - outputs actual state
      // 
      // Get sensor measurements - outputs noisy measurements based on actual state
      // 
      // Estimate State 

      sim_time_ += timestep;
    }
  }
}

bool Simulation::getObjective(const UAS_state& state_estimate) {

  while (!waypoints_.empty()) {

    Waypoint next_waypoint = waypoints_.front();

    double dx = state_estimate.x - next_waypoint.x;
    double dy = state_estimate.y - next_waypoint.y;

    double euclid_distance = std::sqrt(dx * dx + dy * dy);

    if (euclid_distance < 1e2) {
      waypoints_.erase(waypoints_.begin());
    }
    else {
      return false;
    }
  }

  return true;
}
