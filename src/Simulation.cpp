#include "uas_sim/Simulation.h"

Simulation::Simulation(const UAS_operating_constraints& operating_constraints,
  const std::vector<Waypoint>& waypoints,
  UAS_state initial_state, GPSSensor gps_sensor) : operating_constraints_(operating_constraints),
  waypoints_(waypoints), true_state_(initial_state), gps_sensor_(gps_sensor), uas_(operating_constraints, initial_state),
  estimated_state_(initial_state)
{
  
}

void Simulation::run(double sim_time, double timestep) {

}

