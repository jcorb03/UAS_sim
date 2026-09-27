#include "uas_sim/Simulation.h"

Simulation::Simulation(const SimulationConfig& config)
  : sim_config_(config.simulation)
{
  const bool use_occupancy_grid =
    config.navigation_mode == NavigationMode::UnknownMap;

  UASs_.emplace_back(
    config.operating_constraints,
    config.checkpoints,
    config.initial_state,
    config.gps_sensor,
    config.follower_method,
    config.route_planner,
    config.range_sensor_config,
    use_occupancy_grid,
    config.occupancy_grid_resolution
  );
  UASs_.front().initialiseKalman(config.kalman);
  UASs_.front().setWaypointTolerance(sim_config_);
  estimation_history_.resize(UASs_.size());
}

bool Simulation::run() {
  bool sim_done = false;

  for (UAS& uas : UASs_) {
    uas.planRoute();
  }

  while (sim_time_ < sim_config_.sim_length && sim_done == false) {

    UAS_statuses.clear();

    for (UAS& uas : UASs_) {
      bool drone_done = uas.step(sim_config_, sim_time_);
      UAS_statuses.push_back(drone_done);
    }

    if (!UASs_.empty()) {
      for (std::size_t uav = 0; uav < UASs_.size(); ++uav) {
        estimation_history_[uav].push_back(
          UASs_[uav].getEstimatedState()
        );
      }
      time_history_.push_back(sim_time_ + sim_config_.timestep);
    }
    
   
    if (std::find(UAS_statuses.begin(), UAS_statuses.end(), false) == UAS_statuses.end()) {
      sim_done = true;
    }
    sim_time_ += sim_config_.timestep;
  }

  return sim_done;
}

bool Simulation::hasOccupancyGrid() const {
  return !UASs_.empty() && UASs_.front().hasOccupancyGrid();
}

std::vector<double> Simulation::getTimeHistory() const {
  return time_history_;
}

std::vector<std::vector<UAS_state>> Simulation::getStateEstimateHistory() const {
  return estimation_history_;
}

OccupancyGrid Simulation::getOccupancyGrid() const {
  return UASs_.at(0).getOccupancyGrid();
}


