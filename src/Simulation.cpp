#include "uas_sim/Simulation.h"

Simulation::Simulation(std::vector<UAS> uas_s , SimConfig sim_config) : UASs_(uas_s), sim_config_(sim_config)
{
  for (UAS& uas : UASs_) {
    uas.setWaypointTolerance(sim_config_);
  }
  estimation_history_.resize(UASs_.size());
}

bool Simulation::run() {
  bool sim_done = false;

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


std::vector<double> Simulation::getTimeHistory() const {
  return time_history_;
}

std::vector<std::vector<UAS_state>> Simulation::getStateEstimateHistory() const {
  return estimation_history_;
}
