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
  operating_constraints_(operating_constraints), mission_waypoints_(checkpoints),
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
    if (range_sensor_config->update_period <= 0.0) {
      throw std::invalid_argument(
        "Range-sensor update period must be positive."
      );
    }

    range_sensor_.emplace(*range_sensor_config);
    range_sensor_update_period_ = range_sensor_config->update_period;
    // Ensures the first map is created before the first route is requested.
    range_sensor_elapsed_ = range_sensor_update_period_;
    occupancy_grid_.emplace(
      planner_.getMap().bounds,
      occupancy_grid_resolution
    );
    grid_path_tolerance_ = std::max(0.5, occupancy_grid_resolution);
    cost_grid_.emplace(*occupancy_grid_, inflation_radius_);
    gridplanner_.emplace();

  }
}

void UAS::initialiseKalman(const KalmanFilterState& kalman) {
  estimator_.initialiseKalmanProperties(kalman);
}

void UAS::setWaypointTolerance(SimConfig sim_config) {
  waypoint_tolerance_ = sim_config.waypoint_tolerance_m;
}

bool UAS::getObjective() {

  while (mission_goal_index_ < mission_waypoints_.size()) {
    const Waypoint& next_waypoint = mission_waypoints_.at(mission_goal_index_);
    double dx = estimated_state_.x - next_waypoint.x;
    double dy = estimated_state_.y - next_waypoint.y;
    double euclid_distance = std::sqrt(dx * dx + dy * dy);
    if (euclid_distance < waypoint_tolerance_) {
      ++mission_goal_index_;
      replan_required_ = true;
    } else 
    { return false; } 
  } 
  return true;
}

std::vector<Waypoint> UAS::getWaypoints() const {
  return active_path_;
}

UAS_state UAS::getEstimatedState() const {
  return estimated_state_;
}

UAS_state UAS::getTrueState() const {
  return dynamics_.getState();
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
  if (occupancy_grid_.has_value()) {
    replan_required_ = true;
    return;
  }

  active_path_ = planner_.Plan(
    Waypoint{ estimated_state_.x, estimated_state_.y }
  );
}

bool UAS::step(SimConfig sim_config, double sim_time) {
  if (getObjective()) {
    return true;
  }

  advanceActivePath();

  if (occupancy_grid_.has_value() && range_sensor_.has_value()) {
    return FrontierStep(sim_config, sim_time);
  }

  return PlannedStep(sim_config, sim_time);
}

bool UAS::PlannedStep(SimConfig sim_config, double sim_time) {
  
  if (!active_path_.empty()) {

    // Get desire velocity and heading
    UAS_command command = guidance_.GetCommand(estimated_state_, active_path_);

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


  }

  return getObjective();
}

bool UAS::FrontierStep(SimConfig sim_config, double sim_time) {
  const bool map_updated = updateOccupancyMap(sim_config.timestep);
  if (map_updated && (active_path_.empty() || !activePathIsTraversable())) {
    replan_required_ = true;
  }

  if (replan_required_) {
    replanUnknownMap();
  }

  if (active_path_.empty()) {
    return false;
  }

  const UAS_command command =
    guidance_.GetCommand(estimated_state_, active_path_);
  dynamics_.step(sim_config.timestep, command);

  const bool updated = gps_sensor_.update(
    sim_config.timestep, dynamics_.getState()
  );
  if (updated) {
    gps_measurement_ = gps_sensor_.getMeasurement();
  }

  estimator_.update(gps_measurement_, sim_time, updated,
    gps_sensor_.getUpdateInt(), operating_constraints_, command);
  estimated_state_ = estimator_.get_state_estimate();

  estimation_history_.push_back(estimated_state_);
  time_history_.push_back(sim_time + sim_config.timestep);

  return getObjective();
}

bool UAS::updateOccupancyMap(double timestep) {
  range_sensor_elapsed_ += timestep;
  if (range_sensor_elapsed_ + 1e-9 < range_sensor_update_period_) {
    return false;
  }

  range_sensor_elapsed_ = std::fmod(
    range_sensor_elapsed_, range_sensor_update_period_
  );
  range_sensor_->getSurroundings(dynamics_.getState(), planner_.getMap());
  occupancy_grid_->UpdateGrid(estimated_state_, range_sensor_->getMeasurement(),
    range_sensor_->getMaxRange());
  cost_grid_.emplace(*occupancy_grid_, inflation_radius_);
  return true;
}

bool UAS::activePathIsTraversable() const {
  for (const GridIndex& cell : active_grid_path_) {
    if (!cost_grid_->isTraversable(cell)) {
      return false;
    }
  }
  return true;
}

bool UAS::replanUnknownMap() {
  const GridIndex start = estimatedGridIndex();
  const auto goal_pair = occupancy_grid_->worldToGrid(
    mission_waypoints_.at(mission_goal_index_)
  );
  const GridIndex goal{
    static_cast<int>(goal_pair.first), static_cast<int>(goal_pair.second)
  };

  gridplanner_->Plan(*occupancy_grid_, *cost_grid_, start, goal);
  const PlanResult result = gridplanner_->GetRoute();
  replan_required_ = false;

  if (!result.found_path) {
    active_grid_path_.clear();
    active_path_.clear();
    return false;
  }

  active_grid_path_ = result.path;
  active_path_.clear();
  active_path_.reserve(active_grid_path_.size());
  for (const GridIndex& cell : active_grid_path_) {
    active_path_.push_back(gridCellCentre(cell));
  }
  advanceActivePath();
  return !active_path_.empty();
}

void UAS::advanceActivePath() {
  const double path_tolerance = occupancy_grid_.has_value()
    ? grid_path_tolerance_
    : waypoint_tolerance_;

  while (!active_path_.empty()) {
    const Waypoint& next = active_path_.front();
    const double dx = estimated_state_.x - next.x;
    const double dy = estimated_state_.y - next.y;

    if (std::hypot(dx, dy) >= path_tolerance) {
      break;
    }

    active_path_.erase(active_path_.begin());
    if (!active_grid_path_.empty()) {
      active_grid_path_.erase(active_grid_path_.begin());
    }
  }
}

GridIndex UAS::estimatedGridIndex() const {
  const auto index = occupancy_grid_->worldToGrid(
    Waypoint{ estimated_state_.x, estimated_state_.y }
  );
  return { static_cast<int>(index.first), static_cast<int>(index.second) };
}

Waypoint UAS::gridCellCentre(GridIndex index) const {
  const auto corner = occupancy_grid_->gridtoWorld({
    static_cast<std::size_t>(index.i), static_cast<std::size_t>(index.j)
  });
  return {
    corner.first + 0.5 * occupancy_grid_->resolution_,
    corner.second + 0.5 * occupancy_grid_->resolution_
  };
}
