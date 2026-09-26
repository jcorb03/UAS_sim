#include "uas_sim/control/Guidance.h"

PathFollower::PathFollower(UAS_operating_constraints operating_constraints,
  const std::vector<Waypoint>& waypoints):
  operating_constraints_(operating_constraints), waypoints_(waypoints){
  
}

UAS_command PathFollower::GetCommand(const UAS_state& current_state, 
  const std::vector<Waypoint>& waypoints) {
  
  if (method_ == FollowerMethod::DIRECT) {
    return DirectCommand(current_state, waypoints.front());
  }
  else {
    return PPCommand(current_state, waypoints);
  }
}

void PathFollower::SetMethod(FollowerMethod method) {
  method_ = method;
}

UAS_command PathFollower::DirectCommand(const UAS_state& current_state, 
  const Waypoint& next_waypoint) const {
  // Calculate the desired heading and velocity to reach the next waypoint
  double dx = next_waypoint.x - current_state.x;
  double dy = next_waypoint.y - current_state.y;

  //heading calculated clockwise from north (y axis)
  double desired_heading = std::atan2(dx, dy);
  
  while (desired_heading < 0.0) {
    desired_heading += 2 * std::numbers::pi;
  }
  double desired_velocity = operating_constraints_.max_speed; // Example constant velocity
  
  // Create and return the UAS_command
  UAS_command command;
  command.velocity = desired_velocity;
  command.heading = desired_heading;
  
  return command;
}

UAS_command PathFollower::PPCommand(const UAS_state& current_state,
  const std::vector<Waypoint>& waypoints) const{

  //Find where we are on the path currently (nearest neighbour)
  Waypoint state_coords{ current_state.x, current_state.y };
  
  double min_dist = std::numeric_limits<double>::infinity();
  int closest = 0;

  for (int i = 0; i < waypoints_.size(); ++i) {
    
    double dist = helpers::distance(state_coords, waypoints.at(i));

    if (dist < min_dist) {
      closest = i;
      min_dist = dist;
    }
  }
  Waypoint nearest = waypoints.at(closest);


  // Find intercept points (next point in waypoints which is 
  // larger than lookahead distance)
  double lookahead_distance = current_state.v * lookahead_time_;
  bool set = false;
  Waypoint intercept{ 0.0,0.0 };

  for (int i = closest; i < (waypoints_.size()); ++i) {
    if (helpers::distance(state_coords, waypoints.at(i))
         > lookahead_distance) {
      intercept = waypoints.at(i);
      set = true;
      break;
    }
  }
  if (!set) {
    intercept = nearest;
  }

  //Calculate bearing to lookahead point
  double desired_bearing = helpers::bearing(intercept, state_coords);

 
  // Create and return the UAS_command
  UAS_command command;

  double desired_velocity = operating_constraints_.max_speed;
  command.velocity = desired_velocity;
  command.heading = desired_bearing;

  return command;
}