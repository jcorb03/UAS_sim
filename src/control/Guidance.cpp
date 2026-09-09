#include "uas_sim/control/Guidance.h"

GuidanceModule::GuidanceModule(UAS uas): uas_(uas)  {
  // Constructor implementation (if needed)
}

UAS_command GuidanceModule::getCommand(const UAS_state& current_state, const Waypoint& next_waypoint) const {
  // Calculate the desired heading and velocity to reach the next waypoint
  double dx = next_waypoint.x - current_state.x;
  double dy = next_waypoint.y - current_state.y;

  //heading calculated clockwise from north (y axis)
  double desired_heading = std::atan2(dx, dy);
  
  

  double desired_velocity = uas_.getOperatingConstraints().max_speed; // Example constant velocity
  
  // Create and return the UAS_command
  UAS_command command;
  command.velocity = desired_velocity;
  command.heading = desired_heading;
  
  return command;
}