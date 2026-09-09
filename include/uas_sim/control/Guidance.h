#pragma once
#include "uas_sim/UAS_structs.h"
#include "uas_sim/dynamics/UAS.h"
#include <stdexcept>
#include <cmath>
#include <numbers>
#include <algorithm>

// Receives next waypoint, 
// produces desired UAS_command (velocity and heading)

class GuidanceModule {
public:
	GuidanceModule(UAS uas);
	                                                                                      
	UAS_command getCommand(const UAS_state& current_state,
	const Waypoint& next_waypoint) const;
private:
	UAS_command command_;
	UAS uas_;
};