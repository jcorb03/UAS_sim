#pragma once
#include "uas_sim/UAS_structs.h"
#include "uas_sim/dynamics/dynamics.h"
#include "uas_sim/helperfunctions.h"
#include <stdexcept>
#include <cmath>
#include <numbers>
#include <algorithm>


//Produces UAS_command to follow path from route planner

enum class FollowerMethod {
	DIRECT,
	PURE_PURSUIT
};

class PathFollower {
public:
	PathFollower(UAS_operating_constraints operating_constraints,
		const std::vector<Waypoint>& waypoints
		);

	void SetLookaheadTime(double lookahead_time) {
		lookahead_time_ = std::max(1.0, lookahead_time);
	}
	void SetMethod(FollowerMethod method);

	UAS_command GetCommand(const UAS_state& current_state,
		const std::vector<Waypoint>& waypoints);

private:
	UAS_command DirectCommand(const UAS_state& current_state,
	const Waypoint& next_waypoint) const;

	UAS_command PPCommand(const UAS_state& current_state,
		const std::vector<Waypoint>& waypoints) const;


	FollowerMethod method_ = FollowerMethod::DIRECT;
	UAS_operating_constraints operating_constraints_;
	std::vector<Waypoint> waypoints_;
	double lookahead_time_ = 2.0;
};