#include "uas_sim/dynamics/UAS.h"
#include <algorithm>
#include <cmath>
UAS::UAS(UAS_operating_constraints operating_constraints, UAS_state uas_state)
	: operating_constraints_(operating_constraints), state_(uas_state)
{

}

void UAS::step(double timestep, UAS_command uas_command) {

	state_.x += state_.v * std::cos(state_.heading) * timestep;
	state_.y += state_.v * std::sin(state_.heading) * timestep;

	//Update heading
	double turn_rate = std::clamp(uas_command.turn_rate, -(operating_constraints_.max_turn_rate), operating_constraints_.max_turn_rate);
	state_.heading += turn_rate * timestep;

	double accel = std::clamp(uas_command.accel, -(operating_constraints_.max_accel), operating_constraints_.max_accel);

	state_.v = std::clamp((state_.v + accel * timestep), (operating_constraints_.min_speed), operating_constraints_.max_speed);

}

UAS_state UAS::getState() const {
	return state_;
}