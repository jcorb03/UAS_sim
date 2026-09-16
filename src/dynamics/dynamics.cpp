#include "uas_sim/dynamics/dynamics.h"



Dynamics::Dynamics(UAS_operating_constraints operating_constraints, UAS_state uas_state)
	: operating_constraints_(operating_constraints), state_(uas_state)
{

}

void Dynamics::step(double timestep, UAS_command uas_command) {

	state_.x += state_.v * std::sin(state_.heading) * timestep;
	state_.y += state_.v * std::cos(state_.heading) * timestep;

	//Change in heading calculation
  double d_heading = uas_command.heading - state_.heading;

	while (d_heading > std::numbers::pi) {
		d_heading -= 2 * std::numbers::pi;
  }
	while (d_heading < -std::numbers::pi) {
		d_heading += 2 * std::numbers::pi;
  }

	
	//Update heading
	double turn_rate = std::clamp(d_heading,
		-(operating_constraints_.max_turn_rate),
		operating_constraints_.max_turn_rate);

	state_.heading += turn_rate * timestep;

	double accel = std::clamp(uas_command.velocity - state_.v, 
		-(operating_constraints_.max_accel), 
		operating_constraints_.max_accel);

	state_.v = std::clamp((state_.v + accel * timestep), 
		(operating_constraints_.min_speed),
		operating_constraints_.max_speed);

	state_.heading = std::fmod(state_.heading, 2 * std::numbers::pi);

	if (state_.heading < 0.0) {
		state_.heading += 2 * std::numbers::pi;
	}

}

UAS_state Dynamics::getState() const {
	return state_;
}

UAS_operating_constraints Dynamics::getOperatingConstraints() const {
	return operating_constraints_;
}