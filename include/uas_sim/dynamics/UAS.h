#pragma once

#include <vector>

struct UAS_operating_constraints {
	double min_speed;
	double max_speed;
	double max_accel;
	double max_turn_rate;
};

struct UAS_command {
	double accel;
	double turn_rate;
};

struct UAS_state {
	double x, y, v, heading;
};

class UAS
{
public:

	UAS( UAS_operating_constraints uas_operating_constraints, UAS_state uas_state);
	void step(double timestep_, UAS_command uas_command);

	UAS_state getState() const;

private:

	UAS_state state_;
	UAS_operating_constraints operating_constraints_;
};