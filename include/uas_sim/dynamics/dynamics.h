#pragma once
#include "uas_sim/UAS_structs.h"

#include <vector>
#include <algorithm>
#include <cmath>
#include <numbers>

class Dynamics
{
public:

	Dynamics( UAS_operating_constraints uas_operating_constraints, UAS_state uas_state);
	void step(double timestep_, UAS_command uas_command);

	UAS_state getState() const;
	UAS_operating_constraints getOperatingConstraints() const;

private:

	UAS_state state_;
	UAS_operating_constraints operating_constraints_;
};