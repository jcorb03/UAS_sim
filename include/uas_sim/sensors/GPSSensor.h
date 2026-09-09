#pragma once
#include "uas_sim/dynamics/UAS.h"
#include "uas_sim/UAS_structs.h"
#include <random>
#include <stdexcept>


class GPSSensor
{
public:
  GPSSensor(double noise_stddev, double update_period, double time_til_die);
  void update(double timestep, const UAS_state& uas_state);
  UAS_measurement getMeasurement() const;
private:
  UAS_measurement measurement_;
  double noise_stddev_;
  double update_period_;
  double time_since_measurement_ = 0.0;
  double sim_time_ = 0.0;
  double time_til_die_;
  double measurement_age_ = 0.0;
  std::normal_distribution<double> noise_{ 0.0, 1.0 };
  std::default_random_engine generator_;
};
