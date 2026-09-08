#pragma once
#include "uas_sim/dynamics/UAS.h"
#include <random>
#include <stdexcept>

struct GPS_measurement {
  double time, x, y;
  bool valid = false;
};
class GPSSensor
{
public:
  GPSSensor(double noise_stddev, double update_period, double time_til_die);
  void update(double timestep, const UAS_state& uas_state);
  GPS_measurement getMeasurement() const;
private:
  GPS_measurement measurement_ = {-1, 0, 0};
  double noise_stddev_;
  double update_period_;
  double time_since_measurement_ = 0.0;
  double sim_time_ = 0.0;
  double time_til_die_;
  double measurement_age_ = 0.0;
  std::normal_distribution<double> noise_{ 0.0, 1.0 };
  std::default_random_engine generator_;
};
