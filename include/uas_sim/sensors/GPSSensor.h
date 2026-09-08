#pragma once
#include "uas_sim/dynamics/UAS.h"
#include <random>
#include <stdexcept>

struct GPS_measurement {
  double time, x, y;
};
class GPSSensor
{
public:
  GPSSensor(double noise_stddev, double update_period);
  void update(double timestep, const UAS_state& uas_state);
  GPS_measurement getMeasurement() const;
private:
  GPS_measurement measurement_ = {-1, 0, 0};
  double noise_stddev_;
  double update_period_;
  double time_since_measurement_ = 0.0;
  double sim_time_ = 0.0;
  std::normal_distribution<double> noise_;
  std::default_random_engine generator_;
};
