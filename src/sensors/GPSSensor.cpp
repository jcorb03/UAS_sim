#include "uas_sim/sensors/GPSSensor.h"

GPSSensor::GPSSensor(double noise_stddev, double update_period) : noise_stddev_(noise_stddev), update_period_(update_period)
{
  if (noise_stddev_ < 0.0) {
    noise_stddev_ = 0.0;
  }

  if (update_period_ <= 0.0) {
    update_period_ = 1.0;
  }

  noise_ = std::normal_distribution(0.0, noise_stddev_);
}

void GPSSensor::update(double timestep, const UAS_state& uas_state) 
{
  time_since_measurement_ += timestep;
  sim_time_ += timestep;

  if (time_since_measurement_ >= update_period_) {
    
    
    measurement_.x = uas_state.x + noise_(generator_);
    measurement_.y = uas_state.y + noise_(generator_);
    measurement_.time = sim_time_;
    time_since_measurement_ -= update_period_;
  }
}

GPS_measurement GPSSensor::getMeasurement() const {
  return measurement_;
}