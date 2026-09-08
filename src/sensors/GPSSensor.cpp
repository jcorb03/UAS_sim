#include "uas_sim/sensors/GPSSensor.h"

GPSSensor::GPSSensor(double noise_stddev, double update_period, double time_til_die) : 
  noise_stddev_(noise_stddev), update_period_(update_period), time_til_die_(time_til_die)
{
  if (noise_stddev_ < 0.0) {
    throw std::invalid_argument("Noise standard deviation must be non-negative.");
  }

  if (update_period_ <= 0.0) {
    throw std::invalid_argument("Update period must be positive.");
  }
  
  if (time_til_die_ <= 0.0) {
    throw std::invalid_argument("Time until die must be positive.");
  }
}

void GPSSensor::update(double timestep, const UAS_state& uas_state) 
{
  time_since_measurement_ += timestep;
  measurement_age_ += timestep;
  sim_time_ += timestep;
  if (measurement_age_ > time_til_die_) {
    measurement_.valid = false;
  }

  if (time_since_measurement_ >= update_period_) {
    measurement_.x = uas_state.x + noise_stddev_ * noise_(generator_);
    measurement_.y = uas_state.y + noise_stddev_ * noise_(generator_);
    measurement_.time = sim_time_;
    time_since_measurement_ -= update_period_;
    measurement_age_ = 0.0;
    measurement_.valid = true;
  }

  
}

UAS_measurement GPSSensor::getMeasurement() const {
  return measurement_;
}