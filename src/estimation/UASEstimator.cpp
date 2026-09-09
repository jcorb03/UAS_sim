#include "uas_sim/estimation/UASEstimator.h"

Estimator::Estimator() {
  measurement_count_ = 0;

}

void Estimator::initialiseKalmanProperties(KalmanFilterState kalman) {
  kalman_ = kalman;
}

void Estimator::update(UAS_measurement& measurement, double sim_time) {
  
  // Have we got a new measurement?
  if (measurement.time == sim_time && measurement.valid == true) {
    measurement_count_++;
  }
  
  //time since last call
  double elapsed_time = sim_time - time_;

  if (measurement_count_ == 1) {
    estimated_state_.x = measurement.x
  }

  //update stored time to current time
  time_ = sim_time;
}

UAS_state Estimator::get_state_estimate() const {
  return estimated_state_;
}
