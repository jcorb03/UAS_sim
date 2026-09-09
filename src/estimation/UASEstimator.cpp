#include "uas_sim/estimation/UASEstimator.h"

Estimator::Estimator() {
  measurement_count_ = 0;

}

void Estimator::initialiseKalmanProperties(KalmanFilterState kalman) {
  kalman_ = kalman;
}

void Estimator::update(UAS_measurement& measurement,
  double sim_time,
  bool gps_update) {

  bool new_measurement = false;

  // Have we got a new GPS measurement?
  if (gps_update) {
    measurement_count_++;
    new_measurement = true;
  }

  // Time since last estimator update
  double elapsed_time = sim_time - time_;

  // First GPS measurement
  if (measurement_count_ == 1 && new_measurement) {

    estimated_state_.x = measurement.x.value();
    estimated_state_.y = measurement.y.value();
    estimated_state_.v = 0.0;
    estimated_state_.heading = 0.0;

    time_ = sim_time;
    return;
  }

  // Second GPS measurement
  if (measurement_count_ == 2 && new_measurement) {

    double dx = measurement.x.value() - estimated_state_.x;
    double dy = measurement.y.value() - estimated_state_.y;

    estimated_state_.v =
      std::sqrt(dx * dx + dy * dy) / elapsed_time;

    estimated_state_.heading =
      std::atan2(dx, dy);

    estimated_state_.x = measurement.x.value();
    estimated_state_.y = measurement.y.value();

    // Initialise the EKF state
    kalman_.state_estimate <<
      estimated_state_.x,
      estimated_state_.y,
      estimated_state_.v,
      estimated_state_.heading;

    time_ = sim_time;
    return;
  }

  if (measurement_count_ >= 3) {

    // Prediction Step

    double c = std::cos(estimated_state_.heading);
    double s = std::sin(estimated_state_.heading);

    kalman_.state_estimate(0) =
      estimated_state_.x +
      estimated_state_.v * s * elapsed_time;

    kalman_.state_estimate(1) =
      estimated_state_.y +
      estimated_state_.v * c * elapsed_time;

    kalman_.state_estimate(2) =
      estimated_state_.v;

    kalman_.state_estimate(3) =
      estimated_state_.heading;

    kalman_.A <<
      1.0, 0.0, s* elapsed_time,
      estimated_state_.v* c* elapsed_time,

      0.0, 1.0, c* elapsed_time,
      -estimated_state_.v * s * elapsed_time,

      0.0, 0.0, 1.0, 0.0,

      0.0, 0.0, 0.0, 1.0;

    kalman_.P =
      kalman_.A * kalman_.P * kalman_.A.transpose()
      + kalman_.Q;
  }

  if (measurement_count_ >= 3 && new_measurement) {

    // Measurement model
    kalman_.H <<
      1.0, 0.0, 0.0, 0.0,
      0.0, 1.0, 0.0, 0.0;

    // Predicted measurement
    Eigen::Vector2d z_prediction =
      kalman_.H * kalman_.state_estimate;

    // Actual GPS measurement
    Eigen::Vector2d z;
    z(0) = measurement.x.value();
    z(1) = measurement.y.value();

    // Measurement residual
    Eigen::Vector2d residual =
      z - z_prediction;

    // Innovation covariance
    Eigen::Matrix2d S =
      kalman_.H * kalman_.P * kalman_.H.transpose()
      + kalman_.R;

    // Kalman gain
    kalman_.K =
      kalman_.P * kalman_.H.transpose() * S.inverse();

    // State update
    kalman_.state_estimate +=
      kalman_.K * residual;

    // Covariance update
    Eigen::Matrix4d I =
      Eigen::Matrix4d::Identity();

    kalman_.P =
      (I - kalman_.K * kalman_.H) * kalman_.P;
  }

  // Convert Kalman state estimate to UAS_state
  estimated_state_.x = kalman_.state_estimate(0);
  estimated_state_.y = kalman_.state_estimate(1);
  estimated_state_.v = kalman_.state_estimate(2);
  estimated_state_.heading = kalman_.state_estimate(3);

  // Update stored time
  time_ = sim_time;
}


UAS_state Estimator::get_state_estimate() const {
  return estimated_state_;
}
