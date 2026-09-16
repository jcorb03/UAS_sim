#include "uas_sim/estimation/UASEstimator.h"

Estimator::Estimator() {
  measurement_count_ = 0;

}

void Estimator::initialiseKalmanProperties(KalmanFilterState kalman) {
  kalman_ = kalman;
}

void Estimator::update(UAS_measurement& measurement,
  double sim_time,
  bool gps_update,
  double update_period,
  UAS_operating_constraints operating_constraints_,
  UAS_command uas_command) {

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

    kalman_.state_estimate <<
      estimated_state_.x,
      estimated_state_.y,
      estimated_state_.v,
      estimated_state_.heading;

    time_ = sim_time;
    while (estimated_state_.heading < 0.0) {
      estimated_state_.heading += 2 * std::numbers::pi;
    }
    return;
  }

  // Second GPS measurement
  if (measurement_count_ == 2 && new_measurement) {

    double dx = measurement.x.value() - estimated_state_.x;
    double dy = measurement.y.value() - estimated_state_.y;

    time_ = sim_time;

    
    estimated_state_.v =
      std::sqrt(dx * dx + dy * dy) / (update_period);

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

    
    while (estimated_state_.heading < 0.0) {
      estimated_state_.heading += 2 * std::numbers::pi;
    }
    return;
  }

  if (measurement_count_ >= 3) {

    // Prediction Step

    double x = kalman_.state_estimate(0);
    double y = kalman_.state_estimate(1);
    double v = kalman_.state_estimate(2);
    double heading = kalman_.state_estimate(3);

    double c = std::cos(heading);
    double s = std::sin(heading);

    // -------------------------
    // Calculate turn rate
    // -------------------------

    double d_heading = uas_command.heading - heading;

    while (d_heading > std::numbers::pi) {
      d_heading -= 2 * std::numbers::pi;
    }

    while (d_heading < -std::numbers::pi) {
      d_heading += 2 * std::numbers::pi;
    }

    double unclamped_turn_rate = d_heading;

    double turn_rate = std::clamp(
      unclamped_turn_rate,
      -operating_constraints_.max_turn_rate,
      operating_constraints_.max_turn_rate
    );

    // -------------------------
    // Calculate acceleration
    // -------------------------

    double unclamped_accel = uas_command.velocity - v;

    double accel = std::clamp(
      unclamped_accel,
      -operating_constraints_.max_accel,
      operating_constraints_.max_accel
    );

    // -------------------------
    // Predict state
    // -------------------------

    kalman_.state_estimate(0) =
      x + v * s * elapsed_time;

    kalman_.state_estimate(1) =
      y + v * c * elapsed_time;

    kalman_.state_estimate(2) =
      std::clamp(
        v + accel * elapsed_time,
        operating_constraints_.min_speed,
        operating_constraints_.max_speed
      );

    kalman_.state_estimate(3) =
      heading + turn_rate * elapsed_time;


    // velocity calc
    double dv_dv;

    if (std::abs(unclamped_accel) < operating_constraints_.max_accel) {
      dv_dv = 1.0 - elapsed_time;
    }
    else {
      dv_dv = 1.0;
    }

    //heading calc

    double dheading_dheading;

    if (std::abs(unclamped_turn_rate) < operating_constraints_.max_turn_rate) {
      dheading_dheading = 1.0 - elapsed_time;
    }
    else {
      dheading_dheading = 1.0;
    }

    kalman_.A <<
      1.0, 0.0, s* elapsed_time, v* c* elapsed_time,
      0.0, 1.0, c* elapsed_time, -v * s * elapsed_time,
      0.0, 0.0, dv_dv, 0.0,
      0.0, 0.0, 0.0, dheading_dheading;

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

  while (estimated_state_.heading < 0.0) {
    estimated_state_.heading += 2 * std::numbers::pi;
  }
  
  // Update stored time
  time_ = sim_time;
}


UAS_state Estimator::get_state_estimate() const {
  return estimated_state_;
}
