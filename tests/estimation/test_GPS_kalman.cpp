#include <gtest/gtest.h>
#include "uas_sim/estimation/UASEstimator.h"
#include "uas_sim/UAS_structs.h"
#include <numbers>


namespace UAS_tests {
  TEST(EstimatorTest, FirstMeasurementInitialisesPosition)
  {
    Estimator estimator;

    UAS_measurement measurement;
    measurement.x = 100.0;
    measurement.y = 200.0;

    estimator.update(measurement, 1.0, true);

    UAS_state state = estimator.get_state_estimate();

    EXPECT_DOUBLE_EQ(state.x, 100.0);
    EXPECT_DOUBLE_EQ(state.y, 200.0);
    EXPECT_DOUBLE_EQ(state.v, 0.0);
    EXPECT_DOUBLE_EQ(state.heading, 0.0);
  }

  TEST(EstimatorTest, SecondMeasurementInitialisesVelocityAndHeading)
  {
    Estimator estimator;

    UAS_measurement first;
    first.x = 100.0;
    first.y = 200.0;

    estimator.update(first, 1.0, true);

    UAS_measurement second;
    second.x = 110.0;
    second.y = 200.0;

    estimator.update(second, 2.0, true);

    UAS_state state = estimator.get_state_estimate();

    EXPECT_DOUBLE_EQ(state.x, 110.0);
    EXPECT_DOUBLE_EQ(state.y, 200.0);
    EXPECT_NEAR(state.v, 10.0, 1e-9);
    EXPECT_NEAR(state.heading, std::numbers::pi / 2.0, 1e-9);
  }

  TEST(EstimatorTest, KalmanFilterStateValid) {
    KalmanFilterState kalman;

    kalman.state_estimate.setZero();
    kalman.P = Eigen::Matrix4d::Identity();
    kalman.Q = Eigen::Matrix4d::Zero();
    kalman.H.setZero();
    kalman.R = Eigen::Matrix2d::Identity();
    kalman.K.setZero();
    kalman.A.setIdentity();

    Estimator estimator;
    EXPECT_NO_THROW(estimator.initialiseKalmanProperties(kalman));
  }

  TEST(EstimatorTest, EKFPrediction)
  {
    Estimator estimator;

    // Initialise Kalman filter properties
    KalmanFilterState kalman;

    kalman.state_estimate.setZero();
    kalman.P = Eigen::Matrix4d::Identity();
    kalman.A = Eigen::Matrix4d::Identity();
    kalman.Q = Eigen::Matrix4d::Zero();
    kalman.H.setZero();
    kalman.R = Eigen::Matrix2d::Identity();
    kalman.K.setZero();

    estimator.initialiseKalmanProperties(kalman);

    // First GPS measurement
    UAS_measurement first_measurement;
    first_measurement.x = 0.0;
    first_measurement.y = 0.0;

    estimator.update(first_measurement, 1.0, true);

    // Second GPS measurement
    UAS_measurement second_measurement;
    second_measurement.x = 10.0;
    second_measurement.y = 0.0;

    estimator.update(second_measurement, 2.0, true);

    UAS_measurement third_measurement;
    third_measurement.x = 20.0;
    third_measurement.y = 0.0;

    estimator.update(third_measurement, 3.0, true);
    // No GPS measurement at t = 3
    UAS_measurement no_measurement;

    estimator.update(no_measurement, 4.0, false);

    UAS_state state = estimator.get_state_estimate();

    // 10 m/s east for 1 second -> x should increase by 10 m
    EXPECT_NEAR(state.x, 30.0, 1e-9);
    EXPECT_NEAR(state.y, 0.0, 1e-9);

    // Velocity and heading should remain unchanged
    EXPECT_NEAR(state.v, 10.0, 1e-9);
    EXPECT_NEAR(state.heading, std::numbers::pi / 2.0, 1e-9);
  }

  TEST(EstimatorTest, EKFMeasurementUpdate)
  {
    Estimator estimator;

    KalmanFilterState kalman;

    kalman.state_estimate.setZero();
    kalman.P = Eigen::Matrix4d::Identity();
    kalman.A = Eigen::Matrix4d::Identity();
    kalman.Q = Eigen::Matrix4d::Zero();
    kalman.H.setZero();
    kalman.R = Eigen::Matrix2d::Identity();
    kalman.K.setZero();

    estimator.initialiseKalmanProperties(kalman);

    // First GPS measurement
    UAS_measurement first_measurement;
    first_measurement.x = 0.0;
    first_measurement.y = 0.0;

    estimator.update(first_measurement, 1.0, true);

    // Second GPS measurement
    // Establishes velocity = 10 m/s and heading = pi/2
    UAS_measurement second_measurement;
    second_measurement.x = 10.0;
    second_measurement.y = 0.0;

    estimator.update(second_measurement, 2.0, true);

    // Third GPS measurement
    // Prediction should give (20, 0), then GPS correction
    // should pull the estimate towards (19, 1).
    UAS_measurement third_measurement;
    third_measurement.x = 19.0;
    third_measurement.y = 1.0;

    estimator.update(third_measurement, 3.0, true);

    UAS_state state = estimator.get_state_estimate();

    EXPECT_NEAR(state.x, 19.3333333333, 1e-9);
    EXPECT_NEAR(state.y, 0.9901960784, 1e-9);
    EXPECT_NEAR(state.v, 9.6666666667, 1e-9);
    EXPECT_NEAR(state.heading, 1.472757, 1e-6);
  }

  TEST(EstimatorTest, EKFMeasurementUpdate)
  {
    Estimator estimator;

    KalmanFilterState kalman;

    kalman.state_estimate.setZero();
    kalman.P = Eigen::Matrix4d::Identity();
    kalman.A = Eigen::Matrix4d::Identity();
    kalman.Q = Eigen::Matrix4d::Zero();
    kalman.H.setZero();
    kalman.R = Eigen::Matrix2d::Identity();
    kalman.K.setZero();

    estimator.initialiseKalmanProperties(kalman);

    // First GPS measurement
    UAS_measurement first_measurement;
    first_measurement.x = 0.0;
    first_measurement.y = 0.0;

    estimator.update(first_measurement, 1.0, true);

    // Second GPS measurement
    // Establishes velocity = 10 m/s and heading = pi/2
    UAS_measurement second_measurement;
    second_measurement.x = 10.0;
    second_measurement.y = 0.0;

    estimator.update(second_measurement, 2.0, true);

    // Third GPS measurement
    // Prediction should give (20, 0), then GPS correction
    // should pull the estimate towards (19, 1).
    UAS_measurement third_measurement;
    third_measurement.x = 19.0;
    third_measurement.y = 1.0;

    estimator.update(third_measurement, 3.0, true);

    UAS_state state = estimator.get_state_estimate();

    EXPECT_NEAR(state.x, 19.3333333333, 1e-9);
    EXPECT_NEAR(state.y, 0.9901960784, 1e-9);
    EXPECT_NEAR(state.v, 9.6666666667, 1e-9);
    EXPECT_NEAR(state.heading, 1.472757, 1e-6);
  }

}