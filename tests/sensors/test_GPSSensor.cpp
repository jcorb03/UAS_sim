#include <gtest/gtest.h>
#include "uas_sim/sensors/GPSSensor.h"

namespace UAS_tests {
  TEST(GPSSensorTests, ConstructorTest) {

    GPSSensor gps_sensor(1.0, 1.0, 10.0);
    EXPECT_EQ(gps_sensor.getMeasurement().x, 0.0);
    EXPECT_EQ(gps_sensor.getMeasurement().y, 0.0);
    EXPECT_EQ(gps_sensor.getMeasurement().time, -1.0);
  }

  TEST(GPSSensorTests, UpdateTest) {

    GPSSensor gps_sensor(1.0, 1.0, 10.0);
    UAS_state state{ 10.0, 20.0, 5.0, 0.0 };

    // Before the update period, the measurement should not change
    EXPECT_NO_THROW(gps_sensor.update(0.5, state));
    EXPECT_NEAR(gps_sensor.getMeasurement().time, -1.0, 1e-6);
    EXPECT_NEAR(gps_sensor.getMeasurement().x, 0.0, 1e-6);
    EXPECT_EQ(gps_sensor.getMeasurement().valid, false);

    // After the update period, the measurement should update
    EXPECT_NO_THROW(gps_sensor.update(1.0, state));
    EXPECT_NEAR(gps_sensor.getMeasurement().time, 1.5, 1e-6);
    EXPECT_EQ(gps_sensor.getMeasurement().valid, true);

    state.x = 100;

    EXPECT_NO_THROW(gps_sensor.update(0.7, state));
    EXPECT_NEAR(gps_sensor.getMeasurement().time, 2.2, 1e-6);
    EXPECT_NEAR(gps_sensor.getMeasurement().x, 100, 5.0);

  }
  
    TEST(GPSSensorTests, InvalidArguments) {
      EXPECT_THROW(
        GPSSensor(-1.0, 1.0, 10.0),
        std::invalid_argument
      );

      EXPECT_THROW(
        GPSSensor(1.0, -1.0, 10.0),
        std::invalid_argument
      );

      EXPECT_THROW(
        GPSSensor(1.0, 0.0, 10.0),
        std::invalid_argument
      );
    }

  

  TEST(GPSSensorTests, NoNoiseTest) {
    GPSSensor gps_sensor(0.0, 1.0, 10.0);
    UAS_state state{ 10.0, 20.0, 5.0, 0.0 };
    gps_sensor.update(2.0, state);

    EXPECT_NEAR(gps_sensor.getMeasurement().x, 10.0, 1e-8);
  }

  TEST(GPSSensorTests, TimeUntilDieTest) {
    GPSSensor gps_sensor(1.0, 2.0, 1.0);
    UAS_state state{ 10.0, 20.0, 5.0, 0.0 };

    // Generate initial measurement
    gps_sensor.update(2.0, state);
    EXPECT_TRUE(gps_sensor.getMeasurement().valid);

    // Measurement is now 0.5 seconds old
    gps_sensor.update(0.5, state);
    EXPECT_TRUE(gps_sensor.getMeasurement().valid);

    // Measurement is now 1.1 seconds old
    gps_sensor.update(0.6, state);
    EXPECT_FALSE(gps_sensor.getMeasurement().valid);
  }
}