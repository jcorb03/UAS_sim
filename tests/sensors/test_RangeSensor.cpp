#include <gtest/gtest.h>
#include "uas_sim/sensors/RangeSensor.h"
#include "uas_sim/map/ObstacleMap.h"
#include "uas_sim/UAS_structs.h"

namespace RangeSensorTests {
  TEST(RangeSensorTest, QuickTest) {

    RangeSensorConfig config{
      10.0,
      0.0,
      1.0,
      1.0,
      1.0
    };

    RangeSensor sensor{ config };

    RectangleObstacle rectangle{ -5, 5, 5, 10 };
    WorldBounds bounds{ -20, -20, 20,20 };

    ObstacleMap map{ bounds, {}, {rectangle} };
    map.safety_margin = 0.1;
    UAS_state state{ 0.0 , 0.0, 0.0, 0.0 };

    sensor.getSurroundings(state, map);

    EXPECT_NE(sensor.getMeasurement().size(), 0);
    EXPECT_EQ(sensor.getMeasurement().at(0).range, 5.0);
    EXPECT_EQ(sensor.getMeasurement().at(0).bearing, 0.0);



  }
}