#include <gtest/gtest.h>
#include "uas_sim/UAS_structs.h"
#include "uas_sim/control/Guidance.h"
#include <numbers>

namespace UAS_tests {
  TEST(GuidanceTest, ConstructionTest) {
    UAS_operating_constraints operating_constraints{ 0.0, 10.0, 5.0, 1.0 };
    EXPECT_NO_THROW(GuidanceModule guidance(operating_constraints));

  }

  TEST(GuidanceTest, CommandTest) {
    // Construct
    UAS_operating_constraints 
      operating_constraints{ 0.0, 10.0, 5.0, 1.0 };
    GuidanceModule guidance(operating_constraints);

    //Step
    UAS_state current_state{ 0.0, 0.0, 0.0, 0.0 };
    Waypoint waypoint{ 1.0,0.0 };
    UAS_command command;
    EXPECT_NO_THROW(command = 
      guidance.getCommand(current_state, waypoint));

    EXPECT_NEAR(command.heading, std::numbers::pi/2, 1e-5);
    EXPECT_EQ(command.velocity, operating_constraints.max_speed);
  }

  TEST(GuidanceTest, CommandTest2) {
    // Construct
    UAS_operating_constraints
      operating_constraints{ 0.0, 10.0, 5.0, 1.0 };
    GuidanceModule guidance(operating_constraints);

    //Step
    UAS_state current_state{ 0.0, 0.0, 0.0, 0.0 };
    Waypoint waypoint{ -2.0,-2.0 };
    UAS_command command;
    EXPECT_NO_THROW(command =
      guidance.getCommand(current_state, waypoint));

    EXPECT_NEAR(command.heading, 5* std::numbers::pi / 4, 1e-5);
    EXPECT_EQ(command.velocity, operating_constraints.max_speed);
  }


}