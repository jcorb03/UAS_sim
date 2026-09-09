#include <gtest/gtest.h>
#include "uas_sim/dynamics/UAS.h"

namespace UAS_tests{

  TEST(UASclassTests, ConstructorTest) {
    UAS_operating_constraints constraints{ 0.0, 10.0, 5.0, 1.0 };
    
    UAS_state state{ 0.0, 0.0, 5.0, 0.0 };

    EXPECT_NO_THROW(UAS uas(constraints, state));
    UAS uas(constraints, state);
    EXPECT_EQ(uas.getState().x, 0.0);

    EXPECT_EQ(uas.getState().y, 0.0);
    EXPECT_EQ(uas.getState().v, 5.0);
    EXPECT_EQ(uas.getState().heading, 0.0);

    EXPECT_NO_THROW(uas.step(1.0, { 1.0, 0.5 }));

    
  }

  TEST(UASclassTests, StraightLineTest) {
    UAS_operating_constraints constraints{ 0.0, 10.0, 5.0, 1.0 };
    UAS_state state{ 0.0, 0.0, 5.0, 0.0 };
    UAS uas(constraints, state);
    double timestep = 1.0;
    UAS_command command{ 5.0, 0.0 };
    // No acceleration or turn rate
    for (int i = 0; i < 5; ++i) {
      uas.step(timestep, command);
    }
    UAS_state final_state = uas.getState();
    EXPECT_NEAR(final_state.x, 25.0, 1e-6); // x should be approximately 25
    EXPECT_NEAR(final_state.y, 0.0, 1e-6);   // y should remain approximately 0
    EXPECT_NEAR(final_state.v, 5.0, 1e-6);   // speed should remain approximately 5
    EXPECT_NEAR(final_state.heading, 0.0, 1e-6); // heading should remain approximately 0
  }

  TEST(UASclassTests, AccelerationTest) {
    UAS_operating_constraints constraints{ 0.0, 10.0, 1.0, 1.0 };
    UAS_state state{ 0.0, 0.0, 5.0, 0.0 };
    UAS uas(constraints, state);
    double timestep = 1.0;
    UAS_command command{ 10.0, 0.0 }; 
    for (int i = 0; i < 5; ++i) {
      uas.step(timestep, command);
    }
    UAS_state final_state = uas.getState();
    EXPECT_NEAR(final_state.x, 35.0, 1e-6); // x should be 35
    EXPECT_NEAR(final_state.y, 0.0, 1e-6);   // y should remain approximately 0
    EXPECT_NEAR(final_state.v, 10.0, 1e-6);   // speed accelerate to 10
    EXPECT_NEAR(final_state.heading, 0.0, 1e-6); // heading should remain approximately 0
  }

  TEST(UASclassTests, MaxAccelerationTest) {
    UAS_operating_constraints constraints{ 0.0, 100.0, 5.0, 1.0 };
    UAS_state state{ 0.0, 0.0, 5.0, 0.0 };
    UAS uas(constraints, state);
    double timestep = 1.0;

    UAS_command command{ 100.0, 0.0 };
    for (int i = 0; i < 5; ++i) {
      double old_velocity = uas.getState().v;
      uas.step(timestep, command);
      double new_velocity = uas.getState().v;
      EXPECT_NEAR(
        (new_velocity - old_velocity) / timestep,
        constraints.max_accel,
        1e-6
      );
    }

    command.velocity = 0;

    for (int i = 0; i < 5; ++i) {
      double old_velocity = uas.getState().v;
      uas.step(timestep, command);
      double new_velocity = uas.getState().v;
      EXPECT_NEAR(
        (new_velocity - old_velocity) / timestep,
        -constraints.max_accel,
        1e-6
      );
    }
  }


}
