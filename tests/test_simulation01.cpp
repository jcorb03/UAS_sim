#include <gtest/gtest.h>
#include <cmath>
#include <iostream>
#include "uas_sim/UAS_structs.h"
#include "uas_sim/Simulation.h"
#include "uas_sim/UAS/UAS.h"

namespace UAS_tests {
  TEST(SimulationTests, Test1){

    UAS_operating_constraints constraints{
      2.0,    // min_speed [m/s]
      5.0,   // max_speed [m/s]
      2.0,    // max_accel [m/s^2]
      0.5     // max_turn_rate [rad/s]
    };


    std::vector<Waypoint> waypoints{
        {100.0, 0.0},
        {100.0, 100.0},
        {0.0, 100.0},
        {0.0, 0.0}
    };

    UAS_state initial_state{
        0.0,    // x [m]
        0.0,    // y [m]
        5.0,   // velocity [m/s]
        0.0     // heading [rad]
    };

    GPSSensor gps_sensor(
      1,    // noise standard deviation [m]
      0.1,     // update rate [Hz]
      5.0
    );

    SimConfig sim_config{
        190.0,    // sim length [s]
        0.1,   // timestep[s]
        10.0     // waypoint tolerance [m]
    };

    WorldBounds bounds{ -10, -10, 110.0, 110.0 };

    ObstacleMap map(bounds, {}, {});

    RoutePlanner planner(waypoints, map);

    KalmanFilterState kalman;

    kalman.state_estimate.setZero();

    kalman.P = Eigen::Matrix4d::Identity();

    kalman.A = Eigen::Matrix4d::Identity();

    kalman.Q = Eigen::Matrix4d::Zero();

    kalman.H.setZero();

    kalman.R = Eigen::Matrix2d::Identity() * 4.0;

    kalman.K.setZero();

    SimulationConfig config{
      constraints,
      initial_state,
      gps_sensor,
      std::nullopt,
      sim_config,
      waypoints,
      FollowerMethod::DIRECT,
      planner,
      kalman,
      NavigationMode::KnownMap,
      1.0
    };

    Simulation simulation(config);

    EXPECT_FALSE(simulation.hasOccupancyGrid());
    EXPECT_TRUE(simulation.run());

    config.range_sensor_config = RangeSensorConfig{
      25.0, 1.0, 0.1, 1.0, 0.1
    };
    config.navigation_mode = NavigationMode::UnknownMap;

    Simulation unknown_map_simulation(config);
    EXPECT_TRUE(unknown_map_simulation.hasOccupancyGrid());
    
    
  }

  TEST(SimulationTests, ZeroNoiseGridPlannerMovesTowardMission) {
    const UAS_operating_constraints constraints{
      2.0, 5.0, 2.0, 0.5
    };
    const std::vector<Waypoint> mission{ { 5.0, 5.0 } };
    const UAS_state initial_state{ 5.0, 30.0, 5.0, 3.14159265358979323846 };
    const SimConfig sim_config{ 40.0, 0.1, 2.0 };

    // Empty ground-truth map: any deviation is planner/control behaviour,
    // not obstacle avoidance.  GPS noise is exactly zero.
    const ObstacleMap map{ WorldBounds{ 0.0, 0.0, 40.0, 40.0 }, {}, {} };
    const RoutePlanner planner{ mission, map };
    const GPSSensor gps_sensor{ 0.0, 0.1, 100.0 };
    const RangeSensorConfig range_sensor{
      10.0, 1.57079632679489661923, 0.01, 0.5, 0.5
    };

    UAS uas{
      constraints, mission, initial_state, gps_sensor,
      FollowerMethod::PURE_PURSUIT, planner, range_sensor, true, 1.0
    };

    KalmanFilterState kalman;
    kalman.state_estimate.setZero();
    kalman.P = Eigen::Matrix4d::Identity();
    kalman.A = Eigen::Matrix4d::Identity();
    kalman.Q = Eigen::Matrix4d::Zero();
    kalman.H.setZero();
    kalman.R = Eigen::Matrix2d::Identity();
    kalman.K.setZero();
    uas.initialiseKalman(kalman);
    uas.setWaypointTolerance(sim_config);

    bool reached_mission = false;
    for (double time = 0.0; time < sim_config.sim_length; time += sim_config.timestep) {
      if (uas.step(sim_config, time)) {
        reached_mission = true;
        break;
      }
    }

    const UAS_state final_state = uas.getTrueState();
    const double displacement = std::hypot(
      final_state.x - initial_state.x,
      final_state.y - initial_state.y
    );

    EXPECT_GT(displacement, 5.0);
    EXPECT_LT(final_state.y, initial_state.y - 5.0);
    EXPECT_TRUE(reached_mission);
  }
}
