#pragma once

#include <iostream>
#include "uas_sim/UAS_structs.h"
#include "uas_sim/Simulation.h"
#include "uas_sim/data/make_csv.h"
#include "uas_sim/map/ObstacleMap.h"
#include "uas_sim/routeplanner/routeplanner.h"
#include "uas_sim/courses/course1.h"

namespace SimulationCatalogue {
  void WaypointNavigation() {

    // Simple premade simulation - Preplanned waypoint navigation route using direct navigation
    // No obstacles or sensing used

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

    UAS_state second_initial_state{
        -20.0,  // x [m]
        0.0,    // y [m]
        5.0,    // velocity [m/s]
        0.0     // heading [rad]
    };

    GPSSensor gps_sensor(
      1,    // noise standard deviation [m]
      0.1,     // update rate [Hz]
      5.0
    );

    SimConfig sim_config{
        210.0,    // sim length [s]
        0.1,   // timestep[s]
        10.0     // waypoint tolerance [m]
    };
    WorldBounds bounds{ -10, -10, 110.0, 110.0 };

    ObstacleMap map(bounds, {}, {});

    RoutePlanner planner(waypoints, map);

    UAS uas(
      constraints,
      waypoints,
      initial_state,
      gps_sensor,
      FollowerMethod::DIRECT,
      planner
    );

    UAS second_uas(
      constraints,
      waypoints,
      second_initial_state,
      gps_sensor,
      FollowerMethod::DIRECT,
      planner
    );

    KalmanFilterState kalman;

    kalman.state_estimate.setZero();

    kalman.P = Eigen::Matrix4d::Identity();

    kalman.A = Eigen::Matrix4d::Identity();

    kalman.Q = Eigen::Matrix4d::Zero();

    kalman.H.setZero();

    kalman.R = Eigen::Matrix2d::Identity() * 4.0;

    kalman.K.setZero();

    uas.initialiseKalman(kalman);
    second_uas.initialiseKalman(kalman);

    std::vector<UAS> Uas_vec = { uas, second_uas };

    Simulation simulation(
      Uas_vec,
      sim_config
    );

    bool done = simulation.run();

    std::vector<double> time_history = simulation.getTimeHistory();


    std::vector<std::vector<UAS_state>> estimation_history = simulation.getStateEstimateHistory();


    bool made = results::makeCsv(time_history, estimation_history);
    std::cout << "Simulation complete: " << done << "\n"
      << "CSV status: " << made << '\n';
  }

  void ObstacleCourseNavigation() {

    UAS_operating_constraints constraints{
     2.0,    // min_speed [m/s]
     5.0,   // max_speed [m/s]
     2.0,    // max_accel [m/s^2]
     0.5     // max_turn_rate [rad/s]
    };

    RoutePlanner planner = Courses::Course1();
    std::vector<Waypoint> path = planner.Plan(Waypoint(10.0, 90.0));

    PathFollower guidance(constraints,
       FollowerMethod::PURE_PURSUIT);

    UAS_state state(10.0, 90.0, 5.0, 3.14);

    guidance.SetMethod(FollowerMethod::PURE_PURSUIT);

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

    std::vector<Waypoint> checkpoints{
        Waypoint(5.0, 5.0),
        Waypoint(60.0, 35.0),
        Waypoint(80.0, 65.0),
        Waypoint(40.0, 90.0),
    };

    UAS uas(
      constraints,
      checkpoints,
      state,
      gps_sensor,
      FollowerMethod::DIRECT,
      planner
    );

    KalmanFilterState kalman;

    kalman.state_estimate.setZero();

    kalman.P = Eigen::Matrix4d::Identity();

    kalman.A = Eigen::Matrix4d::Identity();

    kalman.Q = Eigen::Matrix4d::Zero();

    kalman.H.setZero();

    kalman.R = Eigen::Matrix2d::Identity() * 4.0;

    kalman.K.setZero();

    uas.initialiseKalman(kalman);

    std::vector<UAS> Uas_vec = { uas };

    Simulation simulation(
      Uas_vec,
      sim_config
    );

    bool done = simulation.run();

    std::vector<double> time_history = simulation.getTimeHistory();


    std::vector<std::vector<UAS_state>> estimation_history = simulation.getStateEstimateHistory();


    bool made = results::makeCsv(time_history, estimation_history);
    std::cout << "Simulation complete: " << done << "\n"
      << "CSV status: " << made << '\n';
  }
}