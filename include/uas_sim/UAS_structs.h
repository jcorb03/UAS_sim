#pragma once
#include <optional>

enum class ControlAlgorithm {
  SIMPLE
};

enum class EstimationAlgorithm {
  KALMAN
};

enum class SensorType {
  GPS
};

enum class TaskType {
  WAYPOINT_NAVIGATION
};

struct Waypoint {
  double x;
  double y;
};

struct UAS_operating_constraints {
	double min_speed;
	double max_speed;
	double max_accel;
	double max_turn_rate;
};

struct UAS_command {
	double velocity;
	double heading;
};

struct UAS_state {
	double x, y, v, heading;
};

struct UAS_measurement {
    std::optional<double> x = std::nullopt;
    std::optional<double> y = std::nullopt;
    std::optional<double> v = std::nullopt;
    std::optional<double> heading = std::nullopt;
    double time = -1.0;
    bool valid = false;
};

struct SimConfig {
  double sim_length;
  double timestep;
  double waypoint_tolerance_m;
};

  