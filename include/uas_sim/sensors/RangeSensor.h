#pragma once
#include "uas_sim/map/ObstacleMap.h"
#include "uas_sim/UAS_structs.h"
#include <vector>

struct RangeMeasurement {
  double range;
  double bearing;
};

struct RangeSensorConfig {
  double max_range;
  double field_of_view; // half angle from direction of movement
  double angular_resolution;
  double range_resolution;
  double update_period;
};

class RangeSensor {
public:
  RangeSensor(RangeSensorConfig config);

  void getSurroundings(const UAS_state& state,
    const ObstacleMap& map);

  std::vector<RangeMeasurement> getMeasurement() const;

private:
  RangeSensorConfig config_;
  std::vector<RangeMeasurement> measurement_;
};