#pragma once
#include "uas_sim/map/ObstacleMap.h"
#include <vector>
#include "uas_sim/routeplanner/routeplanner.h"

namespace Courses {
  RoutePlanner Course1() {
    WorldBounds bounds{ 0.0,0.0,100.0,100.0 };

    CircleObstacle circle1{ 10.0, 15.0, 3.0 };
    CircleObstacle circle2{ 20.0, 75.0, 4.0 };
    CircleObstacle circle3{ 50.0, 50.0, 5.0 };
    std::vector<CircleObstacle> circles = { circle1, circle2, circle3 };


    RectangleObstacle rectangle1{ 20.0, 20.0, 22.0, 23.0 };
    RectangleObstacle rectangle2{ 65.0, 40.0, 80.0, 48.0 };
    std::vector<RectangleObstacle> rectangles = { rectangle1, rectangle2 };

    ObstacleMap map;
    map.bounds = bounds;
    map.circles = circles;
    map.rectangles = rectangles;

    map.writeToCSV();

    std::vector<Waypoint> checkpoints{
      Waypoint(5.0, 5.0),
      Waypoint(60.0, 35.0),
      Waypoint(80.0, 65.0),
      Waypoint(40.0, 90.0),
    };

    RoutePlanner planner(checkpoints, map);
    planner.SetPlanningAlgorithm(RoutePlanningAlgos::RRT_STAR);
    return planner;
  }
}