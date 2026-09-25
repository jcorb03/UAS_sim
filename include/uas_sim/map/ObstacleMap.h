#pragma once

#include "uas_sim/UAS_structs.h"
#include <vector>
#include <cmath>

struct CircleObstacle {
    double x;
    double y;
    double radius;
};

struct RectangleObstacle {
    double min_x;
    double min_y;
    double max_x;
    double max_y;
};

struct WorldBounds {
    double min_x;
    double min_y;
    double max_x;
    double max_y;
};

struct ObstacleMap {
    WorldBounds bounds;
    std::vector<CircleObstacle> circles;
    std::vector<RectangleObstacle> rectangles;

    double safety_margin = 5.0;

    bool isPointFree(const Waypoint& to) const;

    bool isSegmentFree(
        const Waypoint& from,
        const Waypoint& to) const;
};