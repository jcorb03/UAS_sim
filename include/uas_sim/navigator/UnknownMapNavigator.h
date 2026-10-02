#pragma once
#include <vector>
#include "uas_sim/UAS_structs.h"
#include "uas_sim/map/OccupancyGrid.h"
#include "uas_sim/routeplanner/gridplanner.h"

class UnknownMapNavigator {
public:
    UnknownMapNavigator();
    void setMissionGoals(std::vector<Waypoint> goals);

    bool update(const UAS_state& estimated_state,
        const OccupancyGrid& occupancy_grid,
        bool new_scan);

    const std::vector<Waypoint>& activePath() const;
    bool missionComplete() const;
private:
    GridPlanner gridplanner_;
    std::vector<Waypoint> goals_;
};