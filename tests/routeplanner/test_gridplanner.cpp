#include <gtest/gtest.h>

#include <limits>

#include "uas_sim/map/CostMap.h"
#include "uas_sim/routeplanner/gridplanner.h"

namespace {

Costmap makeFreeCostmap(std::size_t width = 5, std::size_t height = 5) {
  OccupancyGrid occupancy_grid{
    WorldBounds{ 0.0, 0.0, static_cast<double>(width),
      static_cast<double>(height) },
    1.0
  };

  for (auto& column : occupancy_grid.grid) {
    for (OccupancyCell& cell : column) {
      cell.log_odds = -2.0;
    }
  }

  return Costmap{ occupancy_grid, 0.0 };
}

}  // namespace

TEST(GridPlannerTests, FindsPathAcrossFreeCostmap) {
  const Costmap costmap = makeFreeCostmap();
  GridPlanner planner;

  EXPECT_TRUE(planner.Plan(costmap, { 0, 0 }, { 4, 4 }));

  const PlanResult route = planner.GetRoute();
  ASSERT_TRUE(route.found_path);
  ASSERT_FALSE(route.path.empty());
  EXPECT_EQ(route.path.front().i, 0);
  EXPECT_EQ(route.path.front().j, 0);
  EXPECT_EQ(route.path.back().i, 4);
  EXPECT_EQ(route.path.back().j, 4);
}

TEST(GridPlannerTests, RejectsAnUnreachableGoal) {
  Costmap costmap = makeFreeCostmap();

  for (double& cell_cost : costmap.costs.at(1)) {
    cell_cost = std::numeric_limits<double>::infinity();
  }

  GridPlanner planner;

  EXPECT_FALSE(planner.Plan(costmap, { 0, 0 }, { 4, 4 }));
  EXPECT_FALSE(planner.GetRoute().found_path);
  EXPECT_TRUE(planner.GetRoute().path.empty());
}

TEST(GridPlannerTests, RejectsAnOutOfBoundsStart) {
  const Costmap costmap = makeFreeCostmap();
  GridPlanner planner;

  EXPECT_FALSE(planner.Plan(costmap, { -1, 0 }, { 4, 4 }));
  EXPECT_FALSE(planner.GetRoute().found_path);
}
