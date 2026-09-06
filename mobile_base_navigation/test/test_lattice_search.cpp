// Copyright 2026 Safwan
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// Edge generation, heading snapping and merging, against a synthetic
/// costmap. No node, no lifecycle manager and no simulator: the search core
/// is deliberately free of ROS types so it can be driven directly.

#include <cmath>
#include <vector>

#include "gtest/gtest.h"
#include "mobile_base_navigation/lattice_search.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

using mobile_base_navigation::LatticeSearch;
using mobile_base_navigation::LatticeSettings;
using mobile_base_navigation::headingAngle;
using mobile_base_navigation::headingCos;
using mobile_base_navigation::headingSin;
using mobile_base_navigation::isDiagonalHeading;
using mobile_base_navigation::kNumHeadings;
using mobile_base_navigation::snapHeading;
using mobile_base_navigation::snapHeadingWithHysteresis;

namespace
{

constexpr double kQuarterPi = M_PI / 4.0;

/// A 6 x 6 m room at the repo's map resolution, entirely free space.
nav2_costmap_2d::Costmap2D freeCostmap()
{
  return nav2_costmap_2d::Costmap2D(120, 120, 0.05, 0.0, 0.0, nav2_costmap_2d::FREE_SPACE);
}

LatticeSettings testSettings()
{
  LatticeSettings settings;
  settings.step_size_m = 0.05;
  settings.robot_radius_m = 0.14;
  settings.tolerance_m = 0.05;
  settings.cost_penalty_weight = 0.0;
  settings.allow_unknown = false;
  return settings;
}

/// The angle of the segment leaving waypoint i, wrapped into [0, 2*pi).
double segmentAngle(
  const std::vector<mobile_base_navigation::LatticeState> & waypoints, std::size_t i)
{
  const double angle = std::atan2(
    waypoints[i + 1].y - waypoints[i].y,
    waypoints[i + 1].x - waypoints[i].x);
  return angle < 0.0 ? angle + 2.0 * M_PI : angle;
}

}  // namespace

TEST(HeadingSnap, EightHeadingsAreExactAndDistinct)
{
  for (int heading = 0; heading < kNumHeadings; ++heading) {
    EXPECT_NEAR(headingAngle(heading), heading * kQuarterPi, 1e-12);
    EXPECT_EQ(snapHeading(headingAngle(heading)), heading);
    // Unit length, so a commanded speed is the speed actually travelled and
    // not a component of it.
    EXPECT_NEAR(std::hypot(headingCos(heading), headingSin(heading)), 1.0, 1e-12);
  }
  // A diagonal must have |x| == |y| bit for bit: the controller locks vx and
  // vy to each other on a diagonal, and trigonometry rounding would spend the
  // tolerance that check is allowed.
  for (const int heading : {1, 3, 5, 7}) {
    EXPECT_TRUE(isDiagonalHeading(heading));
    EXPECT_DOUBLE_EQ(std::fabs(headingCos(heading)), std::fabs(headingSin(heading)));
  }
  for (const int heading : {0, 2, 4, 6}) {
    EXPECT_FALSE(isDiagonalHeading(heading));
    // A cardinal heading must have an exactly zero cross component, or a
    // "forward" command would leak a small strafe.
    const double cross = heading % 4 == 0 ? headingSin(heading) : headingCos(heading);
    EXPECT_DOUBLE_EQ(cross, 0.0);
  }
}

TEST(HeadingSnap, WrapsNegativeAndOverfullAngles)
{
  EXPECT_EQ(snapHeading(-kQuarterPi), 7);
  EXPECT_EQ(snapHeading(2.0 * M_PI), 0);
  EXPECT_EQ(snapHeading(-2.0 * M_PI + 1e-9), 0);
  // Just inside each half-cell still snaps to the same heading.
  EXPECT_EQ(snapHeading(kQuarterPi / 2.0 - 1e-6), 0);
  EXPECT_EQ(snapHeading(kQuarterPi / 2.0 + 1e-6), 1);
}

TEST(HeadingSnap, HysteresisHoldsTheHeadingNearABoundary)
{
  const double hysteresis = 5.0 * M_PI / 180.0;
  // Just over the 22.5 degree boundary: without hysteresis this is heading 1.
  const double yaw = kQuarterPi / 2.0 + 2.0 * M_PI / 180.0;
  EXPECT_EQ(snapHeading(yaw), 1);
  // With no previous heading, nothing to hold on to.
  EXPECT_EQ(snapHeadingWithHysteresis(yaw, -1, hysteresis), 1);
  // Having previously chosen 0, a 2 degree excursion is not enough to switch.
  EXPECT_EQ(snapHeadingWithHysteresis(yaw, 0, hysteresis), 0);
  // Past the boundary by more than the band, it does switch, so the
  // hysteresis is sticky and not a latch.
  const double far = kQuarterPi / 2.0 + 8.0 * M_PI / 180.0;
  EXPECT_EQ(snapHeadingWithHysteresis(far, 0, hysteresis), 1);
}

TEST(LatticeSearch, StepSizeIsQuantisedToWholeCells)
{
  auto costmap = freeCostmap();
  LatticeSettings settings = testSettings();

  settings.step_size_m = 0.05;
  LatticeSearch search(settings);
  EXPECT_EQ(search.cellStep(costmap), 1);

  // 0.14 m over a 0.05 m grid is 2.8 cells, which must land on 3 rather than
  // on a fractional step: a diagonal edge of 2.8 cells would not end on a
  // cell centre and the lattice would walk off its own grid.
  settings.step_size_m = 0.14;
  search.setSettings(settings);
  EXPECT_EQ(search.cellStep(costmap), 3);

  // Never zero, however small the request.
  settings.step_size_m = 0.001;
  search.setSettings(settings);
  EXPECT_EQ(search.cellStep(costmap), 1);
}

TEST(LatticeSearch, FootprintIsSweptNotJustTheCentreCell)
{
  auto costmap = freeCostmap();
  // One lethal cell in the middle of the room.
  unsigned int mx = 0;
  unsigned int my = 0;
  ASSERT_TRUE(costmap.worldToMap(3.0, 3.0, mx, my));
  costmap.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);

  LatticeSearch search(testSettings());
  search.buildBlockedMask(costmap);

  // The obstacle cell itself is blocked, and so is every cell within the
  // 0.14 m radius: a centre-only check would let the robot's body pass
  // through the obstacle while its centre missed it.
  EXPECT_TRUE(search.isBlocked(mx, my));
  EXPECT_TRUE(search.isBlocked(mx + 2, my));
  EXPECT_TRUE(search.isBlocked(mx, my - 2));
  // 0.14 m is 2.8 cells, so three cells away in both x and y (0.212 m) is
  // outside the disc and must stay free.
  EXPECT_FALSE(search.isBlocked(mx + 3, my + 3));
  EXPECT_FALSE(search.isBlocked(mx + 10, my));
}

TEST(LatticeSearch, InflatedCostIsNotTreatedAsLethal)
{
  auto costmap = freeCostmap();
  unsigned int mx = 0;
  unsigned int my = 0;
  ASSERT_TRUE(costmap.worldToMap(3.0, 3.0, mx, my));
  // The inflation layer marks the inscribed band using the same robot_radius
  // the search dilates with. Dilating that band as well would apply the
  // footprint twice and close gaps the robot fits through, so only
  // LETHAL_OBSTACLE is dilated.
  costmap.setCost(mx, my, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);

  LatticeSearch search(testSettings());
  search.buildBlockedMask(costmap);
  EXPECT_FALSE(search.isBlocked(mx, my));
}

TEST(LatticeSearch, UnknownSpaceIsImpassableUnlessAllowed)
{
  auto costmap = freeCostmap();
  unsigned int mx = 0;
  unsigned int my = 0;
  ASSERT_TRUE(costmap.worldToMap(3.0, 3.0, mx, my));
  costmap.setCost(mx, my, nav2_costmap_2d::NO_INFORMATION);

  LatticeSettings settings = testSettings();
  settings.allow_unknown = false;
  LatticeSearch strict(settings);
  strict.buildBlockedMask(costmap);
  EXPECT_TRUE(strict.isBlocked(mx, my));

  settings.allow_unknown = true;
  LatticeSearch permissive(settings);
  permissive.buildBlockedMask(costmap);
  EXPECT_FALSE(permissive.isBlocked(mx, my));
}

TEST(LatticeSearch, SegmentCheckWalksEveryInterveningCell)
{
  auto costmap = freeCostmap();
  unsigned int mx = 0;
  unsigned int my = 0;
  ASSERT_TRUE(costmap.worldToMap(3.0, 3.0, mx, my));
  costmap.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);

  LatticeSearch search(testSettings());
  search.buildBlockedMask(costmap);

  // A long edge whose endpoints are both clear but which passes through the
  // blocked region must be rejected. Checking endpoints only would accept it.
  EXPECT_FALSE(search.segmentClear(mx - 20, my, mx + 20, my));
  EXPECT_TRUE(search.segmentClear(mx - 20, my + 20, mx - 10, my + 20));
  // Diagonals are walked the same way.
  EXPECT_FALSE(search.segmentClear(mx - 20, my - 20, mx + 20, my + 20));
}

TEST(LatticeSearch, EveryEmittedSegmentIsAxisSnapped)
{
  auto costmap = freeCostmap();
  LatticeSearch search(testSettings());

  // A goal that is neither straight ahead nor at a clean 45 degrees, so the
  // route has to be composed out of primitives rather than falling out as one.
  const auto result = search.search(costmap, 1.0, 1.0, 0, 4.10, 2.35);
  ASSERT_TRUE(result.success) << result.message;
  ASSERT_GE(result.waypoints.size(), 2u);

  for (std::size_t i = 0; i + 1 < result.waypoints.size(); ++i) {
    const double angle = segmentAngle(result.waypoints, i);
    const double remainder = std::fabs(std::remainder(angle, kQuarterPi));
    EXPECT_LT(remainder, 1e-6)
      << "segment " << i << " leaves the lattice at " << angle << " rad";
    // The stored heading must agree with the geometry, since the plugin
    // writes that heading into the pose orientation.
    EXPECT_EQ(snapHeading(angle), result.waypoints[i].heading);
  }
}

TEST(LatticeSearch, StraightRunsAreMergedIntoSingleSegments)
{
  auto costmap = freeCostmap();
  LatticeSearch search(testSettings());

  // 3 m due east across free space at a 0.05 m step is 60 edges. Merged, it
  // is one segment and two poses; unmerged the controller would see a bead
  // chain of 61 waypoints and stop at each one.
  const auto result = search.search(costmap, 1.0, 1.0, 0, 4.0, 1.0);
  ASSERT_TRUE(result.success) << result.message;
  EXPECT_EQ(result.waypoints.size(), 2u);
  EXPECT_NEAR(result.waypoints.front().y, result.waypoints.back().y, 1e-9);
  EXPECT_EQ(result.waypoints.front().heading, 0);
}

TEST(LatticeSearch, DiagonalGoalIsOneDiagonalRunNotAStaircase)
{
  auto costmap = freeCostmap();
  LatticeSearch search(testSettings());

  // Exactly 45 degrees away. A staircase of alternating cardinal edges would
  // cost the same distance in an 8-connected grid without a turning penalty,
  // so this is the check that turning_cost_weight is actually doing its job
  // and that a diagonal is its own edge type rather than a composite.
  const auto result = search.search(costmap, 1.0, 1.0, 1, 3.0, 3.0);
  ASSERT_TRUE(result.success) << result.message;
  EXPECT_EQ(result.waypoints.size(), 2u);
  EXPECT_EQ(result.waypoints.front().heading, 1);
}

TEST(LatticeSearch, RoutesAroundAnObstacleRatherThanThroughIt)
{
  auto costmap = freeCostmap();
  // A wall spanning most of the room, with a gap at the top.
  for (unsigned int my = 0; my < 90; ++my) {
    costmap.setCost(60, my, nav2_costmap_2d::LETHAL_OBSTACLE);
  }

  LatticeSearch search(testSettings());
  const auto result = search.search(costmap, 1.0, 1.0, 0, 5.0, 1.0);
  ASSERT_TRUE(result.success) << result.message;

  // It must go round: every waypoint has to clear the wall's north end.
  bool went_north = false;
  for (const auto & waypoint : result.waypoints) {
    if (waypoint.y > 4.5) {
      went_north = true;
    }
  }
  EXPECT_TRUE(went_north);

  // And no waypoint may sit inside the swept footprint of the wall.
  search.buildBlockedMask(costmap);
  for (const auto & waypoint : result.waypoints) {
    unsigned int mx = 0;
    unsigned int my = 0;
    ASSERT_TRUE(costmap.worldToMap(waypoint.x, waypoint.y, mx, my));
    EXPECT_FALSE(search.isBlocked(mx, my));
  }
}

TEST(LatticeSearch, FailsClosedWhenTheGoalIsWalledIn)
{
  auto costmap = freeCostmap();
  // Seal a pocket around (5.0, 5.0).
  for (unsigned int i = 80; i <= 110; ++i) {
    costmap.setCost(i, 80, nav2_costmap_2d::LETHAL_OBSTACLE);
    costmap.setCost(i, 110, nav2_costmap_2d::LETHAL_OBSTACLE);
    costmap.setCost(80, i, nav2_costmap_2d::LETHAL_OBSTACLE);
    costmap.setCost(110, i, nav2_costmap_2d::LETHAL_OBSTACLE);
  }

  LatticeSearch search(testSettings());
  const auto result = search.search(costmap, 1.0, 1.0, 0, 4.75, 4.75);
  EXPECT_FALSE(result.success);
  EXPECT_NE(result.message.find("no valid path"), std::string::npos)
    << result.message;
}

TEST(LatticeSearch, RefusesToStartInsideAnObstacle)
{
  auto costmap = freeCostmap();
  unsigned int mx = 0;
  unsigned int my = 0;
  ASSERT_TRUE(costmap.worldToMap(1.0, 1.0, mx, my));
  costmap.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);

  LatticeSearch search(testSettings());
  const auto result = search.search(costmap, 1.0, 1.0, 0, 4.0, 4.0);
  EXPECT_FALSE(result.success);
  EXPECT_NE(result.message.find("start is occupied"), std::string::npos)
    << result.message;
}

TEST(LatticeSearch, RejectsGoalsOffTheCostmap)
{
  auto costmap = freeCostmap();
  LatticeSearch search(testSettings());
  const auto result = search.search(costmap, 1.0, 1.0, 0, 40.0, 40.0);
  EXPECT_FALSE(result.success);
  EXPECT_NE(result.message.find("goal is outside"), std::string::npos)
    << result.message;
}

TEST(LatticeSearch, HonoursCancellation)
{
  auto costmap = freeCostmap();
  LatticeSearch search(testSettings());
  const auto result = search.search(
    costmap, 1.0, 1.0, 0, 5.5, 5.5, []() {return true;});
  EXPECT_FALSE(result.success);
  EXPECT_NE(result.message.find("cancelled"), std::string::npos) << result.message;
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
