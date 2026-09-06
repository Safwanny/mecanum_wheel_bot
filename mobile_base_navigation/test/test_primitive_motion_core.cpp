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

/// The one-primitive-at-a-time invariant, asserted on every tick of a driven
/// state machine rather than sampled from a running robot. A sampler racing a
/// live controller attributes commands to the wrong phase; ticking the core
/// directly cannot.

#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "mobile_base_navigation/primitive_motion_core.hpp"

using mobile_base_navigation::Command;
using mobile_base_navigation::Pose2D;
using mobile_base_navigation::PrimitiveKind;
using mobile_base_navigation::PrimitiveMotionCore;
using mobile_base_navigation::PrimitivePhase;
using mobile_base_navigation::PrimitiveSettings;
using mobile_base_navigation::toString;

namespace
{

constexpr double kDt = 0.05;   // 20 Hz, matching controller_frequency
constexpr double kZero = 1e-9;

/// Commanded velocities are exact by construction, so the diagonal lock is
/// checked far tighter than any measurement tolerance would allow.
constexpr double kLockTolerance = 1e-12;

/**
 * The invariant: at any tick at most one motion family is commanded, and a
 * diagonal has its two linear terms locked to each other.
 */
void expectSinglePrimitive(const Command & command, const std::string & context)
{
  const bool rotating = std::fabs(command.omega) > kZero;
  const bool translating =
    std::fabs(command.vx) > kZero || std::fabs(command.vy) > kZero;

  ASSERT_FALSE(rotating && translating)
    << context << ": blended vx=" << command.vx << " vy=" << command.vy
    << " omega=" << command.omega;

  if (!translating) {
    return;
  }

  const bool has_x = std::fabs(command.vx) > kZero;
  const bool has_y = std::fabs(command.vy) > kZero;
  if (has_x && has_y) {
    // Diagonal: vx and vy locked to the 45 degree ratio. Anything else is a
    // blended translation, which is what this profile forbids.
    EXPECT_NEAR(std::fabs(command.vx), std::fabs(command.vy), kLockTolerance)
      << context << ": diagonal is not locked";
  }
  // Otherwise it is a pure forward/back or a pure strafe, and the unused axis
  // is exactly zero rather than merely small.
}

std::vector<Pose2D> plan(std::vector<std::pair<double, double>> points)
{
  std::vector<Pose2D> waypoints;
  for (const auto & point : points) {
    Pose2D pose;
    pose.x = point.first;
    pose.y = point.second;
    waypoints.push_back(pose);
  }
  return waypoints;
}

/// Perfect execution: the robot goes exactly where it is told, at the
/// commanded velocity. Isolates the state machine from plant behaviour.
struct Simulator
{
  Pose2D pose;
  PrimitiveMotionCore core;
  std::vector<PrimitiveKind> kinds;
  std::vector<PrimitivePhase> phases;

  void integrate(const Command & command)
  {
    const double cos_yaw = std::cos(pose.yaw);
    const double sin_yaw = std::sin(pose.yaw);
    pose.x += (command.vx * cos_yaw - command.vy * sin_yaw) * kDt;
    pose.y += (command.vx * sin_yaw + command.vy * cos_yaw) * kDt;
    pose.yaw += command.omega * kDt;
  }

  void step()
  {
    const Command command = core.computeCommand(pose, kDt);
    expectSinglePrimitive(command, std::string("phase ") + toString(core.phase()));
    if (kinds.empty() || kinds.back() != core.kind()) {
      kinds.push_back(core.kind());
    }
    if (phases.empty() || phases.back() != core.phase()) {
      phases.push_back(core.phase());
    }
    integrate(command);
  }

  bool run(int max_ticks)
  {
    for (int tick = 0; tick < max_ticks && !core.finished(); ++tick) {
      step();
    }
    return core.finished();
  }

  bool used(PrimitiveKind kind) const
  {
    for (const auto & seen : kinds) {
      if (seen == kind) {
        return true;
      }
    }
    return false;
  }
};

}  // namespace

TEST(PrimitiveMotionCore, IdleWithoutAPlanCommandsNothing)
{
  PrimitiveMotionCore core;
  Pose2D pose;
  const Command command = core.computeCommand(pose, kDt);
  EXPECT_DOUBLE_EQ(command.vx, 0.0);
  EXPECT_DOUBLE_EQ(command.vy, 0.0);
  EXPECT_DOUBLE_EQ(command.omega, 0.0);
  EXPECT_EQ(core.kind(), PrimitiveKind::kNone);
}

TEST(PrimitiveMotionCore, StraightEastRunIsPureForward)
{
  Simulator sim;
  sim.pose = Pose2D{0.0, 0.0, 0.0};
  sim.core.setPlan(plan({{0.0, 0.0}, {1.0, 0.0}}), 0.0);
  ASSERT_TRUE(sim.run(2000));

  EXPECT_NEAR(sim.pose.x, 1.0, 0.05);
  EXPECT_NEAR(sim.pose.y, 0.0, 1e-9);
  EXPECT_TRUE(sim.used(PrimitiveKind::kForward));
  EXPECT_FALSE(sim.used(PrimitiveKind::kStrafeLeft));
  EXPECT_FALSE(sim.used(PrimitiveKind::kStrafeRight));
}

TEST(PrimitiveMotionCore, DefaultTurnsToFaceALateralGoalBeforeMoving)
{
  Simulator sim;
  sim.pose = Pose2D{0.0, 0.0, 0.0};
  // Due north while facing east. Orient-then-move is the default, so the
  // robot turns 90 degrees first and then drives forward - its heading shows
  // where it is going, which is the point of the mode.
  sim.core.setPlan(plan({{0.0, 0.0}, {0.0, 1.0}}), 0.0);
  ASSERT_TRUE(sim.run(4000));

  EXPECT_NEAR(sim.pose.y, 1.0, 0.05);
  EXPECT_TRUE(sim.used(PrimitiveKind::kRotate));
  EXPECT_TRUE(sim.used(PrimitiveKind::kForward));
  EXPECT_FALSE(sim.used(PrimitiveKind::kStrafeLeft));
  // ALIGN must finish before EXECUTE begins: no creeping north while still
  // turning, which is the blend the whole profile forbids.
  ASSERT_GE(sim.phases.size(), 2u);
  EXPECT_EQ(sim.phases[0], PrimitivePhase::kAlign);
  EXPECT_EQ(sim.phases[1], PrimitivePhase::kExecute);
}

TEST(PrimitiveMotionCore, StrafeModeMovesLaterallyWithoutTurning)
{
  PrimitiveSettings settings;
  // The opt-out: snap to the nearest heading instead of facing the segment,
  // and a holonomic base crabs sideways with no rotation at all.
  settings.align_to_segment = false;

  Simulator sim;
  sim.core.setSettings(settings);
  sim.pose = Pose2D{0.0, 0.0, 0.0};
  sim.core.setPlan(plan({{0.0, 0.0}, {0.0, 1.0}}), 0.0);
  ASSERT_TRUE(sim.run(2000));

  EXPECT_NEAR(sim.pose.y, 1.0, 0.05);
  EXPECT_NEAR(sim.pose.yaw, 0.0, 1e-9) << "the body must not have rotated";
  EXPECT_TRUE(sim.used(PrimitiveKind::kStrafeLeft));
  EXPECT_FALSE(sim.used(PrimitiveKind::kForward));
}

TEST(PrimitiveMotionCore, FortyFiveDegreeSegmentIsOneDiagonalPrimitive)
{
  PrimitiveSettings settings;
  settings.align_to_segment = false;
  Simulator sim;
  sim.core.setSettings(settings);
  sim.pose = Pose2D{0.0, 0.0, 0.0};
  sim.core.setPlan(plan({{0.0, 0.0}, {1.0, 1.0}}), 0.0);
  ASSERT_TRUE(sim.run(2000));

  EXPECT_NEAR(sim.pose.x, 1.0, 0.05);
  EXPECT_NEAR(sim.pose.y, 1.0, 0.05);
  EXPECT_TRUE(sim.used(PrimitiveKind::kDiagonalFrontLeft));
  // A diagonal must not be decomposed into a forward leg and a strafe leg.
  EXPECT_FALSE(sim.used(PrimitiveKind::kForward));
  EXPECT_FALSE(sim.used(PrimitiveKind::kStrafeLeft));
}

TEST(PrimitiveMotionCore, AllFourDiagonalsAreReachable)
{
  const std::vector<std::pair<std::pair<double, double>, PrimitiveKind>> cases = {
    {{1.0, 1.0}, PrimitiveKind::kDiagonalFrontLeft},
    {{1.0, -1.0}, PrimitiveKind::kDiagonalFrontRight},
    {{-1.0, 1.0}, PrimitiveKind::kDiagonalBackLeft},
    {{-1.0, -1.0}, PrimitiveKind::kDiagonalBackRight},
  };
  PrimitiveSettings settings;
  settings.align_to_segment = false;
  for (const auto & entry : cases) {
    Simulator sim;
    sim.core.setSettings(settings);
    sim.pose = Pose2D{0.0, 0.0, 0.0};
    sim.core.setPlan(plan({{0.0, 0.0}, entry.first}), 0.0);
    ASSERT_TRUE(sim.run(2000));
    EXPECT_TRUE(sim.used(entry.second))
      << "goal (" << entry.first.first << ", " << entry.first.second << ")";
  }
}

TEST(PrimitiveMotionCore, MultiSegmentPathNeverBlendsAcrossAVertex)
{
  PrimitiveSettings settings;
  settings.align_to_segment = false;
  Simulator sim;
  sim.core.setSettings(settings);
  sim.pose = Pose2D{0.0, 0.0, 0.0};
  // East, then north, then north-east. The corners are where a conventional
  // controller rounds off and commands two axes at once.
  sim.core.setPlan(plan({{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {2.0, 2.0}}), 0.0);
  ASSERT_TRUE(sim.run(4000));

  EXPECT_NEAR(sim.pose.x, 2.0, 0.05);
  EXPECT_NEAR(sim.pose.y, 2.0, 0.05);
  EXPECT_TRUE(sim.used(PrimitiveKind::kForward));
  EXPECT_TRUE(sim.used(PrimitiveKind::kStrafeLeft));
  EXPECT_TRUE(sim.used(PrimitiveKind::kDiagonalFrontLeft));
  // expectSinglePrimitive ran on every tick above, corners included.
}

TEST(PrimitiveMotionCore, AlignsBeforeMovingWhenTheYawIsOffLattice)
{
  Simulator sim;
  // 20 degrees is not one of the eight headings, so the body has to rotate
  // onto the lattice before any translation is allowed.
  sim.pose = Pose2D{0.0, 0.0, 20.0 * M_PI / 180.0};
  sim.core.setPlan(plan({{0.0, 0.0}, {1.0, 0.0}}), 0.0);
  ASSERT_TRUE(sim.run(2000));

  ASSERT_FALSE(sim.phases.empty());
  EXPECT_EQ(sim.phases.front(), PrimitivePhase::kAlign);
  EXPECT_TRUE(sim.used(PrimitiveKind::kRotate));
}

TEST(PrimitiveMotionCore, StrafeModeNeverRotatesMoreThanAHalfCell)
{
  PrimitiveSettings settings;
  settings.align_to_segment = false;
  Simulator sim;
  sim.core.setSettings(settings);
  // 20 degrees is not one of the eight headings, so the body rotates onto the
  // lattice - but only onto the NEAREST one, never further than 22.5 degrees.
  sim.pose = Pose2D{0.0, 0.0, 20.0 * M_PI / 180.0};
  sim.core.setPlan(plan({{0.0, 0.0}, {1.0, 0.0}}), 0.0);
  ASSERT_TRUE(sim.run(2000));
  EXPECT_LE(std::fabs(sim.pose.yaw), 22.5 * M_PI / 180.0 + 1e-6);
}

TEST(PrimitiveMotionCore, DegenerateReplanGoesStraightToFinalOrient)
{
  // The reported bug. bt_navigator replans at 1 Hz; a replan issued while the
  // robot already sits on the goal returns a path a few millimetres long
  // whose direction is rounding noise. The robot would turn to face that
  // noise, settle, and be handed another one a second later - random spinning
  // exactly when it should have been settling the goal heading.
  PrimitiveMotionCore core;
  core.setPlan(plan({{1.0, 1.0}, {1.0005, 1.0003}}), M_PI / 2.0);
  EXPECT_EQ(core.segmentCount(), 0u)
    << "a sub-threshold hop must not survive as a drivable segment";

  Pose2D pose{1.0, 1.0, 0.0};
  const Command command = core.computeCommand(pose, kDt);
  EXPECT_EQ(core.phase(), PrimitivePhase::kFinalOrient);
  // Rotating towards the goal yaw, not towards the noise.
  EXPECT_DOUBLE_EQ(command.vx, 0.0);
  EXPECT_DOUBLE_EQ(command.vy, 0.0);
  EXPECT_GT(command.omega, 0.0);
}

TEST(PrimitiveMotionCore, RepeatedReplansOnTheGoalKeepSettlingTheSameHeading)
{
  // The same bug seen the way the robot sees it: the plan is reloaded every
  // tick, as controller_server does, and the heading must converge rather
  // than being restarted.
  PrimitiveMotionCore core;
  const auto degenerate = plan({{1.0, 1.0}, {1.0004, 0.9998}});
  Pose2D pose{1.0, 1.0, 0.0};
  for (int tick = 0; tick < 400; ++tick) {
    core.setPlan(degenerate, M_PI / 2.0);
    const Command command = core.computeCommand(pose, kDt);
    expectSinglePrimitive(command, "degenerate replan");
    EXPECT_DOUBLE_EQ(command.vx, 0.0);
    EXPECT_DOUBLE_EQ(command.vy, 0.0);
    pose.yaw += command.omega * kDt;
  }
  EXPECT_NEAR(pose.yaw, M_PI / 2.0, 0.15);
}

TEST(PrimitiveMotionCore, PhaseOrderIsAlignThenExecuteThenFinalOrient)
{
  Simulator sim;
  sim.pose = Pose2D{0.0, 0.0, 0.1};
  sim.core.setPlan(plan({{0.0, 0.0}, {1.0, 0.0}}), M_PI / 2.0);
  ASSERT_TRUE(sim.run(3000));

  const std::vector<PrimitivePhase> expected = {
    PrimitivePhase::kAlign,
    PrimitivePhase::kExecute,
    PrimitivePhase::kFinalOrient,
    PrimitivePhase::kDone,
  };
  EXPECT_EQ(sim.phases, expected);
  EXPECT_NEAR(sim.pose.yaw, M_PI / 2.0, 0.10);
}

TEST(PrimitiveMotionCore, FinalOrientRotatesWithoutTranslating)
{
  Simulator sim;
  sim.pose = Pose2D{0.0, 0.0, 0.0};
  sim.core.setPlan(plan({{0.0, 0.0}, {1.0, 0.0}}), M_PI);
  ASSERT_TRUE(sim.run(3000));

  // Settling the goal heading must not move the robot off the goal position,
  // which is what a blended final approach would do.
  EXPECT_NEAR(sim.pose.x, 1.0, 0.05);
  EXPECT_NEAR(sim.pose.y, 0.0, 1e-9);
  EXPECT_NEAR(std::fabs(sim.pose.yaw), M_PI, 0.10);
}

TEST(PrimitiveMotionCore, VelocityLimitsAreRespectedOnEveryTick)
{
  PrimitiveSettings settings;
  settings.max_linear_vel = 0.10;
  settings.max_angular_vel = 0.30;

  Simulator sim;
  sim.core.setSettings(settings);
  sim.pose = Pose2D{0.0, 0.0, 0.3};
  sim.core.setPlan(plan({{0.0, 0.0}, {2.0, 2.0}, {2.0, 0.0}}), 1.0);

  for (int tick = 0; tick < 6000 && !sim.core.finished(); ++tick) {
    const Command command = sim.core.computeCommand(sim.pose, kDt);
    expectSinglePrimitive(command, "limits");
    // The resultant speed, not the per-axis component: a diagonal at the
    // per-axis limit would travel at sqrt(2) times the limit.
    EXPECT_LE(std::hypot(command.vx, command.vy), settings.max_linear_vel + 1e-9);
    EXPECT_LE(std::fabs(command.omega), settings.max_angular_vel + 1e-9);
    sim.integrate(command);
  }
  EXPECT_TRUE(sim.core.finished());
}

TEST(PrimitiveMotionCore, OvershootEndsTheSegmentInsteadOfReversing)
{
  PrimitiveMotionCore core;
  core.setPlan(plan({{0.0, 0.0}, {1.0, 0.0}}), 0.0);
  Pose2D pose{0.0, 0.0, 0.0};
  core.computeCommand(pose, kDt);
  ASSERT_EQ(core.phase(), PrimitivePhase::kExecute);

  // Teleport past the waypoint. Straight-line distance to it is still 0.5 m
  // and never negative, so a controller regulating on that would drive back
  // west forever. The along-track remainder is -0.5 and ends the segment,
  // which is why the core regulates on the projection.
  pose.x = 1.5;
  core.computeCommand(pose, kDt);
  EXPECT_NE(core.phase(), PrimitivePhase::kExecute);
}

TEST(PrimitiveMotionCore, DriftBeyondTheRealignBandReturnsToAlign)
{
  PrimitiveMotionCore core;
  core.setPlan(plan({{0.0, 0.0}, {2.0, 0.0}}), 0.0);
  Pose2D pose{0.0, 0.0, 0.0};
  core.computeCommand(pose, kDt);
  ASSERT_EQ(core.phase(), PrimitivePhase::kExecute);

  // Well past realign_yaw_rad, but nowhere near the 22.5 degree point at
  // which the snap itself would pick a different heading.
  pose.yaw = 0.25;
  const Command command = core.computeCommand(pose, kDt);
  EXPECT_EQ(core.phase(), PrimitivePhase::kAlign);
  expectSinglePrimitive(command, "realign");
  EXPECT_DOUBLE_EQ(command.vx, 0.0);
  EXPECT_DOUBLE_EQ(command.vy, 0.0);
  EXPECT_GT(std::fabs(command.omega), 0.0);
}

TEST(PrimitiveMotionCore, RealignBandIsWiderThanTheAlignExitTolerance)
{
  // Equal thresholds chatter: kExecute leaves the moment the estimate crosses
  // the line and kAlign hands straight back, so the robot alternates between
  // rotating and translating without progressing.
  const PrimitiveSettings defaults;
  EXPECT_GT(defaults.realign_yaw_rad, defaults.yaw_tolerance_rad);
}

TEST(PrimitiveMotionCore, DefaultLimitsStayUnderTheVelocitySmootherCeiling)
{
  // Speeds were doubled past anything characterised, so the smoother's
  // [0.5, 0.5, 2.0] is the only thing left bounding them. Commanding past it
  // would ask for a limit that silently never arrives.
  const PrimitiveSettings defaults;
  EXPECT_LE(defaults.max_linear_vel, 0.5);
  EXPECT_LE(defaults.max_angular_vel, 2.0);
  // Floors must stay under the ceilings, or a limit could never be reached.
  EXPECT_LT(defaults.min_linear_vel, defaults.max_linear_vel);
  EXPECT_LT(defaults.min_angular_vel, defaults.max_angular_vel);
}

TEST(PrimitiveMotionCore, OrientBeforeMovingIsTheDefault)
{
  const PrimitiveSettings defaults;
  EXPECT_TRUE(defaults.align_to_segment);
  EXPECT_GT(defaults.min_segment_length_m, 0.0);
}

TEST(PrimitiveMotionCore, ResetDropsThePlan)
{
  PrimitiveMotionCore core;
  core.setPlan(plan({{0.0, 0.0}, {1.0, 0.0}}), 0.0);
  Pose2D pose{0.0, 0.0, 0.0};
  ASSERT_GT(std::fabs(core.computeCommand(pose, kDt).vx), 0.0);

  core.reset();
  const Command command = core.computeCommand(pose, kDt);
  EXPECT_DOUBLE_EQ(command.vx, 0.0);
  EXPECT_DOUBLE_EQ(command.vy, 0.0);
  EXPECT_DOUBLE_EQ(command.omega, 0.0);
}

TEST(PrimitiveMotionCore, GeometryRefreshDoesNotRestartTheStateMachine)
{
  // The plugin re-transforms the plan from map into odom every tick, so this
  // path is taken 20 times a second. If it reset the machine the robot would
  // re-align forever and never leave the first segment.
  PrimitiveMotionCore core;
  const auto waypoints = plan({{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}});
  core.setPlan(waypoints, 0.0);
  Pose2D pose{0.0, 0.0, 0.0};
  for (int tick = 0; tick < 800; ++tick) {
    core.updatePlanGeometry(waypoints, 0.0);
    const Command command = core.computeCommand(pose, kDt);
    expectSinglePrimitive(command, "refresh");
    const double cos_yaw = std::cos(pose.yaw);
    const double sin_yaw = std::sin(pose.yaw);
    pose.x += (command.vx * cos_yaw - command.vy * sin_yaw) * kDt;
    pose.y += (command.vx * sin_yaw + command.vy * cos_yaw) * kDt;
    pose.yaw += command.omega * kDt;
  }
  EXPECT_GT(core.segment(), 0u) << "the machine never advanced past segment 0";
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
