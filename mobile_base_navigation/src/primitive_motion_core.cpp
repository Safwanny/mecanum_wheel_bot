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

#include "mobile_base_navigation/primitive_motion_core.hpp"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "mobile_base_navigation/lattice_search.hpp"

namespace mobile_base_navigation
{

namespace
{

constexpr double kTwoPi = 6.28318530717958623200;

double shortestAngularDistance(double from, double to)
{
  return std::remainder(to - from, kTwoPi);
}

double clampAbs(double value, double limit)
{
  return std::max(-limit, std::min(limit, value));
}

/// Apply a floor without changing sign, used to escape stiction.
double applyFloor(double value, double floor_value)
{
  if (value == 0.0) {
    return 0.0;
  }
  const double magnitude = std::max(std::fabs(value), floor_value);
  return std::copysign(magnitude, value);
}

}  // namespace

const char * toString(PrimitivePhase phase)
{
  switch (phase) {
    case PrimitivePhase::kIdle: return "IDLE";
    case PrimitivePhase::kAlign: return "ALIGN_TO_SEGMENT_HEADING";
    case PrimitivePhase::kExecute: return "EXECUTE_SEGMENT";
    case PrimitivePhase::kFinalOrient: return "FINAL_ORIENT";
    case PrimitivePhase::kDone: return "DONE";
  }
  return "UNKNOWN";
}

const char * toString(PrimitiveKind kind)
{
  switch (kind) {
    case PrimitiveKind::kNone: return "NONE";
    case PrimitiveKind::kRotate: return "ROTATE";
    case PrimitiveKind::kForward: return "FORWARD";
    case PrimitiveKind::kBackward: return "BACKWARD";
    case PrimitiveKind::kStrafeLeft: return "STRAFE_LEFT";
    case PrimitiveKind::kStrafeRight: return "STRAFE_RIGHT";
    case PrimitiveKind::kDiagonalFrontLeft: return "DIAGONAL_FRONT_LEFT";
    case PrimitiveKind::kDiagonalFrontRight: return "DIAGONAL_FRONT_RIGHT";
    case PrimitiveKind::kDiagonalBackLeft: return "DIAGONAL_BACK_LEFT";
    case PrimitiveKind::kDiagonalBackRight: return "DIAGONAL_BACK_RIGHT";
  }
  return "UNKNOWN";
}

PrimitiveKind primitiveForBodyDirection(int body_heading)
{
  switch (((body_heading % kNumHeadings) + kNumHeadings) % kNumHeadings) {
    case 0: return PrimitiveKind::kForward;
    case 1: return PrimitiveKind::kDiagonalFrontLeft;
    case 2: return PrimitiveKind::kStrafeLeft;
    case 3: return PrimitiveKind::kDiagonalBackLeft;
    case 4: return PrimitiveKind::kBackward;
    case 5: return PrimitiveKind::kDiagonalBackRight;
    case 6: return PrimitiveKind::kStrafeRight;
    default: return PrimitiveKind::kDiagonalFrontRight;
  }
}

PrimitiveMotionCore::PrimitiveMotionCore(const PrimitiveSettings & settings)
: settings_(settings)
{
}

void PrimitiveMotionCore::setSettings(const PrimitiveSettings & settings)
{
  settings_ = settings;
}

std::vector<Pose2D> PrimitiveMotionCore::filterWaypoints(
  const std::vector<Pose2D> & waypoints) const
{
  // A replan issued while the robot is already on the goal returns a path a
  // few millimetres long whose atan2 is pure rounding noise. Without this the
  // robot turns to face that noise, and bt_navigator hands it a fresh one
  // every second - which is exactly the random spinning seen near a goal.
  std::vector<Pose2D> kept;
  for (const auto & waypoint : waypoints) {
    if (kept.empty()) {
      kept.push_back(waypoint);
      continue;
    }
    if (std::hypot(waypoint.x - kept.back().x, waypoint.y - kept.back().y) >=
      settings_.min_segment_length_m)
    {
      kept.push_back(waypoint);
    }
  }
  if (kept.size() == 1 && !waypoints.empty()) {
    // Everything collapsed: keep the goal pose so the machine still knows
    // where it is, but with no segments it goes straight to FINAL_ORIENT.
    kept.back() = waypoints.back();
  }
  return kept;
}

void PrimitiveMotionCore::setPlan(const std::vector<Pose2D> & waypoints, double goal_yaw)
{
  waypoints_ = filterWaypoints(waypoints);
  goal_yaw_ = goal_yaw;
  segment_ = 0;
  phase_ = PrimitivePhase::kIdle;
  kind_ = PrimitiveKind::kNone;
  enterPhase(PrimitivePhase::kIdle);
}

void PrimitiveMotionCore::updatePlanGeometry(
  const std::vector<Pose2D> & waypoints, double goal_yaw)
{
  // Filter first, then compare filtered against filtered: comparing the raw
  // incoming size against the stored filtered size would differ on every tick
  // and restart the machine 20 times a second.
  auto filtered = filterWaypoints(waypoints);
  if (filtered.size() != waypoints_.size() || phase_ == PrimitivePhase::kIdle) {
    setPlan(waypoints, goal_yaw);
    return;
  }
  waypoints_ = std::move(filtered);
  goal_yaw_ = goal_yaw;
}

void PrimitiveMotionCore::reset()
{
  waypoints_.clear();
  segment_ = 0;
  enterPhase(PrimitivePhase::kIdle);
  kind_ = PrimitiveKind::kNone;
}

std::size_t PrimitiveMotionCore::segmentCount() const
{
  return waypoints_.size() < 2 ? 0 : waypoints_.size() - 1;
}

double PrimitiveMotionCore::segmentDirection(std::size_t index) const
{
  const Pose2D & from = waypoints_[index];
  const Pose2D & to = waypoints_[index + 1];
  return std::atan2(to.y - from.y, to.x - from.x);
}

double PrimitiveMotionCore::bodyYawFor(std::size_t index, double current_yaw) const
{
  if (settings_.align_to_segment) {
    // Face the way we are about to travel. Every EXECUTE_SEGMENT is then a
    // pure FORWARD, and the robot's heading shows its intent.
    return segmentDirection(index);
  }
  // Snap to the nearest of the eight headings instead - at most 22.5 degrees
  // of rotation, after which a 45-degree-snapped segment direction is exactly
  // a body-frame primitive and the base strafes or moves diagonally without
  // turning at all.
  return headingAngle(snapHeading(current_yaw));
}

void PrimitiveMotionCore::enterPhase(PrimitivePhase phase)
{
  phase_ = phase;
  // Reset both integrators and both derivative histories on every transition.
  // An integrator carried across a phase change is winding up against an
  // error it was never regulating: the align integral would dump itself into
  // the first translation tick, which is exactly the blended command this
  // controller exists to make impossible.
  angular_integral_ = 0.0;
  linear_integral_ = 0.0;
  angular_previous_ = 0.0;
  linear_previous_ = 0.0;
  have_previous_ = false;
}

double PrimitiveMotionCore::angularPid(double error, double dt)
{
  angular_integral_ = clampAbs(
    angular_integral_ + error * dt, settings_.integral_limit);
  double derivative = 0.0;
  if (have_previous_ && dt > 0.0) {
    derivative = (error - angular_previous_) / dt;
  }
  angular_previous_ = error;
  have_previous_ = true;
  return settings_.angular_kp * error +
         settings_.angular_ki * angular_integral_ +
         settings_.angular_kd * derivative;
}

double PrimitiveMotionCore::linearPid(double error, double dt)
{
  linear_integral_ = clampAbs(
    linear_integral_ + error * dt, settings_.integral_limit);
  double derivative = 0.0;
  if (have_previous_ && dt > 0.0) {
    derivative = (error - linear_previous_) / dt;
  }
  linear_previous_ = error;
  have_previous_ = true;
  return settings_.linear_kp * error +
         settings_.linear_ki * linear_integral_ +
         settings_.linear_kd * derivative;
}

double PrimitiveMotionCore::distanceRemaining(const Pose2D & current) const
{
  if (segmentCount() == 0 || segment_ >= segmentCount()) {
    return 0.0;
  }
  const Pose2D & target = waypoints_[segment_ + 1];
  double total = std::hypot(target.x - current.x, target.y - current.y);
  for (std::size_t i = segment_ + 1; i + 1 < waypoints_.size(); ++i) {
    total += std::hypot(
      waypoints_[i + 1].x - waypoints_[i].x,
      waypoints_[i + 1].y - waypoints_[i].y);
  }
  return total;
}

Command PrimitiveMotionCore::computeCommand(const Pose2D & current, double dt)
{
  Command command;
  if (waypoints_.empty()) {
    kind_ = PrimitiveKind::kNone;
    return command;
  }

  if (phase_ == PrimitivePhase::kIdle) {
    segment_ = 0;
    if (segmentCount() == 0) {
      // Nothing left to drive - only the final heading. Reaching this without
      // an align step is what stops a replan-on-the-goal from spinning the
      // robot away from the orientation it was busy settling.
      enterPhase(PrimitivePhase::kFinalOrient);
    } else {
      body_yaw_target_ = bodyYawFor(0, current.yaw);
      enterPhase(PrimitivePhase::kAlign);
    }
  }

  if (phase_ == PrimitivePhase::kDone) {
    kind_ = PrimitiveKind::kNone;
    return command;
  }

  if (phase_ == PrimitivePhase::kAlign) {
    const double error = shortestAngularDistance(current.yaw, body_yaw_target_);
    if (std::fabs(error) <= settings_.yaw_tolerance_rad) {
      enterPhase(PrimitivePhase::kExecute);
    } else {
      kind_ = PrimitiveKind::kRotate;
      command.omega = clampAbs(
        applyFloor(angularPid(error, dt), settings_.min_angular_vel),
        settings_.max_angular_vel);
      return command;
    }
  }

  if (phase_ == PrimitivePhase::kExecute) {
    while (segment_ < segmentCount()) {
      const Pose2D & from = waypoints_[segment_];
      const Pose2D & to = waypoints_[segment_ + 1];
      const double seg_dx = to.x - from.x;
      const double seg_dy = to.y - from.y;
      const double seg_length = std::hypot(seg_dx, seg_dy);
      if (seg_length < 1e-9) {
        ++segment_;
        continue;
      }
      const double ux = seg_dx / seg_length;
      const double uy = seg_dy / seg_length;
      // Along-track remainder, not straight-line distance to the waypoint.
      // Straight-line distance never goes negative, so a robot that overshoots
      // would be driven back and forth across the waypoint forever.
      const double remaining =
        (to.x - current.x) * ux + (to.y - current.y) * uy;
      if (remaining <= settings_.segment_tolerance_m) {
        ++segment_;
        if (segment_ < segmentCount() && settings_.align_to_segment) {
          // Turn to face the next leg before driving it.
          body_yaw_target_ = bodyYawFor(segment_, current.yaw);
          enterPhase(PrimitivePhase::kAlign);
          return computeCommand(current, dt);
        }
        enterPhase(PrimitivePhase::kExecute);
        continue;
      }

      // Drift correction. Re-aligning mid-path costs a rotation, so the
      // threshold is deliberately looser than the one kAlign exits on.
      const double yaw_error = shortestAngularDistance(current.yaw, body_yaw_target_);
      if (std::fabs(yaw_error) > settings_.realign_yaw_rad) {
        enterPhase(PrimitivePhase::kAlign);
        return computeCommand(current, dt);
      }

      // Snap the body-frame direction. Both the segment direction and the
      // body yaw are already 45-degree multiples, so this is a no-op in exact
      // arithmetic; it is here so that accumulated floating-point error can
      // never leak a ninth direction past the one-primitive invariant.
      const double world_direction = std::atan2(seg_dy, seg_dx);
      const int body_heading = snapHeading(world_direction - body_yaw_target_);
      kind_ = primitiveForBodyDirection(body_heading);

      double speed = linearPid(remaining, dt);
      speed = clampAbs(
        applyFloor(speed, settings_.min_linear_vel), settings_.max_linear_vel);
      command.vx = speed * headingCos(body_heading);
      command.vy = speed * headingSin(body_heading);
      command.omega = 0.0;
      return command;
    }
    enterPhase(PrimitivePhase::kFinalOrient);
  }

  if (phase_ == PrimitivePhase::kFinalOrient) {
    const double error = shortestAngularDistance(current.yaw, goal_yaw_);
    if (std::fabs(error) <= settings_.goal_yaw_tolerance_rad) {
      enterPhase(PrimitivePhase::kDone);
      kind_ = PrimitiveKind::kNone;
      return command;
    }
    kind_ = PrimitiveKind::kRotate;
    command.omega = clampAbs(
      applyFloor(angularPid(error, dt), settings_.min_angular_vel),
      settings_.max_angular_vel);
  }

  return command;
}

}  // namespace mobile_base_navigation
