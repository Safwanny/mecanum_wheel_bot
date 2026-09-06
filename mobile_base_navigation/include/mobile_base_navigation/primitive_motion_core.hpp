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

#ifndef MOBILE_BASE_NAVIGATION__PRIMITIVE_MOTION_CORE_HPP_
#define MOBILE_BASE_NAVIGATION__PRIMITIVE_MOTION_CORE_HPP_

#include <cstddef>
#include <string>
#include <vector>

namespace mobile_base_navigation
{

/// Phase of the per-segment state machine.
enum class PrimitivePhase
{
  kIdle,
  /// Rotating the body onto a 45-degree-snapped yaw. Angular only.
  kAlign,
  /// Translating along one segment. Linear only, and one primitive at a time.
  kExecute,
  /// Rotating onto the goal yaw once the last segment is done.
  kFinalOrient,
  kDone,
};

/// Which motion the robot is making, expressed in its own body frame.
enum class PrimitiveKind
{
  kNone,
  kRotate,
  kForward,
  kBackward,
  kStrafeLeft,
  kStrafeRight,
  kDiagonalFrontLeft,
  kDiagonalFrontRight,
  kDiagonalBackLeft,
  kDiagonalBackRight,
};

const char * toString(PrimitivePhase phase);
const char * toString(PrimitiveKind kind);

/// Body-frame direction index (0..7) to the primitive it names.
PrimitiveKind primitiveForBodyDirection(int body_heading);

/// A planar pose, in whatever frame the caller is working in.
struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

/// A velocity command. Angular is about z; linear is in the body frame.
struct Command
{
  double vx{0.0};
  double vy{0.0};
  double omega{0.0};
};

struct PrimitiveSettings
{
  // Defaults are the validated envelope itself: every accuracy measurement in
  // this repo was taken at 0.10 m/s and 0.30 rad/s. MPPI's controller.yaml
  // runs above that with a written justification; this controller has no
  // measurements of its own yet, so it starts inside the envelope rather than
  // borrowing MPPI's headroom.
  double max_linear_vel{0.20};
  double max_angular_vel{0.60};

  // Floors, not deadbands. Below these the command is too small to overcome
  // stiction on a real base, so a segment would end in a crawl that never
  // finishes. Applied only while the error is still outside tolerance.
  double min_linear_vel{0.04};
  double min_angular_vel{0.10};

  // Distance-along-segment regulation. Proportional dominates by design: the
  // error is a monotonically shrinking distance with no steady-state offset
  // to integrate away, so ki exists for completeness and stays at zero.
  double linear_kp{1.2};
  double linear_ki{0.0};
  double linear_kd{0.0};

  // Heading regulation, used in kAlign and kFinalOrient.
  double angular_kp{1.5};
  double angular_ki{0.0};
  double angular_kd{0.0};

  /// A segment counts as finished when the remaining along-track distance
  /// falls under this. Sized above one control tick of travel.
  double segment_tolerance_m{0.03};

  /// How closely the body yaw must sit on its 45-degree-snapped target before
  /// kAlign hands over to kExecute.
  double yaw_tolerance_rad{0.05};

  /// Final orientation tolerance. Kept under the goal checker's
  /// yaw_goal_tolerance so the controller settles before the checker is asked.
  double goal_yaw_tolerance_rad{0.10};

  /// Yaw drift that sends kExecute back to kAlign mid-path. Deliberately
  /// larger than yaw_tolerance_rad: equal values chatter between the two
  /// phases every time the estimate wobbles across the threshold.
  double realign_yaw_rad{0.15};

  /// Integrator ceiling, in the units of each controlled quantity.
  double integral_limit{0.5};

  /**
   * Turn to face each segment before driving it.
   *
   * true (default): ALIGN drives the body yaw onto the segment's own
   * direction, so every EXECUTE_SEGMENT is a pure FORWARD along the way the
   * robot is pointing. This is "orient, then move", and it is what makes the
   * robot's heading legible - you can see where it intends to go.
   *
   * false: ALIGN snaps to the NEAREST of the eight headings instead, at most
   * 22.5 degrees away, and the segment is then executed as whichever
   * body-frame primitive it happens to be - a strafe or a diagonal without
   * turning. Cheaper in rotations, and the reason a holonomic base exists,
   * but the robot crabs sideways and its heading tells you nothing.
   */
  bool align_to_segment{true};

  /**
   * Waypoints closer together than this are dropped when a plan is loaded.
   *
   * A replan issued while the robot is already sitting on the goal returns a
   * path a few millimetres long. Its "direction" is whatever rounding noise
   * says, so the robot turns to face a random heading, and because
   * bt_navigator replans at 1 Hz it does that again a second later. Dropping
   * sub-threshold segments leaves such a plan with no segments at all, and
   * the machine goes straight to FINAL_ORIENT instead of fighting it.
   */
  double min_segment_length_m{0.05};
};

/**
 * The single-primitive state machine, with no ROS in it.
 *
 * ALIGN_TO_SEGMENT_HEADING -> EXECUTE_SEGMENT -> (next segment) ...
 * -> FINAL_ORIENT.
 *
 * The invariant the whole feature exists for: on every tick at most one
 * motion family is commanded. kAlign and kFinalOrient produce angular
 * velocity only; kExecute produces linear velocity only, and that linear
 * velocity always points along one of the eight body-frame directions, so a
 * diagonal has |vx| == |vy| exactly and nothing in between is reachable.
 *
 * Body yaw is held at a 45-degree-snapped value for the whole path. That is
 * what makes strafing and diagonal travel reachable at all: the plan's
 * segment directions are themselves 45-degree-snapped, so the body-frame
 * direction of every segment is exactly a multiple of 45 degrees without the
 * robot having to turn to face where it is going. A holonomic base should not
 * pay a rotation to move sideways, and this is where that is cashed in.
 */
class PrimitiveMotionCore
{
public:
  PrimitiveMotionCore() = default;
  explicit PrimitiveMotionCore(const PrimitiveSettings & settings);

  void setSettings(const PrimitiveSettings & settings);
  const PrimitiveSettings & settings() const {return settings_;}

  /**
   * Load a plan.
   *
   * Segment directions are taken from consecutive waypoint positions, not
   * from waypoint orientations. The lattice planner writes the travel
   * direction into every orientation except the last, which carries the goal
   * yaw instead; deriving directions from positions means the controller
   * never has to know which convention a given pose is using.
   */
  void setPlan(const std::vector<Pose2D> & waypoints, double goal_yaw);

  /**
   * Refresh the waypoint geometry without disturbing the state machine.
   *
   * The plan is planned in map and executed against a pose in the local
   * costmap's odom frame, so it has to be re-transformed every tick as
   * map -> odom moves. Calling setPlan() to do that would reset the phase,
   * the segment index and both integrators sixty times a second. Only the
   * geometry moves here; the phase and segment survive.
   *
   * A size change means a genuinely different plan, and falls back to
   * setPlan() so the machine restarts rather than indexing into stale
   * segments.
   */
  void updatePlanGeometry(const std::vector<Pose2D> & waypoints, double goal_yaw);

  /// Drop the plan and return to kIdle.
  void reset();

  /// One control tick. `dt` is the controller period in seconds.
  Command computeCommand(const Pose2D & current, double dt);

  PrimitivePhase phase() const {return phase_;}
  PrimitiveKind kind() const {return kind_;}
  std::size_t segment() const {return segment_;}
  std::size_t segmentCount() const;
  bool finished() const {return phase_ == PrimitivePhase::kDone;}
  /// Along-track distance still to travel over every remaining segment.
  double distanceRemaining(const Pose2D & current) const;

private:
  /// Drop waypoints too close together to carry a meaningful direction.
  std::vector<Pose2D> filterWaypoints(const std::vector<Pose2D> & waypoints) const;
  void enterPhase(PrimitivePhase phase);
  /// World-frame direction of the segment leaving waypoint `index`.
  double segmentDirection(std::size_t index) const;
  /// Body yaw this segment should be driven at, per align_to_segment.
  double bodyYawFor(std::size_t index, double current_yaw) const;
  double angularPid(double error, double dt);
  double linearPid(double error, double dt);

  PrimitiveSettings settings_;
  std::vector<Pose2D> waypoints_;
  double goal_yaw_{0.0};
  double body_yaw_target_{0.0};
  std::size_t segment_{0};
  PrimitivePhase phase_{PrimitivePhase::kIdle};
  PrimitiveKind kind_{PrimitiveKind::kNone};

  double angular_integral_{0.0};
  double angular_previous_{0.0};
  double linear_integral_{0.0};
  double linear_previous_{0.0};
  bool have_previous_{false};
};

}  // namespace mobile_base_navigation

#endif  // MOBILE_BASE_NAVIGATION__PRIMITIVE_MOTION_CORE_HPP_
