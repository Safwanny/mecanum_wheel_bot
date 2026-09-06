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

#ifndef MOBILE_BASE_NAVIGATION__PRIMITIVE_CONTROLLER_HPP_
#define MOBILE_BASE_NAVIGATION__PRIMITIVE_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "mobile_base_navigation/primitive_motion_core.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "tf2_ros/buffer.h"

namespace mobile_base_navigation
{

/**
 * A controller that drives one motion primitive at a time.
 *
 * A transparent drop-in for the FollowPath slot: it produces the same
 * TwistStamped on the same interface MPPI does, so the velocity smoother,
 * collision monitor and twist_mux downstream need no change whatsoever.
 *
 * It expects a path whose segments are already axis-snapped, which is what
 * LatticePlanner produces. Given an arbitrary curved path it will still drive
 * the polyline through its own vertices, but every heading change becomes a
 * full stop, so pairing it with SmacPlanner2D is legal and slow rather than
 * legal and good.
 */
class PrimitiveController : public nav2_core::Controller
{
public:
  PrimitiveController() = default;
  ~PrimitiveController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  bool cancel() override;
  void reset() override;

private:
  /// Re-express the stored plan in `frame`, at `stamp`. The plan arrives in
  /// map and the robot pose arrives in the local costmap's odom frame, so
  /// this has to be redone every tick: map -> odom moves whenever AMCL
  /// corrects, and mixing the two frames silently offsets every segment.
  bool transformPlan(
    const std::string & frame, const rclcpp::Time & stamp,
    std::vector<Pose2D> & waypoints, double & goal_yaw) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string name_;
  rclcpp::Logger logger_{rclcpp::get_logger("PrimitiveController")};

  nav_msgs::msg::Path plan_;
  PrimitiveMotionCore core_;
  PrimitiveSettings settings_;
  double control_period_{0.05};
  double speed_limit_scale_{1.0};
  double transform_tolerance_{0.3};
  bool cancelling_{false};

  PrimitivePhase last_reported_phase_{PrimitivePhase::kIdle};
  PrimitiveKind last_reported_kind_{PrimitiveKind::kNone};
};

}  // namespace mobile_base_navigation

#endif  // MOBILE_BASE_NAVIGATION__PRIMITIVE_CONTROLLER_HPP_
