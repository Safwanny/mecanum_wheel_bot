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

#include "mobile_base_navigation/lattice_planner.hpp"

#include <cmath>
#include <memory>
#include <string>
#include <utility>

#include "nav2_core/planner_exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace mobile_base_navigation
{

namespace
{

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(quaternion);
}

}  // namespace

void LatticePlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = parent.lock();
  name_ = std::move(name);
  tf_ = std::move(tf);
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();
  logger_ = node->get_logger();

  LatticeSettings settings;

  // Default is one costmap cell. A whole number of cells is enforced at
  // search time: a diagonal edge of an arbitrary length would land between
  // cell centres and the lattice would drift off its own grid.
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".step_size_m", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".turning_cost_weight", rclcpp::ParameterValue(0.35));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".cost_penalty_weight", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".tolerance", rclcpp::ParameterValue(0.125));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_planning_time", rclcpp::ParameterValue(2.0));
  // Matches planner.yaml's GridBased block: unknown space stays as
  // impassable as a wall until that is relaxed deliberately.
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".allow_unknown", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".heading_snap_hysteresis_deg", rclcpp::ParameterValue(5.0));

  node->get_parameter(name_ + ".step_size_m", settings.step_size_m);
  node->get_parameter(name_ + ".turning_cost_weight", settings.turning_cost_weight);
  node->get_parameter(name_ + ".cost_penalty_weight", settings.cost_penalty_weight);
  node->get_parameter(name_ + ".tolerance", settings.tolerance_m);
  node->get_parameter(name_ + ".max_planning_time", settings.max_planning_time_s);
  node->get_parameter(name_ + ".allow_unknown", settings.allow_unknown);

  double hysteresis_deg = 5.0;
  node->get_parameter(name_ + ".heading_snap_hysteresis_deg", hysteresis_deg);
  heading_snap_hysteresis_rad_ = hysteresis_deg * M_PI / 180.0;

  if (settings.cost_penalty_weight < 0.0) {
    // A negative weight would make an edge cost less than its own length, and
    // the Euclidean heuristic would stop being admissible: A* would return
    // whatever it found first rather than the cheapest route.
    RCLCPP_WARN(
      logger_,
      "%s.cost_penalty_weight was negative (%.3f); clamping to 0.0",
      name_.c_str(), settings.cost_penalty_weight);
    settings.cost_penalty_weight = 0.0;
  }

  // Taken from the costmap rather than restated, so the planner cannot drift
  // away from the robot_radius the costmap is already inflating with.
  //
  // Measured, not assumed: this reads 0.154 m against a robot_radius of 0.14.
  // Nav2 builds a 16-gon from robot_radius and then pads it by
  // footprint_padding (0.01 default) per coordinate, so a vertex at 45
  // degrees moves from (0.099, 0.099) to (0.109, 0.109) and its norm is
  // 0.1541. That is the padded footprint the rest of Nav2 collision-checks
  // with, so matching it keeps the planner and the costmap agreeing; it is
  // simply not the 0.14 anyone reading planner.yaml would expect to see
  // logged. Still far inside the 0.925 m north gap: 2 * 0.154 = 0.308.
  settings.robot_radius_m =
    costmap_ros->getLayeredCostmap()->getCircumscribedRadius();

  search_.setSettings(settings);

  RCLCPP_INFO(
    logger_,
    "%s: eight-heading lattice planner configured. step %.3f m, "
    "turning weight %.2f, radius %.3f m, allow_unknown %s",
    name_.c_str(), settings.step_size_m, settings.turning_cost_weight,
    settings.robot_radius_m, settings.allow_unknown ? "true" : "false");
}

void LatticePlanner::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up %s", name_.c_str());
}

void LatticePlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating %s", name_.c_str());
  previous_start_heading_ = -1;
}

void LatticePlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating %s", name_.c_str());
}

nav_msgs::msg::Path LatticePlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame_;
  path.header.stamp = start.header.stamp;

  if (start.header.frame_id != global_frame_ || goal.header.frame_id != global_frame_) {
    throw nav2_core::PlannerTFError(
            "start and goal must already be in the costmap frame " + global_frame_);
  }

  const double start_yaw = tf2::getYaw(start.pose.orientation);
  // Sticky snap. A start yaw parked near a 22.5 degree boundary would
  // otherwise pick a different heading on each replan, and the first segment
  // of the plan would alternate between two primitives while the robot sat
  // still. The hysteresis makes the previous choice win a close contest.
  const int start_heading = snapHeadingWithHysteresis(
    start_yaw, previous_start_heading_, heading_snap_hysteresis_rad_);
  previous_start_heading_ = start_heading;

  LatticeResult result;
  {
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
    result = search_.search(
      *costmap_,
      start.pose.position.x, start.pose.position.y, start_heading,
      goal.pose.position.x, goal.pose.position.y,
      cancel_checker);
  }

  if (!result.success) {
    if (result.message.find("cancelled") != std::string::npos) {
      throw nav2_core::PlannerCancelled(result.message);
    }
    if (result.message.find("start is outside") != std::string::npos) {
      throw nav2_core::StartOutsideMapBounds(result.message);
    }
    if (result.message.find("goal is outside") != std::string::npos) {
      throw nav2_core::GoalOutsideMapBounds(result.message);
    }
    if (result.message.find("start is occupied") != std::string::npos) {
      throw nav2_core::StartOccupied(result.message);
    }
    if (result.message.find("budget") != std::string::npos) {
      throw nav2_core::PlannerTimedOut(result.message);
    }
    throw nav2_core::NoValidPathCouldBeFound(result.message);
  }

  path.poses.reserve(result.waypoints.size());
  for (std::size_t i = 0; i < result.waypoints.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = result.waypoints[i].x;
    pose.pose.position.y = result.waypoints[i].y;
    pose.pose.position.z = 0.0;
    // Every orientation except the last is the TRAVEL direction of the
    // segment leaving that waypoint, not a commanded body yaw: the controller
    // realises a 90 degree travel direction as a strafe, without turning. The
    // final pose is the exception and carries the requested goal yaw, which
    // is what the goal checker compares against.
    const bool last = i + 1 == result.waypoints.size();
    pose.pose.orientation = last ?
      goal.pose.orientation :
      yawToQuaternion(headingAngle(result.waypoints[i].heading));
    path.poses.push_back(pose);
  }

  RCLCPP_DEBUG(
    logger_,
    "%s: planned %zu segments in %zu expansions",
    name_.c_str(),
    path.poses.empty() ? 0u : path.poses.size() - 1, result.expansions);

  return path;
}

}  // namespace mobile_base_navigation

PLUGINLIB_EXPORT_CLASS(
  mobile_base_navigation::LatticePlanner,
  nav2_core::GlobalPlanner)
