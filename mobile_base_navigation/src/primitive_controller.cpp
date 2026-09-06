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

#include "mobile_base_navigation/primitive_controller.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace mobile_base_navigation
{

void PrimitiveController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = parent.lock();
  name_ = std::move(name);
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  logger_ = node->get_logger();

  const auto declare = [&node, this](const std::string & key, double value) {
      nav2_util::declare_parameter_if_not_declared(
        node, name_ + "." + key, rclcpp::ParameterValue(value));
      double result = value;
      node->get_parameter(name_ + "." + key, result);
      return result;
    };

  settings_.max_linear_vel = declare("max_linear_vel", 0.20);
  settings_.max_angular_vel = declare("max_angular_vel", 0.60);
  settings_.min_linear_vel = declare("min_linear_vel", 0.04);
  settings_.min_angular_vel = declare("min_angular_vel", 0.10);
  settings_.linear_kp = declare("linear_kp", 1.2);
  settings_.linear_ki = declare("linear_ki", 0.0);
  settings_.linear_kd = declare("linear_kd", 0.0);
  settings_.angular_kp = declare("angular_kp", 1.5);
  settings_.angular_ki = declare("angular_ki", 0.0);
  settings_.angular_kd = declare("angular_kd", 0.0);
  settings_.segment_tolerance_m = declare("segment_tolerance_m", 0.03);
  settings_.yaw_tolerance_rad = declare("yaw_tolerance_rad", 0.05);
  settings_.goal_yaw_tolerance_rad = declare("goal_yaw_tolerance_rad", 0.10);
  settings_.realign_yaw_rad = declare("realign_yaw_rad", 0.15);
  settings_.integral_limit = declare("integral_limit", 0.5);
  settings_.min_segment_length_m = declare("min_segment_length_m", 0.05);

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".align_to_segment", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + ".align_to_segment", settings_.align_to_segment);
  transform_tolerance_ = declare("transform_tolerance", 0.30);

  if (settings_.realign_yaw_rad <= settings_.yaw_tolerance_rad) {
    // Equal thresholds chatter: kExecute leaves for kAlign the instant the
    // estimate crosses the line, kAlign hands straight back, and the robot
    // alternates between rotating and translating without progressing.
    RCLCPP_WARN(
      logger_,
      "%s.realign_yaw_rad (%.3f) must exceed yaw_tolerance_rad (%.3f); "
      "raising it to avoid align/execute chatter",
      name_.c_str(), settings_.realign_yaw_rad, settings_.yaw_tolerance_rad);
    settings_.realign_yaw_rad = settings_.yaw_tolerance_rad * 3.0;
  }

  double controller_frequency = 20.0;
  node->get_parameter("controller_frequency", controller_frequency);
  control_period_ = controller_frequency > 0.0 ? 1.0 / controller_frequency : 0.05;

  // These live on controller_server, not on this plugin, and they filter the
  // MEASURED velocity before a controller is called. Nav2's stock
  // min_y_velocity_threshold is 0.5, a diff-drive value that would report
  // every strafe this robot can make as zero. Read them back and complain
  // rather than silently inheriting a broken feedback path, which is the same
  // class of silent failure controller.yaml already documents for odom_topic.
  for (const auto & key : {
      std::string("min_x_velocity_threshold"),
      std::string("min_y_velocity_threshold"),
      std::string("min_theta_velocity_threshold")})
  {
    double threshold = 0.0;
    if (node->get_parameter(key, threshold) && threshold > 0.01) {
      RCLCPP_WARN(
        logger_,
        "controller_server.%s is %.3f, which is above this robot's whole "
        "validated envelope of 0.10 m/s. Lateral and rotational feedback "
        "will read as zero. controller.yaml sets 0.001 for this reason.",
        key.c_str(), threshold);
    }
  }

  core_.setSettings(settings_);

  RCLCPP_INFO(
    logger_,
    "%s: single-primitive controller configured at %.1f Hz, "
    "max %.3f m/s and %.3f rad/s, align_to_segment %s",
    name_.c_str(), controller_frequency,
    settings_.max_linear_vel, settings_.max_angular_vel,
    settings_.align_to_segment ? "true (turn to face each leg)" : "false (strafe)");
}

void PrimitiveController::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up %s", name_.c_str());
  core_.reset();
}

void PrimitiveController::activate()
{
  RCLCPP_INFO(logger_, "Activating %s", name_.c_str());
}

void PrimitiveController::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating %s", name_.c_str());
  core_.reset();
}

void PrimitiveController::reset()
{
  core_.reset();
  cancelling_ = false;
  last_reported_phase_ = PrimitivePhase::kIdle;
  last_reported_kind_ = PrimitiveKind::kNone;
}

bool PrimitiveController::cancel()
{
  // Stopping is a full stop, not a coast: the state machine is dropped and
  // the next tick returns a zero twist. Nothing downstream needs to unwind.
  cancelling_ = true;
  core_.reset();
  return true;
}

void PrimitiveController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (speed_limit <= 0.0) {
    speed_limit_scale_ = 1.0;
  } else if (percentage) {
    speed_limit_scale_ = std::clamp(speed_limit / 100.0, 0.0, 1.0);
  } else {
    speed_limit_scale_ = settings_.max_linear_vel > 0.0 ?
      std::clamp(speed_limit / settings_.max_linear_vel, 0.0, 1.0) :
      1.0;
  }
  PrimitiveSettings scaled = settings_;
  scaled.max_linear_vel = settings_.max_linear_vel * speed_limit_scale_;
  scaled.max_angular_vel = settings_.max_angular_vel * speed_limit_scale_;
  // Floors have to come down with the ceiling, or a speed limit below the
  // floor would be quietly ignored.
  scaled.min_linear_vel = std::min(settings_.min_linear_vel, scaled.max_linear_vel);
  scaled.min_angular_vel = std::min(settings_.min_angular_vel, scaled.max_angular_vel);
  core_.setSettings(scaled);
}

void PrimitiveController::setPlan(const nav_msgs::msg::Path & path)
{
  plan_ = path;
  cancelling_ = false;
  core_.reset();
  last_reported_phase_ = PrimitivePhase::kIdle;
  last_reported_kind_ = PrimitiveKind::kNone;
}

bool PrimitiveController::transformPlan(
  const std::string & frame, const rclcpp::Time & stamp,
  std::vector<Pose2D> & waypoints, double & goal_yaw) const
{
  waypoints.clear();
  if (plan_.poses.empty()) {
    return false;
  }

  geometry_msgs::msg::TransformStamped transform;
  if (plan_.header.frame_id != frame) {
    try {
      transform = tf_->lookupTransform(
        frame, plan_.header.frame_id, stamp,
        tf2::durationFromSec(transform_tolerance_));
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN(
        logger_, "%s: cannot transform the plan from %s to %s: %s",
        name_.c_str(), plan_.header.frame_id.c_str(), frame.c_str(),
        exception.what());
      return false;
    }
  }

  waypoints.reserve(plan_.poses.size());
  for (const auto & source : plan_.poses) {
    geometry_msgs::msg::PoseStamped pose = source;
    if (plan_.header.frame_id != frame) {
      tf2::doTransform(pose, pose, transform);
    }
    Pose2D waypoint;
    waypoint.x = pose.pose.position.x;
    waypoint.y = pose.pose.position.y;
    waypoint.yaw = tf2::getYaw(pose.pose.orientation);
    waypoints.push_back(waypoint);
  }
  goal_yaw = waypoints.back().yaw;
  return true;
}

geometry_msgs::msg::TwistStamped PrimitiveController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  geometry_msgs::msg::TwistStamped command;
  command.header.frame_id = costmap_ros_->getBaseFrameID();
  command.header.stamp = pose.header.stamp;

  if (cancelling_ || plan_.poses.empty()) {
    return command;
  }

  std::vector<Pose2D> waypoints;
  double goal_yaw = 0.0;
  if (!transformPlan(pose.header.frame_id, pose.header.stamp, waypoints, goal_yaw)) {
    throw nav2_core::ControllerTFError(
            "could not transform the plan into " + pose.header.frame_id);
  }

  // The plan is planned in map and executed against a pose in the local
  // costmap's odom frame, so it is re-transformed every tick rather than
  // once: map -> odom moves whenever AMCL corrects, and holding the plan in
  // the wrong frame offsets every segment by the accumulated correction.
  //
  // The body-frame direction is snapped afterwards, which absorbs up to 22.5
  // degrees of map -> odom rotation before a segment could be misclassified.
  // AMCL corrections on this robot are far smaller than that, but it is the
  // limit, and it is why the snap is not skipped as redundant.
  core_.updatePlanGeometry(waypoints, goal_yaw);

  Pose2D current;
  current.x = pose.pose.position.x;
  current.y = pose.pose.position.y;
  current.yaw = tf2::getYaw(pose.pose.orientation);

  const Command result = core_.computeCommand(current, control_period_);
  command.twist.linear.x = result.vx;
  command.twist.linear.y = result.vy;
  command.twist.angular.z = result.omega;

  if (core_.phase() != last_reported_phase_ || core_.kind() != last_reported_kind_) {
    last_reported_phase_ = core_.phase();
    last_reported_kind_ = core_.kind();
    RCLCPP_INFO(
      logger_, "%s: %s / %s, segment %zu of %zu, %.3f m remaining",
      name_.c_str(), toString(core_.phase()), toString(core_.kind()),
      core_.segment(), core_.segmentCount(), core_.distanceRemaining(current));
  }

  return command;
}

}  // namespace mobile_base_navigation

PLUGINLIB_EXPORT_CLASS(
  mobile_base_navigation::PrimitiveController,
  nav2_core::Controller)
