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

#include "mobile_base_evaluation/ground_truth_selector.hpp"

#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>

namespace mobile_base_evaluation
{

class GroundTruthSelectorNode : public rclcpp::Node
{
public:
  GroundTruthSelectorNode()
  : Node("ground_truth_selector")
  {
    const auto world = declare_parameter<std::string>("world", "empty");
    entity_name_ = declare_parameter<std::string>("robot_entity", "mobile_base");
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/mobile_base/evaluation/ground_truth");
    gz_topic_ = declare_parameter<std::string>(
      "gz_topic", "/world/" + world + "/dynamic_pose/info");

    publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      output_topic_, rclcpp::SensorDataQoS());
    if (!gz_node_.Subscribe(gz_topic_, &GroundTruthSelectorNode::on_poses, this)) {
      throw std::runtime_error("failed to subscribe to Gazebo topic " + gz_topic_);
    }
    RCLCPP_INFO(
      get_logger(), "Selecting Gazebo entity '%s' from %s",
      entity_name_.c_str(), gz_topic_.c_str());
  }

private:
  void on_poses(const gz::msgs::Pose_V & poses)
  {
    std::string error;
    const auto selected = select_named_pose(poses, entity_name_, error);
    if (!selected.has_value()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "%s", error.c_str());
      return;
    }

    geometry_msgs::msg::PoseStamped output;
    output.header.stamp = get_clock()->now();
    output.header.frame_id = frame_id_;
    output.pose.position.x = selected->position().x();
    output.pose.position.y = selected->position().y();
    output.pose.position.z = selected->position().z();
    output.pose.orientation.x = selected->orientation().x();
    output.pose.orientation.y = selected->orientation().y();
    output.pose.orientation.z = selected->orientation().z();
    output.pose.orientation.w = selected->orientation().w();
    publisher_->publish(output);
  }

  std::string entity_name_;
  std::string frame_id_;
  std::string output_topic_;
  std::string gz_topic_;
  gz::transport::Node gz_node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
};

}  // namespace mobile_base_evaluation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(
      std::make_shared<mobile_base_evaluation::GroundTruthSelectorNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("ground_truth_selector"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
