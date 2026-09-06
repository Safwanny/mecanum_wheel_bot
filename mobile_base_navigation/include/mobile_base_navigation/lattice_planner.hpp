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

#ifndef MOBILE_BASE_NAVIGATION__LATTICE_PLANNER_HPP_
#define MOBILE_BASE_NAVIGATION__LATTICE_PLANNER_HPP_

#include <memory>
#include <string>

#include "mobile_base_navigation/lattice_search.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace mobile_base_navigation
{

/**
 * A global planner that only emits paths this robot can execute as single
 * motion primitives.
 *
 * Every returned segment is a straight run along one of eight headings, 45
 * degrees apart, and every vertex is a heading change. Nothing curves, and no
 * segment asks for translation and rotation at once. This is the planning half
 * of the "one motion at a time" profile; SmacPlanner2D remains the default and
 * is not touched.
 *
 * The plugin loads into planner_server exactly where SmacPlanner2D does and
 * reuses the global_costmap it is handed, so the footprint, inflation and
 * resolution decisions documented in planner.yaml all still apply.
 */
class LatticePlanner : public nav2_core::GlobalPlanner
{
public:
  LatticePlanner() = default;
  ~LatticePlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string name_;
  std::string global_frame_;
  rclcpp::Logger logger_{rclcpp::get_logger("LatticePlanner")};

  LatticeSearch search_;
  double heading_snap_hysteresis_rad_{0.0};
  /// Heading the last plan started from, so the snap can be sticky.
  int previous_start_heading_{-1};
};

}  // namespace mobile_base_navigation

#endif  // MOBILE_BASE_NAVIGATION__LATTICE_PLANNER_HPP_
