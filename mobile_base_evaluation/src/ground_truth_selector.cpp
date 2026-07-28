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

#include <cmath>

namespace mobile_base_evaluation
{

std::optional<gz::msgs::Pose> select_named_pose(
  const gz::msgs::Pose_V & poses,
  const std::string & entity_name,
  std::string & error)
{
  if (entity_name.empty()) {
    error = "configured entity name is empty";
    return std::nullopt;
  }

  const gz::msgs::Pose * selected = nullptr;
  for (const auto & pose : poses.pose()) {
    if (pose.name() != entity_name) {
      continue;
    }
    if (selected != nullptr) {
      error = "duplicate Gazebo poses named '" + entity_name + "'";
      return std::nullopt;
    }
    selected = &pose;
  }
  if (selected == nullptr) {
    error = "Gazebo entity '" + entity_name + "' is missing";
    return std::nullopt;
  }

  const auto & position = selected->position();
  const auto & orientation = selected->orientation();
  const double quaternion_norm = std::sqrt(
    orientation.x() * orientation.x() +
    orientation.y() * orientation.y() +
    orientation.z() * orientation.z() +
    orientation.w() * orientation.w());
  if (
    !std::isfinite(position.x()) || !std::isfinite(position.y()) ||
    !std::isfinite(position.z()) || !std::isfinite(orientation.x()) ||
    !std::isfinite(orientation.y()) || !std::isfinite(orientation.z()) ||
    !std::isfinite(orientation.w()) || quaternion_norm <= 1e-12)
  {
    error = "Gazebo entity '" + entity_name + "' has an invalid pose";
    return std::nullopt;
  }

  error.clear();
  return *selected;
}

}  // namespace mobile_base_evaluation
