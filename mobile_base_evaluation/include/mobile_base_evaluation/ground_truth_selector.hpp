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

#ifndef MOBILE_BASE_EVALUATION__GROUND_TRUTH_SELECTOR_HPP_
#define MOBILE_BASE_EVALUATION__GROUND_TRUTH_SELECTOR_HPP_

#include <gz/msgs/pose_v.pb.h>

#include <optional>
#include <string>

namespace mobile_base_evaluation
{

std::optional<gz::msgs::Pose> select_named_pose(
  const gz::msgs::Pose_V & poses,
  const std::string & entity_name,
  std::string & error);

}  // namespace mobile_base_evaluation

#endif  // MOBILE_BASE_EVALUATION__GROUND_TRUTH_SELECTOR_HPP_
