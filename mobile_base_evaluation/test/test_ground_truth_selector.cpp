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

#include <gtest/gtest.h>
#include <gz/msgs/pose_v.pb.h>

#include <limits>
#include <string>

#include "mobile_base_evaluation/ground_truth_selector.hpp"

namespace
{

void add_pose(gz::msgs::Pose_V & poses, const std::string & name, double x)
{
  auto * pose = poses.add_pose();
  pose->set_name(name);
  pose->mutable_position()->set_x(x);
  pose->mutable_orientation()->set_w(1.0);
}

TEST(GroundTruthSelector, SelectsConfiguredEntityRegardlessOfOrder)
{
  for (const bool robot_first : {true, false}) {
    gz::msgs::Pose_V poses;
    if (robot_first) {
      add_pose(poses, "mobile_base", 1.25);
      add_pose(poses, "moving_obstacle", 9.0);
    } else {
      add_pose(poses, "moving_obstacle", 9.0);
      add_pose(poses, "mobile_base", 1.25);
    }
    std::string error;
    const auto selected = mobile_base_evaluation::select_named_pose(
      poses, "mobile_base", error);
    ASSERT_TRUE(selected.has_value()) << error;
    EXPECT_DOUBLE_EQ(selected->position().x(), 1.25);
  }
}

TEST(GroundTruthSelector, RejectsMissingAndDuplicateEntity)
{
  gz::msgs::Pose_V poses;
  add_pose(poses, "moving_obstacle", 9.0);
  std::string error;
  EXPECT_FALSE(
    mobile_base_evaluation::select_named_pose(
      poses, "mobile_base", error).has_value());
  EXPECT_NE(error.find("missing"), std::string::npos);

  add_pose(poses, "mobile_base", 1.0);
  add_pose(poses, "mobile_base", 2.0);
  EXPECT_FALSE(
    mobile_base_evaluation::select_named_pose(
      poses, "mobile_base", error).has_value());
  EXPECT_NE(error.find("duplicate"), std::string::npos);
}

TEST(GroundTruthSelector, RejectsInvalidPose)
{
  gz::msgs::Pose_V poses;
  add_pose(poses, "mobile_base", std::numeric_limits<double>::quiet_NaN());
  std::string error;
  EXPECT_FALSE(
    mobile_base_evaluation::select_named_pose(
      poses, "mobile_base", error).has_value());
  EXPECT_NE(error.find("invalid"), std::string::npos);
}

}  // namespace
