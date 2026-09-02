// Copyright 2026 Intelligent Robotics Lab
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

// The loop guard.
//
// A plan that fails, is replanned, and fails the same way is a loop: the
// planner is being asked the same question and answering it the same way, and
// the executor would keep driving the robot into the same failure. What marks
// the loop is not the number of replans -- a mission that legitimately
// replans many times while getting further is healthy -- but replanning with
// nothing having completed in between.
//
// The counting itself is what these tests pin down, since the state machine
// around it needs a whole graph to exercise and this does not.

#include <map>
#include <memory>
#include <string>

#include "gtest/gtest.h"

#include "plansys2_executor/ActionExecutor.hpp"
#include "plansys2_executor/ExecutorNode.hpp"

#include "rclcpp/rclcpp.hpp"

namespace
{

/// An ExecutorNode with the counting reachable, so the guard can be driven
/// without standing up a plan, a tree and a set of performers.
class CountingExecutor : public plansys2::ExecutorNode
{
public:
  using plansys2::ExecutorNode::completed_actions;
  using plansys2::ExecutorNode::replanning_is_making_progress;
};

}  // namespace

TEST(ReplanGuard, CountsOnlyActionsThatFinished)
{
  auto node = std::make_shared<CountingExecutor>();

  auto action_map =
    std::make_shared<std::map<std::string, plansys2::ActionExecutionInfo>>();

  // No map at all, and an empty one, are both "nothing has completed" rather
  // than an error: the first replan happens before any action exists.
  EXPECT_EQ(node->completed_actions(nullptr), 0u);
  EXPECT_EQ(node->completed_actions(action_map), 0u);

  // An entry whose executor was never assigned has not completed either. The
  // executor publishes those so a subscriber knows the action exists.
  (*action_map)["(move r1 a b):0"] = plansys2::ActionExecutionInfo();
  EXPECT_EQ(node->completed_actions(action_map), 0u);
}

TEST(ReplanGuard, StopsAfterReplanningWithNothingCompleting)
{
  auto node = std::make_shared<CountingExecutor>();
  node->set_parameter(rclcpp::Parameter("max_replans_without_progress", 3));

  plansys2::PlanRuntineInfo runtime_info;
  runtime_info.action_map =
    std::make_shared<std::map<std::string, plansys2::ActionExecutionInfo>>();

  // Nothing ever completes, so every replan is the same question asked again.
  // Three are tolerated; the fourth is the one that says so.
  EXPECT_TRUE(node->replanning_is_making_progress(runtime_info)) << "replan 1";
  EXPECT_TRUE(node->replanning_is_making_progress(runtime_info)) << "replan 2";
  EXPECT_TRUE(node->replanning_is_making_progress(runtime_info)) << "replan 3";
  EXPECT_FALSE(node->replanning_is_making_progress(runtime_info))
    << "the executor would keep driving the same failure";
}

TEST(ReplanGuard, ZeroMeansKeepTrying)
{
  auto node = std::make_shared<CountingExecutor>();
  node->set_parameter(rclcpp::Parameter("max_replans_without_progress", 0));

  plansys2::PlanRuntineInfo runtime_info;
  runtime_info.action_map =
    std::make_shared<std::map<std::string, plansys2::ActionExecutionInfo>>();

  // A deployment that would rather keep trying can say so, and then the guard
  // never fires however long the loop runs.
  for (int i = 0; i < 20; ++i) {
    EXPECT_TRUE(node->replanning_is_making_progress(runtime_info)) << "replan " << i;
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
