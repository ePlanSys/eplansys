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

// The subprocess route to the same planner. plansys2_epistemic_planner links
// Aletheia in; this plugin runs the `epistemic_planner` binary and reads its
// plan file back. Both are Aletheia, and a fleet mission has to come out the
// same way through either — that is what this checks, on the scenario the
// examples are built around.
//
// The binary is not a build dependency of anything, so a workspace without it
// skips rather than fails. Point ALETHEIA_PLANNER at it, or put it on PATH.

#include <unistd.h>

#include <cstdlib>
#include <memory>
#include <string>

#include "gtest/gtest.h"
#include "plansys2_aletheia_plan_solver/aletheia_plan_solver.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace
{

std::string task(const std::string & name)
{
  return std::string(EPISTEMIC_TASK_DIR) + "/" + name + ".json";
}

std::string mapping(const std::string & name)
{
  return std::string(EPISTEMIC_MAPPING_DIR) + "/" + name + ".json";
}

/// The planner binary, or empty when there is none to run.
std::string planner_command()
{
  const char * env = std::getenv("ALETHEIA_PLANNER");
  return env != nullptr && *env != '\0' ? std::string(env) : std::string("epistemic_planner");
}

}  // namespace

class FleetSolverTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("aletheia_fleet_test");
    solver_.configure(node_, "ALETHEIA");
    node_->set_parameter(rclcpp::Parameter("ALETHEIA.command", planner_command()));
  }

  void use(const std::string & name)
  {
    node_->set_parameter(rclcpp::Parameter("ALETHEIA.task_file", task(name)));
    node_->set_parameter(rclcpp::Parameter("ALETHEIA.action_mapping", mapping(name)));
  }

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  plansys2::AletheiaPlanSolver solver_;
};

TEST_F(FleetSolverTest, SolvesTheCorridorMissionThroughTheBinary)
{
  use("robot-fleet");
  node_->set_parameter(rclcpp::Parameter("ALETHEIA.conditional_plan", "policy"));

  const auto plan = solver_.getPlan("", "");
  if (!plan) {
    GTEST_SKIP() << "the epistemic_planner binary is not available";
  }

  ASSERT_FALSE(plan->items.empty());

  // Drive out, look, and report: three actions on the branch execution takes,
  // and the epistemic names survive translation so the state can follow along.
  EXPECT_EQ(plan->items.front().epistemic_action.rfind("goto-junction_", 0), 0u)
    << plan->items.front().epistemic_action;
  EXPECT_TRUE(plan->items.front().action.rfind("(goto_junction", 0) == 0)
    << plan->items.front().action;

  bool branches = false;
  for (const auto & item : plan->items) {
    if (item.children.size() > 1) {
      branches = true;
    }
  }
  EXPECT_TRUE(branches) << "the corridor's state is undetermined, so the policy branches";
}

TEST_F(FleetSolverTest, FlatteningYieldsARunnableSequence)
{
  use("robot-fleet");
  node_->set_parameter(rclcpp::Parameter("ALETHEIA.conditional_plan", "flatten"));

  const auto plan = solver_.getPlan("", "");
  if (!plan) {
    GTEST_SKIP() << "the epistemic_planner binary is not available";
  }

  // One branch of the policy, as a plain sequence a stock PlanSys2 executor
  // can run: valid only if the corridor turns out the way that branch assumed.
  ASSERT_EQ(plan->items.size(), 3u);
  for (const auto & item : plan->items) {
    EXPECT_TRUE(item.children.empty()) << "flattened items carry no branches";
    EXPECT_GT(item.duration, 0.0f);
  }
}

// Leave through _exit, so the DDS threads never outlive the process.
//
// rclcpp::shutdown() does not finalise the global context; that happens in
// static destruction, inside _dl_fini, while Fast DDS listener threads are
// still running in libraries the loader is unmapping. Under --coverage, which
// is how the rolling job builds, that same exit path writes a .gcda for every
// object file, stretching the window until about one run in ten segfaults with
// every test already passed. Dumping the counters and leaving through _exit
// keeps the coverage data and skips static destruction. __gcov_dump is weak:
// it is null in an uninstrumented build.
extern "C" void __gcov_dump(void) __attribute__((weak));

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();

  if (__gcov_dump) {
    __gcov_dump();
  }
  _exit(result);
}
