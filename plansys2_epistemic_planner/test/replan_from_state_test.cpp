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

// Replanning from where the mission got to.
//
// The executor's answer to a failed plan is to replan, and for an epistemic
// mission the plan that failed usually failed because the world did something
// the policy did not anticipate. Planning again from the model grounding
// produced would then start from a belief the divergence has already
// disproved: the robot would be told to go and find out what it has just found
// out, or worse, to act on a possibility it has ruled out.
//
// What the solver has to do instead is take the model the epistemic state
// reached. These tests publish a state on the topic the solver listens to and
// check that the plan changes accordingly.

#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "plansys2_epistemic_planner/epistemic_plan_solver.hpp"
#include "plansys2_epistemic_planner/parser.hpp"
#include "plansys2_epistemic_planner/state_json.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"

namespace
{

std::string task_path()
{
  return std::string(EPISTEMIC_TEST_TASK_DIR) + "/robot-fleet.json";
}

std::string mapping_path()
{
  return std::string(EPISTEMIC_EXAMPLE_MAPPING_DIR) + "/robot-fleet.json";
}

/// The model the corridor mission reaches once r1 has looked and r2 has not
/// been told: two worlds still, but r1's accessibility is now reflexive, so r1
/// knows which one is actual while r2 still cannot tell them apart.
///
/// Built by hand instead of by running the mission, so that the test says what
/// state it is planning from without depending on the executor to produce it.
EpistemicState after_r1_has_looked(const PlanningTask & task)
{
  const auto blocked = task.atom_index.at("blocked");
  const auto r1 = task.agent_index.at("r1");
  const auto r2 = task.agent_index.at("r2");

  EpistemicState s;
  s.allocate(
    2, static_cast<std::uint32_t>(task.num_atoms()),
    static_cast<std::uint32_t>(task.num_agents()));

  // w0: the corridor is clear. w1: it is blocked.
  s.set_atom(1, blocked);

  // r1 has looked, so it can tell the two apart.
  s.add_edge(r1, 0, 0);
  s.add_edge(r1, 1, 1);

  // r2 has not been told, so it still cannot.
  s.add_edge(r2, 0, 0);
  s.add_edge(r2, 0, 1);
  s.add_edge(r2, 1, 0);
  s.add_edge(r2, 1, 1);

  // The corridor is in fact clear.
  s.set_designated(0);
  return s;
}

bool branches(const plansys2_msgs::msg::Plan & plan)
{
  for (const auto & item : plan.items) {
    if (item.children.size() > 1) {
      return true;
    }
  }
  return false;
}

}  // namespace

class ReplanFromState : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("epistemic_replan_test");
    solver_.configure(node_, "EPISTEMIC");

    node_->set_parameter(rclcpp::Parameter("EPISTEMIC.task_file", task_path()));
    node_->set_parameter(rclcpp::Parameter("EPISTEMIC.action_mapping", mapping_path()));
    node_->set_parameter(rclcpp::Parameter("EPISTEMIC.strategy", "aostar"));
    node_->set_parameter(rclcpp::Parameter("EPISTEMIC.conditional_plan", "policy"));

    publisher_ = node_->create_publisher<std_msgs::msg::String>(
      "epistemic_state/state", rclcpp::QoS(1).transient_local());
  }

  /// Publish a state and let the solver's subscription receive it.
  void publish_state(const std::string & payload)
  {
    std_msgs::msg::String msg;
    msg.data = payload;
    publisher_->on_activate();
    publisher_->publish(msg);

    // Spun rather than slept on: the subscription is on this same node, so a
    // few spins deliver it and a sleep would only make the test slower.
    for (int i = 0; i < 50; ++i) {
      rclcpp::spin_some(node_->get_node_base_interface());
    }
  }

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr publisher_;
  plansys2::EpistemicPlanSolver solver_;
};

TEST_F(ReplanFromState, WithNoStatePublishedThePlanStartsFromTheTask)
{
  // The baseline the rest of the file is measured against: nobody knows
  // whether the corridor is blocked, so the solution has to branch on what the
  // looking turns up.
  const auto plan = solver_.getPlan("", "");

  ASSERT_TRUE(plan.has_value());
  EXPECT_TRUE(branches(*plan))
    << "the corridor's state is undetermined, so planning from the task must branch";
}

TEST_F(ReplanFromState, APublishedModelIsWhatThePlanStartsFrom)
{
  const auto task = load_task(task_path());
  const auto reached = after_r1_has_looked(task);

  publish_state("{\"model\": " + plansys2::state_to_json(task, reached) + "}");

  const auto plan = solver_.getPlan("", "");
  ASSERT_TRUE(plan.has_value());

  // r1 has already looked, so there is nothing left to find out and nothing to
  // branch on. Planning from the task instead would send it to look again.
  EXPECT_FALSE(branches(*plan))
    << "r1 already knows, so the replan has no contingency to plan for";

  for (const auto & item : plan->items) {
    EXPECT_EQ(item.action.find("(inspect_corridor"), std::string::npos)
      << "the replan sends r1 to find out what it already knows: " << item.action;
  }
}

TEST_F(ReplanFromState, AModelFromAnotherProblemIsRefusedRatherThanPlannedFrom)
{
  // The quiet failure this guards against: a state loaded from a different
  // task. Planning would succeed and return a policy for a mission nobody is
  // on, and nothing downstream would notice.
  publish_state(
    R"({"model": {"worlds":["w0"],"labels":{"w0":["some-other-atom"]},)"
    R"("designated":["w0"],"relations":{}}})");

  EXPECT_FALSE(solver_.getPlan("", "").has_value());
}

TEST_F(ReplanFromState, AStateWithoutAModelLeavesTheTaskAlone)
{
  // What a state with publish_model turned off looks like. It still carries a
  // goal, and the initial state must fall back to the task's own instead of
  // being emptied.
  publish_state(R"({"worlds": 2, "designated": 2, "goal": ""})");

  const auto plan = solver_.getPlan("", "");
  ASSERT_TRUE(plan.has_value());
  EXPECT_TRUE(branches(*plan));
}

// Leave through _exit, so the DDS threads never outlive the process. See
// plan_solver_test.cpp for why.
extern "C" void __gcov_dump(void) __attribute__((weak));

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  if (__gcov_dump) {
    __gcov_dump();
  }
  _exit(result);
}
