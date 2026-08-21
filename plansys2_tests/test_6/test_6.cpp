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

// The same thing test_5 does, with the branch nested inside a branch.
//
// The depot has two corridors and three robots, and knowing whether one
// corridor is blocked says nothing about the other. So the policy senses
// twice, and the second sensing happens inside whichever continuation the
// first one selected: every one of the four ways the depot can turn out is a
// different path through the tree, and the second half of the mission has to
// run correctly in all of them.
//
// This is where a flattened plan stops being merely optimistic and becomes
// wrong more often than right: it covers one of four contingencies. It is also
// where the parts have to compose — a switch inside a switch's child, with the
// epistemic state updated by everything that ran before it.

#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "plansys2_pddl_parser/AmentIndexCompat.hpp"

#include "gtest/gtest.h"
#include "plansys2_domain_expert/DomainExpertNode.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertNode.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_planner/PlannerNode.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_executor/ExecutorNode.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_epistemic_executor/EpistemicStateNode.hpp"

#include "std_msgs/msg/string.hpp"

#include "plansys2_tests/test_action_node.hpp"
#include "plansys2_tests/execution_logger.hpp"

namespace
{

/// What one run of the mission produced.
struct Mission
{
  bool plan_branches{false};
  int result{-1};
  std::vector<std::string> executed;

  bool ran(const std::string & action) const
  {
    for (const auto & name : executed) {
      if (name.rfind(action, 0) == 0) {
        return true;
      }
    }
    return false;
  }
};

/// Run the depot mission with the two corridors in given states.
///
/// @param observation What the corridors turn out to be, as `action=outcome`
///   entries the performers report — the ground truth the model cannot supply,
///   since the task designates all four combinations and none of them is the
///   model's to choose.
Mission run_mission(const std::string & observation)
{
  Mission mission;

  auto test_node = rclcpp::Node::make_shared("test_node");
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  auto planner_node = std::make_shared<plansys2::PlannerNode>();
  auto executor_node = std::make_shared<plansys2::ExecutorNode>();
  auto epistemic_node = std::make_shared<plansys2::EpistemicStateNode>();

  auto domain_client = std::make_shared<plansys2::DomainExpertClient>();
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();
  auto planner_client = std::make_shared<plansys2::PlannerClient>();
  auto executor_client = std::make_shared<plansys2::ExecutorClient>();

  // The performers. None of them knows anything epistemic: they are the same fake
  // actions the classical tests use, which is the point — the branch is not
  // something the hardware side has to be aware of.
  auto goto_junction_node = plansys2_tests::TestAction::make_shared("goto_junction");
  auto inspect_node = plansys2_tests::TestAction::make_shared("inspect_corridor");
  auto report_clear_node = plansys2_tests::TestAction::make_shared("report_clear");
  auto report_blocked_node = plansys2_tests::TestAction::make_shared("report_blocked");

  auto execution_logger = plansys2_tests::ExecutionLogger::make_shared();

  const std::string pkgpath = plansys2::get_package_share_dir("plansys2_tests");
  const std::string model_file = pkgpath + "/test_6/pddl/test_6.pddl";

  domain_node->set_parameter({"model_file", model_file});
  problem_node->set_parameter({"model_file", model_file});

  // The planner solves the grounded epistemic task rather than the PDDL
  // problem: there is no PDDL surface for event models or per-agent
  // observability, so the task is where the mission actually is.
  planner_node->set_parameter(
    rclcpp::Parameter("plan_solver_plugins", std::vector<std::string>{"EPISTEMIC"}));
  planner_node->declare_parameter("EPISTEMIC.plugin", "plansys2/EpistemicPlanSolver");

  // The epistemic state is loaded from the same task the planner solves.
  // A model built from a different one would answer knowledge questions in a
  // vocabulary the policy does not speak.
  epistemic_node->set_parameter(
    rclcpp::Parameter("task_file", std::string(EPISTEMIC_TASK_DIR) + "/robot-fleet-depot.json"));

  executor_node->set_parameter(
    rclcpp::Parameter("bt_builder_plugin", "EpistemicBTBuilder"));
  executor_node->set_parameter(
    rclcpp::Parameter(
      "bt_node_plugins",
      std::vector<std::string>{EPISTEMIC_BT_NODES_LIBRARY, OBSERVATION_BT_NODES_LIBRARY}));
  // The packaged template leaves `observed` unbound, because what a robot saw
  // is domain-specific; this one binds it to the topic the performers report on.
  executor_node->set_parameter(
    rclcpp::Parameter(
      "default_action_bt_xml_filename",
      pkgpath + "/behavior_trees/observing_action_bt.xml"));

  rclcpp::experimental::executors::EventsExecutor exe;

  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());
  exe.add_node(planner_node->get_node_base_interface());
  exe.add_node(executor_node->get_node_base_interface());
  exe.add_node(epistemic_node->get_node_base_interface());
  exe.add_node(goto_junction_node->get_node_base_interface());
  exe.add_node(inspect_node->get_node_base_interface());
  exe.add_node(report_clear_node->get_node_base_interface());
  exe.add_node(report_blocked_node->get_node_base_interface());
  exe.add_node(execution_logger->get_node_base_interface());

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });

  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  executor_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  epistemic_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  // The solver declares its parameters when the planner node configures it, so
  // these can only be set now.
  planner_node->set_parameter(
    rclcpp::Parameter(
      "EPISTEMIC.task_file", std::string(EPISTEMIC_TASK_DIR) + "/robot-fleet-depot.json"));
  planner_node->set_parameter(
    rclcpp::Parameter(
      "EPISTEMIC.action_mapping", std::string(EPISTEMIC_MAPPING_DIR) + "/robot-fleet-depot.json"));
  // Keep the branches. The other modes exist for a stock executor, which can
  // only run a sequence and so has to be handed one contingency.
  planner_node->set_parameter(rclcpp::Parameter("EPISTEMIC.conditional_plan", "policy"));
  // AO* is the strategy that searches for a policy rather than a path.
  planner_node->set_parameter(rclcpp::Parameter("EPISTEMIC.strategy", "aostar"));

  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  executor_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  epistemic_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  // What the corridors turn out to be. Published before the tree exists and
  // kept available to whoever subscribes afterwards, which is how a sensor
  // reading that predates the behavior tree reaches it.
  auto observation_pub = test_node->create_publisher<std_msgs::msg::String>(
    "/epistemic_observation", rclcpp::QoS(1).transient_local());
  std_msgs::msg::String observation_msg;
  observation_msg.data = observation;
  observation_pub->publish(observation_msg);

  problem_client->addInstance(plansys2::Instance("r1", "robot"));
  problem_client->addInstance(plansys2::Instance("r2", "robot"));
  problem_client->addInstance(plansys2::Instance("r3", "robot"));
  problem_client->addInstance(plansys2::Instance("north", "corridor"));
  problem_client->addInstance(plansys2::Instance("south", "corridor"));
  problem_client->addPredicate(plansys2::Predicate("(at_base r1)"));
  problem_client->addPredicate(plansys2::Predicate("(at_base r2)"));
  problem_client->addPredicate(plansys2::Predicate("(at_base r3)"));
  problem_client->setGoal(plansys2::Goal("(and(reported r1 north)(reported r2 south))"));

  auto domain = domain_client->getDomain();
  auto problem = problem_client->getProblem();
  auto plan = planner_client->getPlan(domain, problem);

  EXPECT_FALSE(domain.empty());
  EXPECT_FALSE(problem.empty());

  if (plan.has_value()) {
    for (const auto & item : plan->items) {
      mission.plan_branches = mission.plan_branches || item.children.size() > 1;
    }

    if (executor_client->start_plan_execution(plan.value())) {
      rclcpp::Rate rate(5);
      while (executor_client->execute_and_check_plan()) {
        rate.sleep();
      }

      const auto result = executor_client->getResult();
      if (result.has_value()) {
        mission.result = result.value().result;
      }
    }
  }

  for (const auto & action : execution_logger->get_action_execution_info_log()) {
    if (action.completion > 0.000001) {
      mission.executed.push_back(action.action_full_name);
    }
  }

  finish = true;
  t.join();

  return mission;
}

}  // namespace

TEST(test_6, each_corridor_is_reported_as_it_is_found)
{
  // North blocked, south clear: the two sensings disagree, which is the case a
  // single observation for the whole mission could not express.
  const auto mission = run_mission(
    "(inspect_corridor r1 north)=e-inspect-north-blocked;"
    "(inspect_corridor r2 south)=e-inspect-south-clear");

  ASSERT_TRUE(mission.plan_branches) << "two corridors of unknown state, so the policy "
    "must branch; a flat plan means the branches were lost before the executor";

  EXPECT_EQ(mission.result, plansys2_msgs::action::ExecutePlan::Result::SUCCESS);

  EXPECT_TRUE(mission.ran("(inspect_corridor r1 north)"));
  EXPECT_TRUE(mission.ran("(report_blocked r1 north)"));
  // The south half ran inside the north branch's continuation: reaching it at
  // all means the switch selected a subtree and the tree kept going.
  EXPECT_TRUE(mission.ran("(inspect_corridor r2 south)"));
  EXPECT_TRUE(mission.ran("(report_clear r2 south)"));

  EXPECT_FALSE(mission.ran("(report_clear r1 north)"));
  EXPECT_FALSE(mission.ran("(report_blocked r2 south)"));
}

TEST(test_6, the_other_way_round_takes_the_other_path)
{
  // The mirror image, which shares no leaf with the run above: the same policy,
  // the same tree, and a different execution from the first sensing onwards.
  const auto mission = run_mission(
    "(inspect_corridor r1 north)=e-inspect-north-clear;"
    "(inspect_corridor r2 south)=e-inspect-south-blocked");

  ASSERT_TRUE(mission.plan_branches);

  EXPECT_EQ(mission.result, plansys2_msgs::action::ExecutePlan::Result::SUCCESS);

  EXPECT_TRUE(mission.ran("(report_clear r1 north)"));
  EXPECT_TRUE(mission.ran("(report_blocked r2 south)"));

  EXPECT_FALSE(mission.ran("(report_blocked r1 north)"));
  EXPECT_FALSE(mission.ran("(report_clear r2 south)"));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}
