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

// When the world answers a question the policy did not plan for.
//
// Tests 5 and 6 run a policy against a world that behaves as the model said it
// could, and check that the right branch runs. This one runs the corridor
// mission against a world that does not, and checks that nothing runs on a
// belief the observation has just contradicted.
//
// Two ways the world can disagree, and they end differently:
//
//   - The performer reports something no event of the action describes. The
//     model cannot represent what happened at all, so the update is refused
//     and the mission stops right after the sensing action, before either
//     report.
//   - The performer reports a real outcome that the planner was sure would not
//     happen: the map said the corridor was clear, so the policy has a branch
//     for "clear" and nothing else. The epistemic state repairs the model to
//     fit what was seen, and the action the policy planned for "clear" must
//     still not run, since its knowledge guard is checked against the repaired
//     model and no longer holds.
//
// The first case never reaches EpistemicSwitch, and neither does the second:
// the corridor's inspection has two outcomes, so a policy that branches on it
// plans for both, and one that was sure of the answer has a single
// continuation and renders no switch. The switch's own refusal is unit tested
// in plansys2_epistemic_executor (TheSwitchRefusesAnUnplannedOutcome). What
// this test establishes end to end is the property that refusal exists for: an
// unplanned observation does not lead to a planned action.

#include <unistd.h>

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "plansys2_core/Compat.hpp"
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
#include "plansys2_executor/MissionSupervisor.hpp"
#include "plansys2_epistemic_executor/EpistemicStateNode.hpp"
#include "plansys2_epistemic_executor/EpistemicStateClient.hpp"

#include "plansys2_tests/test_action_node.hpp"
#include "plansys2_tests/execution_logger.hpp"

namespace
{

/// The world a run is set in.
struct Scenario
{
  /// The grounded task, under EPISTEMIC_TASK_DIR. The planner solves it and
  /// the epistemic state tracks it.
  std::string task;
  /// Facts the problem expert starts with: where the robots are.
  std::vector<std::string> facts;
  /// What the inspecting robot reports seeing.
  std::string observed;
  /// Formulas to ask the epistemic state once the run has ended.
  std::vector<std::string> ask;
  /// Run under a MissionSupervisor, which replans when execution fails,
  /// rather than planning and executing once.
  bool supervised{false};
};

/// What one run produced.
struct Mission
{
  bool planned{false};
  bool plan_branches{false};
  std::vector<std::string> plan;
  int result{-1};
  std::vector<std::string> executed;
  /// One answer per Scenario::ask, in order: did the formula hold.
  std::vector<bool> holds;
  /// Supervised runs only.
  int attempts{0};
  plansys2::MissionSupervisor::Outcome outcome{plansys2::MissionSupervisor::Outcome::NoPlan};

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

Mission run_mission(const Scenario & scenario)
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

  auto goto_junction_node = plansys2_tests::TestAction::make_shared("goto_junction");
  auto inspect_node = plansys2_tests::TestAction::make_shared("inspect_corridor");
  inspect_node->set_outcome(scenario.observed);
  auto report_clear_node = plansys2_tests::TestAction::make_shared("report_clear");
  auto report_blocked_node = plansys2_tests::TestAction::make_shared("report_blocked");

  auto execution_logger = plansys2_tests::ExecutionLogger::make_shared();

  // The corridor's classical half is test 5's; only the epistemic task and
  // what the robot sees differ here.
  const std::string pkgpath = plansys2::get_package_share_dir("plansys2_tests");
  const std::string model_file = pkgpath + "/test_5/pddl/test_5.pddl";
  const std::string task_file = std::string(EPISTEMIC_TASK_DIR) + "/" + scenario.task;

  domain_node->set_parameter({"model_file", model_file});
  problem_node->set_parameter({"model_file", model_file});

  planner_node->set_parameter(
    rclcpp::Parameter("plan_solver_plugins", std::vector<std::string>{"EPISTEMIC"}));
  planner_node->declare_parameter("EPISTEMIC.plugin", "plansys2/EpistemicPlanSolver");

  epistemic_node->set_parameter(rclcpp::Parameter("task_file", task_file));

  executor_node->set_parameter(
    rclcpp::Parameter("bt_builder_plugin", "EpistemicBTBuilder"));
  executor_node->set_parameter(
    rclcpp::Parameter(
      "bt_node_plugins", std::vector<std::string>{EPISTEMIC_BT_NODES_LIBRARY}));

  plansys2::SpinExecutor exe;

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

  planner_node->set_parameter(rclcpp::Parameter("EPISTEMIC.task_file", task_file));
  planner_node->set_parameter(
    rclcpp::Parameter(
      "EPISTEMIC.action_mapping", std::string(EPISTEMIC_MAPPING_DIR) + "/robot-fleet.json"));
  planner_node->set_parameter(rclcpp::Parameter("EPISTEMIC.conditional_plan", "policy"));
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

  problem_client->addInstance(plansys2::Instance("r1", "robot"));
  problem_client->addInstance(plansys2::Instance("r2", "robot"));
  for (const auto & fact : scenario.facts) {
    problem_client->addPredicate(plansys2::Predicate(fact));
  }
  problem_client->setGoal(plansys2::Goal("(and(inspected r1))"));

  if (scenario.supervised) {
    plansys2::MissionSupervisor::Options options;
    options.settle = std::chrono::milliseconds(500);
    plansys2::MissionSupervisor supervisor(
      rclcpp::Node::make_shared("test_7_supervisor"), options);

    const auto report = supervisor.run(
      [&](const plansys2::MissionSupervisor::Event & event) {
        if (event.kind == plansys2::MissionSupervisor::Event::Kind::Planned) {
          std::string line = "attempt " + std::to_string(event.attempt) + ":";
          for (const auto & item : event.plan->items) {
            line += " " + item.action;
            mission.plan_branches = mission.plan_branches || item.children.size() > 1;
          }
          mission.plan.push_back(line);
          mission.planned = true;
        }
      });
    mission.attempts = report.attempts;
    mission.outcome = report.outcome;
    mission.result = report.succeeded() ?
      plansys2_msgs::action::ExecutePlan::Result::SUCCESS :
      plansys2_msgs::action::ExecutePlan::Result::FAILURE;
  }

  auto plan = scenario.supervised ? std::optional<plansys2_msgs::msg::Plan>() :
    planner_client->getPlan(domain_client->getDomain(), problem_client->getProblem());

  if (plan.has_value()) {
    mission.planned = true;
    for (const auto & item : plan->items) {
      mission.plan.push_back(item.action);
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

  // Asked while the graph is still spinning: the state node answers from the
  // executor thread above.
  if (!scenario.ask.empty()) {
    plansys2::EpistemicStateClient state("test_7_state_client");
    for (const auto & formula : scenario.ask) {
      const auto answer = state.check_formula(formula);
      EXPECT_TRUE(answer.answered && answer.success) << formula << ": " << answer.error;
      mission.holds.push_back(answer.holds);
    }
  }

  finish = true;
  t.join();

  return mission;
}

std::string describe(const Mission & mission)
{
  std::string out = "plan:";
  for (const auto & action : mission.plan) {
    out += " " + action;
  }
  out += "; executed:";
  for (const auto & action : mission.executed) {
    out += " " + action;
  }
  return out;
}

}  // namespace

TEST(test_7, an_observation_the_model_cannot_account_for_stops_the_mission)
{
  // The corridor mission of test 5, whose policy branches on clear and
  // blocked, and a sensor that returns neither.
  const auto mission = run_mission(
    {"robot-fleet.json", {"(at_base r1)", "(at_base r2)"}, "e-inspect-foggy", {}});

  ASSERT_TRUE(mission.planned);
  ASSERT_TRUE(mission.plan_branches) << "without a branch there is nothing to fall into";

  EXPECT_EQ(mission.result, plansys2_msgs::action::ExecutePlan::Result::FAILURE)
    << describe(mission);

  // The robot did look; what it saw is what stopped the mission.
  EXPECT_TRUE(mission.ran("(inspect_corridor r1)")) << describe(mission);
  // Neither branch: running one would be choosing a belief at random.
  EXPECT_FALSE(mission.ran("(report_clear r1)")) << describe(mission);
  EXPECT_FALSE(mission.ran("(report_blocked r1)")) << describe(mission);
}

// The map says the corridor is clear, so the planner, sure of the answer, plans
// for that one: r1 looks and tells r2 it is clear. r2 is away from the
// junction, which makes telling it cheaper than sending it to look. The robot
// looks and sees the corridor blocked.
const Scenario kStalePrior{
  "robot-fleet-stale-prior-lone.json", {"(at_junction r1)", "(at_base r2)"},
  "e-inspect-blocked", {"blocked", "(Kw r1 blocked)"}};

TEST(test_7, an_outcome_the_policy_did_not_plan_for_is_not_acted_on)
{
  const auto mission = run_mission(kStalePrior);

  ASSERT_TRUE(mission.planned);
  EXPECT_FALSE(mission.plan_branches)
    << "the stale prior designates a single world, so the policy should commit to it; "
    << describe(mission);

  ASSERT_NE(describe(mission).find("(report_clear r1)"), std::string::npos)
    << "the policy should tell r2 the corridor is clear, or there is nothing to refuse; "
    << describe(mission);

  EXPECT_TRUE(mission.ran("(inspect_corridor r1)")) << describe(mission);
  // The action planned for a clear corridor must not run in a blocked one.
  EXPECT_FALSE(mission.ran("(report_clear r1)")) << describe(mission);
  EXPECT_EQ(mission.result, plansys2_msgs::action::ExecutePlan::Result::FAILURE)
    << describe(mission);

  // And the model the mission leaves behind, which is what a replan would
  // start from, records what the robot saw rather than what the map said.
  ASSERT_EQ(mission.holds.size(), 2u);
  EXPECT_TRUE(mission.holds[0]) << "the repaired model does not have the corridor blocked";
  EXPECT_TRUE(mission.holds[1]) << "the repaired model does not have r1 knowing it";
}

TEST(test_7, a_supervised_mission_replans_from_what_the_robot_saw)
{
  // The same mission under a MissionSupervisor. The first policy fails where
  // the test above says it must; the supervisor asks again, the planner plans
  // from the repaired belief, and the second policy tells r2 the truth.
  Scenario scenario = kStalePrior;
  scenario.supervised = true;
  const auto mission = run_mission(scenario);

  ASSERT_TRUE(mission.planned);
  EXPECT_EQ(mission.outcome, plansys2::MissionSupervisor::Outcome::Succeeded)
    << plansys2::MissionSupervisor::to_string(mission.outcome) << "; " << describe(mission);
  EXPECT_EQ(mission.attempts, 2) << describe(mission);

  EXPECT_TRUE(mission.ran("(report_blocked r1)")) << describe(mission);
  EXPECT_FALSE(mission.ran("(report_clear r1)")) << describe(mission);
  // The second policy starts from what was learned: r1 has already looked.
  ASSERT_EQ(mission.plan.size(), 2u) << describe(mission);
  EXPECT_EQ(mission.plan[1].find("inspect_corridor"), std::string::npos)
    << "the replan sent r1 to look again at what it had already seen; " << describe(mission);

  ASSERT_EQ(mission.holds.size(), 2u);
  EXPECT_TRUE(mission.holds[0]);
  EXPECT_TRUE(mission.holds[1]);
}

// Leave through _exit, so the DDS threads never outlive the process; see
// test_5.cpp for why.
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
