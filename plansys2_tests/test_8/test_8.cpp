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

// A temporal policy, end to end.
//
// With durations, the corridor's best policy is not the shortest one: sending
// both robots to look at once (30 + 5 s) beats one looking and telling the
// other (30 + 5 + 1 s). The planner returns a temporal policy that overlaps
// the two drives and the two looks, the epistemic BT builder renders it for
// event-driven dispatch, and the two looks, both sensing actions, really do
// run at the same time, with their updates applied to one epistemic state in
// whichever order they finish.

#include <unistd.h>

#include <map>
#include <optional>
#include <memory>
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
#include "plansys2_epistemic_executor/EpistemicStateNode.hpp"

#include "plansys2_tests/test_action_node.hpp"
#include "plansys2_tests/execution_logger.hpp"

namespace
{

/// What one run of the mission produced.
struct Mission
{
  bool plan_branches{false};
  bool temporal{false};
  int result{-1};
  std::vector<std::string> executed;
  std::vector<std::string> plan;
  /// Per action, when the executor saw it start and succeed.
  std::map<std::string, std::pair<double, double>> span;

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

/// Run the corridor mission with the corridor in a given state.
///
/// @param observed What the inspecting robot turns out to see, named as an
///   outcome of the inspect action's model. This is the ground truth the model
///   cannot supply, since the task designates both a blocked and a clear world
///   and neither is the model's to choose; the performer reports it on finish,
///   the way a real one would report what its sensor returned.
Mission run_mission(
  const std::string & observed, const std::string & estimate = "admissible",
  int max_expansions = 0)
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

  // The performers. Neither knows anything epistemic: they are the same fake
  // actions the classical tests use, which is the point — the branch is not
  // something the hardware side has to be aware of.
  // Two performers for each action the policy overlaps: a performer serves one
  // request at a time, and the two robots drive, and look, at once.
  auto goto_junction_node = plansys2_tests::TestAction::make_shared("goto_junction");
  auto goto_junction_node_2 = plansys2_tests::TestAction::make_shared("goto_junction");
  auto inspect_node = plansys2_tests::TestAction::make_shared("inspect_corridor");
  auto inspect_node_2 = plansys2_tests::TestAction::make_shared("inspect_corridor");
  inspect_node->set_outcome(observed);
  inspect_node_2->set_outcome(observed);
  auto report_clear_node = plansys2_tests::TestAction::make_shared("report_clear");
  auto report_blocked_node = plansys2_tests::TestAction::make_shared("report_blocked");

  auto execution_logger = plansys2_tests::ExecutionLogger::make_shared();

  const std::string pkgpath = plansys2::get_package_share_dir("plansys2_tests");
  const std::string model_file = pkgpath + "/test_5/pddl/test_5.pddl";

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
    rclcpp::Parameter("task_file", std::string(EPISTEMIC_TASK_DIR) + "/robot-fleet.json"));

  executor_node->set_parameter(
    rclcpp::Parameter("bt_builder_plugin", "EpistemicBTBuilder"));
  executor_node->set_parameter(
    rclcpp::Parameter(
      "bt_node_plugins", std::vector<std::string>{EPISTEMIC_BT_NODES_LIBRARY}));
  // No action template of its own. The packaged one already carries the
  // observation from the performer to the epistemic update, which is the point
  // of running this mission on the defaults: a sensing action needs no wiring
  // beyond a performer that says what it saw.

  plansys2::SpinExecutor exe;

  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());
  exe.add_node(planner_node->get_node_base_interface());
  exe.add_node(executor_node->get_node_base_interface());
  exe.add_node(epistemic_node->get_node_base_interface());
  exe.add_node(goto_junction_node->get_node_base_interface());
  exe.add_node(goto_junction_node_2->get_node_base_interface());
  exe.add_node(inspect_node->get_node_base_interface());
  exe.add_node(inspect_node_2->get_node_base_interface());
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
      "EPISTEMIC.task_file", std::string(EPISTEMIC_TASK_DIR) + "/robot-fleet.json"));
  planner_node->set_parameter(
    rclcpp::Parameter(
      "EPISTEMIC.action_mapping", std::string(EPISTEMIC_MAPPING_DIR) + "/robot-fleet.json"));
  // Keep the branches. The other modes exist for a stock executor, which can
  // only run a sequence and so has to be handed one contingency.
  planner_node->set_parameter(rclcpp::Parameter("EPISTEMIC.conditional_plan", "policy"));
  // AO* is the strategy that searches for a policy rather than a path.
  planner_node->set_parameter(rclcpp::Parameter("EPISTEMIC.strategy", "aostar"));
  // Plan with the mapping's durations and overlap what commutes.
  planner_node->set_parameter(rclcpp::Parameter("EPISTEMIC.temporal", true));
  planner_node->set_parameter(rclcpp::Parameter("EPISTEMIC.temporal_estimate", estimate));
  planner_node->set_parameter(
    rclcpp::Parameter("EPISTEMIC.temporal_max_expansions", max_expansions));

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
  problem_client->addPredicate(plansys2::Predicate("(at_base r1)"));
  problem_client->addPredicate(plansys2::Predicate("(at_base r2)"));
  // Classically, r1 has looked: nobody reports in this policy.
  problem_client->setGoal(plansys2::Goal("(and(inspected r1))"));

  auto domain = domain_client->getDomain();
  auto problem = problem_client->getProblem();
  auto plan = planner_client->getPlan(domain, problem);

  EXPECT_FALSE(domain.empty());
  EXPECT_FALSE(problem.empty());

  if (plan.has_value()) {
    mission.temporal = plan->temporal;
    for (const auto & item : plan->items) {
      mission.plan_branches = mission.plan_branches || !item.given_items.empty();
      mission.plan.push_back(item.action + "@" + std::to_string(item.time));
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
    if (action.status == plansys2_msgs::msg::ActionExecutionInfo::SUCCEEDED) {
      mission.span[action.action_full_name] = {
        rclcpp::Time(action.start_stamp).seconds(), rclcpp::Time(action.status_stamp).seconds()};
    }
  }

  finish = true;
  t.join();

  return mission;
}

std::string describe(const Mission & mission)
{
  std::string out = "plan:";
  for (const auto & p : mission.plan) {
    out += " " + p;
  }
  out += "; executed:";
  for (const auto & e : mission.executed) {
    out += " " + e;
  }
  return out;
}

/// When the executor saw an action, named by prefix, start and succeed.
std::optional<std::pair<double, double>> span_of(const Mission & mission, const std::string & name)
{
  for (const auto & [full, span] : mission.span) {
    if (full.rfind(name, 0) == 0) {
      return span;
    }
  }
  return std::nullopt;
}

bool overlapped(const Mission & mission, const std::string & a, const std::string & b)
{
  const auto x = span_of(mission, a);
  const auto y = span_of(mission, b);
  return x && y && x->first < y->second && y->first < x->second;
}

}  // namespace

TEST(test_8, both_robots_look_at_once)
{
  const auto mission = run_mission("e-inspect-blocked");

  ASSERT_TRUE(mission.temporal) << "the planner did not return a temporal policy";
  EXPECT_EQ(mission.result, plansys2_msgs::action::ExecutePlan::Result::SUCCESS)
    << describe(mission);

  // Both robots drive and both look; nobody needs to be told.
  EXPECT_TRUE(mission.ran("(inspect_corridor r1)")) << describe(mission);
  EXPECT_TRUE(mission.ran("(inspect_corridor r2)")) << describe(mission);
  EXPECT_FALSE(mission.ran("(report_clear")) << describe(mission);
  EXPECT_FALSE(mission.ran("(report_blocked")) << describe(mission);

  // And at the same time, not one after the other: the two drives overlap,
  // and so do the two looks, both of them sensing actions.
  EXPECT_TRUE(overlapped(mission, "(goto_junction r1)", "(goto_junction r2)"))
    << describe(mission);
  EXPECT_TRUE(overlapped(mission, "(inspect_corridor r1)", "(inspect_corridor r2)"))
    << describe(mission);
}

TEST(test_8, the_looks_overlap_whatever_the_corridor_is)
{
  const auto mission = run_mission("e-inspect-clear");
  ASSERT_TRUE(mission.temporal);
  EXPECT_EQ(mission.result, plansys2_msgs::action::ExecutePlan::Result::SUCCESS)
    << describe(mission);
  EXPECT_TRUE(mission.ran("(inspect_corridor r1)")) << describe(mission);
  EXPECT_TRUE(mission.ran("(inspect_corridor r2)")) << describe(mission);
}

TEST(test_8, the_critical_path_search_plans_the_same_mission)
{
  // The satisficing search, chosen by parameter: on this mission it finds
  // the same overlapping policy, and the mission runs the same way.
  const auto mission = run_mission("e-inspect-blocked", "critical_path");
  ASSERT_TRUE(mission.temporal);
  EXPECT_EQ(mission.result, plansys2_msgs::action::ExecutePlan::Result::SUCCESS)
    << describe(mission);
  EXPECT_TRUE(overlapped(mission, "(inspect_corridor r1)", "(inspect_corridor r2)"))
    << describe(mission);
}

TEST(test_8, a_search_that_does_not_finish_falls_back_to_the_scheduled_untimed_policy)
{
  // One expansion is not enough for the optimal search, so the solver plans
  // untimed and schedules that policy: r1 drives, looks and announces. It is
  // still a temporal policy, admissible, and the mission still succeeds.
  const auto mission = run_mission("e-inspect-blocked", "admissible", 1);
  ASSERT_TRUE(mission.temporal) << "the fallback must still return a temporal policy";
  EXPECT_EQ(mission.result, plansys2_msgs::action::ExecutePlan::Result::SUCCESS)
    << describe(mission);
  EXPECT_TRUE(mission.ran("(report_blocked")) << describe(mission);
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

  if (__gcov_dump) {
    __gcov_dump();
  }
  _exit(result);
}
