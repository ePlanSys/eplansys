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

// Starts the site survey, so that the demo is one command.
//
// The same three steps anyone would type into `ros2 plansys2 terminal`: say
// what exists, say what is wanted, run. Doing it from a node instead means the
// demo has nothing to type and nothing to get wrong, and it is the shortest
// honest description of how a mission is started.
//
// It does one thing more, and it is the thing a mission on hardware needs:
// when an action fails it asks for a policy again rather than stopping. The
// new policy is planned from the epistemic state the mission reached, so what
// was learned before the failure is kept and only what is left is planned for.
// An action can fail for a reason that outlives one attempt --- a fleet that
// stopped answering, a door that will not open --- so the attempts are counted
// and the mission gives up saying which action it was that kept failing.

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>

#include "plansys2_msgs/action/execute_plan.hpp"
#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;   // NOLINT (build/namespaces)

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("survey_mission");

  auto problem = std::make_shared<plansys2::ProblemExpertClient>();
  auto planner = std::make_shared<plansys2::PlannerClient>();
  auto domain = std::make_shared<plansys2::DomainExpertClient>();
  auto executor = std::make_shared<plansys2::ExecutorClient>();

  // The bringup is lifecycle-managed and comes up on its own schedule. Waiting
  // for the domain to answer is the cheapest way to know it has.
  RCLCPP_INFO(node->get_logger(), "waiting for the planning system");
  for (int i = 0; i < 120 && rclcpp::ok(); ++i) {
    if (!domain->getDomain().empty()) {
      break;
    }
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(500ms);
  }

  // What exists. Nothing here says anything about the corridor: whether it is
  // blocked is not a fact the problem expert holds, which is the whole reason
  // the mission needs an epistemic layer at all.
  for (const auto & robot : {"scout", "relay", "observer"}) {
    problem->addInstance(plansys2::Instance{robot, "robot"});
    problem->addPredicate(plansys2::Predicate("(at_depot " + std::string(robot) + ")"));
  }

  // What is wanted, classically: the scout has said something. Which channel
  // it used, and who was allowed to hear, is not expressible here at all --- it
  // travels with the EPDDL the planner grounds and solves.
  problem->setGoal(plansys2::Goal("(and(told scout))"));

  // How many policies the mission is willing to ask for. One is the classical
  // behaviour: plan, run, report. More than one is what makes a failure
  // recoverable, and the bound is what keeps a permanent failure from being
  // retried for ever.
  int attempts = 3;
  for (int i = 1; i + 1 < argc; ++i) {
    if (std::string(argv[i]) == "--attempts") {
      attempts = std::atoi(argv[i + 1]);
    }
  }

  bool succeeded = false;

  for (int attempt = 1; attempt <= attempts && rclcpp::ok() && !succeeded; ++attempt) {
    RCLCPP_INFO(node->get_logger(), "planning (attempt %d of %d)", attempt, attempts);

    // On any attempt after the first this is a replan, and the planner reads
    // the epistemic state as the mission left it: a scan that already happened
    // is not planned for again.
    const auto plan = planner->getPlan(domain->getDomain(), problem->getProblem());
    if (!plan.has_value()) {
      RCLCPP_ERROR(node->get_logger(), "no plan; is the epistemic solver configured?");
      rclcpp::shutdown();
      return 1;
    }

    bool branches = false;
    for (const auto & item : plan->items) {
      branches = branches || item.children.size() > 1;
    }
    RCLCPP_INFO(
      node->get_logger(), "policy with %zu nodes, %s",
      plan->items.size(), branches ? "branching" : "linear");

    if (plan->items.empty()) {
      // A replan returns nothing when the goal already holds, which is a
      // mission that finished between the failure and the replan.
      RCLCPP_INFO(node->get_logger(), "the goal already holds; nothing left to run");
      succeeded = true;
      break;
    }

    if (!executor->start_plan_execution(plan.value())) {
      RCLCPP_ERROR(node->get_logger(), "the executor refused the policy");
      rclcpp::shutdown();
      return 1;
    }

    RCLCPP_INFO(node->get_logger(), "executing");
    rclcpp::Rate rate(4);
    while (rclcpp::ok() && executor->execute_and_check_plan()) {
      rclcpp::spin_some(node);
      rate.sleep();
    }

    const auto result = executor->getResult();
    succeeded = result &&
      result->result == plansys2_msgs::action::ExecutePlan::Result::SUCCESS;

    if (succeeded) {
      break;
    }

    // Which action died is the whole of what a reader needs afterwards, and
    // the executor knows it: the result carries a status per action.
    if (result) {
      for (const auto & status : result->action_execution_status) {
        if (status.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
          RCLCPP_WARN(
            node->get_logger(), "%s failed: %s",
            status.action_full_name.c_str(), status.message_status.c_str());
        }
      }
    }

    if (attempt < attempts) {
      RCLCPP_WARN(
        node->get_logger(),
        "the policy failed; replanning from the state the mission reached");
      std::this_thread::sleep_for(2s);
      rclcpp::spin_some(node);
    }
  }

  if (succeeded) {
    RCLCPP_INFO(node->get_logger(), "mission complete");
  } else {
    RCLCPP_ERROR(node->get_logger(), "mission failed after %d attempt(s)", attempts);
  }

  rclcpp::shutdown();
  return succeeded ? 0 : 1;
}
