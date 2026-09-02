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

// Starts the corridor mission, so that the demo is one command.
//
// The same three steps anyone would type into `ros2 plansys2 terminal`: say
// what exists, say what is wanted, run. Doing it from a node instead means the
// demo has nothing to type and nothing to get wrong, and it is the shortest
// honest description of how a mission is started.

#include <memory>
#include <string>

#include "plansys2_msgs/action/execute_plan.hpp"
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
  auto node = rclcpp::Node::make_shared("corridor_mission");

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
  problem->addInstance(plansys2::Instance{"r1", "robot"});
  problem->addInstance(plansys2::Instance{"r2", "robot"});
  problem->addPredicate(plansys2::Predicate("(at_base r1)"));
  problem->addPredicate(plansys2::Predicate("(at_base r2)"));

  // What is wanted, classically: r1 has reported. What is wanted
  // epistemically --- that both robots come to know whether the corridor is
  // blocked --- travels with the grounded task the planner solves.
  problem->setGoal(plansys2::Goal("(and(reported r1))"));

  RCLCPP_INFO(node->get_logger(), "planning");
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
  const bool succeeded =
    result && result->result == plansys2_msgs::action::ExecutePlan::Result::SUCCESS;

  if (succeeded) {
    RCLCPP_INFO(node->get_logger(), "mission complete");
  } else {
    RCLCPP_ERROR(node->get_logger(), "mission failed");
  }

  rclcpp::shutdown();
  return succeeded ? 0 : 1;
}
