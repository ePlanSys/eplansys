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
//
// Running it is a MissionSupervisor's job: when an action fails it asks for a
// policy again, planned from the epistemic state the mission reached, and the
// mission.* parameters bound how often.

#include <memory>
#include <string>

#include "plansys2_executor/MissionSupervisor.hpp"

#include "rclcpp/rclcpp.hpp"

#include "mission_log.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("corridor_mission");
  plansys2::MissionSupervisor supervisor(node);

  RCLCPP_INFO(node->get_logger(), "waiting for the planning system");
  supervisor.wait_for_system();

  // What exists. Nothing here says anything about the corridor: whether it is
  // blocked is not a fact the problem expert holds, which is the whole reason
  // the mission needs an epistemic layer at all.
  auto problem = supervisor.problem();
  problem->addInstance(plansys2::Instance{"r1", "robot"});
  problem->addInstance(plansys2::Instance{"r2", "robot"});
  problem->addPredicate(plansys2::Predicate("(at_base r1)"));
  problem->addPredicate(plansys2::Predicate("(at_base r2)"));

  // What is wanted, classically: r1 has reported. What is wanted
  // epistemically --- that both robots come to know whether the corridor is
  // blocked --- travels with the grounded task the planner solves.
  problem->setGoal(plansys2::Goal("(and(reported r1))"));

  const auto report = supervisor.run(eplansys_demo::log_events(node));

  if (report.succeeded()) {
    RCLCPP_INFO(node->get_logger(), "mission complete");
  } else {
    RCLCPP_ERROR(node->get_logger(), "%s", report.message.c_str());
  }

  rclcpp::shutdown();
  return report.succeeded() ? 0 : 1;
}
