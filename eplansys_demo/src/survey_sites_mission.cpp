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


// Starts the two-site survey, and times it.
//
// The mission is the one-site survey twice over, in different hands: each
// scout carries the instrument its own site needs, so the policy has two
// halves that share no agent, no atom and no precondition. The elapsed time
// printed at the end is the number that a parallel dispatch is measured
// against; a serial one pays for both halves.
//
// Running it is a MissionSupervisor's job, as in the other missions.

#include <memory>
#include <string>

#include "plansys2_executor/MissionSupervisor.hpp"

#include "rclcpp/rclcpp.hpp"

#include "mission_log.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("survey_sites_mission");
  plansys2::MissionSupervisor supervisor(node);

  RCLCPP_INFO(node->get_logger(), "waiting for the planning system");
  supervisor.wait_for_system();

  // Four robots at the depot. Which of them carries which instrument is not a
  // fact this half holds: it decides who can scan what, and so it belongs to
  // the epistemic problem, where it is common knowledge.
  auto problem = supervisor.problem();
  for (const auto & robot : {"north", "south", "relay", "observer"}) {
    problem->addInstance(plansys2::Instance{robot, "robot"});
    problem->addPredicate(plansys2::Predicate("(at_depot " + std::string(robot) + ")"));
  }

  // What is wanted, classically: both scouts have reported. Which channel each
  // used, and who was allowed to hear it, travels with the EPDDL.
  problem->setGoal(plansys2::Goal("(and(told north)(told south))"));

  const auto report = supervisor.run(eplansys_demo::log_events(node));

  if (report.succeeded()) {
    // The makespan, which is the point of this mission: the two halves sum to
    // eighteen seconds of declared duration run one after another, and to nine
    // run at the same time. It includes planning, as it always has.
    RCLCPP_INFO(node->get_logger(), "mission complete in %.1fs", report.elapsed_seconds);
  } else {
    RCLCPP_ERROR(
      node->get_logger(), "%s (after %.1fs)", report.message.c_str(), report.elapsed_seconds);
  }

  rclcpp::shutdown();
  return report.succeeded() ? 0 : 1;
}
