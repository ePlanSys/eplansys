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

// What the demo missions print while a MissionSupervisor runs them.

#ifndef MISSION_LOG_HPP_
#define MISSION_LOG_HPP_

#include "plansys2_executor/MissionSupervisor.hpp"
#include "rclcpp/rclcpp.hpp"

namespace eplansys_demo
{

inline plansys2::MissionSupervisor::EventCallback log_events(const rclcpp::Node::SharedPtr & node)
{
  return [node](const plansys2::MissionSupervisor::Event & event) {
           using Kind = plansys2::MissionSupervisor::Event::Kind;
           const auto logger = node->get_logger();
           switch (event.kind) {
             case Kind::Planning:
               RCLCPP_INFO(logger, "planning");
               break;
             case Kind::Replanning:
               // The planner reads the epistemic state as the failed attempt
               // left it, so what was learned before the failure is kept.
               RCLCPP_WARN(logger, "replanning from the state the mission reached");
               break;
             case Kind::Planned: {
                 bool branches = false;
                 for (const auto & item : event.plan->items) {
                   branches = branches || item.children.size() > 1;
                 }
                 RCLCPP_INFO(
                   logger, "policy with %zu nodes, %s",
                   event.plan->items.size(), branches ? "branching" : "linear");
                 break;
               }
             case Kind::Executing:
               RCLCPP_INFO(logger, "executing");
               break;
             case Kind::AttemptFailed:
               RCLCPP_WARN(logger, "attempt %d failed", event.attempt);
               break;
             case Kind::Finished:
               break;
           }
         };
}

}  // namespace eplansys_demo

#endif  // MISSION_LOG_HPP_
