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

#ifndef PLANSYS2_EPISTEMIC_EXECUTOR__EPISTEMICSTATENODE_HPP_
#define PLANSYS2_EPISTEMIC_EXECUTOR__EPISTEMICSTATENODE_HPP_

#include <memory>
#include <optional>
#include <string>

#include "plansys2_epistemic_msgs/srv/apply_action.hpp"
#include "plansys2_epistemic_msgs/srv/check_formula.hpp"
#include "plansys2_epistemic_msgs/srv/load_task.hpp"
#include "plansys2_epistemic_planner/state.hpp"
#include "plansys2_epddl_grounder/epddl_grounder.hpp"
#include "plansys2_epddl_grounder/parameters.hpp"
#include "plansys2_epistemic_planner/task.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"

namespace plansys2
{

/**
 * @class plansys2::EpistemicStateNode
 * @brief Holds what the agents know, and keeps it current as they act.
 *
 * The problem expert holds the facts: what is true, and it is asked whether a
 * PDDL condition holds. This is its counterpart one level up — it holds a
 * pointed Kripke model, and it is asked whether an agent knows something.
 *
 * The two are not interchangeable. "The corridor is blocked" is a fact and
 * lives in the problem expert. "r1 knows whether the corridor is blocked" is
 * not a fact about the corridor at all; no set of predicates records it, and a
 * plan that must sense before committing needs exactly that distinction.
 *
 * Three things are asked of it, matching what executing a policy requires:
 *
 *   load_task     Set the model, from the same grounded task the planner
 *                 solved. Executing a policy against a model built from a
 *                 different task would check knowledge in a vocabulary the
 *                 policy does not speak.
 *   check_formula Does this epistemic condition hold now? This is what a
 *                 knowledge precondition and an epistemic goal both reduce to.
 *   apply_action  An action has been executed: advance the model by its DEL
 *                 product update, and report which outcome occurred.
 *
 * The model is advanced by executed actions rather than by observing the
 * world, which is what makes it a belief state rather than a log. When it
 * disagrees with what the robot observes, the disagreement surfaces at
 * apply_action as an outcome the model cannot account for — which is a reason
 * to replan, not to overwrite the model quietly.
 */
class EpistemicStateNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  EpistemicStateNode();

  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);

private:
  void load_task_callback(
    const std::shared_ptr<plansys2_epistemic_msgs::srv::LoadTask::Request> request,
    std::shared_ptr<plansys2_epistemic_msgs::srv::LoadTask::Response> response);

  void check_formula_callback(
    const std::shared_ptr<plansys2_epistemic_msgs::srv::CheckFormula::Request> request,
    std::shared_ptr<plansys2_epistemic_msgs::srv::CheckFormula::Response> response);

  void apply_action_callback(
    const std::shared_ptr<plansys2_epistemic_msgs::srv::ApplyAction::Request> request,
    std::shared_ptr<plansys2_epistemic_msgs::srv::ApplyAction::Response> response);

  /// Announce the shape of the current model, so that a monitor can follow the
  /// state without polling a service on every change.
  void publish_state();

  rclcpp::Service<plansys2_epistemic_msgs::srv::LoadTask>::SharedPtr load_task_service_;
  rclcpp::Service<plansys2_epistemic_msgs::srv::CheckFormula>::SharedPtr check_formula_service_;
  rclcpp::Service<plansys2_epistemic_msgs::srv::ApplyAction>::SharedPtr apply_action_service_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr state_pub_;

  /// The loaded task and the model as it now stands. Absent until a task is
  /// loaded, which is why every callback checks before answering: a confident
  /// answer from an empty model would be worse than an error.
  /// The EPDDL sources this node was given, and the grounder that turns them
  /// into a task. Shared with the planner through the parameter names alone:
  /// both are pointed at the same files, and neither grounds for the other.
  EpddlParameterNames epddl_parameter_names_;
  EpddlGrounder grounder_;

  std::optional<PlanningTask> task_;
  std::optional<EpistemicState> state_;
};

}  // namespace plansys2

#endif  // PLANSYS2_EPISTEMIC_EXECUTOR__EPISTEMICSTATENODE_HPP_
