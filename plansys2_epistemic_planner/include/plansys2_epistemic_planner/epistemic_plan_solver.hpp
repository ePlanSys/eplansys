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

#ifndef PLANSYS2_EPISTEMIC_PLANNER__EPISTEMIC_PLAN_SOLVER_HPP_
#define PLANSYS2_EPISTEMIC_PLANNER__EPISTEMIC_PLAN_SOLVER_HPP_

#include <memory>
#include <optional>
#include <string>

#include "plansys2_core/PlanSolverBase.hpp"
#include "plansys2_epistemic_planner/search.hpp"
#include "plansys2_epistemic_planner/task.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace plansys2
{

/**
 * @class plansys2::EpistemicPlanSolver
 * @brief PlanSolverBase plugin backed by the Aletheia DEL planner.
 *
 * Two things about this plugin do not follow the PDDL solvers, and both are
 * consequences of the interface rather than of the planner:
 *
 * 1. `getPlan` receives a PDDL domain and problem as strings. Aletheia plans
 *    over pointed Kripke models and reads a grounded epistemic task in the
 *    IePC JSON format; there is no PDDL surface for event models or per-agent
 *    observability, so there is no translation to perform. The plugin
 *    therefore takes the epistemic task from the `problem` string when that
 *    string is epistemic JSON, or from a task file named by parameter. A
 *    genuine PDDL problem is rejected with an explanatory error rather than
 *    silently mis-planned.
 *
 * 2. Aletheia's action names and PlanSys2's are different vocabularies. plank
 *    grounds an action into a single token ("pickup-A-hold_r2"), while the
 *    executor splits "(pickup r2 A)" into a name and parameters and looks the
 *    name up in the PDDL domain to find the BT that drives the hardware. The
 *    plan is therefore translated through an `action_mapping` before it
 *    leaves this plugin; see ActionMapping.
 *
 * 3. `plansys2_msgs::msg::Plan` is a flat sequence of timed PlanItems and
 *    cannot represent a branch. Aletheia returns a policy tree for any sensing
 *    domain. See `conditional_plan` below for how that is resolved; the
 *    faithful fix is a policy message and a branching executor, which is a
 *    change to plansys2_msgs and plansys2_executor rather than to this plugin.
 *
 * Parameters, all prefixed with the plugin name:
 *
 *   task_file        Path to a grounded epistemic task JSON. Used when the
 *                    `problem` string is not itself epistemic JSON.
 *   heuristic        ug | ed | ks | wc | rpg | radd. Empty (default) leaves
 *                    the choice to the selection policy.
 *   strategy         gbfs | ehc | aostar. Empty (default) as above.
 *   policy_file      Selection-policy JSON overriding the built-in rules.
 *   action_mapping   Path to a JSON map from grounded epistemic action names
 *                    to PlanSys2 action expressions, optionally with
 *                    durations. Empty (default) falls back to a naming
 *                    convention that guesses parameter order and is not
 *                    suitable for dispatching to real actions. An action the
 *                    map does not cover fails the request.
 *   conditional_plan How to return a branching policy through a flat Plan:
 *                      "flatten" (default) — follow the lowest event index at
 *                          each branch and warn. The result is valid only if
 *                          execution takes that contingency.
 *                      "reject"  — return no plan, on the grounds that a plan
 *                          valid on one contingency is worse than none.
 *                    A policy that does not actually branch is emitted with no
 *                    warning under either setting, since flattening it loses
 *                    nothing.
 */
class EpistemicPlanSolver : public PlanSolverBase
{
public:
  EpistemicPlanSolver();

  void configure(
    rclcpp_lifecycle::LifecycleNode::SharedPtr lc_node,
    const std::string & plugin_name) override;

  std::optional<plansys2_msgs::msg::Plan> getPlan(
    const std::string & domain, const std::string & problem,
    const std::string & node_namespace = "",
    const rclcpp::Duration solver_timeout = std::chrono::seconds(15)) override;

  bool isDomainValid(
    const std::string & domain, const std::string & node_namespace = "") override;

private:
  /// Resolve the grounded epistemic task: `problem` if it is epistemic JSON,
  /// otherwise the configured task file. Nullopt with `error` set on failure.
  std::optional<PlanningTask> resolve_task(
    const std::string & problem, std::string & error) const;

  std::string parameter(const std::string & name) const;

  std::string task_file_parameter_name_;
  std::string heuristic_parameter_name_;
  std::string strategy_parameter_name_;
  std::string policy_file_parameter_name_;
  std::string conditional_parameter_name_;
  std::string action_mapping_parameter_name_;
};

}  // namespace plansys2

#endif  // PLANSYS2_EPISTEMIC_PLANNER__EPISTEMIC_PLAN_SOLVER_HPP_
