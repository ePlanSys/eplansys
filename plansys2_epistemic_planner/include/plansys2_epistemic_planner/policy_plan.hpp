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

#ifndef PLANSYS2_EPISTEMIC_PLANNER__POLICY_PLAN_HPP_
#define PLANSYS2_EPISTEMIC_PLANNER__POLICY_PLAN_HPP_

#include <memory>
#include <optional>
#include <string>

#include "plansys2_epistemic_planner/action_mapping.hpp"
#include "aletheia/search.hpp"
#include "aletheia/task.hpp"
#include "aletheia/temporal_search.hpp"
#include "plansys2_msgs/msg/plan.hpp"

namespace plansys2
{

/**
 * @brief Serialise a policy tree into a Plan carrying its branch structure.
 *
 * The tree AO* returns is the honest shape of a plan for a partially
 * observable domain: after a sensing action, which continuation is correct
 * depends on what was observed. Flattening it to a sequence keeps only one
 * contingency and silently assumes it happens.
 *
 * This walks the tree instead, writing each node as one PlanItem and recording
 * its successors in `children`, with the event name that selects each in
 * `outcomes`. items[0] is the root. Times are laid out along the longest path
 * so that any single execution through the policy sees non-decreasing times;
 * two items on different branches may share a time, since only one of them
 * will ever run.
 *
 * Sensing actions are marked, because their outcome must be observed before
 * the executor may continue, and the epistemic goal travels on the Plan so
 * that reaching a leaf can be checked rather than assumed.
 *
 * @param[in] task The task the policy solves, for goal and action lookup.
 * @param[in] tree The policy. Null means the goal already held: an empty plan.
 * @param[in] mapping Translation into PlanSys2 action expressions.
 * @param[out] error Set when an action cannot be translated.
 * @return The Plan, or nullopt when some action has no mapping.
 */
std::optional<plansys2_msgs::msg::Plan> to_policy_plan(
  const PlanningTask & task,
  const std::shared_ptr<PlanNode> & tree,
  const ActionMapping & mapping,
  std::string & error);

/**
 * @brief The duration of every action of `task`, as the mapping states it.
 *
 * The temporal search plans with the durations the executor will schedule
 * and time out against, so they come from the one place both read. An action
 * the mapping does not cover keeps one second: if the plan uses it, the
 * translation into a Plan fails on it anyway, with a clearer message.
 */
temporal::Durations durations_from_mapping(
  const PlanningTask & task, const ActionMapping & mapping);

/**
 * @brief Serialise a temporal policy into a Plan the executor dispatches by
 *        events.
 *
 * One item per action started, on every branch it is started on. Each item
 * names the items it waits for (`starts_after`: every item on its path that
 * ends before it starts, minus those another listed item already waits for)
 * and the observations that select its branch (`given_items`,
 * `given_outcomes`); `time` is the planned start. Items are written parents
 * first, so every index an item names is smaller than its own.
 *
 * @param[out] error Set when an action cannot be translated.
 */
std::optional<plansys2_msgs::msg::Plan> to_temporal_plan(
  const PlanningTask & task,
  const temporal::TemporalPolicy & policy,
  const ActionMapping & mapping,
  std::string & error);

/**
 * @brief Write a formula the way the epistemic domains read it.
 *
 * "(K r1 (clear corridor))", "(Kw A tails)", "(and ...)" — over the names of
 * the task rather than the indices the planner works in. This is the form
 * knowledge preconditions and epistemic goals travel in, and the form the
 * epistemic state parses back, so the two must agree; it is public so that
 * agreement can be tested rather than assumed.
 */
std::string render_formula(const PlanningTask & task, const Formula & formula);

/// True when the policy offers a genuine choice somewhere, so that flattening
/// it would discard a contingency.
bool policy_branches(const std::shared_ptr<PlanNode> & tree);

}  // namespace plansys2

#endif  // PLANSYS2_EPISTEMIC_PLANNER__POLICY_PLAN_HPP_
