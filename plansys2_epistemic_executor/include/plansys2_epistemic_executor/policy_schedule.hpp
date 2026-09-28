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

#ifndef PLANSYS2_EPISTEMIC_EXECUTOR__POLICY_SCHEDULE_HPP_
#define PLANSYS2_EPISTEMIC_EXECUTOR__POLICY_SCHEDULE_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "plansys2_epistemic_executor/policy.hpp"
#include "plansys2_epistemic_executor/policy_parallel.hpp"

namespace plansys2
{

/**
 * @brief One node of a policy arranged for dispatch.
 *
 * `item` is the policy node whose action is rendered here: its guard, its
 * action and its update. `next` is what follows, aligned with the item's
 * outcomes when it branches, one entry when it does not, and empty when it ends
 * the policy; a null entry is an outcome that completes the policy.
 *
 * The same item may appear at several places, because moving a node above a
 * branch copies what it passed into every branch. Only one of them runs in any
 * one execution.
 */
struct ScheduledNode
{
  std::uint32_t item{0};
  std::vector<std::shared_ptr<ScheduledNode>> next;

  /// How many nodes, this one first, start together. A group is followed along
  /// `next`, and every branch of a member leads to the same action as the next
  /// member: the members are one action each, run once, and their outcomes
  /// choose the continuation after all of them have finished.
  std::size_t group{1};
};

using Schedule = std::shared_ptr<ScheduledNode>;

/// The policy exactly as it is written: one node per item, nothing moved and
/// nothing grouped. Rendering it gives the tree `policy_to_bt` always gave.
Schedule policy_schedule(const Policy & policy);

/// The same, with the groups `parallel_groups` found dispatched together.
Schedule policy_schedule(const Policy & policy, const ParallelGroups & groups);

/**
 * @brief A policy rearranged so that independent actions start as early as
 * their dependencies allow.
 *
 * A policy fixes an order among actions that often had no reason to be
 * ordered, and a branch fixes one more: nothing written below a sensing action
 * starts before its outcome is known, even an action the outcome has no
 * bearing on. In the two-site survey the south scan waits for the north one to
 * be read, although it is written under both of its outcomes.
 *
 * Two things are done about it:
 *
 *  - An action is moved above its parent when the two are independent and the
 *    action could start earlier. Its earliest start is the end of the latest
 *    action above it that it depends on, by declared duration. Above a branch
 *    it may only move when every branch begins with it, and the branch is then
 *    copied under each of its outcomes. Moving one of two independent actions
 *    past the other leaves both earliest starts where they were, so the
 *    rearrangement ends.
 *
 *  - A chain of pairwise independent actions is started together, as
 *    `parallel_groups` does, except that a member may branch as long as every
 *    one of its branches continues with the next member. The members' outcomes
 *    then choose the continuation, one after another, once all have finished.
 *
 * Independence is `parallel_groups`': classical, as `independent` answers from
 * the domain, and epistemic, from the agents the actions name.
 *
 * A plan with no continuations is PlanSys2's own sequence and is returned as
 * written, as `parallel_groups` leaves it too.
 */
Schedule schedule_policy(const Policy & policy, const Independence & independent);

/// The groups of a schedule, each as the items that start together, in the
/// order a depth-first walk meets them. What a builder reports.
std::vector<std::vector<std::uint32_t>> scheduled_groups(const Schedule & schedule);

}  // namespace plansys2

#endif  // PLANSYS2_EPISTEMIC_EXECUTOR__POLICY_SCHEDULE_HPP_
