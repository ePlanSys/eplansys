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

#ifndef PLANSYS2_EPISTEMIC_EXECUTOR__POLICY_PARALLEL_HPP_
#define PLANSYS2_EPISTEMIC_EXECUTOR__POLICY_PARALLEL_HPP_

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "plansys2_epistemic_executor/policy.hpp"

namespace plansys2
{

/// Policy nodes that may be dispatched at the same time, in policy order.
using ParallelGroup = std::vector<std::uint32_t>;

/// Every group of more than one node. Nodes not named here are dispatched one
/// at a time, as they always were.
using ParallelGroups = std::vector<ParallelGroup>;

/// Whether two policy nodes' classical actions can overlap: whether neither
/// one's effects disturb what the other requires. Supplied by the caller,
/// because answering it means reading the PDDL domain and this library holds
/// no domain.
using Independence =
  std::function<bool (const plansys2_msgs::msg::PlanItem &, const plansys2_msgs::msg::PlanItem &)>;

/**
 * @brief The runs of a policy that could be dispatched together.
 *
 * A policy is dispatched a node at a time: the tree the executor runs is a
 * chain of sequences, and the second action of a mission starts when the first
 * has finished whether or not it had any reason to wait. Where the two are
 * independent the mission pays for that wait and buys nothing.
 *
 * This finds the runs where it buys nothing. A run is a maximal chain of
 * policy nodes, each the only continuation of the one before it, whose actions
 * are pairwise independent in both of these senses:
 *
 *  - classically, which `independent` answers from the domain, and
 *
 *  - epistemically, which is answered here and conservatively: the agents the
 *    two actions name must be disjoint, and neither node's knowledge
 *    requirements may mention an agent the other names. An action about one
 *    set of agents cannot establish or destroy a knowledge requirement about
 *    another set, so the guards of the members hold at the moment the group
 *    starts exactly as they held one after another.
 *
 * The second test is exact for ontic actions and for private and semi-private
 * ones, whose event models reach the agents they name. It is not exact for a
 * public announcement, which reaches every agent whether or not the action
 * names it, and whose update can therefore bear on a requirement this test
 * calls independent. That case is left to fail loudly rather than silently:
 * every member keeps its own `CheckKnowledge`, so a guard that should have
 * been established by another member fails the group, which fails the tree and
 * sends the executor to replan from the state the mission reached.
 *
 * Only the last node of a run may branch. A node with several continuations
 * ends its run, because the continuation is chosen by what the node observed
 * and cannot be started before it is known.
 */
ParallelGroups parallel_groups(const Policy & policy, const Independence & independent);

/// The agents an item's action names, from the grounded epistemic name and the
/// PDDL expression alike. Exposed because it is the whole of the epistemic
/// half of the independence test and is worth testing on its own.
std::vector<std::string> item_agents(const plansys2_msgs::msg::PlanItem & item);

}  // namespace plansys2

#endif  // PLANSYS2_EPISTEMIC_EXECUTOR__POLICY_PARALLEL_HPP_
