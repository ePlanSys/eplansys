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

#ifndef PLANSYS2_EPISTEMIC_EXECUTOR__POLICY_HPP_
#define PLANSYS2_EPISTEMIC_EXECUTOR__POLICY_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "plansys2_msgs/msg/plan.hpp"

namespace plansys2
{

/**
 * @brief A read-only view of a Plan understood as a policy.
 *
 * PlanSys2 plans are sequences: item i is followed by item i+1. A plan for a
 * partially observable domain is a tree, because after a sensing action the
 * right continuation depends on what was observed. The Plan message carries
 * that shape in its epistemic fields (see PlanItem.msg), and this is the
 * agreed reading of them, shared by whatever builds a behavior tree from a
 * policy and whatever executes one.
 *
 * A classical plan is a policy too — a chain with no choices — so nothing here
 * requires the epistemic fields to be set. The distinction that matters is not
 * classical-versus-epistemic but branching-versus-not, and `branches()`
 * answers that.
 *
 * When no item anywhere names a continuation, the plan is read the way
 * PlanSys2 writes it: item i is followed by item i+1. That reading is
 * plan-wide rather than per item, because an item with no continuation is also
 * how a policy says "this branch ends here", and the two cannot be told apart
 * one item at a time. A policy therefore either links its nodes or does not,
 * and a planner that links any must link all.
 */
class Policy
{
public:
  /// Reasons a Plan is not a well-formed policy, in the form of a message.
  /// Empty means it is one.
  static std::string validate(const plansys2_msgs::msg::Plan & plan);

  /// Wrap a plan. The plan must have passed validate(); behaviour on an
  /// ill-formed one is unspecified rather than defended against at every call.
  explicit Policy(const plansys2_msgs::msg::Plan & plan)
  : plan_(plan) {}

  bool empty() const {return plan_.items.empty();}

  /// The number of policy nodes, not the length of any one execution.
  std::size_t size() const {return plan_.items.size();}

  const plansys2_msgs::msg::PlanItem & item(std::uint32_t index) const
  {
    return plan_.items[index];
  }

  /// Index of the root. Only meaningful when the policy is not empty.
  static constexpr std::uint32_t root() {return 0;}

  /// True when some node offers more than one continuation, so that executing
  /// the policy requires observing which one applies.
  bool branches() const;

  /// True when the plan names no continuations at all and is therefore read as
  /// the sequence PlanSys2 would execute.
  bool sequential() const;

  /// The epistemic goal the policy was built for, empty for a classical plan.
  const std::string & goal() const {return plan_.epistemic_goal;}

  /// The continuation for an observed outcome. Returns POLICY_DONE when the
  /// outcome completes the policy, and nullopt when the policy does not cover
  /// it — which is not a lookup failure but an execution failure: the world
  /// did something the plan did not anticipate.
  std::optional<std::uint32_t> successor(
    std::uint32_t index, const std::string & outcome) const;

  /// The only continuation of a node that offers exactly one, which is the
  /// shape of an ontic action. Nullopt when the node ends the policy or
  /// branches.
  std::optional<std::uint32_t> only_successor(std::uint32_t index) const;

  /// Node indices in the order they would be written out, parents first.
  std::vector<std::uint32_t> preorder() const;

  const plansys2_msgs::msg::Plan & message() const {return plan_;}

private:
  plansys2_msgs::msg::Plan plan_;
};

}  // namespace plansys2

#endif  // PLANSYS2_EPISTEMIC_EXECUTOR__POLICY_HPP_
