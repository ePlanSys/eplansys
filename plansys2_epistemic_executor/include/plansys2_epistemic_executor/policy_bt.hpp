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

#ifndef PLANSYS2_EPISTEMIC_EXECUTOR__POLICY_BT_HPP_
#define PLANSYS2_EPISTEMIC_EXECUTOR__POLICY_BT_HPP_

#include <string>

#include "plansys2_epistemic_executor/policy.hpp"

namespace plansys2
{

/**
 * @brief How a policy is rendered as a behavior tree.
 *
 * PlanSys2 renders a plan as a sequence of per-action subtrees, each guarding
 * its action with the PDDL requirements around it:
 *
 *     Sequence
 *       WaitAtStartReq -> ApplyAtStartEffect
 *       ReactiveSequence[CheckOverAllReq, ExecuteAction]
 *       CheckAtEndReq -> ApplyAtEndEffect
 *
 * The epistemic rendering keeps that subtree unchanged — the same nodes drive
 * the same action performers against the same problem expert — and wraps it
 * with three things a sequence cannot express:
 *
 *  1. A knowledge guard before the action. Some preconditions are about what
 *     an agent knows, not about what is true, and the problem expert holds no
 *     such fact to check. `CheckKnowledge` asks the epistemic state instead.
 *
 *  2. An epistemic update after it. Executing an action changes what the
 *     agents know, and by more than its own effects: a public announcement
 *     tells every observer something, and even a failed observation rules
 *     worlds out. `ApplyEpistemicUpdate` performs the DEL product update, so
 *     that the guard on the next action sees the state the action produced.
 *
 *  3. A branch on what was observed. This is the part with no counterpart in
 *     PlanSys2, whose plans commit to one future. `EpistemicSwitch` reads the
 *     outcome the update reported and runs the continuation planned for it —
 *     and fails when the outcome is one the policy never planned for, which is
 *     the executor's cue to replan rather than to carry on regardless.
 *
 * The whole tree is then followed by `CheckEpistemicGoal`, because reaching a
 * leaf of a policy is not the same as having achieved the goal: a plan that
 * ran to completion in the wrong branch would otherwise report success.
 *
 * Rendered for one node, with a branch:
 *
 *     Sequence "node_0"
 *       CheckKnowledge node="0"
 *       ...the PlanSys2 action subtree...
 *       ApplyEpistemicUpdate node="0" outcome="{outcome_0}"
 *       EpistemicSwitch node="0" outcome="{outcome_0}"
 *         Sequence "node_1"   <- the continuation for the first outcome
 *         AlwaysSuccess       <- an outcome that completes the policy
 *
 * A node with one continuation is rendered without a switch, since there is
 * nothing to choose: its subtree follows in the same sequence. That keeps a
 * classical plan rendering as the same flat sequence PlanSys2 would build.
 */

/// The default per-action subtree, matching plansys2_action_bt.xml with the
/// epistemic guard and update around it. ACTION_ID and NODE_ID are replaced
/// per node; SUBTREES is where continuations are spliced in.
extern const char * const kDefaultEpistemicActionBT;

/**
 * @brief Render a policy as a BehaviorTree.CPP v4 tree.
 *
 * @param[in] policy The policy. An empty one renders a tree that only checks
 *   the goal, which is the honest rendering of "nothing to do".
 * @param[in] action_bt The per-action template. Empty for the default above.
 * @param[in] precision Digits of the start time in an action id, matching what
 *   the executor uses to key its action map; 3 is what PlanSys2 uses.
 * @return The behavior tree XML.
 */
std::string policy_to_bt(
  const Policy & policy,
  const std::string & action_bt = "",
  int precision = 3);

/// The action id the executor keys its action map by, for one policy node.
/// This has to agree with plansys2::BTBuilder::to_action_id exactly: it is the
/// name by which a behavior tree node finds the action it is meant to drive.
std::string policy_action_id(
  const plansys2_msgs::msg::PlanItem & item, int precision = 3);

}  // namespace plansys2

#endif  // PLANSYS2_EPISTEMIC_EXECUTOR__POLICY_BT_HPP_
