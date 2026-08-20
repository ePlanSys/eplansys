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

#ifndef PLANSYS2_EPISTEMIC_EXECUTOR__BEHAVIOR_TREE__EPISTEMIC_NODES_HPP_
#define PLANSYS2_EPISTEMIC_EXECUTOR__BEHAVIOR_TREE__EPISTEMIC_NODES_HPP_

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_cpp/control_node.h"

#include "plansys2_epistemic_executor/EpistemicStateClient.hpp"
#include "plansys2_epistemic_executor/policy.hpp"

namespace plansys2
{

/// Blackboard keys these nodes use.
///
/// A host that already builds policies sets `kEpistemicPolicyKey` directly.
/// One that does not — a stock PlanSys2 executor — only has to publish the
/// plan it is running under `kCurrentPlanKey`, and the policy is read from it;
/// that is the whole contract, and it costs the executor no dependency on this
/// package. The client is created on first use and shared through the
/// blackboard, so the four nodes of a tree talk to the state through one.
constexpr const char * kEpistemicClientKey = "epistemic_state_client";
constexpr const char * kEpistemicPolicyKey = "epistemic_policy";
constexpr const char * kCurrentPlanKey = "current_plan";

/// Register all four with a factory. One call, so a host cannot pick up the
/// guard without the update and end up checking a state nothing advances.
void register_epistemic_nodes(BT::BehaviorTreeFactory & factory);

/**
 * @class plansys2::CheckKnowledge
 * @brief Guards an action with what its agents must know.
 *
 * The counterpart of CheckOverAllReq for conditions the problem expert cannot
 * answer. "The corridor is clear" is a fact and belongs there; "r1 knows the
 * corridor is clear" is not a fact about the corridor, and a plan that senses
 * before committing turns on exactly that difference.
 *
 * Fails when a requirement does not hold, and equally when the state cannot be
 * reached — an unanswered guard is not a satisfied one.
 */
class CheckKnowledge : public BT::ConditionNode
{
public:
  CheckKnowledge(const std::string & xml_tag_name, const BT::NodeConfig & conf);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<std::string>("node", "Index of the policy node being guarded"),
        BT::InputPort<std::string>("action", "Action id, for the error message"),
      });
  }

private:
  EpistemicStateClient::Ptr client_;
  std::shared_ptr<Policy> policy_;
};

/**
 * @class plansys2::ApplyEpistemicUpdate
 * @brief Advances the epistemic state by the action that just ran.
 *
 * The counterpart of ApplyAtEndEffect, one level up. An action changes what
 * agents know by more than its own effects: an announcement informs everyone
 * who was listening, and even an observation that finds nothing rules worlds
 * out. This performs the DEL product update, so the next action's guard is
 * checked against the state this action produced rather than the one it
 * started from.
 *
 * For a sensing action it also reports which outcome occurred, on the port the
 * following EpistemicSwitch reads. The outcome comes from the observation the
 * performer reported when there is one, and otherwise from the model, which
 * can determine it whenever it designates a single world.
 */
class ApplyEpistemicUpdate : public BT::ActionNodeBase
{
public:
  ApplyEpistemicUpdate(const std::string & xml_tag_name, const BT::NodeConfig & conf);

  void halt() override {}
  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<std::string>("node", "Index of the policy node that ran"),
        BT::InputPort<std::string>("action", "Action id, for the error message"),
        BT::InputPort<std::string>(
          "observed", "", "Outcome the performer observed; empty to let the state decide"),
        BT::OutputPort<std::string>("outcome", "The outcome that occurred"),
      });
  }

private:
  EpistemicStateClient::Ptr client_;
  std::shared_ptr<Policy> policy_;
  bool applied_{false};
};

/**
 * @class plansys2::EpistemicSwitch
 * @brief Runs the continuation planned for what was observed.
 *
 * This is the node PlanSys2 has no counterpart for. Its plans commit to one
 * future, so a sequence is enough to express them; a policy keeps one
 * continuation per outcome and picks between them at execution time.
 *
 * Children are in the order of the `outcomes` list, so the outcome selects a
 * child by position. An outcome the policy does not list fails the node rather
 * than defaulting to a branch: the world did something the plan did not
 * anticipate, and continuing down a branch built for a different observation
 * is how a robot acts confidently on a belief it has no support for. The
 * failure propagates to the executor, whose answer to a failed plan is to
 * replan.
 */
class EpistemicSwitch : public BT::ControlNode
{
public:
  EpistemicSwitch(const std::string & xml_tag_name, const BT::NodeConfig & conf);

  void halt() override;
  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<std::string>("node", "Index of the policy node that branched"),
        BT::InputPort<std::string>("outcome", "The observed outcome"),
        BT::InputPort<std::string>("outcomes", "Outcomes in child order, ';' separated"),
      });
  }

private:
  int running_child_{-1};
};

/**
 * @class plansys2::CheckEpistemicGoal
 * @brief Asks whether the policy actually achieved what it was built for.
 *
 * Running a policy to a leaf means one execution finished, not that the goal
 * holds: every leaf was believed to reach it, but that belief was formed
 * against the model at planning time. This asks the state that resulted.
 *
 * A tree with no epistemic goal — a classical plan — succeeds here, since the
 * executor already checks the PDDL goal.
 */
class CheckEpistemicGoal : public BT::ConditionNode
{
public:
  CheckEpistemicGoal(const std::string & xml_tag_name, const BT::NodeConfig & conf);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {BT::InputPort<std::string>("goal", "The epistemic goal, empty for none")});
  }

private:
  EpistemicStateClient::Ptr client_;
};

}  // namespace plansys2

#endif  // PLANSYS2_EPISTEMIC_EXECUTOR__BEHAVIOR_TREE__EPISTEMIC_NODES_HPP_
