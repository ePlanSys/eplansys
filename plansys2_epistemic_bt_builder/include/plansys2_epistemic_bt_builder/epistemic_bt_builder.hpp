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

#ifndef PLANSYS2_EPISTEMIC_BT_BUILDER__EPISTEMIC_BT_BUILDER_HPP_
#define PLANSYS2_EPISTEMIC_BT_BUILDER__EPISTEMIC_BT_BUILDER_HPP_

#include <map>
#include <memory>
#include <string>

#include "plansys2_epistemic_executor/policy_bt.hpp"
#include "plansys2_executor/BTBuilder.hpp"

namespace plansys2
{

/**
 * @class plansys2::EpistemicBTBuilder
 * @brief The BTBuilder that renders a policy rather than a sequence.
 *
 * Selected the way any other builder is, with the executor's
 * `bt_builder_plugin` parameter. Alongside it the executor needs
 * `bt_node_plugins` to name the epistemic node library, since the nodes this
 * builder writes into the tree are not ones the executor knows.
 *
 * A plan with no branches renders as the same flat sequence SimpleBTBuilder
 * would produce, plus the knowledge guard and the epistemic update around each
 * action. So this is a strict extension: it runs classical plans, and adds
 * what a policy needs on top.
 *
 * It builds no temporal graph. SimpleBTBuilder's graph orders actions by which
 * of them establish each other's preconditions; for a policy the ordering is
 * already fixed by the tree, and there is nothing left for a graph to decide.
 * get_graph() therefore returns nothing, as SequentialBTBuilder's does.
 */
class EpistemicBTBuilder : public BTBuilder
{
public:
  EpistemicBTBuilder() = default;

  void initialize(
    const std::string & bt_action_1 = "",
    const std::string & bt_action_2 = "",
    int precision = 3) override;

  std::string get_tree(const plansys2_msgs::msg::Plan & current_plan) override;

  /// The policy as a temporal graph: one node per policy node, an arc from
  /// each node to every continuation it can be followed by.
  ///
  /// SimpleBTBuilder's graph orders actions by which of them establish each
  /// other's preconditions, because a sequential plan leaves that to be
  /// worked out. A policy has the ordering already: the planner fixed it, and
  /// the branches say which orderings are alternatives to each other. So this
  /// records the structure instead of deriving one.
  Graph::Ptr get_graph() override;

  /// Compute the time bounds on each arc, from the durations the plan carries.
  bool propagate(Graph::Ptr graph) override;

  /// The policy as a graph, for the dotgraph the executor publishes. Drawn
  /// from the policy rather than from a temporal graph, so branches show as
  /// branches and each is labelled with the observation that selects it.
  std::string get_dotgraph(
    std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map,
    bool enable_legend = false,
    bool enable_print_graph = false) override;

private:
  std::string bt_action_;
  int precision_{3};
  plansys2_msgs::msg::Plan plan_;

  /// Built by get_tree and handed out by get_graph, so that the two describe
  /// the same policy. A graph built independently could disagree with the tree
  /// the executor is running.
  Graph::Ptr graph_;

  /// The graph for a policy, wired from its branches.
  Graph::Ptr build_graph(const Policy & policy);
};

}  // namespace plansys2

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(plansys2::EpistemicBTBuilder, plansys2::BTBuilder)

#endif  // PLANSYS2_EPISTEMIC_BT_BUILDER__EPISTEMIC_BT_BUILDER_HPP_
