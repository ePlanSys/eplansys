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

#include "plansys2_epistemic_bt_builder/epistemic_bt_builder.hpp"

#include <map>
#include <memory>
#include <string>

#include "plansys2_epistemic_executor/policy.hpp"
#include "rclcpp/rclcpp.hpp"

namespace plansys2
{

namespace
{

rclcpp::Logger logger()
{
  return rclcpp::get_logger("epistemic_bt_builder");
}

std::string dot_escape(const std::string & text)
{
  std::string out;
  for (const char c : text) {
    if (c == '"' || c == '\\') {
      out += '\\';
    }
    out += c;
  }
  return out;
}

}  // namespace

void EpistemicBTBuilder::initialize(
  const std::string & bt_action_1, const std::string & bt_action_2, int precision)
{
  (void)bt_action_2;   // there is one template here: a policy node is a policy node
  precision_ = precision;

  // The executor hands every builder the action template it is configured
  // with, and its default is PlanSys2's — which has no CONTINUATIONS placeholder
  // and so no place for the rest of the policy to go. Rendering it would
  // produce a tree containing only the root action, which runs, succeeds, and
  // leaves the mission undone. A template that cannot hold a policy is
  // therefore refused in favour of the packaged one, loudly, because a
  // deployment that meant to supply its own needs to know it was not used.
  if (!bt_action_1.empty() && bt_action_1.find("CONTINUATIONS") == std::string::npos) {
    RCLCPP_WARN(
      logger(),
      "the action template has no CONTINUATIONS placeholder, so a policy could "
      "not be rendered into it; using the packaged epistemic template instead");
    bt_action_.clear();
    return;
  }

  bt_action_ = bt_action_1;
}

std::string EpistemicBTBuilder::get_tree(const plansys2_msgs::msg::Plan & current_plan)
{
  const auto problem = Policy::validate(current_plan);
  if (!problem.empty()) {
    // Returning an empty tree is how a builder reports failure to the
    // executor. Saying why first matters, because the executor's own message
    // is only that the tree could not be computed.
    RCLCPP_ERROR(logger(), "the plan is not a well-formed policy: %s", problem.c_str());
    return "";
  }

  plan_ = current_plan;
  const Policy policy(current_plan);

  RCLCPP_INFO(
    logger(), "building a tree for %zu policy nodes%s", policy.size(),
    policy.branches() ? ", branching" : "");

  return policy_to_bt(policy, bt_action_, precision_);
}

std::string EpistemicBTBuilder::get_dotgraph(
  std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map,
  bool enable_legend, bool enable_print_graph)
{
  (void)enable_legend;

  const Policy policy(plan_);
  std::string dot = "digraph plan {\n  node [shape=box];\n";

  for (const auto index : policy.preorder()) {
    const auto & item = policy.item(index);
    const auto action_id = policy_action_id(item, precision_);

    std::string label = std::to_string(index) + ": " + item.action;
    if (!item.knowledge_requirements.empty()) {
      // The guard is drawn with the node, because a policy that fails is
      // usually one whose knowledge precondition did not hold, and this is
      // where someone reading the graph looks for why.
      for (const auto & requirement : item.knowledge_requirements) {
        label += "\\n" + requirement;
      }
    }

    // Colour by how far execution got, from the same map the executor fills.
    std::string colour = "white";
    if (action_map) {
      const auto found = action_map->find(action_id);
      if (found != action_map->end()) {
        colour = found->second.at_end_effects_applied ? "palegreen" :
          (found->second.at_start_effects_applied ? "khaki" : "white");
      }
    }

    dot += "  n" + std::to_string(index) + " [label=\"" + dot_escape(label) +
      "\", style=filled, fillcolor=" + colour + "];\n";
  }

  for (const auto index : policy.preorder()) {
    const auto & item = policy.item(index);
    if (policy.sequential()) {
      const auto next = policy.only_successor(index);
      if (next) {
        dot += "  n" + std::to_string(index) + " -> n" + std::to_string(*next) + ";\n";
      }
      continue;
    }
    for (std::size_t i = 0; i < item.children.size(); ++i) {
      if (item.children[i] == plansys2_msgs::msg::PlanItem::POLICY_DONE) {
        dot += "  done" + std::to_string(index) + std::to_string(i) +
          " [label=\"done\", shape=ellipse];\n";
        dot += "  n" + std::to_string(index) + " -> done" + std::to_string(index) +
          std::to_string(i) + " [label=\"" + dot_escape(item.outcomes[i]) + "\"];\n";
        continue;
      }
      dot += "  n" + std::to_string(index) + " -> n" + std::to_string(item.children[i]) +
        " [label=\"" + dot_escape(item.outcomes[i]) + "\"];\n";
    }
  }

  if (!policy.goal().empty()) {
    dot += "  goal [label=\"" + dot_escape(policy.goal()) + "\", shape=doubleoctagon];\n";
  }

  dot += "}\n";

  if (enable_print_graph) {
    std::cout << dot << std::endl;
  }
  return dot;
}

}  // namespace plansys2
