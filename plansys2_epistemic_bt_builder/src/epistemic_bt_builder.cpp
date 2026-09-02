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

#include <limits>
#include <map>
#include <set>
#include <tuple>
#include <vector>
#include <memory>
#include <string>

#include <fstream>
#include <sstream>

#include "plansys2_epistemic_executor/policy.hpp"
#include "plansys2_pddl_parser/AmentIndexCompat.hpp"
#include "rclcpp/rclcpp.hpp"

namespace plansys2
{

namespace
{

rclcpp::Logger logger()
{
  return rclcpp::get_logger("epistemic_bt_builder");
}

/// Whether a template is one of PlanSys2's own packaged ones.
///
/// The executor hands every builder a template whether or not the deployment
/// chose one: with the parameter unset it loads its classical default. So "has
/// no place for a policy" says nothing on its own about whether anyone made a
/// mistake, and the two cases deserve different messages --- a deployment that
/// wrote its own template needs to hear loudly that it was not used, and one
/// that simply never set the parameter needs to hear nothing alarming at all.
bool is_a_packaged_plansys2_template(const std::string & content)
{
  std::string share;
  try {
    share = plansys2::get_package_share_dir("plansys2_executor");
  } catch (const std::exception &) {
    return false;   // cannot tell, so assume the deployment meant it
  }

  for (const auto * name : {"plansys2_action_bt.xml", "plansys2_action_bt_with_undo.xml",
      "plansys2_start_action_bt.xml", "plansys2_end_action_bt.xml"})
  {
    std::ifstream file(share + "/behavior_trees/" + name);
    if (!file) {
      continue;
    }
    std::ostringstream packaged;
    packaged << file.rdbuf();
    if (packaged.str() == content) {
      return true;
    }
  }
  return false;
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
  // with, and with the parameter unset that is PlanSys2's classical default —
  // which has no CONTINUATIONS placeholder and so no place for the rest of the
  // policy to go. Rendering it would produce a tree containing only the root
  // action, which runs, succeeds, and leaves the mission undone. A template
  // that cannot hold a policy is therefore refused in favour of the packaged
  // epistemic one.
  //
  // Which of the two happened decides how loudly to say so. A deployment that
  // wrote its own template needs to know it was not used; one that only never
  // set the parameter did nothing wrong, and is on the ordinary path.
  if (!bt_action_1.empty() && bt_action_1.find("CONTINUATIONS") == std::string::npos) {
    if (is_a_packaged_plansys2_template(bt_action_1)) {
      RCLCPP_INFO(
        logger(),
        "rendering with the packaged epistemic action template, which carries "
        "the observation from the performer to the epistemic update");
    } else {
      RCLCPP_WARN(
        logger(),
        "the action template has no CONTINUATIONS placeholder, so a policy could "
        "not be rendered into it; using the packaged epistemic template instead");
    }
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
  graph_ = build_graph(policy);

  RCLCPP_INFO(
    logger(), "building a tree for %zu policy nodes%s", policy.size(),
    policy.branches() ? ", branching" : "");

  return policy_to_bt(policy, bt_action_, precision_);
}

namespace
{

/// One graph node per policy node, keyed by policy index so the arcs can be
/// wired by index afterwards.
std::vector<Node::Ptr> policy_nodes(const Policy & policy)
{
  std::vector<Node::Ptr> nodes;
  nodes.reserve(policy.size());

  for (std::uint32_t i = 0; i < policy.size(); ++i) {
    const auto & item = policy.item(i);

    auto node = Node::make_shared(static_cast<int>(i));
    node->action.time = item.time;
    node->action.expression = item.action;
    node->action.duration = item.duration;
    // Every policy node drives a whole action, start to end, so it is durative
    // in the executor's sense even when its duration is the default.
    node->action.type = ActionType::DURATIVE;
    nodes.push_back(node);
  }
  return nodes;
}

}  // namespace

Graph::Ptr EpistemicBTBuilder::build_graph(const Policy & policy)
{
  auto graph = Graph::make_shared();
  if (policy.empty()) {
    return graph;
  }

  const auto nodes = policy_nodes(policy);

  for (std::uint32_t i = 0; i < policy.size(); ++i) {
    const auto & item = policy.item(i);

    for (const auto child : item.children) {
      if (child == plansys2_msgs::msg::PlanItem::POLICY_DONE || child >= nodes.size()) {
        continue;         // the policy ends on this outcome
      }

      // The bounds are filled in by propagate. They start fully open because
      // an arc with a bound nothing computed would be a constraint the
      // executor enforces for no reason.
      const auto arc = std::make_tuple(nodes[i], 0.0, std::numeric_limits<double>::infinity());
      nodes[child]->input_arcs.insert(arc);
      nodes[i]->output_arcs.insert(
        std::make_tuple(nodes[child], 0.0, std::numeric_limits<double>::infinity()));
    }
  }

  // A plan that names no continuations is the sequence PlanSys2 would execute,
  // and its graph is the chain that says so.
  if (policy.sequential()) {
    for (std::uint32_t i = 0; i + 1 < policy.size(); ++i) {
      const auto arc = std::make_tuple(nodes[i], 0.0, std::numeric_limits<double>::infinity());
      nodes[i + 1]->input_arcs.insert(arc);
      nodes[i]->output_arcs.insert(
        std::make_tuple(nodes[i + 1], 0.0, std::numeric_limits<double>::infinity()));
    }
  }

  for (const auto & node : nodes) {
    graph->nodes.push_back(node);
  }

  propagate(graph);
  return graph;
}

Graph::Ptr EpistemicBTBuilder::get_graph()
{
  return graph_;
}

bool EpistemicBTBuilder::propagate(Graph::Ptr graph)
{
  if (!graph) {
    return false;
  }

  // A continuation cannot start before the action it follows has finished, and
  // the epistemic layer makes that stricter than it is for a classical plan:
  // the knowledge guard on a continuation is checked against the state its
  // parent's update produced, so the two cannot overlap at all. The lower
  // bound is therefore the parent's whole duration.
  for (auto & node : graph->nodes) {
    std::set<std::tuple<Node::Ptr, double, double>> bounded;
    for (const auto & arc : node->input_arcs) {
      const auto parent = std::get<0>(arc);
      bounded.insert(
        std::make_tuple(
          parent, static_cast<double>(parent->action.duration),
          std::numeric_limits<double>::infinity()));
    }
    node->input_arcs = std::move(bounded);
  }

  for (auto & node : graph->nodes) {
    std::set<std::tuple<Node::Ptr, double, double>> bounded;
    for (const auto & arc : node->output_arcs) {
      bounded.insert(
        std::make_tuple(
          std::get<0>(arc), static_cast<double>(node->action.duration),
          std::numeric_limits<double>::infinity()));
    }
    node->output_arcs = std::move(bounded);
  }

  return true;
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
