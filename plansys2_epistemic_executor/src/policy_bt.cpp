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

#include "plansys2_epistemic_executor/policy_bt.hpp"

#include <cmath>
#include <sstream>
#include <string>
#include <vector>

namespace plansys2
{

const char * const kDefaultEpistemicActionBT =
  R"bt(<Sequence name="NODE_NAME">
  <CheckKnowledge node="NODE_ID" action="ACTION_ID"/>
  <WaitAtStartReq action="ACTION_ID"/>
  <ApplyAtStartEffect action="ACTION_ID"/>
  <ReactiveSequence name="ACTION_ID">
    <CheckOverAllReq action="ACTION_ID"/>
    <ExecuteAction action="ACTION_ID"/>
  </ReactiveSequence>
  <CheckAtEndReq action="ACTION_ID"/>
  <ApplyAtEndEffect action="ACTION_ID"/>
  <ApplyEpistemicUpdate node="NODE_ID" action="ACTION_ID" outcome="{OUTCOME_KEY}"/>
CONTINUATIONS
</Sequence>)bt";

namespace
{

std::string indent(int level)
{
  return std::string(level * 2, ' ');
}

void replace_all(std::string & text, const std::string & from, const std::string & to)
{
  if (from.empty()) {
    return;
  }
  std::size_t at = 0;
  while ((at = text.find(from, at)) != std::string::npos) {
    text.replace(at, from.length(), to);
    at += to.length();
  }
}

/// XML attribute values carry action expressions and formulas, which contain
/// characters XML reserves. Escaping them here rather than trusting the
/// content keeps a domain with an apostrophe in a name from producing a tree
/// that will not parse.
std::string escape(const std::string & text)
{
  std::string out;
  out.reserve(text.size());
  for (const char c : text) {
    switch (c) {
      case '&': out += "&amp;"; break;
      case '<': out += "&lt;"; break;
      case '>': out += "&gt;"; break;
      case '"': out += "&quot;"; break;
      case '\'': out += "&apos;"; break;
      default: out += c;
    }
  }
  return out;
}

/// Re-indent a rendered block so the tree reads as a tree.
std::string shift(const std::string & block, int level)
{
  std::istringstream lines(block);
  std::string line;
  std::string out;
  while (std::getline(lines, line)) {
    if (!line.empty()) {
      out += indent(level) + line + "\n";
    }
  }
  return out;
}

}  // namespace

std::string policy_action_id(const plansys2_msgs::msg::PlanItem & item, int precision)
{
  const float scale = std::pow(10.0f, static_cast<float>(precision));
  return item.action + ":" + std::to_string(static_cast<int>(item.time * scale));
}

std::string policy_to_bt(const Policy & policy, const std::string & action_bt, int precision)
{
  const std::string tmpl = action_bt.empty() ? kDefaultEpistemicActionBT : action_bt;

  // Each node renders its own subtree and splices its continuations into it,
  // so the recursion returns a block rather than writing into a shared buffer.
  const auto render = [&](std::uint32_t index, const auto & self) -> std::string {
      const auto & item = policy.item(index);
      const std::string action_id = policy_action_id(item, precision);
      const std::string node_id = std::to_string(index);
      // One blackboard entry per node: two sensing actions in flight on
      // different branches never share a key, and the name says which node
      // wrote it when reading a Groot trace.
      const std::string outcome_key = "epistemic_outcome_" + node_id;

      std::string continuations;

      const auto only = policy.only_successor(index);
      if (only) {
        // Nothing to choose: the continuation simply follows. This is what
        // makes a classical plan render as the flat sequence PlanSys2 builds.
        continuations = self(*only, self);
      } else if (item.children.size() > 1) {
        std::string branches;
        for (std::size_t i = 0; i < item.children.size(); ++i) {
          branches += indent(1) + "<!-- observed " + escape(item.outcomes[i]) + " -->\n";
          if (item.children[i] == plansys2_msgs::msg::PlanItem::POLICY_DONE) {
            // The policy is complete on this outcome. Saying so explicitly
            // keeps the branch arity equal to the outcome list, so the switch
            // can index one by the other.
            branches += indent(1) + "<AlwaysSuccess/>\n";
          } else {
            branches += shift(self(item.children[i], self), 1);
          }
        }

        std::string outcome_list;
        for (std::size_t i = 0; i < item.outcomes.size(); ++i) {
          outcome_list += (i ? ";" : "") + item.outcomes[i];
        }

        continuations =
          "<EpistemicSwitch node=\"" + node_id + "\" outcome=\"{" + outcome_key +
          "}\" outcomes=\"" + escape(outcome_list) + "\">\n" +
          branches +
          "</EpistemicSwitch>\n";
      }
      // Otherwise this node ends the policy and nothing follows it.

      std::string block = tmpl;
      replace_all(block, "NODE_NAME", escape("node_" + node_id + " " + item.action));
      replace_all(block, "ACTION_ID", escape(action_id));
      replace_all(block, "NODE_ID", node_id);
      replace_all(block, "OUTCOME_KEY", outcome_key);
      replace_all(block, "CONTINUATIONS", shift(continuations, 1));
      return block + "\n";
    };

  std::string body;
  if (!policy.empty()) {
    body = shift(render(Policy::root(), render), 3);
  }

  // The goal check is outside the policy rather than at each leaf. A leaf is
  // where one execution stops, and every leaf claims to reach the goal, but
  // that claim was made against the model at planning time; this asks the
  // state that actually resulted.
  std::string goal_check;
  if (!policy.goal().empty()) {
    goal_check = indent(3) + "<CheckEpistemicGoal goal=\"" + escape(policy.goal()) + "\"/>\n";
  }

  return
    "<root BTCPP_format=\"4\" main_tree_to_execute=\"MainTree\">\n" +
    indent(1) + "<BehaviorTree ID=\"MainTree\">\n" +
    indent(2) + "<Sequence name=\"EpistemicPolicy\">\n" +
    body +
    goal_check +
    indent(2) + "</Sequence>\n" +
    indent(1) + "</BehaviorTree>\n" +
    "</root>\n";
}

}  // namespace plansys2
