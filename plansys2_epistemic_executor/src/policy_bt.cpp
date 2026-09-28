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

#include <map>

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
    <CheckBeliefUnchanged action="ACTION_ID"/>
    <ExecuteAction action="ACTION_ID" outcome="{OBSERVED_KEY}"/>
  </ReactiveSequence>
  <CheckAtEndReq action="ACTION_ID"/>
  <ApplyAtEndEffect action="ACTION_ID"/>
  <ApplyEpistemicUpdate node="NODE_ID" action="ACTION_ID" observed="{OBSERVED_KEY}"
    outcome="{OUTCOME_KEY}"/>
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
  return policy_to_bt(policy, action_bt, precision, ParallelGroups{});
}

std::string policy_to_bt(
  const Policy & policy, const std::string & action_bt, int precision,
  const ParallelGroups & groups)
{
  return policy_to_bt(policy, policy_schedule(policy, groups), action_bt, precision);
}

std::string policy_to_bt(
  const Policy & policy, const Schedule & schedule, const std::string & action_bt,
  int precision)
{
  const std::string tmpl = action_bt.empty() ? kDefaultEpistemicActionBT : action_bt;

  // One node's own block, with whatever is to follow it spliced in.
  //
  // One blackboard entry per node, named after it: two sensing actions in
  // flight never share a key, whether they are on different branches or in the
  // same group, and the name says which node wrote it when reading a Groot
  // trace. `epistemic_observed_N` is what the robot says it saw and
  // `epistemic_outcome_N` is what the epistemic state made of it; they agree
  // whenever the report names an outcome the action defines, and keeping them
  // apart is what lets the state disagree.
  const auto block_of =
    [&](std::uint32_t at, const std::string & continuations) -> std::string {
      const auto & node = policy.item(at);
      const std::string at_id = std::to_string(at);

      std::string block = tmpl;
      replace_all(block, "NODE_NAME", escape("node_" + at_id + " " + node.action));
      replace_all(block, "ACTION_ID", escape(policy_action_id(node, precision)));
      replace_all(block, "NODE_ID", at_id);
      replace_all(block, "OUTCOME_KEY", "epistemic_outcome_" + at_id);
      replace_all(block, "OBSERVED_KEY", "epistemic_observed_" + at_id);
      replace_all(block, "CONTINUATIONS", shift(continuations, 1));
      return block + "\n";
    };

  // A switch over what `decided_by` observed, with one child per outcome of
  // `at`. They are the same node unless `at` is a copy of a group member,
  // which never runs itself: the member it copies ran, and its outcome is the
  // one to read.
  const auto switch_of =
    [&](const ScheduledNode & at, std::uint32_t decided_by, const auto & child) -> std::string {
      const auto & node = policy.item(at.item);

      std::string branches;
      for (std::size_t i = 0; i < at.next.size(); ++i) {
        branches += indent(1) + "<!-- observed " + escape(node.outcomes[i]) + " -->\n";
        if (!at.next[i]) {
          // The policy is complete on this outcome. Saying so explicitly keeps
          // the branch arity equal to the outcome list, so the switch can
          // index one by the other.
          branches += indent(1) + "<AlwaysSuccess/>\n";
        } else {
          branches += shift(child(at.next[i]), 1);
        }
      }

      std::string outcome_list;
      for (std::size_t i = 0; i < node.outcomes.size(); ++i) {
        outcome_list += (i ? ";" : "") + node.outcomes[i];
      }

      return
        "<EpistemicSwitch node=\"" + std::to_string(decided_by) +
        "\" outcome=\"{epistemic_outcome_" + std::to_string(decided_by) +
        "}\" outcomes=\"" + escape(outcome_list) + "\">\n" +
        branches +
        "</EpistemicSwitch>\n";
    };

  // Each node renders its own subtree and splices its continuations into it,
  // so the recursion returns a block rather than writing into a shared buffer.
  const auto render = [&](const Schedule & at, const auto & self) -> std::string {
      // What follows a node once it has run: nothing, the next block, or a
      // switch over what it observed.
      const auto continuations_of =
        [&](const ScheduledNode & node, std::uint32_t decided_by) -> std::string {
          if (node.next.size() == 1) {
            // Nothing to choose: the continuation simply follows. This is what
            // makes a classical plan render as the flat sequence PlanSys2
            // builds.
            return node.next.front() ? self(node.next.front(), self) : "";
          }
          if (node.next.empty()) {
            return "";      // this node ends the policy
          }
          return switch_of(
            node, decided_by, [&](const Schedule & next) {return self(next, self);});
        };

      if (at->group <= 1) {
        return block_of(at->item, continuations_of(*at, at->item));
      }

      // A group runs as one, so no member carries a continuation: the chain
      // that linked them is what the group replaces, and what follows the last
      // of them follows the whole group.
      std::vector<std::uint32_t> members;
      const ScheduledNode * member = at.get();
      for (std::size_t i = 0; i < at->group; ++i) {
        members.push_back(member->item);
        if (i + 1 < at->group) {
          member = member->next.front().get();
        }
      }

      std::string blocks;
      for (const auto item : members) {
        blocks += shift(block_of(item, ""), 1);
      }

      // Once every member has finished, their outcomes choose the way on, one
      // member after another. A member that branches is present as the same
      // action on each of its branches, so the walk reaches a copy of the next
      // member whichever outcome it takes.
      const auto after_member =
        [&](const ScheduledNode & node, std::size_t position, const auto & again) -> std::string {
          if (position + 1 == members.size()) {
            return continuations_of(node, members[position]);
          }
          if (node.next.size() == 1) {
            return again(*node.next.front(), position + 1, again);
          }
          return switch_of(
            node, members[position],
            [&](const Schedule & next) {return again(*next, position + 1, again);});
        };

      // The group and what follows it are one element, not two. A node's
      // rendering is spliced in wherever a single child is expected --- a
      // switch branch is exactly that --- and two siblings there would give the
      // switch more branches than the policy has outcomes.
      const std::string after = after_member(*at, 0, after_member);

      return
        "<Sequence name=\"" + escape("group_" + std::to_string(at->item)) + "\">\n" +
        indent(1) + "<Parallel success_count=\"" + std::to_string(members.size()) +
        "\" failure_count=\"1\">\n" +
        shift(blocks, 1) +
        indent(1) + "</Parallel>\n" +
        shift(after, 1) +
        "</Sequence>\n";
    };

  std::string body;
  if (!policy.empty() && schedule) {
    body = shift(render(schedule, render), 3);
  }

  // The goal check is outside the policy rather than at each leaf. A leaf is
  // where one execution stops, and every leaf claims to reach the goal, but
  // that claim was made against the model at planning time; this asks the
  // state that actually resulted.
  std::string goal_check;
  if (!policy.goal().empty()) {
    goal_check = indent(3) + "<CheckEpistemicGoal goal=\"" + escape(policy.goal()) + "\"/>\n";
  }

  // Nothing to run and nothing to check happens when the planner is handed a
  // goal that already holds. The sequence still has to have a child:
  // BehaviorTree.CPP rejects an empty one, and a tree that cannot be parsed is
  // a worse answer to "there is nothing to do" than a tree that does nothing.
  if (body.empty() && goal_check.empty()) {
    body = indent(3) + "<AlwaysSuccess/>\n";
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
