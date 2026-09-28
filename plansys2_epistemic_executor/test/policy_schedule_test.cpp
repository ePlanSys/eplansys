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

#include <gtest/gtest.h>

#include <algorithm>
#include <map>
#include <string>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/control_node.h"

#include "plansys2_epistemic_executor/policy_bt.hpp"
#include "plansys2_epistemic_executor/policy_schedule.hpp"

using plansys2::Policy;
using plansys2::Schedule;
using plansys2::policy_to_bt;
using plansys2::schedule_policy;
using plansys2::scheduled_groups;
using PlanItem = plansys2_msgs::msg::PlanItem;

namespace
{

constexpr auto kDone = PlanItem::POLICY_DONE;

PlanItem action(
  const std::string & expression, const std::string & grounded, float duration,
  const std::vector<std::uint32_t> & children = {},
  const std::vector<std::string> & outcomes = {})
{
  PlanItem item;
  item.action = expression;
  item.epistemic_action = grounded;
  item.duration = duration;
  item.children = children;
  item.outcomes = outcomes;
  item.sensing = children.size() > 1;
  return item;
}

/// The two-site survey as the planner returns it, with the demo's durations:
/// both drives, the north scan, and under each of its outcomes the north report
/// followed by the south scan and the south report.
plansys2_msgs::msg::Plan two_site_policy()
{
  plansys2_msgs::msg::Plan plan;
  plan.epistemic_goal = "(and (Kw north dirty-north) (Kw south dirty-south))";
  const std::vector<std::string> north = {"e-scan-north-dirty", "e-scan-north-clean"};
  const std::vector<std::string> south = {"e-scan-south-dirty", "e-scan-south-clean"};
  plan.items = {
    action("(goto_north north)", "goto-north_north", 4.0f, {1}),
    action("(goto_south south)", "goto-south_south", 4.0f, {2}),
    action("(scan_north north)", "scan-north_north", 3.0f, {3, 7}, north),
    action("(relay_north north relay)", "relay-north-dirty_north_relay", 2.0f, {4}),
    action("(scan_south south)", "scan-south_south", 3.0f, {5, 6}, south),
    action("(relay_south south relay)", "relay-south-dirty_south_relay", 2.0f, {}),
    action("(relay_south south relay)", "relay-south-clean_south_relay", 2.0f, {}),
    action("(relay_north north relay)", "relay-north-clean_north_relay", 2.0f, {8}),
    action("(scan_south south)", "scan-south_south", 3.0f, {9, 10}, south),
    action("(relay_south south relay)", "relay-south-dirty_south_relay", 2.0f, {}),
    action("(relay_south south relay)", "relay-south-clean_south_relay", 2.0f, {}),
  };
  return plan;
}

const plansys2::Independence kAlways =
  [](const PlanItem &, const PlanItem &) {return true;};

const plansys2::Independence kNever =
  [](const PlanItem &, const PlanItem &) {return false;};

/// What one execution runs, given what each sensing action observes: the
/// actions of the policy as written, in order.
std::vector<std::string> run_policy(
  const Policy & policy, const std::map<std::string, std::size_t> & observed)
{
  std::vector<std::string> ran;
  std::uint32_t at = Policy::root();
  while (true) {
    const auto & item = policy.item(at);
    ran.push_back(item.epistemic_action);
    if (const auto only = policy.only_successor(at)) {
      at = *only;
      continue;
    }
    if (item.children.size() <= 1) {
      return ran;
    }
    const auto next = item.children[observed.at(item.epistemic_action)];
    if (next == kDone) {
      return ran;
    }
    at = next;
  }
}

/// The same execution through a schedule, walked the way its rendering runs:
/// a group's members all run, then their outcomes choose the way on. Each entry
/// is the set of actions started together.
std::vector<std::vector<std::string>> run_schedule(
  const Policy & policy, const Schedule & schedule,
  const std::map<std::string, std::size_t> & observed)
{
  std::vector<std::vector<std::string>> ran;
  const plansys2::ScheduledNode * at = schedule.get();
  while (at) {
    std::vector<std::string> together;
    const plansys2::ScheduledNode * member = at;
    for (std::size_t i = 0; i < at->group; ++i) {
      together.push_back(policy.item(member->item).epistemic_action);
      if (i + 1 == at->group) {
        break;
      }
      // Every branch of a member continues with the next one, so which branch
      // the walk takes does not matter until the last member.
      const auto & item = policy.item(member->item);
      member = member->next.size() == 1 ? member->next.front().get() :
        member->next[observed.at(item.epistemic_action)].get();
    }
    ran.push_back(together);

    if (member->next.empty()) {
      break;
    }
    const auto & item = policy.item(member->item);
    at = member->next.size() == 1 ? member->next.front().get() :
      member->next[observed.at(item.epistemic_action)].get();
  }
  return ran;
}

std::size_t count_of(const std::string & haystack, const std::string & needle)
{
  std::size_t count = 0;
  for (std::size_t at = haystack.find(needle); at != std::string::npos;
    at = haystack.find(needle, at + needle.size()))
  {
    ++count;
  }
  return count;
}

class SwitchStub : public BT::ControlNode
{
public:
  SwitchStub(const std::string & name, const BT::NodeConfig & conf)
  : BT::ControlNode(name, conf) {}

  BT::NodeStatus tick() override {return BT::NodeStatus::SUCCESS;}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("node"),
      BT::InputPort<std::string>("outcome"),
      BT::InputPort<std::string>("outcomes")};
  }
};

void expect_parses(const std::string & xml)
{
  BT::BehaviorTreeFactory factory;
  const auto ok = [](BT::TreeNode &) {return BT::NodeStatus::SUCCESS;};
  factory.registerSimpleCondition(
    "CheckKnowledge", ok,
    {BT::InputPort<std::string>("node"), BT::InputPort<std::string>("action")});
  factory.registerSimpleCondition(
    "CheckEpistemicGoal", ok, {BT::InputPort<std::string>("goal")});
  factory.registerSimpleAction(
    "ApplyEpistemicUpdate", ok,
    {BT::InputPort<std::string>("node"), BT::InputPort<std::string>("action"),
      BT::InputPort<std::string>("observed"), BT::OutputPort<std::string>("outcome")});
  factory.registerSimpleCondition(
    "CheckBeliefUnchanged", ok,
    {BT::InputPort<std::string>("action"), BT::InputPort<std::string>("enabled")});
  for (const auto & name : {"WaitAtStartReq", "CheckOverAllReq", "CheckAtEndReq"}) {
    factory.registerSimpleCondition(name, ok, {BT::InputPort<std::string>("action")});
  }
  for (const auto & name : {"ApplyAtStartEffect", "ApplyAtEndEffect"}) {
    factory.registerSimpleAction(name, ok, {BT::InputPort<std::string>("action")});
  }
  factory.registerSimpleAction(
    "ExecuteAction", ok,
    {BT::InputPort<std::string>("action"), BT::OutputPort<std::string>("outcome")});
  factory.registerNodeType<SwitchStub>("EpistemicSwitch");

  EXPECT_NO_THROW(
    {
      auto tree = factory.createTreeFromText(xml);
      (void)tree;
    }) << xml;
}

}  // namespace

TEST(PolicySchedule, the_south_scan_runs_with_the_north_one)
{
  // The south scan is written under both outcomes of the north one, and
  // nothing about it depends on them. Moving it up puts it beside the north
  // scan, and the drives were already beside each other.
  const Policy policy(two_site_policy());
  const auto schedule = schedule_policy(policy, kAlways);

  const auto groups = scheduled_groups(schedule);
  ASSERT_EQ(groups.size(), 2u);
  EXPECT_EQ(groups[0], (std::vector<std::uint32_t>{0, 1}));
  EXPECT_EQ(groups[1], (std::vector<std::uint32_t>{2, 4}));
}

TEST(PolicySchedule, every_execution_runs_what_the_policy_would)
{
  // For each combination of what the two sites hold, the schedule runs the
  // same actions the policy does, and never one before an action it depends
  // on: each report after its own site's scan, and the two reports to the
  // relay one after the other.
  const Policy policy(two_site_policy());
  const auto schedule = schedule_policy(policy, kAlways);

  for (const std::size_t north : {0u, 1u}) {
    for (const std::size_t south : {0u, 1u}) {
      SCOPED_TRACE("north " + std::to_string(north) + ", south " + std::to_string(south));
      const std::map<std::string, std::size_t> observed = {
        {"scan-north_north", north}, {"scan-south_south", south}};

      auto written = run_policy(policy, observed);
      const auto steps = run_schedule(policy, schedule, observed);

      std::vector<std::string> scheduled;
      for (const auto & step : steps) {
        scheduled.insert(scheduled.end(), step.begin(), step.end());
      }
      std::sort(written.begin(), written.end());
      std::sort(scheduled.begin(), scheduled.end());
      EXPECT_EQ(scheduled, written);

      // Drives, scans, then the reports one at a time.
      ASSERT_EQ(steps.size(), 4u);
      EXPECT_EQ(steps[0].size(), 2u);
      EXPECT_EQ(steps[1].size(), 2u);
      EXPECT_EQ(steps[2].front().rfind("relay-north-", 0), 0u);
      EXPECT_EQ(steps[3].front().rfind("relay-south-", 0), 0u);
    }
  }
}

TEST(PolicySchedule, the_moved_scan_runs_once_and_both_branches_read_it)
{
  const Policy policy(two_site_policy());
  const auto xml = policy_to_bt(policy, schedule_policy(policy, kAlways), "", 3);

  EXPECT_EQ(count_of(xml, "<Parallel"), 2u);
  // Node 4 is the south scan under the north scan's first outcome, and the one
  // that runs. Its copy under the second outcome, node 8, never does; its
  // outcome is read from node 4 in both branches.
  EXPECT_EQ(count_of(xml, "<ApplyEpistemicUpdate node=\"4\""), 1u);
  EXPECT_EQ(count_of(xml, "<ApplyEpistemicUpdate node=\"8\""), 0u);
  EXPECT_EQ(count_of(xml, "outcome=\"{epistemic_outcome_4}\" outcomes="), 2u);
  expect_parses(xml);
}

TEST(PolicySchedule, nothing_moves_when_nothing_is_independent)
{
  const Policy policy(two_site_policy());
  EXPECT_EQ(
    policy_to_bt(policy, schedule_policy(policy, kNever), "", 3),
    policy_to_bt(policy, "", 3));
}

TEST(PolicySchedule, a_branch_without_the_action_keeps_it_below)
{
  // When the north site turns out clean the policy stops there, so the south
  // scan is not on every branch and cannot be moved above the north one.
  auto plan = two_site_policy();
  plan.items[2].children = {3, kDone};
  const Policy policy(plan);

  const auto groups = scheduled_groups(schedule_policy(policy, kAlways));
  for (const auto & group : groups) {
    EXPECT_EQ(std::find(group.begin(), group.end(), 2u), group.end())
      << "the north scan was grouped with an action not every branch runs";
  }
}

TEST(PolicySchedule, a_shared_agent_keeps_the_written_order)
{
  // The same scout at both sites: every action names it, so nothing may run
  // beside anything else and the policy renders as written.
  auto plan = two_site_policy();
  for (auto & item : plan.items) {
    item.epistemic_action += "_scout";
  }
  const Policy policy(plan);
  EXPECT_EQ(
    policy_to_bt(policy, schedule_policy(policy, kAlways), "", 3),
    policy_to_bt(policy, "", 3));
}
