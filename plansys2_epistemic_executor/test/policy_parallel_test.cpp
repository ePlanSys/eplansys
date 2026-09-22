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

#include <string>
#include <vector>

#include "plansys2_epistemic_executor/policy_bt.hpp"
#include "plansys2_epistemic_executor/policy_parallel.hpp"

using plansys2::ParallelGroups;
using plansys2::Policy;
using plansys2::item_agents;
using plansys2::parallel_groups;
using plansys2::policy_to_bt;
using PlanItem = plansys2_msgs::msg::PlanItem;

namespace
{

PlanItem action(
  const std::string & expression, const std::string & grounded, float time,
  const std::vector<std::uint32_t> & children = {},
  const std::vector<std::string> & outcomes = {})
{
  PlanItem item;
  item.action = expression;
  item.epistemic_action = grounded;
  item.time = time;
  item.duration = 1.0f;
  item.children = children;
  item.outcomes = outcomes;
  item.sensing = children.size() > 1;
  return item;
}

/// The two halves of the two-site survey, in the order the planner returns
/// them: a drive and a scan at one site, then a report, then the same at the
/// other. The report and the second drive are the pair with no reason to be
/// ordered.
plansys2_msgs::msg::Plan two_site_policy()
{
  plansys2_msgs::msg::Plan plan;
  plan.epistemic_goal = "(and (Kw north dirty-north) (Kw south dirty-south))";
  plan.items = {
    action("(goto_north north)", "goto-north_north", 0.0f, {1}),
    action(
      "(scan_north north)", "scan-north_north", 4.0f, {2, 2},
      {"e-scan-north-dirty", "e-scan-north-clean"}),
    action("(relay_north north relay)", "relay-north-dirty_north_relay", 7.0f, {3}),
    action("(goto_south south)", "goto-south_south", 9.0f, {4}),
    action("(scan_south south)", "scan-south_south", 13.0f, {}),
  };
  return plan;
}

/// Everything is independent. Isolating the epistemic half of the test.
const plansys2::Independence kAlways =
  [](const PlanItem &, const PlanItem &) {return true;};

const plansys2::Independence kNever =
  [](const PlanItem &, const PlanItem &) {return false;};

}  // namespace

TEST(PolicyParallel, agents_come_from_both_vocabularies)
{
  const auto named = item_agents(
    action("(relay_north north relay)", "relay-north-dirty_north_relay", 0.0f));

  EXPECT_EQ(named.size(), 2u);
  EXPECT_NE(std::find(named.begin(), named.end(), "north"), named.end());
  EXPECT_NE(std::find(named.begin(), named.end(), "relay"), named.end());
}

TEST(PolicyParallel, a_run_of_independent_nodes_is_one_group)
{
  const Policy policy(two_site_policy());
  const auto groups = parallel_groups(policy, kAlways);

  ASSERT_EQ(groups.size(), 1u);
  EXPECT_EQ(groups.front(), (plansys2::ParallelGroup{2, 3}));
}

TEST(PolicyParallel, a_shared_agent_ends_a_group)
{
  // The drive and the scan at the south site are the same robot's, so they
  // stay ordered however independent their classical halves look.
  const Policy policy(two_site_policy());
  const auto groups = parallel_groups(policy, kAlways);

  ASSERT_EQ(groups.size(), 1u);
  EXPECT_EQ(groups.front().back(), 3u);
}

TEST(PolicyParallel, a_branch_ends_a_group)
{
  // Node 1 branches, so nothing after it joins node 0: which continuation runs
  // is not known until it has been observed.
  const Policy policy(two_site_policy());
  const auto groups = parallel_groups(policy, kAlways);

  for (const auto & group : groups) {
    EXPECT_EQ(std::find(group.begin(), group.end(), 0u), group.end());
  }
}

TEST(PolicyParallel, the_classical_test_can_refuse_every_group)
{
  const Policy policy(two_site_policy());
  EXPECT_TRUE(parallel_groups(policy, kNever).empty());
}

TEST(PolicyParallel, a_knowledge_requirement_naming_the_other_agent_refuses)
{
  auto plan = two_site_policy();
  plan.items[3].knowledge_requirements = {"(K south (told north))"};

  const Policy policy(plan);
  EXPECT_TRUE(parallel_groups(policy, kAlways).empty());
}

TEST(PolicyParallel, a_sequential_plan_has_no_groups)
{
  // A plan that names no continuations is PlanSys2's own sequence, which has a
  // builder of its own for this.
  plansys2_msgs::msg::Plan plan;
  plan.items = {
    action("(goto_north north)", "goto-north_north", 0.0f),
    action("(goto_south south)", "goto-south_south", 4.0f),
  };

  const Policy policy(plan);
  EXPECT_TRUE(parallel_groups(policy, kAlways).empty());
}

TEST(PolicyParallel, the_group_is_rendered_as_one_parallel)
{
  const Policy policy(two_site_policy());
  const auto tree = policy_to_bt(policy, "", 3, parallel_groups(policy, kAlways));

  EXPECT_NE(tree.find("<Parallel success_count=\"2\" failure_count=\"1\">"), std::string::npos);
  // Both members are inside it, and what followed the last of them follows the
  // group rather than either member.
  const auto parallel_at = tree.find("<Parallel");
  const auto closed_at = tree.find("</Parallel>");
  ASSERT_NE(closed_at, std::string::npos);
  EXPECT_LT(tree.find("(relay_north north relay)"), closed_at);
  EXPECT_LT(tree.find("(goto_south south)"), closed_at);
  EXPECT_GT(tree.find("(scan_south south)"), parallel_at);
}

TEST(PolicyParallel, a_group_inside_a_branch_is_one_child_of_the_switch)
{
  // The switch indexes its branches by its outcomes, so a branch that rendered
  // as two elements would leave the tree claiming more branches than the
  // policy has outcomes. The group and its continuation are one element.
  const Policy policy(two_site_policy());
  const auto tree = policy_to_bt(policy, "", 3, parallel_groups(policy, kAlways));

  const auto switch_at = tree.find("<EpistemicSwitch");
  ASSERT_NE(switch_at, std::string::npos);
  const auto switch_end = tree.find("</EpistemicSwitch>", switch_at);
  ASSERT_NE(switch_end, std::string::npos);

  const auto body = tree.substr(switch_at, switch_end - switch_at);
  // One sequence per outcome, and the two outcomes of this policy lead to the
  // same node, so the rendering repeats it: two group sequences, no more.
  std::size_t groups_rendered = 0;
  for (std::size_t at = body.find("<Sequence name=\"group_");
    at != std::string::npos;
    at = body.find("<Sequence name=\"group_", at + 1))
  {
    ++groups_rendered;
  }
  EXPECT_EQ(groups_rendered, 2u);
}

TEST(PolicyParallel, no_groups_renders_exactly_what_it_always_did)
{
  const Policy policy(two_site_policy());

  EXPECT_EQ(policy_to_bt(policy, "", 3, ParallelGroups{}), policy_to_bt(policy, "", 3));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
