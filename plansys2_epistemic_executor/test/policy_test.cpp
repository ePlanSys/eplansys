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

#include "plansys2_epistemic_executor/policy.hpp"

using plansys2::Policy;
using PlanItem = plansys2_msgs::msg::PlanItem;

namespace
{

PlanItem action(
  const std::string & expression, float time,
  const std::vector<std::uint32_t> & children = {},
  const std::vector<std::string> & outcomes = {})
{
  PlanItem item;
  item.action = expression;
  item.time = time;
  item.duration = 1.0f;
  item.children = children;
  item.outcomes = outcomes;
  item.sensing = children.size() > 1;
  return item;
}

plansys2_msgs::msg::Plan sequence_of(std::initializer_list<PlanItem> items)
{
  plansys2_msgs::msg::Plan plan;
  plan.items = items;
  return plan;
}

}  // namespace

// A plan from a classical planner sets none of the epistemic fields, and must
// still be a valid policy — otherwise the epistemic executor could not run the
// plans PlanSys2 already produces.
TEST(PolicyTest, AClassicalPlanIsAValidPolicy)
{
  auto plan = sequence_of({action("(move r1 a b)", 0.0f), action("(pick r1 o)", 1.0f)});
  EXPECT_EQ(Policy::validate(plan), "");

  const Policy policy(plan);
  EXPECT_FALSE(policy.branches());
  EXPECT_TRUE(policy.sequential());
  EXPECT_EQ(policy.size(), 2u);

  // Read the way PlanSys2 writes it: item i is followed by item i+1.
  EXPECT_EQ(policy.only_successor(0), 1u);
  EXPECT_FALSE(policy.only_successor(1).has_value());
  EXPECT_EQ(policy.preorder(), (std::vector<std::uint32_t>{0, 1}));
}

// The sequential reading is plan-wide, not per item: once any node names a
// continuation, a node without one ends its branch rather than falling through
// to the next item, which would be a different plan entirely.
TEST(PolicyTest, ALinkedPlanIsNotReadSequentially)
{
  auto plan = sequence_of(
    {
      action("(peek A)", 0.0f, {1, 2}, {"a", "b"}),
      action("(x)", 1.0f),
      action("(y)", 1.0f),
    });
  ASSERT_EQ(Policy::validate(plan), "");

  const Policy policy(plan);
  EXPECT_FALSE(policy.sequential());
  EXPECT_FALSE(policy.only_successor(1).has_value())
    << "item 1 ends its branch; item 2 belongs to the other one";
}

TEST(PolicyTest, AnEmptyPlanIsAValidPolicy)
{
  const plansys2_msgs::msg::Plan plan;
  EXPECT_EQ(Policy::validate(plan), "");
  EXPECT_TRUE(Policy(plan).empty());
}

TEST(PolicyTest, BranchesAreFoundAndFollowed)
{
  auto plan = sequence_of(
    {
      action("(peek A)", 0.0f, {1, 2}, {"e_tails", "e_heads"}),
      action("(shout A)", 1.0f),
      action("(open A)", 1.0f),
    });
  ASSERT_EQ(Policy::validate(plan), "");

  const Policy policy(plan);
  EXPECT_TRUE(policy.branches());
  EXPECT_EQ(policy.successor(0, "e_tails"), 1u);
  EXPECT_EQ(policy.successor(0, "e_heads"), 2u);

  // An outcome nobody planned for is not a lookup failure to paper over: it
  // is the case the executor must refuse to guess at.
  EXPECT_FALSE(policy.successor(0, "e_exploded").has_value());
}

TEST(PolicyTest, TerminalOutcomesAreDistinguishedFromUnplannedOnes)
{
  auto plan = sequence_of(
    {
      action("(peek A)", 0.0f, {1, PlanItem::POLICY_DONE}, {"e_tails", "e_heads"}),
      action("(shout A)", 1.0f),
    });
  ASSERT_EQ(Policy::validate(plan), "");

  const Policy policy(plan);
  EXPECT_EQ(policy.successor(0, "e_heads"), PlanItem::POLICY_DONE);
  EXPECT_FALSE(policy.successor(0, "e_missing").has_value());
}

TEST(PolicyTest, OnlySuccessorIsTheUnconditionalContinuation)
{
  auto plan = sequence_of(
    {
      action("(move r1 a b)", 0.0f, {1}, {"e_done"}),
      action("(pick r1 o)", 1.0f, {PlanItem::POLICY_DONE}, {"e_done"}),
    });
  ASSERT_EQ(Policy::validate(plan), "");

  const Policy policy(plan);
  EXPECT_EQ(policy.only_successor(0), 1u);
  // A node whose one continuation ends the policy has nothing to run next.
  EXPECT_FALSE(policy.only_successor(1).has_value());
}

TEST(PolicyTest, PreorderVisitsEveryNodeParentsFirst)
{
  auto plan = sequence_of(
    {
      action("(peek A)", 0.0f, {1, 2}, {"a", "b"}),
      action("(x)", 1.0f),
      action("(y)", 1.0f, {3}, {"c"}),
      action("(z)", 2.0f),
    });
  ASSERT_EQ(Policy::validate(plan), "");

  const auto order = Policy(plan).preorder();
  ASSERT_EQ(order.size(), 4u);
  EXPECT_EQ(order.front(), 0u);
  EXPECT_LT(
    std::find(order.begin(), order.end(), 2u) - order.begin(),
    std::find(order.begin(), order.end(), 3u) - order.begin());
}

// Validation exists to catch a policy that would misexecute rather than fail
// to parse. Each of these would run, and run wrongly.
TEST(PolicyTest, ValidateRejectsMisalignedOutcomes)
{
  auto plan = sequence_of({action("(peek A)", 0.0f, {1, 2}, {"only_one"}), action("(x)", 1.0f),
      action("(y)", 1.0f)});
  EXPECT_NE(Policy::validate(plan), "");
}

TEST(PolicyTest, ValidateRejectsADuplicatedOutcome)
{
  auto plan = sequence_of(
    {action("(peek A)", 0.0f, {1, 2}, {"same", "same"}), action("(x)", 1.0f),
      action("(y)", 1.0f)});
  EXPECT_NE(Policy::validate(plan), "");
}

TEST(PolicyTest, ValidateRejectsABranchOnANonSensingAction)
{
  auto plan = sequence_of(
    {action("(move r1 a b)", 0.0f, {1, 2}, {"a", "b"}), action("(x)", 1.0f),
      action("(y)", 1.0f)});
  plan.items[0].sensing = false;
  EXPECT_NE(Policy::validate(plan), "")
    << "nothing would ever observe which branch applies";
}

TEST(PolicyTest, ValidateRejectsACycle)
{
  auto plan = sequence_of(
    {action("(a)", 0.0f, {1}, {"x"}), action("(b)", 1.0f, {0}, {"y"})});
  EXPECT_NE(Policy::validate(plan), "") << "a policy is a tree, not a graph";
}

TEST(PolicyTest, ValidateRejectsAContinuationPastTheEnd)
{
  auto plan = sequence_of({action("(a)", 0.0f, {7}, {"x"})});
  EXPECT_NE(Policy::validate(plan), "");
}

TEST(PolicyTest, ValidateRejectsAnUnreachableNode)
{
  // A linked plan, so the sequential reading does not apply: item 2 is named
  // by nobody and the executor could never run it.
  auto plan = sequence_of(
    {
      action("(a)", 0.0f, {1}, {"x"}),
      action("(b)", 1.0f),
      action("(orphan)", 2.0f),
    });
  EXPECT_NE(Policy::validate(plan), "")
    << "the executor could never run it, so the policy does not mean what it says";
}

TEST(PolicyTest, ValidateRejectsASharedSubtree)
{
  auto plan = sequence_of(
    {
      action("(peek A)", 0.0f, {1, 1}, {"a", "b"}),
      action("(x)", 1.0f),
    });
  EXPECT_NE(Policy::validate(plan), "")
    << "one node reached from two branches would run as one action from both";
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
