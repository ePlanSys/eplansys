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

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/control_node.h"
#include "behaviortree_cpp/xml_parsing.h"

#include "plansys2_epistemic_executor/policy_bt.hpp"

using plansys2::Policy;
using plansys2::policy_action_id;
using plansys2::policy_to_bt;
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
  item.epistemic_action = "grounded";
  item.time = time;
  item.duration = 1.0f;
  item.children = children;
  item.outcomes = outcomes;
  item.sensing = children.size() > 1;
  return item;
}

/// Stands in for EpistemicSwitch, which lives in the ROS-facing library. The
/// parser only needs a control node with the same name and ports to accept the
/// tree, and the rendering is what is under test here.
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

/// A tree the executor cannot parse is worthless however good it looks, so
/// every rendering here is handed to the real BehaviorTree.CPP parser. The
/// epistemic nodes are registered as stubs: what is under test is the shape of
/// the tree, not what the nodes do when ticked.
void expect_parses(const std::string & xml)
{
  BT::BehaviorTreeFactory factory;

  factory.registerSimpleCondition(
    "CheckKnowledge", [](BT::TreeNode &) {
      return BT::NodeStatus::SUCCESS;
    }, {BT::InputPort<std::string>("node"), BT::InputPort<std::string>("action")});
  factory.registerSimpleCondition(
    "CheckEpistemicGoal", [](BT::TreeNode &) {
      return BT::NodeStatus::SUCCESS;
    }, {BT::InputPort<std::string>("goal")});
  factory.registerSimpleAction(
    "ApplyEpistemicUpdate", [](BT::TreeNode &) {
      return BT::NodeStatus::SUCCESS;
    }, {BT::InputPort<std::string>("node"), BT::InputPort<std::string>("action"),
        BT::InputPort<std::string>("observed"), BT::OutputPort<std::string>("outcome")});

  // The PlanSys2 nodes the template reuses unchanged.
  for (const auto & name : {"WaitAtStartReq", "CheckOverAllReq", "CheckAtEndReq"}) {
    factory.registerSimpleCondition(
      name, [](BT::TreeNode &) {
        return BT::NodeStatus::SUCCESS;
      }, {BT::InputPort<std::string>("action")});
  }
  for (const auto & name : {"ApplyAtStartEffect", "ApplyAtEndEffect"}) {
    factory.registerSimpleAction(
      name, [](BT::TreeNode &) {
        return BT::NodeStatus::SUCCESS;
      }, {BT::InputPort<std::string>("action")});
  }
  // ExecuteAction apart from the others: it carries what the performer
  // observed out on a port, which the template binds to the update's input.
  factory.registerSimpleAction(
    "ExecuteAction", [](BT::TreeNode &) {
      return BT::NodeStatus::SUCCESS;
    }, {BT::InputPort<std::string>("action"), BT::OutputPort<std::string>("outcome")});

  factory.registerNodeType<SwitchStub>("EpistemicSwitch");

  EXPECT_NO_THROW(
    {
      auto tree = factory.createTreeFromText(xml);
      (void)tree;
    }) << xml;
}

}  // namespace

TEST(PolicyBtTest, ActionIdMatchesTheExecutorsKey)
{
  auto item = action("(ask c1)", 1.25f);
  // plansys2::BTBuilder::to_action_id, reproduced: expression, colon,
  // milliseconds. A behavior tree node finds its action by this name.
  EXPECT_EQ(policy_action_id(item, 3), "(ask c1):1250");
}

TEST(PolicyBtTest, AChainRendersAsNestedSequencesWithoutASwitch)
{
  plansys2_msgs::msg::Plan plan;
  plan.items = {
    action("(ask c1)", 0.0f, {1}, {"e_told"}),
    action("(ask c2)", 1.0f),
  };
  ASSERT_EQ(Policy::validate(plan), "");

  const auto xml = policy_to_bt(Policy(plan));

  EXPECT_EQ(count_of(xml, "<EpistemicSwitch"), 0u)
    << "nothing is chosen, so nothing should choose";
  EXPECT_EQ(count_of(xml, "<ExecuteAction"), 2u);
  EXPECT_NE(xml.find("(ask c1):0"), std::string::npos);
  EXPECT_NE(xml.find("(ask c2):1000"), std::string::npos);
  expect_parses(xml);
}

TEST(PolicyBtTest, EveryActionKeepsThePlanSys2Guards)
{
  plansys2_msgs::msg::Plan plan;
  plan.items = {action("(ask c1)", 0.0f)};

  const auto xml = policy_to_bt(Policy(plan));

  // The point of the architecture is that it extends PlanSys2's rather than
  // replacing it: the same guards still surround the same action.
  for (const auto & node : {"WaitAtStartReq", "ApplyAtStartEffect", "CheckOverAllReq",
      "ExecuteAction", "CheckAtEndReq", "ApplyAtEndEffect"})
  {
    EXPECT_EQ(count_of(xml, std::string("<") + node), 1u) << node << " is missing";
  }
  // And the two it adds around them.
  EXPECT_EQ(count_of(xml, "<CheckKnowledge"), 1u);
  EXPECT_EQ(count_of(xml, "<ApplyEpistemicUpdate"), 1u);
  expect_parses(xml);
}

TEST(PolicyBtTest, TheObservationTravelsFromThePerformerToTheUpdate)
{
  plansys2_msgs::msg::Plan plan;
  plan.items = {
    action("(peek A)", 0.0f, {1, 2}, {"e_tails", "e_heads"}),
    action("(shout-tails A)", 1.0f),
    action("(open A)", 1.0f),
  };
  ASSERT_EQ(Policy::validate(plan), "");

  const auto xml = policy_to_bt(Policy(plan));

  // The performer reports what it saw when it finishes; ExecuteAction puts it
  // on the blackboard and the update reads it from there. Without the binding
  // the update is left asking a model that designates several worlds which one
  // is the case, and it rightly refuses to guess --- so this is the wire that
  // makes a sensing action work on the packaged template alone.
  EXPECT_NE(
    xml.find("<ExecuteAction action=\"(peek A):0\" outcome=\"{epistemic_observed_0}\""),
    std::string::npos) << "ExecuteAction does not publish what the performer observed";
  EXPECT_NE(xml.find("observed=\"{epistemic_observed_0}\""), std::string::npos)
    << "ApplyEpistemicUpdate is not reading it";

  // One entry per policy node: two sensing actions on different branches must
  // not overwrite each other's observation.
  EXPECT_EQ(count_of(xml, "{epistemic_observed_1}"), 2u);
  EXPECT_EQ(count_of(xml, "{epistemic_observed_2}"), 2u);

  // And it stays distinct from what the state made of it: the robot's report
  // and the model's reading of it are two different things, and the switch
  // branches on the second.
  EXPECT_NE(xml.find("outcome=\"{epistemic_outcome_0}\""), std::string::npos);
  expect_parses(xml);
}

TEST(PolicyBtTest, ABranchRendersAsASwitchWithOneChildPerOutcome)
{
  plansys2_msgs::msg::Plan plan;
  plan.items = {
    action("(peek A)", 0.0f, {1, 2}, {"e_tails", "e_heads"}),
    action("(shout-tails A)", 1.0f),
    action("(open A)", 1.0f),
  };
  ASSERT_EQ(Policy::validate(plan), "");

  const auto xml = policy_to_bt(Policy(plan));

  EXPECT_EQ(count_of(xml, "<EpistemicSwitch"), 1u);
  EXPECT_NE(xml.find("outcomes=\"e_tails;e_heads\""), std::string::npos)
    << "the switch matches an outcome to a child by position, so the order has "
    "to travel with it";
  EXPECT_EQ(count_of(xml, "<ExecuteAction"), 3u)
    << "both continuations are in the tree; that is the point";
  expect_parses(xml);
}

// The outcome flows from the update that reported it to the switch that acts
// on it, through a blackboard entry named for the node. Two sensing actions
// must not share one.
TEST(PolicyBtTest, EachNodeGetsItsOwnOutcomeEntry)
{
  plansys2_msgs::msg::Plan plan;
  plan.items = {
    action("(peek A)", 0.0f, {1, 2}, {"e_tails", "e_heads"}),
    action("(peek B)", 1.0f, {3, PlanItem::POLICY_DONE}, {"e_tails", "e_heads"}),
    action("(open A)", 1.0f),
    action("(shout B)", 2.0f),
  };
  ASSERT_EQ(Policy::validate(plan), "");

  const auto xml = policy_to_bt(Policy(plan));

  EXPECT_NE(xml.find("{epistemic_outcome_0}"), std::string::npos);
  EXPECT_NE(xml.find("{epistemic_outcome_1}"), std::string::npos);
  EXPECT_EQ(count_of(xml, "{epistemic_outcome_0}"), 2u)
    << "written by the update, read by the switch";
  expect_parses(xml);
}

// An outcome that completes the policy still needs a child, so that the switch
// can keep matching outcomes to children by position.
TEST(PolicyBtTest, ATerminalOutcomeRendersAsASuccessfulBranch)
{
  plansys2_msgs::msg::Plan plan;
  plan.items = {
    action("(peek A)", 0.0f, {1, PlanItem::POLICY_DONE}, {"e_tails", "e_heads"}),
    action("(shout A)", 1.0f),
  };
  ASSERT_EQ(Policy::validate(plan), "");

  const auto xml = policy_to_bt(Policy(plan));

  EXPECT_EQ(count_of(xml, "<AlwaysSuccess/>"), 1u);
  EXPECT_NE(xml.find("outcomes=\"e_tails;e_heads\""), std::string::npos);
  expect_parses(xml);
}

TEST(PolicyBtTest, TheEpistemicGoalIsCheckedAfterThePolicy)
{
  plansys2_msgs::msg::Plan plan;
  plan.items = {action("(ask c1)", 0.0f)};
  plan.epistemic_goal = "(Kw c1 muddy_c1)";

  const auto xml = policy_to_bt(Policy(plan));

  EXPECT_NE(xml.find("<CheckEpistemicGoal goal=\"(Kw c1 muddy_c1)\"/>"), std::string::npos);
  // After, not inside: reaching a leaf is not achieving the goal.
  EXPECT_GT(xml.find("CheckEpistemicGoal"), xml.find("ExecuteAction"));
  expect_parses(xml);
}

TEST(PolicyBtTest, AnEmptyPolicyStillChecksItsGoal)
{
  plansys2_msgs::msg::Plan plan;
  plan.epistemic_goal = "(Kw c1 muddy_c1)";

  const auto xml = policy_to_bt(Policy(plan));

  EXPECT_EQ(count_of(xml, "<ExecuteAction"), 0u);
  EXPECT_EQ(count_of(xml, "<CheckEpistemicGoal"), 1u)
    << "the planner said there was nothing to do; this is what confirms it";
  expect_parses(xml);
}

// Action expressions and formulas contain characters XML reserves. An
// unescaped one produces a tree that will not parse, at execution time.
TEST(PolicyBtTest, ReservedCharactersAreEscaped)
{
  plansys2_msgs::msg::Plan plan;
  plan.items = {action("(say \"hi\" & <bye>)", 0.0f)};
  plan.epistemic_goal = "(K a p<q)";

  const auto xml = policy_to_bt(Policy(plan));

  EXPECT_EQ(xml.find("\"hi\" &"), std::string::npos);
  EXPECT_NE(xml.find("&quot;hi&quot;"), std::string::npos);
  EXPECT_NE(xml.find("&amp;"), std::string::npos);
  expect_parses(xml);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
