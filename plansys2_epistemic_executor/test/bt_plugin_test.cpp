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
#include "rclcpp/rclcpp.hpp"

#include "plansys2_epistemic_executor/behavior_tree/epistemic_nodes.hpp"
#include "plansys2_epistemic_executor/policy_bt.hpp"

using plansys2::Policy;
using plansys2::policy_to_bt;
using PlanItem = plansys2_msgs::msg::PlanItem;

namespace
{

/// Register the PlanSys2 nodes the template reuses. They come from the
/// executor at run time; here they only have to exist for the tree to build.
void register_plansys2_stubs(BT::BehaviorTreeFactory & factory)
{
  for (const auto & name : {"WaitAtStartReq", "CheckOverAllReq", "CheckAtEndReq"}) {
    factory.registerSimpleCondition(name, [](BT::TreeNode &) {
        return BT::NodeStatus::SUCCESS;
      }, {BT::InputPort<std::string>("action")});
  }
  for (const auto & name : {"ApplyAtStartEffect", "ExecuteAction", "ApplyAtEndEffect"}) {
    factory.registerSimpleAction(name, [](BT::TreeNode &) {
        return BT::NodeStatus::SUCCESS;
      }, {BT::InputPort<std::string>("action")});
  }
}

plansys2_msgs::msg::Plan branching_policy()
{
  plansys2_msgs::msg::Plan plan;

  PlanItem peek;
  peek.action = "(peek A)";
  peek.epistemic_action = "peek_A";
  peek.duration = 1.0f;
  peek.sensing = true;
  peek.children = {1, PlanItem::POLICY_DONE};
  peek.outcomes = {"e_tails", "e_heads"};
  peek.knowledge_requirements = {"(K A looking_A)"};

  PlanItem shout;
  shout.action = "(shout-tails A)";
  shout.epistemic_action = "shout-tails_A";
  shout.time = 1.0f;
  shout.duration = 1.0f;

  plan.items = {peek, shout};
  plan.epistemic_goal = "(K A tails)";
  return plan;
}

}  // namespace

// The whole extension mechanism in one check: the executor loads a library by
// name and gets four node types it was never compiled against. If this breaks,
// the tree stops parsing inside the executor, far from the cause.
TEST(BtPluginTest, ThePluginRegistersEveryEpistemicNode)
{
  BT::BehaviorTreeFactory factory;

  const auto before = factory.builders().size();
  ASSERT_NO_THROW(factory.registerFromPlugin(EPISTEMIC_BT_NODES_LIBRARY));

  for (const auto & name : {"CheckKnowledge", "ApplyEpistemicUpdate", "EpistemicSwitch",
      "CheckEpistemicGoal"})
  {
    EXPECT_NE(factory.builders().find(name), factory.builders().end())
      << name << " was not registered by the plugin";
  }
  EXPECT_EQ(factory.builders().size(), before + 4u);
}

// A tree rendered from a policy must instantiate with the real nodes, not just
// with the stubs the rendering tests use. This is what proves the ports the
// renderer writes are the ports the nodes declare.
TEST(BtPluginTest, ARenderedPolicyInstantiatesWithTheRealNodes)
{
  const auto plan = branching_policy();
  ASSERT_EQ(Policy::validate(plan), "");

  const auto xml = policy_to_bt(Policy(plan));

  BT::BehaviorTreeFactory factory;
  register_plansys2_stubs(factory);
  ASSERT_NO_THROW(factory.registerFromPlugin(EPISTEMIC_BT_NODES_LIBRARY));

  // The nodes read the plan from the blackboard, exactly as the executor
  // publishes it. Without it they build, but find no policy to consult.
  auto blackboard = BT::Blackboard::create();
  blackboard->set(plansys2::kCurrentPlanKey, plan);

  ASSERT_NO_THROW(
  {
    auto tree = factory.createTreeFromText(xml, blackboard);
    (void)tree;
  }) << xml;
}

// A mismatch between the outcome list and the number of branches would mean
// the switch could select a child that is not there. It fails rather than
// indexing past the end.
TEST(BtPluginTest, TheSwitchRefusesAMismatchedTree)
{
  const auto xml =
    R"bt(<root BTCPP_format="4" main_tree_to_execute="MainTree">
      <BehaviorTree ID="MainTree">
        <EpistemicSwitch node="0" outcome="e_tails" outcomes="e_tails;e_heads">
          <AlwaysSuccess/>
        </EpistemicSwitch>
      </BehaviorTree>
    </root>)bt";

  BT::BehaviorTreeFactory factory;
  ASSERT_NO_THROW(factory.registerFromPlugin(EPISTEMIC_BT_NODES_LIBRARY));

  auto tree = factory.createTreeFromText(xml);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE)
    << "two outcomes and one branch is a tree that does not match its policy";
}

// The case the architecture exists for: an outcome nobody planned for must
// stop execution rather than pick a branch built for a different belief.
TEST(BtPluginTest, TheSwitchRefusesAnUnplannedOutcome)
{
  const auto xml =
    R"bt(<root BTCPP_format="4" main_tree_to_execute="MainTree">
      <BehaviorTree ID="MainTree">
        <EpistemicSwitch node="0" outcome="e_exploded" outcomes="e_tails;e_heads">
          <AlwaysSuccess/>
          <AlwaysSuccess/>
        </EpistemicSwitch>
      </BehaviorTree>
    </root>)bt";

  BT::BehaviorTreeFactory factory;
  ASSERT_NO_THROW(factory.registerFromPlugin(EPISTEMIC_BT_NODES_LIBRARY));

  auto tree = factory.createTreeFromText(xml);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST(BtPluginTest, TheSwitchRunsTheBranchForTheObservedOutcome)
{
  const auto xml =
    R"bt(<root BTCPP_format="4" main_tree_to_execute="MainTree">
      <BehaviorTree ID="MainTree">
        <EpistemicSwitch node="0" outcome="e_heads" outcomes="e_tails;e_heads">
          <AlwaysFailure/>
          <AlwaysSuccess/>
        </EpistemicSwitch>
      </BehaviorTree>
    </root>)bt";

  BT::BehaviorTreeFactory factory;
  ASSERT_NO_THROW(factory.registerFromPlugin(EPISTEMIC_BT_NODES_LIBRARY));

  auto tree = factory.createTreeFromText(xml);
  // The second branch, and only it: taking the first would have failed here.
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

// With no epistemic goal there is nothing for this node to check, and a
// classical plan must not fail on its account.
TEST(BtPluginTest, TheGoalCheckPassesWhenThereIsNoEpistemicGoal)
{
  const auto xml =
    R"bt(<root BTCPP_format="4" main_tree_to_execute="MainTree">
      <BehaviorTree ID="MainTree">
        <CheckEpistemicGoal goal=""/>
      </BehaviorTree>
    </root>)bt";

  BT::BehaviorTreeFactory factory;
  ASSERT_NO_THROW(factory.registerFromPlugin(EPISTEMIC_BT_NODES_LIBRARY));

  auto tree = factory.createTreeFromText(xml);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  // The nodes create a client, and a client is a ROS node.
  rclcpp::init(argc, argv);
  const auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
