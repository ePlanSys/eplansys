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


// Temporal policies: event-driven dispatch, ticked for real.
//
// The action block is replaced through the renderer's template by a scripted
// stand-in that takes a given number of ticks and reports a given outcome, so
// the gates (WaitForEnds, CheckGiven, MarkDone) run exactly as in the
// executor, without a performer or an epistemic state behind them.

#include <unistd.h>

#include <map>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "plansys2_epistemic_executor/behavior_tree/epistemic_nodes.hpp"
#include "plansys2_epistemic_executor/policy_bt.hpp"

using plansys2::Policy;
using PlanItem = plansys2_msgs::msg::PlanItem;

namespace
{

/// What the stand-in for each item does, and what happened when it did.
struct Script
{
  std::map<std::string, int> ticks;            // how long each item runs
  std::map<std::string, std::string> outcome;  // what it observes
  std::map<std::string, int> started, ended;   // when, in ticks of the tree
  int now{0};
};

Script * g_script = nullptr;

class Act : public BT::StatefulActionNode
{
public:
  Act(const std::string & name, const BT::NodeConfig & conf)
  : StatefulActionNode(name, conf) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("node"), BT::OutputPort<std::string>("outcome")};
  }

  BT::NodeStatus onStart() override
  {
    getInput("node", node_);
    g_script->started[node_] = g_script->now;
    left_ = g_script->ticks.count(node_) ? g_script->ticks.at(node_) : 1;
    return onRunning();
  }

  BT::NodeStatus onRunning() override
  {
    if (--left_ > 0) {
      return BT::NodeStatus::RUNNING;
    }
    g_script->ended[node_] = g_script->now;
    setOutput("outcome", g_script->outcome.count(node_) ? g_script->outcome.at(node_) : "");
    return BT::NodeStatus::SUCCESS;
  }

  void onHalted() override {}

private:
  std::string node_;
  int left_{0};
};

const char * const kScriptedBlock =
  R"(<Sequence name="NODE_NAME">
  <Act node="NODE_ID" outcome="{OUTCOME_KEY}"/>
CONTINUATIONS
</Sequence>)";

PlanItem item(
  const std::string & action, std::vector<std::uint32_t> after = {},
  std::vector<std::uint32_t> given = {}, std::vector<std::string> outcomes = {})
{
  PlanItem i;
  i.action = action;
  i.epistemic_action = action;
  i.duration = 1.0f;
  i.starts_after = std::move(after);
  i.given_items = std::move(given);
  i.given_outcomes = std::move(outcomes);
  return i;
}

/// Look (sensing, short) and drive (long) start together. When the look ends,
/// one report runs for what it saw. When the drive ends, `finish` runs, but
/// only on the clear branch. It waits for both: the drive's end opens its
/// decision point, and the look was planned to end before that.
plansys2_msgs::msg::Plan corridor()
{
  plansys2_msgs::msg::Plan plan;
  plan.temporal = true;
  auto look = item("(look r1)");
  look.sensing = true;
  plan.items = {
    look,                                                  // 0
    item("(drive r2)"),                                    // 1
    item("(report_clear r1)", {0}, {0}, {"clear"}),        // 2
    item("(report_blocked r1)", {0}, {0}, {"blocked"}),    // 3
    item("(finish r2)", {0, 1}, {0}, {"clear"}),           // 4
  };
  return plan;
}

/// Tick the rendered tree to completion under `script`.
BT::NodeStatus run(const plansys2_msgs::msg::Plan & plan, Script & script)
{
  g_script = &script;
  BT::BehaviorTreeFactory factory;
  plansys2::register_epistemic_nodes(factory);
  factory.registerNodeType<Act>("Act");

  const auto xml = plansys2::temporal_policy_to_bt(Policy(plan), kScriptedBlock);
  auto tree = factory.createTreeFromText(xml);
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  for (script.now = 0; script.now < 100 && status == BT::NodeStatus::RUNNING; ++script.now) {
    status = tree.tickOnce();
  }
  g_script = nullptr;
  return status;
}

}  // namespace

TEST(TemporalPolicy, TheReaderChecksWhatItemsDependOn)
{
  auto plan = corridor();
  EXPECT_EQ(Policy::validate(plan), "");

  auto forward = plan;
  forward.items[1].starts_after = {2};
  EXPECT_NE(Policy::validate(forward), "");

  auto unpaired = plan;
  unpaired.items[2].given_outcomes.clear();
  EXPECT_NE(Policy::validate(unpaired), "");
}

TEST(TemporalPolicy, TheRenderedTreeInstantiatesWithTheRealNodes)
{
  const auto plan = corridor();
  BT::BehaviorTreeFactory factory;
  ASSERT_NO_THROW(factory.registerFromPlugin(EPISTEMIC_BT_NODES_LIBRARY));
  for (const auto & name : {"WaitAtStartReq", "CheckOverAllReq", "CheckAtEndReq"}) {
    factory.registerSimpleCondition(
      name, [](BT::TreeNode &) {return BT::NodeStatus::SUCCESS;},
      {BT::InputPort<std::string>("action")});
  }
  for (const auto & name : {"ApplyAtStartEffect", "ApplyAtEndEffect"}) {
    factory.registerSimpleAction(
      name, [](BT::TreeNode &) {return BT::NodeStatus::SUCCESS;},
      {BT::InputPort<std::string>("action")});
  }
  factory.registerSimpleAction(
    "ExecuteAction", [](BT::TreeNode &) {return BT::NodeStatus::SUCCESS;},
    {BT::InputPort<std::string>("action"), BT::OutputPort<std::string>("outcome")});

  auto blackboard = BT::Blackboard::create();
  blackboard->set(plansys2::kCurrentPlanKey, plan);
  const auto xml = plansys2::temporal_policy_to_bt(Policy(plan));
  try {
    auto tree = factory.createTreeFromText(xml, blackboard);
    (void)tree;
  } catch (const std::exception & e) {
    FAIL() << e.what() << "\n" << xml;
  }
}

TEST(TemporalPolicy, TheClearBranchRunsAndTheOtherDrainsAway)
{
  Script script;
  script.ticks = {{"0", 2}, {"1", 6}};
  script.outcome = {{"0", "clear"}};
  ASSERT_EQ(run(corridor(), script), BT::NodeStatus::SUCCESS);

  // Look and drive start together.
  EXPECT_EQ(script.started.at("0"), script.started.at("1"));
  // The report for what the look saw runs, the other never starts.
  EXPECT_TRUE(script.started.count("2"));
  EXPECT_FALSE(script.started.count("3"));
  // It runs while the drive is still going: the look's end opened its
  // decision point, the drive's did not.
  EXPECT_LT(script.started.at("2"), script.ended.at("1"));
  // finish waits for the drive, and runs because the branch is clear.
  ASSERT_TRUE(script.started.count("4"));
  EXPECT_GE(script.started.at("4"), script.ended.at("1"));
}

TEST(TemporalPolicy, TheBlockedBranchSkipsWhatTheClearOneWouldRun)
{
  Script script;
  script.ticks = {{"0", 2}, {"1", 6}};
  script.outcome = {{"0", "blocked"}};
  ASSERT_EQ(run(corridor(), script), BT::NodeStatus::SUCCESS);

  EXPECT_TRUE(script.started.count("3"));
  EXPECT_FALSE(script.started.count("2"));
  // finish is on the clear branch: skipped, even though its wait (the drive)
  // ended normally.
  EXPECT_FALSE(script.started.count("4"));
}

TEST(TemporalPolicy, DispatchFollowsEndsNotTheClock)
{
  // Same plan, but the drive finishes before the look this time. Nothing that
  // waits for the look starts before it ends, however the durations come out.
  Script script;
  script.ticks = {{"0", 7}, {"1", 2}};
  script.outcome = {{"0", "clear"}};
  ASSERT_EQ(run(corridor(), script), BT::NodeStatus::SUCCESS);

  EXPECT_GE(script.started.at("2"), script.ended.at("0"));
  // finish waits for the drive and for the look's verdict on its branch.
  ASSERT_TRUE(script.started.count("4"));
  EXPECT_GE(script.started.at("4"), script.ended.at("0"));
}

// Leave through _exit, for the reason bt_plugin_test.cpp gives: the real
// nodes create ROS nodes, and DDS threads must not outlive the process.
extern "C" void __gcov_dump(void) __attribute__((weak));

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();

  if (__gcov_dump) {
    __gcov_dump();
  }
  _exit(result);
}
