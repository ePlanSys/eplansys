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

// The builder as the executor uses it: a Plan in, behavior tree XML out, and a
// dotgraph of what is running. policy_bt_test already covers how a policy is
// rendered; what is left here is the part that belongs to the plugin — the
// contract with plansys2_executor. That it reports a malformed plan by
// returning no tree, that the id it writes into the tree is the one the
// executor keys its action map by, that the dotgraph is drawn from the policy
// rather than from a temporal graph it does not build, and that pluginlib can
// find the class the executor asks for by name.

#include <gtest/gtest.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/control_node.h"

#include "pluginlib/class_loader.hpp"

#include "plansys2_epistemic_bt_builder/epistemic_bt_builder.hpp"
#include "plansys2_executor/BTBuilder.hpp"

using plansys2::ActionExecutionInfo;
using plansys2::EpistemicBTBuilder;
using Plan = plansys2_msgs::msg::Plan;
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

/// The corridor mission, as the planner hands it over: drive out, look, and
/// report what was seen. The branch is the whole point — one continuation per
/// outcome of the sensing action.
Plan corridor_policy()
{
  Plan plan;
  plan.items.push_back(action("(goto_junction r1)", 0.0f, {1}, {"e-goto-junction"}));
  plan.items.push_back(action("(inspect_corridor r1)", 30.0f, {2, 3}, {"e-clear", "e-blocked"}));
  plan.items.push_back(action("(report_clear r1)", 35.0f));
  plan.items.push_back(action("(report_blocked r1)", 35.0f));
  plan.items[1].knowledge_requirements.push_back("(at-junction r1)");
  plan.epistemic_goal = "(and (Kw r1 blocked) (Kw r2 blocked))";
  return plan;
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

/// Stands in for EpistemicSwitch, which lives in plansys2_epistemic_executor's
/// node library. The parser only needs a control node of the same name and
/// ports; what is under test here is the XML the builder wrote.
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

/// A tree the executor cannot parse is worthless however good it looks, so
/// every rendering here goes through the real BehaviorTree.CPP parser with the
/// node set the executor registers.
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

TEST(EpistemicBTBuilderTest, RendersABranchingPolicyAsATreeWithASwitch)
{
  EpistemicBTBuilder builder;
  builder.initialize();

  const auto xml = builder.get_tree(corridor_policy());

  ASSERT_FALSE(xml.empty());
  expect_parses(xml);

  // One switch, at the node that senses, and its outcomes in the order the
  // policy listed them so the observation selects a branch by position.
  EXPECT_EQ(count_of(xml, "<EpistemicSwitch"), 1u);
  EXPECT_NE(xml.find("outcomes=\"e-clear;e-blocked\""), std::string::npos) << xml;

  // Both continuations are in the tree: a policy that kept only the branch it
  // expected would be a plan again.
  EXPECT_NE(xml.find("(report_clear r1)"), std::string::npos) << xml;
  EXPECT_NE(xml.find("(report_blocked r1)"), std::string::npos) << xml;

  // Every node is guarded and updated, and the goal is checked once at the end
  // rather than at each leaf.
  EXPECT_EQ(count_of(xml, "<CheckKnowledge"), 4u);
  EXPECT_EQ(count_of(xml, "<ApplyEpistemicUpdate"), 4u);
  EXPECT_EQ(count_of(xml, "<CheckEpistemicGoal"), 1u);
}

TEST(EpistemicBTBuilderTest, RendersAPlanWithNoBranchesAsAFlatSequence)
{
  Plan plan;
  plan.items.push_back(action("(goto_junction r1)", 0.0f));
  plan.items.push_back(action("(report_clear r1)", 30.0f));

  EpistemicBTBuilder builder;
  builder.initialize();

  const auto xml = builder.get_tree(plan);

  ASSERT_FALSE(xml.empty());
  expect_parses(xml);

  // A classical plan is a policy with nothing to choose, and renders as the
  // sequence PlanSys2 would have built — with the guard and the update, which
  // is what makes this builder a strict extension rather than a replacement.
  EXPECT_EQ(count_of(xml, "<EpistemicSwitch"), 0u);
  EXPECT_EQ(count_of(xml, "<ExecuteAction"), 2u);
  EXPECT_EQ(count_of(xml, "<ApplyEpistemicUpdate"), 2u);
  EXPECT_EQ(count_of(xml, "<CheckEpistemicGoal"), 0u) << "no epistemic goal to check";
}

TEST(EpistemicBTBuilderTest, ReportsAMalformedPolicyByBuildingNoTree)
{
  Plan plan;
  // A continuation at an index that does not exist. The executor's own message
  // for an empty tree says only that one could not be computed, so the builder
  // logs why; what is checked here is that it refuses rather than renders a
  // tree that names a node it cannot fill.
  plan.items.push_back(action("(goto_junction r1)", 0.0f, {7}));

  EpistemicBTBuilder builder;
  builder.initialize();

  EXPECT_TRUE(builder.get_tree(plan).empty());
}

TEST(EpistemicBTBuilderTest, RendersAnEmptyPlanAsATreeThatDoesNothing)
{
  EpistemicBTBuilder builder;
  builder.initialize();

  // Nothing to do is a valid answer from a planner whose goal already holds,
  // and the executor still has to be handed a tree it can run.
  const auto xml = builder.get_tree(Plan());

  ASSERT_FALSE(xml.empty());
  expect_parses(xml);
  EXPECT_EQ(count_of(xml, "<ExecuteAction"), 0u);
}

TEST(EpistemicBTBuilderTest, UsesTheActionTemplateAndPrecisionItWasInitializedWith)
{
  Plan plan;
  plan.items.push_back(action("(inspect_corridor r1)", 1.5f));

  EpistemicBTBuilder builder;
  // One template, not two: there are no start and end halves of a policy node.
  // The second argument is ignored, and a builder that quietly used it would
  // render a tree from the wrong XML.
  builder.initialize(
    "<Sequence name=\"NODE_NAME\">\n"
    "  <ExecuteAction action=\"ACTION_ID\"/>\n"
    "CONTINUATIONS\n"
    "</Sequence>",
    "<Sequence><ExecuteAction action=\"unused\"/></Sequence>", 2);

  const auto xml = builder.get_tree(plan);

  ASSERT_FALSE(xml.empty());
  expect_parses(xml);
  EXPECT_EQ(count_of(xml, "unused"), 0u) << xml;

  // The id has to be the one the executor keys its action map by, at the
  // precision the builder was given: a mismatch leaves ExecuteAction driving
  // an action nobody registered.
  PlanItem item = plan.items.front();
  EXPECT_NE(
    xml.find("action=\"" + plansys2::policy_action_id(item, 2) + "\""),
    std::string::npos) << xml;
  EXPECT_EQ(plansys2::policy_action_id(item, 2), plansys2::BTBuilder::to_action_id(item, 2));
}

TEST(EpistemicBTBuilderTest, RefusesATemplateThatCannotHoldAPolicy)
{
  EpistemicBTBuilder builder;
  // What the executor hands every builder when nothing else is configured:
  // PlanSys2's own action template, which has no place for a continuation to
  // go. Rendering into it would produce a tree holding the root action alone,
  // which runs, succeeds, and leaves the mission undone.
  builder.initialize(
    "<Sequence name=\"ACTION_ID\">\n"
    "  <WaitAtStartReq action=\"ACTION_ID\"/>\n"
    "  <ExecuteAction action=\"ACTION_ID\"/>\n"
    "</Sequence>");

  const auto xml = builder.get_tree(corridor_policy());

  ASSERT_FALSE(xml.empty());
  expect_parses(xml);
  // The packaged template was used instead, so the policy is all there.
  EXPECT_EQ(count_of(xml, "<EpistemicSwitch"), 1u) << xml;
  EXPECT_EQ(count_of(xml, "<ApplyEpistemicUpdate"), 4u) << xml;
}

TEST(EpistemicBTBuilderTest, ThePolicyIsReportedAsAGraph)
{
  EpistemicBTBuilder builder;
  builder.initialize();

  const auto plan = corridor_policy();
  ASSERT_FALSE(builder.get_tree(plan).empty());

  // The graph does not decide the order — the planner already did — but it
  // records it, which is what CheckAction reads bounds from and what a monitor
  // draws. Returning none left both with nothing.
  const auto graph = builder.get_graph();
  ASSERT_NE(graph, nullptr);
  EXPECT_EQ(graph->nodes.size(), plan.items.size())
    << "every policy node belongs in the graph, branches included";

  EXPECT_FALSE(builder.propagate(nullptr)) << "there is nothing to propagate through";
  EXPECT_TRUE(builder.propagate(graph));
}

TEST(EpistemicBTBuilderTest, TheBranchIsInTheGraph)
{
  EpistemicBTBuilder builder;
  builder.initialize();
  ASSERT_FALSE(builder.get_tree(corridor_policy()).empty());

  const auto graph = builder.get_graph();
  ASSERT_NE(graph, nullptr);
  ASSERT_FALSE(graph->nodes.empty());

  // The corridor mission drives out first and only branches once it looks, so
  // the branching node is the sensing one and not the root.
  EXPECT_TRUE(graph->nodes.front()->input_arcs.empty()) << "the root follows nothing";

  std::size_t branching = 0;
  for (const auto & node : graph->nodes) {
    if (node->output_arcs.size() > 1u) {
      ++branching;
    }
  }
  EXPECT_EQ(branching, 1u)
    << "the policy branches exactly once, and the graph should say where";
}

TEST(EpistemicBTBuilderTest, AContinuationCannotStartBeforeItsParentIsDone)
{
  EpistemicBTBuilder builder;
  builder.initialize();

  auto plan = corridor_policy();
  ASSERT_FALSE(plan.items.empty());
  plan.items[0].duration = 7.0f;

  ASSERT_FALSE(builder.get_tree(plan).empty());
  const auto graph = builder.get_graph();
  ASSERT_NE(graph, nullptr);

  // The knowledge guard on a continuation is checked against the state its
  // parent's update produced, so the two cannot overlap at all. That is a
  // requirement of the epistemic layer and not a scheduling choice, and the
  // lower bound on the arc is where it is written down.
  std::size_t checked = 0;
  for (const auto & node : graph->nodes) {
    for (const auto & arc : node->input_arcs) {
      if (std::get<0>(arc) == graph->nodes.front()) {
        EXPECT_DOUBLE_EQ(std::get<1>(arc), 7.0)
          << "a continuation may start before its parent finished";
        ++checked;
      }
    }
  }
  EXPECT_GT(checked, 0u) << "no arc leaves the root";
}

TEST(EpistemicBTBuilderTest, DrawsTheBranchesAndTheGoalInTheDotgraph)
{
  EpistemicBTBuilder builder;
  builder.initialize();
  ASSERT_FALSE(builder.get_tree(corridor_policy()).empty());

  const auto dot = builder.get_dotgraph(nullptr);

  EXPECT_NE(dot.find("digraph plan {"), std::string::npos);
  // Branches show as branches, each edge labelled with the observation that
  // selects it — that is what this graph is for, and a temporal graph could
  // not have said it.
  EXPECT_NE(dot.find("n1 -> n2 [label=\"e-clear\"]"), std::string::npos) << dot;
  EXPECT_NE(dot.find("n1 -> n3 [label=\"e-blocked\"]"), std::string::npos) << dot;
  // The knowledge guard is drawn with the node it guards, because that is
  // where someone reading a failed run looks for why it failed.
  EXPECT_NE(dot.find("(at-junction r1)"), std::string::npos) << dot;
  EXPECT_NE(dot.find("shape=doubleoctagon"), std::string::npos) << dot;
}

TEST(EpistemicBTBuilderTest, ColoursTheDotgraphByHowFarExecutionGot)
{
  EpistemicBTBuilder builder;
  builder.initialize();
  const auto plan = corridor_policy();
  ASSERT_FALSE(builder.get_tree(plan).empty());

  auto action_map = std::make_shared<std::map<std::string, ActionExecutionInfo>>();
  (*action_map)[plansys2::policy_action_id(plan.items[0], 3)].at_start_effects_applied = true;
  (*action_map)[plansys2::policy_action_id(plan.items[0], 3)].at_end_effects_applied = true;
  (*action_map)[plansys2::policy_action_id(plan.items[1], 3)].at_start_effects_applied = true;

  const auto dot = builder.get_dotgraph(action_map);

  EXPECT_NE(dot.find("n0 [label=\"0: (goto_junction r1)"), std::string::npos) << dot;
  EXPECT_NE(dot.find("fillcolor=palegreen"), std::string::npos) << dot;
  EXPECT_NE(dot.find("fillcolor=khaki"), std::string::npos) << dot;
  // The branch that was not taken is left uncoloured rather than absent.
  EXPECT_NE(dot.find("n3 [label=\"3: (report_blocked r1)"), std::string::npos) << dot;
}

TEST(EpistemicBTBuilderTest, DrawsSequentialPlansAsAChain)
{
  Plan plan;
  plan.items.push_back(action("(goto_junction r1)", 0.0f));
  plan.items.push_back(action("(report_clear r1)", 30.0f));

  EpistemicBTBuilder builder;
  builder.initialize();
  ASSERT_FALSE(builder.get_tree(plan).empty());

  const auto dot = builder.get_dotgraph(nullptr);

  // A plan that names no continuations is read the way PlanSys2 writes one:
  // item i is followed by item i+1, and the graph has to say the same.
  EXPECT_NE(dot.find("n0 -> n1;"), std::string::npos) << dot;
  EXPECT_EQ(count_of(dot, "->"), 1u) << dot;
}

TEST(EpistemicBTBuilderTest, DrawsAnEndedBranchAsDone)
{
  Plan plan;
  plan.items.push_back(
    action(
      "(inspect_corridor r1)", 0.0f,
      {1, PlanItem::POLICY_DONE}, {"e-blocked", "e-clear"}));
  plan.items.push_back(action("(report_blocked r1)", 5.0f));

  EpistemicBTBuilder builder;
  builder.initialize();
  const auto xml = builder.get_tree(plan);

  ASSERT_FALSE(xml.empty());
  expect_parses(xml);
  // The outcome that finishes the policy still needs a branch, so that the
  // switch can index its children by the outcome list.
  EXPECT_EQ(count_of(xml, "<AlwaysSuccess/>"), 1u) << xml;

  const auto dot = builder.get_dotgraph(nullptr);
  EXPECT_NE(dot.find("[label=\"done\", shape=ellipse]"), std::string::npos) << dot;
  EXPECT_NE(dot.find("[label=\"e-clear\"]"), std::string::npos) << dot;
}

TEST(EpistemicBTBuilderTest, IsTheClassTheExecutorAsksPluginlibFor)
{
  // The executor names a builder by string and asks pluginlib for it; a class
  // that builds but is not exported under that name is one the executor cannot
  // reach. This is the whole of the plugin contract.
  pluginlib::ClassLoader<plansys2::BTBuilder> loader(
    "plansys2_executor", "plansys2::BTBuilder");

  std::shared_ptr<plansys2::BTBuilder> builder;
  ASSERT_NO_THROW(
    builder = loader.createSharedInstance("plansys2::EpistemicBTBuilder"));
  ASSERT_NE(builder, nullptr);

  builder->initialize();
  EXPECT_FALSE(builder->get_tree(corridor_policy()).empty());
}
