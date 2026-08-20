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
#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "plansys2_epistemic_planner/action_mapping.hpp"
#include "plansys2_epistemic_planner/formula.hpp"
#include "plansys2_epistemic_planner/heuristic.hpp"
#include "plansys2_epistemic_planner/parser.hpp"
#include "plansys2_epistemic_planner/policy_plan.hpp"
#include "plansys2_epistemic_planner/search.hpp"

#include "task_fixtures.hpp"

using plansys2::ActionMapping;
using plansys2::policy_branches;
using plansys2::to_policy_plan;
using PlanItem = plansys2_msgs::msg::PlanItem;

namespace
{

Deadline soon()
{
  return std::chrono::steady_clock::now() + std::chrono::seconds(60);
}

/// Walk the policy the way an executor would: from the root, following one
/// outcome at each node, and check that it stays inside the message.
void expect_well_formed(const plansys2_msgs::msg::Plan & plan)
{
  ASSERT_FALSE(plan.items.empty());

  std::set<std::uint32_t> reachable;
  const auto visit = [&](std::uint32_t index, const auto & self) -> void {
      ASSERT_LT(index, plan.items.size());
      if (!reachable.insert(index).second) {
        return;
      }
      const auto & item = plan.items[index];
      ASSERT_EQ(item.children.size(), item.outcomes.size())
        << "every continuation must say which observation selects it";

      for (const auto child : item.children) {
        if (child == PlanItem::POLICY_DONE) {
          continue;
        }
        ASSERT_GT(child, index) << "a policy is a tree, so children come later";
        self(child, self);
      }
    };
  visit(0, visit);

  EXPECT_EQ(reachable.size(), plan.items.size())
    << "every item must be reachable from the root; an unreachable item is one "
    "the executor could never run";
}

}  // namespace

class PolicyPlanTest : public ::testing::Test
{
protected:
  void SetUp() override {formula_registry_reset();}
  void TearDown() override {formula_registry_reset();}
};

// A linear policy must come out as the same sequence the flat path produces:
// one item per action, each naming the next as its only continuation.
TEST_F(PolicyPlanTest, LinearPolicyIsAChain)
{
  const auto task = load_task(task_path("muddy-children-2"));
  UnsatisfiedGoalHeuristic h;

  const auto result = aostar::search(task, h, 0, soon());
  ASSERT_TRUE(result.has_value());
  ASSERT_NE(result->plan_tree, nullptr);
  ASSERT_FALSE(policy_branches(result->plan_tree));

  std::string error;
  const auto plan = to_policy_plan(task, result->plan_tree, ActionMapping::conventional(), error);
  ASSERT_TRUE(plan.has_value()) << error;
  expect_well_formed(*plan);

  for (std::size_t i = 0; i + 1 < plan->items.size(); ++i) {
    ASSERT_EQ(plan->items[i].children.size(), 1u);
    EXPECT_EQ(plan->items[i].children[0], i + 1);
  }

  // The last node still records its outcome, marked terminal. Dropping it
  // would leave the executor unable to tell an outcome that completes the
  // policy from one the policy never planned for.
  for (const auto child : plan->items.back().children) {
    EXPECT_EQ(child, PlanItem::POLICY_DONE);
  }
}

// The branching case is the one a flat Plan cannot hold. Both continuations
// must survive, each tagged with the observation that selects it.
TEST_F(PolicyPlanTest, BranchingPolicyKeepsEveryContingency)
{
  const auto task = load_task(task_path("coin-in-the-box-multipointed"));
  EpistemicDistanceHeuristic h;

  const auto result = aostar::search(task, h, 0, soon());
  ASSERT_TRUE(result.has_value());
  ASSERT_NE(result->plan_tree, nullptr);
  ASSERT_TRUE(policy_branches(result->plan_tree));

  std::string error;
  const auto plan = to_policy_plan(task, result->plan_tree, ActionMapping::conventional(), error);
  ASSERT_TRUE(plan.has_value()) << error;
  expect_well_formed(*plan);

  const auto branching = std::find_if(
    plan->items.begin(), plan->items.end(),
    [](const PlanItem & item) {return item.children.size() > 1;});
  ASSERT_NE(branching, plan->items.end()) << "the branch must survive serialisation";

  EXPECT_TRUE(branching->sensing) << "only a sensing action can branch";
  for (const auto & outcome : branching->outcomes) {
    EXPECT_FALSE(outcome.empty()) << "an unnamed outcome cannot be matched at runtime";
  }
  EXPECT_EQ(
    std::set<std::string>(branching->outcomes.begin(), branching->outcomes.end()).size(),
    branching->outcomes.size()) << "two continuations cannot share one observation";
}

// Times must not go backwards along any single execution. They may repeat
// across branches, since only one branch ever runs.
TEST_F(PolicyPlanTest, TimesIncreaseAlongEveryExecution)
{
  const auto task = load_task(task_path("coin-in-the-box-multipointed"));
  EpistemicDistanceHeuristic h;
  const auto result = aostar::search(task, h, 0, soon());
  ASSERT_TRUE(result.has_value());

  std::string error;
  const auto plan = to_policy_plan(task, result->plan_tree, ActionMapping::conventional(), error);
  ASSERT_TRUE(plan.has_value()) << error;

  const auto walk = [&](std::uint32_t index, const auto & self) -> void {
      const auto & item = plan->items[index];
      for (const auto child : item.children) {
        if (child == PlanItem::POLICY_DONE) {
          continue;
        }
        EXPECT_GE(plan->items[child].time, item.time + item.duration)
          << "a continuation cannot start before its predecessor ends";
        self(child, self);
      }
    };
  walk(0, walk);
}

// The goal travels with the policy: reaching a leaf is not the same as having
// achieved what the policy was built for, and the executor has no other way to
// learn an epistemic goal.
TEST_F(PolicyPlanTest, EpistemicGoalIsCarried)
{
  const auto task = load_task(task_path("muddy-children-2"));
  UnsatisfiedGoalHeuristic h;
  const auto result = aostar::search(task, h, 0, soon());
  ASSERT_TRUE(result.has_value());

  std::string error;
  const auto plan = to_policy_plan(task, result->plan_tree, ActionMapping::conventional(), error);
  ASSERT_TRUE(plan.has_value()) << error;

  EXPECT_FALSE(plan->epistemic_goal.empty());
  EXPECT_NE(plan->epistemic_goal.find("Kw"), std::string::npos)
    << "muddy-children asks each child to know whether it is muddy: " <<
    plan->epistemic_goal;
}

// Knowledge preconditions cannot be read off the PDDL domain, so they have to
// travel on the item. Only the modal part: the ordinary facts are the PDDL
// action's own precondition and are checked against the problem expert.
TEST_F(PolicyPlanTest, KnowledgeRequirementsAreModalOnly)
{
  const auto task = load_task(task_path("coin-in-the-box-multipointed"));
  EpistemicDistanceHeuristic h;
  const auto result = aostar::search(task, h, 0, soon());
  ASSERT_TRUE(result.has_value());

  std::string error;
  const auto plan = to_policy_plan(task, result->plan_tree, ActionMapping::conventional(), error);
  ASSERT_TRUE(plan.has_value()) << error;

  for (const auto & item : plan->items) {
    for (const auto & requirement : item.knowledge_requirements) {
      EXPECT_TRUE(
        requirement.rfind("(K ", 0) == 0 || requirement.rfind("(B ", 0) == 0 ||
        requirement.rfind("(Kw ", 0) == 0 || requirement.rfind("(C ", 0) == 0)
        << "a non-modal requirement belongs in the PDDL precondition: " << requirement;
    }
  }
}

// The grounded name has to survive alongside the translated one: it is what
// identifies the event model to apply once the action has run.
TEST_F(PolicyPlanTest, BothVocabulariesAreCarried)
{
  const auto task = load_task(task_path("muddy-children-2"));
  UnsatisfiedGoalHeuristic h;
  const auto result = aostar::search(task, h, 0, soon());
  ASSERT_TRUE(result.has_value());

  std::string error;
  const auto plan = to_policy_plan(task, result->plan_tree, ActionMapping::conventional(), error);
  ASSERT_TRUE(plan.has_value()) << error;

  for (const auto & item : plan->items) {
    EXPECT_NE(task.action_index.find(item.epistemic_action), task.action_index.end())
      << "the grounded name must still resolve in the task: " << item.epistemic_action;
    EXPECT_EQ(item.action.front(), '(') << "the executor's name is a PDDL expression";
  }
}

// A goal that already holds is a valid, empty policy — not a failure, and not
// a policy with a phantom root.
TEST_F(PolicyPlanTest, EmptyPolicyIsValid)
{
  const auto task = load_task(task_path("muddy-children-2"));

  std::string error;
  const auto plan = to_policy_plan(task, nullptr, ActionMapping::conventional(), error);
  ASSERT_TRUE(plan.has_value()) << error;
  EXPECT_TRUE(plan->items.empty());
  EXPECT_FALSE(plan->epistemic_goal.empty());
}

TEST_F(PolicyPlanTest, UnmappedActionFailsWithItsName)
{
  const auto task = load_task(task_path("muddy-children-2"));
  UnsatisfiedGoalHeuristic h;
  const auto result = aostar::search(task, h, 0, soon());
  ASSERT_TRUE(result.has_value());

  // A map that covers nothing: the first action must be named in the error.
  const auto empty_path =
    (std::filesystem::temp_directory_path() / "eplansys-empty-mapping.json").string();
  {
    std::ofstream out(empty_path);
    out << "{}";
  }
  const auto empty = ActionMapping::load(empty_path);

  std::string error;
  const auto plan = to_policy_plan(task, result->plan_tree, empty, error);
  EXPECT_FALSE(plan.has_value());
  EXPECT_NE(error.find(result->plan_tree->action), std::string::npos) << error;

  std::error_code ec;
  std::filesystem::remove(empty_path, ec);
}

// The executor keys its action map by action expression and start time in
// milliseconds. Two policy nodes colliding there would drive one action
// executor from two branches, and only on the day the second branch is taken.
TEST_F(PolicyPlanTest, EveryNodeGetsADistinctExecutorKey)
{
  const auto task = load_task(task_path("coin-in-the-box-multipointed"));
  EpistemicDistanceHeuristic h;
  const auto result = aostar::search(task, h, 0, soon());
  ASSERT_TRUE(result.has_value());

  std::string error;
  const auto plan = to_policy_plan(task, result->plan_tree, ActionMapping::conventional(), error);
  ASSERT_TRUE(plan.has_value()) << error;

  // The key the executor builds, reproduced exactly: see BTBuilder::to_action_id.
  std::set<std::string> keys;
  for (const auto & item : plan->items) {
    const auto key = item.action + ":" +
      std::to_string(static_cast<int>(item.time * 1000));
    EXPECT_TRUE(keys.insert(key).second) << "two policy nodes share the key " << key;
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
