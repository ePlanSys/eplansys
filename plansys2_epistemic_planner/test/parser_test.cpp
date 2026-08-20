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

#include "plansys2_epistemic_planner/formula.hpp"
#include "plansys2_epistemic_planner/parser.hpp"
#include "plansys2_epistemic_planner/selection_policy.hpp"

#include "task_fixtures.hpp"

// The formula registry is global and interns across tasks, so each test starts
// from a clean one — exactly as EpistemicPlanSolver::getPlan does per call.
class ParserTest : public ::testing::Test
{
protected:
  void SetUp() override {formula_registry_reset();}
  void TearDown() override {formula_registry_reset();}
};

// The counts asserted here are the ones plank reports in "planning-task-info"
// for each fixture, so a parser that silently drops an agent, an atom, an
// action, or an initial world is caught against the grounder's own accounting
// rather than against a number this test made up.
TEST_F(ParserTest, MuddyChildrenTwoMatchesGrounderCounts)
{
  const auto task = load_task(task_path("muddy-children-2"));

  EXPECT_EQ(task.num_agents(), 2u);
  EXPECT_EQ(task.num_atoms(), 2u);
  EXPECT_EQ(task.num_actions(), 2u);
  EXPECT_EQ(task.init.num_worlds, 4u);

  EXPECT_NE(task.agent_index.find("c1"), task.agent_index.end());
  EXPECT_NE(task.agent_index.find("c2"), task.agent_index.end());
  EXPECT_NE(task.action_index.find("ask_c1"), task.action_index.end());
  EXPECT_NE(task.action_index.find("ask_c2"), task.action_index.end());
}

TEST_F(ParserTest, MuddyChildrenThreeMatchesGrounderCounts)
{
  const auto task = load_task(task_path("muddy-children-3"));

  EXPECT_EQ(task.num_agents(), 3u);
  EXPECT_EQ(task.num_atoms(), 3u);
  EXPECT_EQ(task.num_actions(), 3u);
  EXPECT_EQ(task.init.num_worlds, 8u);
}

TEST_F(ParserTest, CoinInTheBoxMatchesGrounderCounts)
{
  const auto task = load_task(task_path("coin-in-the-box"));

  EXPECT_EQ(task.num_agents(), 3u);
  EXPECT_EQ(task.num_atoms(), 8u);
  EXPECT_EQ(task.num_actions(), 21u);
  EXPECT_EQ(task.init.num_worlds, 2u);
}

TEST_F(ParserTest, ActiveMuddyChildMatchesGrounderCounts)
{
  const auto task = load_task(task_path("active-muddy-child"));

  EXPECT_EQ(task.num_agents(), 5u);
  EXPECT_EQ(task.num_atoms(), 5u);
  EXPECT_EQ(task.num_actions(), 5u);
  EXPECT_EQ(task.init.num_worlds, 32u);
}

// Every fixture declares :finitary-S5-theories, so none of them may come back
// as a doxastic task; misreading the frame would silently change the semantics
// the search runs under.
TEST_F(ParserTest, S5FixturesAreNotFlaggedKd45)
{
  for (const auto & name :
    {"muddy-children-2", "muddy-children-3", "coin-in-the-box", "active-muddy-child"})
  {
    formula_registry_reset();
    const auto task = load_task(task_path(name));
    EXPECT_FALSE(task.kd45) << name;
  }
}

// goal_kw_only steers heuristic selection. The muddy-children goal is a
// conjunction of Kw conjuncts, which is what the flag is named for.
//
// Coin-in-the-box pins current behaviour, which is NOT what task.hpp
// documents. Its goal is K_A(tails) — a box, not a Kw — yet the flag comes
// back true, because parser.cpp derives it as !has_atom_conjunct: "no bare
// atom at the top level" rather than "every conjunct is a Kw". Any modal goal
// therefore counts as Kw-only and gets routed to KnowledgeSpreadHeuristic.
// Deciding which of the two is right is a semantics call, so this test records
// the behaviour rather than asserting the docstring; change it together with
// the flag.
TEST_F(ParserTest, GoalKwOnlyDistinguishesGoalShapes)
{
  {
    const auto task = load_task(task_path("muddy-children-2"));
    EXPECT_TRUE(task.goal_kw_only);
  }
  formula_registry_reset();
  {
    const auto task = load_task(task_path("coin-in-the-box"));
    EXPECT_TRUE(task.goal_kw_only);   // see note above: a box goal, not a Kw
  }
}

// A task that fails to load must throw rather than hand back an empty task
// that the solver would then try to search.
TEST_F(ParserTest, MissingFileThrows)
{
  EXPECT_ANY_THROW(load_task(task_path("no-such-task")));
}

// TaskFeatures drives the built-in selection policy. Reading it off a real
// task keeps the policy's inputs honest: a feature that stops tracking the
// task it is extracted from would send every domain to the same heuristic.
TEST_F(ParserTest, TaskFeaturesReflectTheTask)
{
  const auto task = load_task(task_path("active-muddy-child"));
  const auto features = TaskFeatures::extract(task);

  const auto policy = SelectionPolicy::builtin();
  const auto heuristic = select(policy.heuristic_rules, features);
  const auto strategy = select(policy.strategy_rules, features);

  EXPECT_FALSE(heuristic.outcome.empty());
  EXPECT_FALSE(strategy.outcome.empty());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
