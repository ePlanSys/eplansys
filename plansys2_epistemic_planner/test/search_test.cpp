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
#include <functional>
#include <set>
#include <memory>
#include <string>
#include <vector>

#include "plansys2_epistemic_planner/formula.hpp"
#include "plansys2_epistemic_planner/heuristic.hpp"
#include "plansys2_epistemic_planner/parser.hpp"
#include "plansys2_epistemic_planner/search.hpp"
#include "plansys2_epistemic_planner/validator.hpp"

#include "task_fixtures.hpp"

namespace
{

// A ceiling, not an expectation: these fixtures solve in well under a second,
// and the bound only stops a regression from hanging the whole test run.
Deadline soon()
{
  return std::chrono::steady_clock::now() + std::chrono::seconds(60);
}

// Every action a search reports must name an action the task actually has.
// A plan carrying a name the task cannot resolve is unexecutable no matter how
// good the search that produced it.
void expect_actions_resolve(const PlanningTask & task, const std::vector<std::string> & plan)
{
  for (const auto & action : plan) {
    EXPECT_NE(task.action_index.find(action), task.action_index.end())
      << "plan names an action absent from the task: " << action;
  }
}

std::size_t tree_depth(const std::shared_ptr<PlanNode> & node)
{
  if (!node) {
    return 0;
  }
  std::size_t deepest = 0;
  for (const auto & [event, child] : node->branches) {
    (void)event;
    deepest = std::max(deepest, tree_depth(child));
  }
  return deepest + 1;
}

bool has_branch(const std::shared_ptr<PlanNode> & node)
{
  if (!node) {
    return false;
  }
  if (node->branches.size() > 1) {
    return true;
  }
  for (const auto & [event, child] : node->branches) {
    (void)event;
    if (has_branch(child)) {
      return true;
    }
  }
  return false;
}

}  // namespace

class SearchTest : public ::testing::Test
{
protected:
  void SetUp() override {formula_registry_reset();}
  void TearDown() override {formula_registry_reset();}
};

TEST_F(SearchTest, GbfsSolvesMuddyChildren)
{
  const auto task = load_task(task_path("muddy-children-2"));
  UnsatisfiedGoalHeuristic h;

  const auto result = gbfs::search(task, h, 0, soon());

  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result->plan.empty());
  expect_actions_resolve(task, result->plan);
  EXPECT_GT(result->stats.nodes_expanded, 0u);
}

TEST_F(SearchTest, EhcSolvesMuddyChildren)
{
  const auto task = load_task(task_path("muddy-children-2"));
  KnowledgeSpreadHeuristic h;

  const auto result = ehc::search(task, h, 0, soon());

  ASSERT_TRUE(result.has_value());
  expect_actions_resolve(task, result->plan);
}

// Search must be a function of the task, not of allocation order or hash
// iteration: the same task and heuristic twice must give the same plan.
// Without this, a plan reproduced from a log could differ from the one the
// robot actually ran.
TEST_F(SearchTest, GbfsIsDeterministic)
{
  const auto first = [] {
      formula_registry_reset();
      const auto task = load_task(task_path("muddy-children-3"));
      UnsatisfiedGoalHeuristic h;
      const auto r = gbfs::search(task, h, 0, soon());
      return r ? r->plan : std::vector<std::string>{};
    }();

  const auto second = [] {
      formula_registry_reset();
      const auto task = load_task(task_path("muddy-children-3"));
      UnsatisfiedGoalHeuristic h;
      const auto r = gbfs::search(task, h, 0, soon());
      return r ? r->plan : std::vector<std::string>{};
    }();

  ASSERT_FALSE(first.empty());
  EXPECT_EQ(first, second);
}

// The validator replays the tree against the task — every action applicable
// where it is applied, every branch a real sensing outcome, every leaf at the
// goal — so this is the end-to-end semantic check on AO*.
TEST_F(SearchTest, AostarPlanForMuddyChildrenValidates)
{
  const auto task = load_task(task_path("muddy-children-2"));
  UnsatisfiedGoalHeuristic h;

  const auto result = aostar::search(task, h, 0, soon());

  ASSERT_TRUE(result.has_value());
  ASSERT_NE(result->plan_tree, nullptr) << "the goal must not already hold";

  const auto validation = validate(task, result->plan_tree);
  EXPECT_TRUE(validation.valid) << validation.error;
  EXPECT_GT(validation.leaves_reached, 0u);
}

// Sensing does not imply branching. Coin-in-the-box as plank grounds it is
// single-pointed — one designated world, so peeking has only one possible
// outcome and the policy is a chain. This pins that: a contingent branch here
// would mean the search invented an outcome the model cannot distinguish.
TEST_F(SearchTest, AostarOnSinglePointedTaskYieldsALinearPolicy)
{
  const auto task = load_task(task_path("coin-in-the-box"));
  EpistemicDistanceHeuristic h;

  const auto result = aostar::search(task, h, 0, soon());

  ASSERT_TRUE(result.has_value());
  ASSERT_NE(result->plan_tree, nullptr);

  const auto validation = validate(task, result->plan_tree);
  EXPECT_TRUE(validation.valid) << validation.error;
  EXPECT_FALSE(has_branch(result->plan_tree));
  EXPECT_GT(tree_depth(result->plan_tree), 1u);
}

// The multi-pointed variant is where contingency actually appears: both coin
// worlds are designated, so A must look and then act on what it saw, and the
// policy genuinely branches. This is the case EpistemicPlanSolver flattens
// away when it emits a plansys2_msgs/Plan, so it needs a test of its own —
// nothing else in this suite exercises the branching path at all.
TEST_F(SearchTest, AostarOnMultiPointedCoinBranchesAndValidates)
{
  const auto task = load_task(task_path("coin-in-the-box-multipointed"));
  EpistemicDistanceHeuristic h;

  const auto result = aostar::search(task, h, 0, soon());

  ASSERT_TRUE(result.has_value());
  ASSERT_NE(result->plan_tree, nullptr) << "the goal must not already hold";

  const auto validation = validate(task, result->plan_tree);
  EXPECT_TRUE(validation.valid) << validation.error;
  EXPECT_GT(validation.leaves_reached, 1u) << "a contingent policy has several leaves";

  EXPECT_TRUE(has_branch(result->plan_tree))
    << "with two designated worlds the sensing outcome is not determined, so "
    "the policy must branch on it";
}

// The fleet scenario, and the reason it was written: a grounded task that is
// multi-pointed as the grounder produces it, rather than edited by hand into
// being so. The corridor's state is genuinely open, the scout has to look, and
// what it says afterwards depends on what it saw.
TEST_F(SearchTest, AostarOnTheRobotFleetBranchesOnWhatTheScoutSees)
{
  const auto task = load_task(task_path("robot-fleet"));
  EpistemicDistanceHeuristic h;

  ASSERT_EQ(task.init.num_designated(), 2u)
    << "the corridor's state must be undetermined for the mission to be one";

  const auto result = aostar::search(task, h, 0, soon());

  ASSERT_TRUE(result.has_value());
  ASSERT_NE(result->plan_tree, nullptr) << "the goal must not already hold";

  const auto validation = validate(task, result->plan_tree);
  EXPECT_TRUE(validation.valid) << validation.error;
  EXPECT_GT(validation.leaves_reached, 1u) << "a contingent policy has several leaves";

  EXPECT_TRUE(has_branch(result->plan_tree))
    << "the scout cannot know what it will see, so the policy must branch on it";

  // Drive, look, and say what you saw. Pinning the shape rather than only the
  // depth is what makes this a scenario rather than a benchmark: the point is
  // that one robot goes and the other is told, not that a tree of some size
  // came back.
  const auto & drive = result->plan_tree;
  EXPECT_EQ(drive->action.rfind("goto-junction_", 0), 0u) << drive->action;
  ASSERT_EQ(drive->branches.size(), 1u) << "driving there has one outcome";

  const auto & look = drive->branches.begin()->second;
  ASSERT_NE(look, nullptr);
  EXPECT_EQ(look->action.rfind("inspect_", 0), 0u) << look->action;
  ASSERT_EQ(look->branches.size(), 2u)
    << "looking is where the mission stops being a sequence";

  // One branch reports a blockage and the other reports it clear. Which is
  // which is decided at execution time, by what the camera actually saw.
  std::vector<std::string> reports;
  for (const auto & [event, child] : look->branches) {
    (void)event;
    ASSERT_NE(child, nullptr);
    reports.push_back(child->action);
  }
  std::sort(reports.begin(), reports.end());
  ASSERT_EQ(reports.size(), 2u);
  EXPECT_EQ(reports[0].rfind("report-blocked_", 0), 0u) << reports[0];
  EXPECT_EQ(reports[1].rfind("report-clear_", 0), 0u) << reports[1];

  // And both reports come from the robot that looked, not from the one that
  // was watching: only the robot that saw the corridor can say what is there.
  const auto scout = look->action.substr(std::string("inspect_").size());
  EXPECT_NE(reports[0].find(scout), std::string::npos) << reports[0];
  EXPECT_NE(reports[1].find(scout), std::string::npos) << reports[1];
}

// One size up: three robots and two routes, so the fleet has to split the
// survey and the robot that stayed behind learns both answers by listening.
// Four leaves — one per way the two corridors can turn out.
TEST_F(SearchTest, AostarOnTheDepotFleetSplitsTheSurvey)
{
  const auto task = load_task(task_path("robot-fleet-depot"));
  EpistemicDistanceHeuristic h;

  ASSERT_EQ(task.init.num_designated(), 4u) << "two open questions, four worlds";

  const auto result = aostar::search(task, h, 0, soon());

  ASSERT_TRUE(result.has_value());
  ASSERT_NE(result->plan_tree, nullptr);

  const auto validation = validate(task, result->plan_tree);
  EXPECT_TRUE(validation.valid) << validation.error;
  EXPECT_EQ(validation.leaves_reached, 4u);
  EXPECT_TRUE(has_branch(result->plan_tree));

  // Both routes get surveyed, and no robot surveys both: a robot dispatched
  // down one cannot reach the other, so the plan has to use two of them.
  std::set<std::string> inspectors;
  std::function<void(const std::shared_ptr<PlanNode> &)> collect =
    [&](const std::shared_ptr<PlanNode> & node) {
      if (!node) {
        return;
      }
      if (node->action.rfind("inspect-", 0) == 0) {
        inspectors.insert(node->action);
      }
      for (const auto & [event, child] : node->branches) {
        (void)event;
        collect(child);
      }
    };
  collect(result->plan_tree);

  EXPECT_EQ(inspectors.size(), 2u) << "one look per route, no more and no less";
}

// A plan built for a different task must not validate against this one. If the
// validator accepted anything, every test above that leans on it would be
// vacuous.
TEST_F(SearchTest, ValidatorRejectsAForeignPlan)
{
  const auto task = load_task(task_path("muddy-children-2"));

  auto foreign = std::make_shared<PlanNode>();
  foreign->action = "open_A";   // a coin-in-the-box action

  const auto validation = validate(task, foreign);
  EXPECT_FALSE(validation.valid);
  EXPECT_FALSE(validation.error.empty());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
