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


// A temporal policy serialised for the executor.
//
// The executor dispatches a temporal Plan by events: an item starts when the
// items in its starts_after have ended, on the branch its given_items select.
// These tests pin what the corridor's policy turns into, since the executor
// has nothing else to go on.

#include <algorithm>
#include <string>

#include "gtest/gtest.h"
#include "aletheia/bisimulation.hpp"
#include "aletheia/parser.hpp"
#include "plansys2_epistemic_planner/action_mapping.hpp"
#include "plansys2_epistemic_planner/policy_plan.hpp"
#include "task_fixtures.hpp"

namespace
{

std::optional<plansys2_msgs::msg::Plan> corridor_plan(std::size_t concurrency)
{
  const auto task = load_task(task_path("robot-fleet"));
  const auto mapping = plansys2::ActionMapping::load(
    mapping_path("robot-fleet"));
  const auto durations = plansys2::durations_from_mapping(task, mapping);

  temporal::SearchOptions options;
  options.max_concurrency = concurrency;
  const auto policy = temporal::search_policy(task, durations, options);
  if (!policy) {
    return std::nullopt;
  }
  std::string error;
  auto plan = plansys2::to_temporal_plan(task, *policy, mapping, error);
  EXPECT_TRUE(plan.has_value()) << error;
  return plan;
}

std::uint32_t index_of(const plansys2_msgs::msg::Plan & plan, const std::string & action)
{
  for (std::uint32_t i = 0; i < plan.items.size(); ++i) {
    if (plan.items[i].action == action) {
      return i;
    }
  }
  ADD_FAILURE() << "no item " << action;
  return 0;
}

}  // namespace

TEST(TemporalPlan, DurationsComeFromTheMapping)
{
  const auto task = load_task(task_path("robot-fleet"));
  const auto mapping = plansys2::ActionMapping::load(
    mapping_path("robot-fleet"));
  const auto d = plansys2::durations_from_mapping(task, mapping);
  EXPECT_DOUBLE_EQ(d[task.action_index.at("goto-junction_r1")], 30.0);
  EXPECT_DOUBLE_EQ(d[task.action_index.at("inspect_r2")], 5.0);
}

TEST(TemporalPlan, BothRobotsDriveThenBothLook)
{
  // Sending both robots to look (30 + 5 s) beats one looking and telling the
  // other (30 + 5 + 1 s), and the two halves overlap.
  const auto plan = corridor_plan(3);
  ASSERT_TRUE(plan.has_value());
  EXPECT_TRUE(plan->temporal);
  EXPECT_FALSE(plan->epistemic_goal.empty());
  ASSERT_EQ(plan->items.size(), 4u);

  const auto drive1 = index_of(*plan, "(goto_junction r1)");
  const auto drive2 = index_of(*plan, "(goto_junction r2)");
  const auto look1 = index_of(*plan, "(inspect_corridor r1)");
  const auto look2 = index_of(*plan, "(inspect_corridor r2)");

  // The drives start with the policy, together.
  EXPECT_TRUE(plan->items[drive1].starts_after.empty());
  EXPECT_TRUE(plan->items[drive2].starts_after.empty());
  EXPECT_FLOAT_EQ(plan->items[drive1].time, 0.0f);
  EXPECT_FLOAT_EQ(plan->items[drive2].time, 0.0f);

  // Both looks wait for both drives, which end at the same planned time.
  for (const auto look : {look1, look2}) {
    auto after = plan->items[look].starts_after;
    std::sort(after.begin(), after.end());
    EXPECT_EQ(after, (std::vector<std::uint32_t>{std::min(drive1, drive2),
        std::max(drive1, drive2)}));
    EXPECT_FLOAT_EQ(plan->items[look].time, 30.0f);
    EXPECT_TRUE(plan->items[look].given_items.empty());
    EXPECT_TRUE(plan->items[look].sensing);
  }

  // Nothing names an item written after it, which the executor relies on.
  for (std::uint32_t i = 0; i < plan->items.size(); ++i) {
    for (const auto other : plan->items[i].starts_after) {
      EXPECT_LT(other, i);
    }
    EXPECT_TRUE(plan->items[i].children.empty());
  }
}

TEST(TemporalPlan, OneAtATimeBranchesOnTheLook)
{
  // Sequential, the corridor takes the announcement: drive, look, and tell
  // what was seen, which depends on what the look observed.
  const auto plan = corridor_plan(1);
  ASSERT_TRUE(plan.has_value());
  ASSERT_EQ(plan->items.size(), 4u);   // drive, look, and one report per outcome

  const auto look = index_of(*plan, "(inspect_corridor r1)");
  std::size_t reports = 0;
  for (const auto & item : plan->items) {
    if (item.action.rfind("(report_", 0) != 0) {
      continue;
    }
    ++reports;
    ASSERT_EQ(item.given_items.size(), 1u);
    EXPECT_EQ(item.given_items[0], look);
    EXPECT_EQ(item.starts_after, (std::vector<std::uint32_t>{look}));
    // Each report is on the branch of the outcome it announces.
    const bool blocked = item.action.find("blocked") != std::string::npos;
    EXPECT_EQ(item.given_outcomes[0], blocked ? "e-inspect-blocked" : "e-inspect-clear");
  }
  EXPECT_EQ(reports, 2u);
}

TEST(TemporalPlan, TheCriticalPathSearchReturnsAPlanNoShorterThanOptimal)
{
  // The satisficing search under max(h_adm, h_cp): valid, serialisable, and
  // never shorter than the optimum, which it usually matches.
  const auto task = load_task(task_path("robot-fleet-depot"));
  const auto mapping = plansys2::ActionMapping::load(mapping_path("robot-fleet-depot"));
  const auto durations = plansys2::durations_from_mapping(task, mapping);

  temporal::SearchOptions cp;
  cp.estimate = temporal::SearchOptions::Estimate::CriticalPath;
  const auto satisficing = temporal::search_policy(task, durations, cp);
  const auto optimal = temporal::search_policy(task, durations);
  ASSERT_TRUE(satisficing.has_value());
  ASSERT_TRUE(optimal.has_value());
  EXPECT_GE(satisficing->makespan, optimal->makespan - 1e-9);

  std::string why;
  EXPECT_TRUE(temporal::validate_policy(task, durations, *satisficing, &why)) << why;
  std::string error;
  const auto plan = plansys2::to_temporal_plan(task, *satisficing, mapping, error);
  ASSERT_TRUE(plan.has_value()) << error;
  EXPECT_TRUE(plan->temporal);
}
