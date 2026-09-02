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

// Writing a belief state back out, so that the next plan can start from it.
//
// The planner reads a task and never had to write one. Replanning needs the
// other direction: after a mission diverges, the state to plan from is the one
// the robot reached, and it exists only in the running node. What has to hold
// is that the model surviving the round trip is the same model, since a plan
// built for a subtly different one is a plan for a mission nobody is on.

#include <string>

#include "gtest/gtest.h"

#include "plansys2_epistemic_planner/parser.hpp"
#include "plansys2_epistemic_planner/product_update.hpp"
#include "plansys2_epistemic_planner/state_json.hpp"

namespace
{

std::string task_path(const std::string & name)
{
  return std::string(EPISTEMIC_TEST_TASK_DIR) + "/" + name;
}

/// Same worlds, same valuations, same relations, same designation.
void expect_same_model(const EpistemicState & a, const EpistemicState & b)
{
  ASSERT_EQ(a.num_worlds, b.num_worlds);
  ASSERT_EQ(a.num_atoms, b.num_atoms);
  ASSERT_EQ(a.num_agents, b.num_agents);

  for (std::uint32_t w = 0; w < a.num_worlds; ++w) {
    EXPECT_EQ(a.is_designated(w), b.is_designated(w)) << "designation of w" << w;
    for (std::uint32_t at = 0; at < a.num_atoms; ++at) {
      EXPECT_EQ(a.has_atom(w, at), b.has_atom(w, at)) << "atom " << at << " at w" << w;
    }
    for (std::uint32_t ag = 0; ag < a.num_agents; ++ag) {
      for (std::uint32_t to = 0; to < a.num_worlds; ++to) {
        EXPECT_EQ(
          bits::test(a.succ(ag, w), to),
          bits::test(b.succ(ag, w), to)) << "edge " << ag << ": w" << w << " -> w" << to;
      }
    }
  }
}

}  // namespace

TEST(StateJsonTest, TheInitialModelSurvivesTheRoundTrip)
{
  const auto task = load_task(task_path("robot-fleet.json"));

  const auto json = plansys2::state_to_json(task, task.init);

  EpistemicState back;
  std::string error;
  ASSERT_TRUE(plansys2::state_from_json(task, json, back, error)) << error;

  expect_same_model(task.init, back);
}

TEST(StateJsonTest, AModelAdvancedByAnActionSurvivesTheRoundTrip)
{
  // The interesting case is not the model grounding produced but the one a
  // mission reached, since that is the only one replanning ever has to write.
  auto task = load_task(task_path("robot-fleet.json"));
  ASSERT_FALSE(task.actions.empty());

  const auto before = task.init.num_designated();

  EpistemicState advanced;
  bool applied = false;
  for (const auto & action : task.actions) {
    if (!action.applicable(task.init)) {
      continue;
    }
    auto result = product_update(task.init, action, task.kd45);
    if (!result) {
      continue;   // capped or pruned; another action will do
    }
    advanced = *result;
    applied = true;
    break;
  }
  ASSERT_TRUE(applied) << "no action applies to the initial state, so there is "
    "no advanced model to round trip";

  const auto json = plansys2::state_to_json(task, advanced);

  EpistemicState back;
  std::string error;
  ASSERT_TRUE(plansys2::state_from_json(task, json, back, error)) << error;

  expect_same_model(advanced, back);
  EXPECT_GT(before, 0u);
}

TEST(StateJsonTest, AModelInAnotherVocabularyIsRefused)
{
  const auto task = load_task(task_path("robot-fleet.json"));

  // The failure this guards against is the quiet one: a state and a planner
  // holding different problems, where planning would succeed and produce a
  // policy for a mission nobody is on.
  const std::string foreign =
    R"({"worlds":["w0"],"labels":{"w0":["not-an-atom-of-this-task"]},)"
    R"("designated":["w0"],"relations":{}})";

  EpistemicState out;
  std::string error;
  EXPECT_FALSE(plansys2::state_from_json(task, foreign, out, error));
  EXPECT_NE(error.find("not-an-atom-of-this-task"), std::string::npos) << error;
}

TEST(StateJsonTest, AModelNamingAnUnknownAgentIsRefused)
{
  const auto task = load_task(task_path("robot-fleet.json"));

  const std::string foreign =
    R"({"worlds":["w0"],"labels":{"w0":[]},"designated":["w0"],)"
    R"("relations":{"r99":{"w0":["w0"]}}})";

  EpistemicState out;
  std::string error;
  EXPECT_FALSE(plansys2::state_from_json(task, foreign, out, error));
  EXPECT_NE(error.find("r99"), std::string::npos) << error;
}

TEST(StateJsonTest, AModelWithNothingDesignatedIsRefused)
{
  const auto task = load_task(task_path("robot-fleet.json"));

  // Every formula holds vacuously in a model that designates nothing, so a
  // goal check against one reports success while saying nothing at all.
  const std::string empty =
    R"({"worlds":["w0"],"labels":{"w0":[]},"designated":[],"relations":{}})";

  EpistemicState out;
  std::string error;
  EXPECT_FALSE(plansys2::state_from_json(task, empty, out, error));
  EXPECT_NE(error.find("designates no world"), std::string::npos) << error;
}

TEST(StateJsonTest, AMalformedModelIsRefusedWithAReason)
{
  const auto task = load_task(task_path("robot-fleet.json"));

  EpistemicState out;
  std::string error;
  EXPECT_FALSE(plansys2::state_from_json(task, "not json at all", out, error));
  EXPECT_FALSE(error.empty());

  EXPECT_FALSE(plansys2::state_from_json(task, R"({"worlds":[]})", out, error));
  EXPECT_FALSE(error.empty());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
