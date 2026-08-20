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

#include <filesystem>
#include <fstream>
#include <string>

#include "plansys2_epistemic_planner/action_mapping.hpp"

#include "task_fixtures.hpp"

using plansys2::ActionMapping;

namespace
{

// Written per test rather than checked in: these are malformed on purpose, and
// a fixture directory of broken files invites someone to "fix" them.
class TempMapping
{
public:
  explicit TempMapping(const std::string & contents)
  {
    path_ = std::filesystem::temp_directory_path() /
      ("eplansys-mapping-test-" + std::to_string(counter_++) + ".json");
    std::ofstream out(path_);
    out << contents;
  }

  ~TempMapping()
  {
    std::error_code ec;
    std::filesystem::remove(path_, ec);
  }

  TempMapping(const TempMapping &) = delete;
  TempMapping & operator=(const TempMapping &) = delete;

  std::string path() const {return path_.string();}

private:
  std::filesystem::path path_;
  static inline unsigned counter_ = 0;
};

}  // namespace

TEST(ActionMappingTest, ConventionSplitsAtTheFirstUnderscore)
{
  const auto mapping = ActionMapping::conventional();
  EXPECT_TRUE(mapping.is_conventional());

  // The head keeps its hyphens; only underscores separate parameters.
  EXPECT_EQ(mapping.translate("ask_c1")->action, "(ask c1)");
  EXPECT_EQ(mapping.translate("peek_A")->action, "(peek A)");
  EXPECT_EQ(mapping.translate("signal_A_B")->action, "(signal A B)");
  EXPECT_EQ(mapping.translate("pickup-A-hold_r2")->action, "(pickup-A-hold r2)");
  EXPECT_EQ(mapping.translate("observe-private-A_r1")->action, "(observe-private-A r1)");
}

TEST(ActionMappingTest, ConventionHandlesAParameterlessAction)
{
  const auto mapping = ActionMapping::conventional();
  EXPECT_EQ(mapping.translate("wait")->action, "(wait)");
}

TEST(ActionMappingTest, ConventionRejectsMalformedNames)
{
  const auto mapping = ActionMapping::conventional();
  EXPECT_FALSE(mapping.translate("").has_value());
  EXPECT_FALSE(mapping.translate("_c1").has_value());     // no action name
  EXPECT_FALSE(mapping.translate("ask__c1").has_value());  // empty parameter
}

TEST(ActionMappingTest, ConventionDefaultsToUnitDuration)
{
  const auto mapping = ActionMapping::conventional();
  EXPECT_FLOAT_EQ(mapping.translate("ask_c1")->duration, 1.0f);
}

TEST(ActionMappingTest, LoadedMapTranslatesAndCarriesDurations)
{
  const TempMapping file(
    R"json({
      "ask_c1": "(ask c1)",
      "move-kitchen_r1": {"action": "(move r1 corridor kitchen)", "duration": 12.5}
    })json");

  const auto mapping = ActionMapping::load(file.path());
  EXPECT_FALSE(mapping.is_conventional());
  EXPECT_EQ(mapping.size(), 2u);

  const auto ask = mapping.translate("ask_c1");
  ASSERT_TRUE(ask.has_value());
  EXPECT_EQ(ask->action, "(ask c1)");
  EXPECT_FLOAT_EQ(ask->duration, 1.0f);

  const auto move = mapping.translate("move-kitchen_r1");
  ASSERT_TRUE(move.has_value());
  EXPECT_EQ(move->action, "(move r1 corridor kitchen)");
  EXPECT_FLOAT_EQ(move->duration, 12.5f);
}

// The whole point of the explicit map is that it can express a correspondence
// the grounded name does not encode — here the parameters come out in the
// domain's order, not the order plank mangled them into.
TEST(ActionMappingTest, LoadedMapCanReorderParameters)
{
  const TempMapping file(R"json({"pickup-A-hold_r2": "(pickup r2 A)"})json");

  const auto mapping = ActionMapping::load(file.path());
  EXPECT_EQ(mapping.translate("pickup-A-hold_r2")->action, "(pickup r2 A)");

  // The convention cannot reach that answer, which is why it is only a
  // fallback.
  EXPECT_EQ(
    ActionMapping::conventional().translate("pickup-A-hold_r2")->action,
    "(pickup-A-hold r2)");
}

// An unmapped action must not fall through to the convention. A guessed name
// the domain does not have would reach the executor as a plan that cannot be
// dispatched, and the reason would surface far from the mapping.
TEST(ActionMappingTest, LoadedMapDoesNotFallBackToTheConvention)
{
  const TempMapping file(R"json({"ask_c1": "(ask c1)"})json");
  const auto mapping = ActionMapping::load(file.path());

  EXPECT_FALSE(mapping.translate("ask_c2").has_value());
}

TEST(ActionMappingTest, LoadRejectsMalformedFiles)
{
  EXPECT_THROW(ActionMapping::load("/nonexistent/mapping.json"), std::runtime_error);

  {
    const TempMapping file("{not json");
    EXPECT_THROW(ActionMapping::load(file.path()), std::runtime_error);
  }
  {
    const TempMapping file(R"json(["ask_c1"])json");   // an array, not an object
    EXPECT_THROW(ActionMapping::load(file.path()), std::runtime_error);
  }
  {
    const TempMapping file(R"json({"ask_c1": {"duration": 2.0}})json");   // no action
    EXPECT_THROW(ActionMapping::load(file.path()), std::runtime_error);
  }
  {
    const TempMapping file(R"json({"ask_c1": ""})json");
    EXPECT_THROW(ActionMapping::load(file.path()), std::runtime_error);
  }
  {
    const TempMapping file(R"json({"ask_c1": 3})json");
    EXPECT_THROW(ActionMapping::load(file.path()), std::runtime_error);
  }
}

// The executor schedules on duration and times actions out against it, so a
// zero or negative one is a mapping bug worth catching at load.
TEST(ActionMappingTest, LoadRejectsNonPositiveDurations)
{
  {
    const TempMapping file(R"json({"ask_c1": {"action": "(ask c1)", "duration": 0}})json");
    EXPECT_THROW(ActionMapping::load(file.path()), std::runtime_error);
  }
  {
    const TempMapping file(R"json({"ask_c1": {"action": "(ask c1)", "duration": -1.5}})json");
    EXPECT_THROW(ActionMapping::load(file.path()), std::runtime_error);
  }
  {
    const TempMapping file(R"json({"ask_c1": {"action": "(ask c1)", "duration": "slow"}})json");
    EXPECT_THROW(ActionMapping::load(file.path()), std::runtime_error);
  }
}

// The mapping shipped for muddy-children must cover every action the task can
// put in a plan. A mapping is only useful if it is total over the domain, and
// this is the check that catches a task and a map drifting apart.
TEST(ActionMappingTest, ExampleMappingCoversItsTask)
{
  const auto mapping = ActionMapping::load(mapping_path("muddy-children-2"));

  for (const auto & grounded : {"ask_c1", "ask_c2"}) {
    EXPECT_TRUE(mapping.translate(grounded).has_value())
      << "the example mapping does not cover " << grounded;
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
