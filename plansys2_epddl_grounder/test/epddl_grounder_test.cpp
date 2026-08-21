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

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>

#include "gtest/gtest.h"
#include "plansys2_epddl_grounder/epddl_grounder.hpp"

namespace
{

/// The stand-in for plank, and the sources handed to it. Their contents never
/// reach the fake, which only needs them to exist.
std::string fake_plank() {return FAKE_PLANK;}

std::filesystem::path scratch()
{
  const auto dir = std::filesystem::temp_directory_path() / "eplansys-grounder-test";
  std::filesystem::create_directories(dir);
  return dir;
}

std::string write(const std::string & name, const std::string & contents)
{
  const auto path = scratch() / name;
  std::ofstream out(path);
  out << contents;
  return path.string();
}

plansys2::EpddlSpec sources()
{
  plansys2::EpddlSpec spec;
  spec.domain = write("d.epddl", "(define (domain d))");
  spec.problem = write("p.epddl", "(define (problem p))");
  spec.libraries.push_back(write("lib.epddl", "(define (action-type-library l))"));
  return spec;
}

}  // namespace

class GrounderTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ::setenv("FAKE_PLANK_TASK", FAKE_PLANK_TASK, 1);
    ::unsetenv("FAKE_PLANK_FAIL");
  }
};

TEST_F(GrounderTest, GroundsThroughTheConfiguredCommand)
{
  plansys2::EpddlGrounder grounder(fake_plank());
  const auto result = grounder.ground(sources());

  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_NE(result.task_json.find("planning-task-info"), std::string::npos);
  EXPECT_EQ(grounder.command(), fake_plank());
}

TEST_F(GrounderTest, LeavesNoOutputDirectoryBehind)
{
  const auto before = std::distance(
    std::filesystem::directory_iterator(std::filesystem::temp_directory_path()),
    std::filesystem::directory_iterator{});

  plansys2::EpddlGrounder grounder(fake_plank());
  ASSERT_TRUE(grounder.ground(sources()).ok);

  const auto after = std::distance(
    std::filesystem::directory_iterator(std::filesystem::temp_directory_path()),
    std::filesystem::directory_iterator{});

  EXPECT_EQ(before, after);
}

TEST_F(GrounderTest, UnchangedSourcesAreNotGroundTwice)
{
  const auto spec = sources();

  plansys2::EpddlGrounder grounder(fake_plank());
  const auto first = grounder.ground(spec);
  ASSERT_TRUE(first.ok) << first.error;

  // Making the fake fail proves the second call never reached it: a cache miss
  // here would surface as a grounding error rather than as the same JSON.
  ::setenv("FAKE_PLANK_FAIL", "1", 1);
  const auto second = grounder.ground(spec);

  ASSERT_TRUE(second.ok) << second.error;
  EXPECT_EQ(first.task_json, second.task_json);
}

TEST_F(GrounderTest, AnEditedSourceIsGroundAgain)
{
  auto spec = sources();

  plansys2::EpddlGrounder grounder(fake_plank());
  ASSERT_TRUE(grounder.ground(spec).ok);

  // A modification time the filesystem can tell apart from the first write.
  std::filesystem::last_write_time(
    spec.problem,
    std::filesystem::last_write_time(spec.problem) + std::chrono::seconds(2));

  ::setenv("FAKE_PLANK_FAIL", "1", 1);
  const auto second = grounder.ground(spec);

  EXPECT_FALSE(second.ok);
  EXPECT_NE(second.error.find("Syntax error"), std::string::npos) << second.error;
}

TEST_F(GrounderTest, ReportsPlankSDiagnosticsWhenGroundingFails)
{
  ::setenv("FAKE_PLANK_FAIL", "1", 1);

  plansys2::EpddlGrounder grounder(fake_plank());
  const auto result = grounder.ground(sources());

  ASSERT_FALSE(result.ok);
  // The message a user needs is the one plank printed, not our own summary.
  EXPECT_NE(result.error.find("invalid keyword identifier"), std::string::npos)
    << result.error;
  EXPECT_NE(result.error.find("signal"), std::string::npos) << result.error;
}

TEST_F(GrounderTest, RejectsAHalfConfiguredSpecification)
{
  plansys2::EpddlSpec spec;
  spec.domain = write("d.epddl", "(define (domain d))");

  plansys2::EpddlGrounder grounder(fake_plank());
  const auto result = grounder.ground(spec);

  ASSERT_FALSE(result.ok);
  EXPECT_NE(result.error.find("problem"), std::string::npos) << result.error;
}

TEST_F(GrounderTest, RejectsAnEmptySpecification)
{
  plansys2::EpddlGrounder grounder(fake_plank());
  const auto result = grounder.ground(plansys2::EpddlSpec{});

  ASSERT_FALSE(result.ok);
  EXPECT_NE(result.error.find("no EPDDL sources configured"), std::string::npos)
    << result.error;
}

TEST_F(GrounderTest, ReportsAMissingSourceFileByName)
{
  auto spec = sources();
  spec.problem = (scratch() / "absent.epddl").string();

  plansys2::EpddlGrounder grounder(fake_plank());
  const auto result = grounder.ground(spec);

  ASSERT_FALSE(result.ok);
  EXPECT_NE(result.error.find("absent.epddl"), std::string::npos) << result.error;
}

TEST_F(GrounderTest, ReportsAMissingLibraryByName)
{
  auto spec = sources();
  spec.libraries.push_back((scratch() / "absent-lib.epddl").string());

  plansys2::EpddlGrounder grounder(fake_plank());
  const auto result = grounder.ground(spec);

  ASSERT_FALSE(result.ok);
  EXPECT_NE(result.error.find("absent-lib.epddl"), std::string::npos) << result.error;
}

TEST_F(GrounderTest, ExplainsHowToGetPlankWhenItIsMissing)
{
  plansys2::EpddlGrounder grounder("no-such-grounder-binary");
  const auto result = grounder.ground(sources());

  ASSERT_FALSE(result.ok);
  EXPECT_NE(result.error.find("dependency_repos.repos"), std::string::npos)
    << result.error;
  EXPECT_NE(result.error.find("plank_command"), std::string::npos) << result.error;
}

TEST_F(GrounderTest, TakesTheCommandFromTheEnvironmentWhenNoneIsGiven)
{
  ::setenv("PLANK", fake_plank().c_str(), 1);

  plansys2::EpddlGrounder grounder;
  const auto result = grounder.ground(sources());
  ::unsetenv("PLANK");

  ASSERT_TRUE(result.ok) << result.error;
}

// The real toolchain, when the workspace has it. plank is built from source
// through dependency_repos.repos, so this runs in CI; a developer who has not
// built it gets a skip rather than a failure.
TEST(RealPlank, GroundsThePackagedExample)
{
  plansys2::EpddlGrounder grounder;

  plansys2::EpddlSpec spec;
  spec.domain = std::string(EXAMPLES_DIR) + "/muddy-children-domain.epddl";
  spec.problem = std::string(EXAMPLES_DIR) + "/muddy-children-problem.epddl";
  spec.libraries.push_back(std::string(LIBRARIES_DIR) + "/intermediate.epddl");

  const auto result = grounder.ground(spec);
  if (!result.ok && result.error.find("was not found") != std::string::npos) {
    GTEST_SKIP() << "plank is not built in this workspace";
  }

  ASSERT_TRUE(result.ok) << result.error;

  // The grounder's own accounting, which is what the planner's fixtures pin:
  // two children, one muddiness atom each, one ask action each, and the four
  // worlds that make up who is muddy.
  EXPECT_NE(result.task_json.find("\"agents-number\": 2"), std::string::npos);
  EXPECT_NE(result.task_json.find("\"atoms-number\": 2"), std::string::npos);
  EXPECT_NE(result.task_json.find("\"actions-number\": 2"), std::string::npos);
  EXPECT_NE(result.task_json.find("\"initial-worlds-number\": 4"), std::string::npos);
  EXPECT_NE(result.task_json.find("ask_c1"), std::string::npos);
  EXPECT_NE(result.task_json.find("ask_c2"), std::string::npos);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
