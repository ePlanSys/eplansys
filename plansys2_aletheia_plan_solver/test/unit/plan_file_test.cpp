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

// The plan file is the contract between this package and a planner binary it
// does not build. These tests pin the reader against the shapes that binary
// writes, so a change in either surfaces here rather than at run time on a
// robot.

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

#include "plansys2_aletheia_plan_solver/aletheia_plan_solver.hpp"

namespace
{

class PlanFile
{
public:
  explicit PlanFile(const std::string & contents)
  {
    path_ = std::filesystem::temp_directory_path() /
      ("aletheia-plan-test-" + std::to_string(counter_++) + ".json");
    std::ofstream out(path_);
    out << contents;
  }

  ~PlanFile()
  {
    std::error_code ec;
    std::filesystem::remove(path_, ec);
  }

  PlanFile(const PlanFile &) = delete;
  PlanFile & operator=(const PlanFile &) = delete;

  std::string path() const {return path_.string();}

private:
  std::filesystem::path path_;
  static inline unsigned counter_ = 0;
};

}  // namespace

TEST(PlanFileTest, reads_a_chain)
{
  // What AO* writes for a single-pointed task: one branch per node, so the
  // policy is a chain even though it is written in the tree form.
  PlanFile file(
    R"({
      "action": "ask_c1",
      "branches": [
        {"event": 0, "subtree": {"action": "ask_c2", "branches": [{"event": 0, "subtree": null}]}}
      ]
    })");

  std::string error;
  const auto plan = plansys2::read_plan_file(file.path(), error);

  ASSERT_TRUE(plan.has_value()) << error;
  EXPECT_TRUE(plan->is_tree);
  ASSERT_NE(plan->tree, nullptr);
  EXPECT_EQ(plan->tree->action, "ask_c1");
  ASSERT_EQ(plan->tree->branches.size(), 1u);

  const auto & [event, child] = plan->tree->branches.front();
  EXPECT_EQ(event, 0u);
  ASSERT_NE(child, nullptr);
  EXPECT_EQ(child->action, "ask_c2");
  ASSERT_EQ(child->branches.size(), 1u);
  EXPECT_EQ(child->branches.front().second, nullptr);   // a leaf
}

TEST(PlanFileTest, reads_a_branching_policy)
{
  PlanFile file(
    R"({
      "action": "peek_A",
      "branches": [
        {"event": 0, "subtree": {"action": "shout-tails_A", "branches": []}},
        {"event": 1, "subtree": {"action": "open_A", "branches": []}}
      ]
    })");

  std::string error;
  const auto plan = plansys2::read_plan_file(file.path(), error);

  ASSERT_TRUE(plan.has_value()) << error;
  ASSERT_NE(plan->tree, nullptr);
  ASSERT_EQ(plan->tree->branches.size(), 2u);
  EXPECT_EQ(plan->tree->branches[0].first, 0u);
  EXPECT_EQ(plan->tree->branches[1].first, 1u);
  EXPECT_EQ(plan->tree->branches[1].second->action, "open_A");
}

TEST(PlanFileTest, reads_a_linear_plan)
{
  // What GBFS and EHC write.
  PlanFile file(R"(["ask_c1", "ask_c2"])");

  std::string error;
  const auto plan = plansys2::read_plan_file(file.path(), error);

  ASSERT_TRUE(plan.has_value()) << error;
  EXPECT_FALSE(plan->is_tree);
  EXPECT_EQ(plan->tree, nullptr);
  ASSERT_EQ(plan->linear.size(), 2u);
  EXPECT_EQ(plan->linear[0], "ask_c1");
  EXPECT_EQ(plan->linear[1], "ask_c2");
}

TEST(PlanFileTest, reads_an_empty_plan)
{
  // The goal already held, so the planner wrote a null tree. That is a valid
  // plan with nothing in it, not a failure.
  PlanFile file("null");

  std::string error;
  const auto plan = plansys2::read_plan_file(file.path(), error);

  ASSERT_TRUE(plan.has_value()) << error;
  EXPECT_TRUE(plan->is_tree);
  EXPECT_EQ(plan->tree, nullptr);
}

TEST(PlanFileTest, a_missing_file_is_not_an_empty_plan)
{
  // The planner reports "no solution found" by exiting zero without writing a
  // plan, so this case must not be read as a plan with no actions: that would
  // report success and leave the executor with nothing to run.
  std::string error;
  const auto plan = plansys2::read_plan_file(
    (std::filesystem::temp_directory_path() / "aletheia-no-such-plan.json").string(), error);

  EXPECT_FALSE(plan.has_value());
  EXPECT_FALSE(error.empty());
}

TEST(PlanFileTest, rejects_a_node_without_an_action)
{
  PlanFile file(R"({"branches": []})");

  std::string error;
  const auto plan = plansys2::read_plan_file(file.path(), error);

  EXPECT_FALSE(plan.has_value());
  EXPECT_NE(error.find("action"), std::string::npos);
}

TEST(PlanFileTest, rejects_a_branch_without_an_event)
{
  PlanFile file(R"({"action": "ask_c1", "branches": [{"subtree": null}]})");

  std::string error;
  const auto plan = plansys2::read_plan_file(file.path(), error);

  EXPECT_FALSE(plan.has_value());
  EXPECT_NE(error.find("event"), std::string::npos);
}

TEST(PlanFileTest, rejects_malformed_json)
{
  PlanFile file("{\"action\": ");

  std::string error;
  const auto plan = plansys2::read_plan_file(file.path(), error);

  EXPECT_FALSE(plan.has_value());
  EXPECT_NE(error.find("JSON"), std::string::npos);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
