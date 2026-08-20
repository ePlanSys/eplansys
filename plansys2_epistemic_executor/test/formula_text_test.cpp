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

#include "plansys2_epistemic_executor/formula_text.hpp"
#include "plansys2_epistemic_planner/formula.hpp"
#include "plansys2_epistemic_planner/parser.hpp"
#include "plansys2_epistemic_planner/policy_plan.hpp"

using plansys2::parse_formula;
using plansys2::render_formula;

namespace
{

// The tasks live with the planner that grounds them. This is the only place
// the two packages meet on disk, and only in a test: nothing installed depends
// on the path.
std::string task_path(const std::string & name)
{
  return std::string(EPISTEMIC_TASK_DIR) + "/" + name + ".json";
}

}  // namespace

class FormulaTextTest : public ::testing::Test
{
protected:
  void SetUp() override {formula_registry_reset();}
  void TearDown() override {formula_registry_reset();}
};

// The planner writes the formula, and the state that checks it reads the text
// back. They are separate processes with separate interning, so nothing but
// this agreement connects them. What is asserted is not that the strings match
// but that the *formulas* do: interning makes structural identity pointer
// identity, so this is an exact check.
TEST_F(FormulaTextTest, EveryGoalRoundTrips)
{
  for (const auto & name :
    {"muddy-children-2", "muddy-children-3", "coin-in-the-box", "active-muddy-child",
      "coin-in-the-box-multipointed"})
  {
    formula_registry_reset();
    const auto task = load_task(task_path(name));
    ASSERT_NE(task.goal, nullptr) << name;

    const auto text = render_formula(task, *task.goal);

    std::string error;
    const auto parsed = parse_formula(task, text, error);
    ASSERT_NE(parsed, nullptr) << name << ": " << error << " (from '" << text << "')";
    EXPECT_EQ(parsed, task.goal) << name << ": '" << text << "' came back as a different formula";
  }
}

// Knowledge preconditions travel the same way, and there are far more of them
// than there are goals.
TEST_F(FormulaTextTest, EveryActionPreconditionRoundTrips)
{
  const auto task = load_task(task_path("coin-in-the-box"));

  std::size_t checked = 0;
  for (const auto & action : task.actions) {
    for (const auto event_id : action.designated_events) {
      const auto & precondition = action.events[event_id].precondition;
      if (!precondition) {
        continue;
      }
      const auto text = render_formula(task, *precondition);

      std::string error;
      const auto parsed = parse_formula(task, text, error);
      ASSERT_NE(parsed, nullptr) << action.name << ": " << error;
      EXPECT_EQ(parsed, precondition) << action.name << ": '" << text << "'";
      ++checked;
    }
  }
  EXPECT_GT(checked, 0u) << "the fixture stopped exercising anything";
}

TEST_F(FormulaTextTest, ConnectivesAndModalitiesParse)
{
  const auto task = load_task(task_path("coin-in-the-box"));

  std::string error;
  for (const auto & text : {
      "tails",
      "(true)",
      "(false)",
      "(not tails)",
      "(and tails (K A tails))",
      "(or tails (Kw B tails))",
      "(K A (not tails))",
      "(Kw A tails)",
      "(C (A B) tails)",
    })
  {
    EXPECT_NE(parse_formula(task, text, error), nullptr) << text << ": " << error;
  }
}

// K and B are one modality under two frames. A policy that carries "(K ...)"
// must keep parsing if the task is later re-grounded as doxastic, or the goal
// stops being checkable for a reason that tells the operator nothing.
TEST_F(FormulaTextTest, KAndBAreTheSameModality)
{
  const auto task = load_task(task_path("coin-in-the-box"));

  std::string error;
  const auto with_k = parse_formula(task, "(K A tails)", error);
  const auto with_b = parse_formula(task, "(B A tails)", error);
  ASSERT_NE(with_k, nullptr) << error;
  EXPECT_EQ(with_k, with_b);
}

// A name the task does not have means the policy and the state disagree about
// which problem is being solved. Inventing the symbol would turn that into a
// condition that quietly never holds.
TEST_F(FormulaTextTest, UnknownNamesAreRejected)
{
  const auto task = load_task(task_path("coin-in-the-box"));

  std::string error;
  EXPECT_EQ(parse_formula(task, "(K Zaphod tails)", error), nullptr);
  EXPECT_NE(error.find("Zaphod"), std::string::npos) << error;

  EXPECT_EQ(parse_formula(task, "sideways", error), nullptr);
  EXPECT_NE(error.find("sideways"), std::string::npos) << error;
}

TEST_F(FormulaTextTest, MalformedInputIsRejectedWithAReason)
{
  const auto task = load_task(task_path("coin-in-the-box"));

  std::string error;
  for (const auto & text : {
      "",
      "(",
      "(K A tails",        // unclosed
      "(K A)",             // no operand
      "(and)",             // no conjuncts
      "(nope tails)",      // unknown connective
      "(K tails)",         // an atom where an agent goes
      "(C () tails)",      // common knowledge among nobody
      "tails extra",       // trailing text
    })
  {
    EXPECT_EQ(parse_formula(task, text, error), nullptr) << "accepted '" << text << "'";
    EXPECT_FALSE(error.empty()) << "rejected '" << text << "' without saying why";
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
