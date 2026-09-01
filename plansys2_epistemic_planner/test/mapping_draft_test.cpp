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

// Drafting the action mapping instead of writing all of it by hand.
//
// The correspondence between an EPDDL schema and a PDDL action is a modelling
// decision and cannot be derived: nothing says that `inspect` is
// `inspect_corridor`. Everything around that decision can be, and hand-writing
// the file meant doing all of it for the sake of the one part that cannot.

#include <string>
#include <vector>

#include "gtest/gtest.h"

#include <nlohmann/json.hpp>

#include "plansys2_epistemic_planner/mapping_draft.hpp"
#include "plansys2_epistemic_planner/parser.hpp"

namespace
{

std::string task_path(const std::string & name)
{
  return std::string(EPISTEMIC_TEST_TASK_DIR) + "/" + name;
}

std::string example(const std::string & name)
{
  return std::string(EPISTEMIC_EXAMPLE_EPDDL_DIR) + "/" + name;
}

}  // namespace

TEST(MappingDraftTest, EveryGroundedActionGetsAKey)
{
  const auto task = load_task(task_path("robot-fleet.json"));
  const auto draft = plansys2::draft_mapping(task, {}, {});

  ASSERT_EQ(draft.entries.size(), task.actions.size())
    << "a draft that omits an action is a planning request that fails later";

  const auto json = nlohmann::json::parse(draft.to_json());
  for (const auto & action : task.actions) {
    EXPECT_TRUE(json.contains(action.name)) << action.name << " has no entry";
  }
}

TEST(MappingDraftTest, DeclaredSchemasDecideWhereTheNameStops)
{
  const auto task = load_task(task_path("robot-fleet.json"));
  const auto schemas = plansys2::read_epddl_schemas(example("robot-fleet-domain.epddl"));

  ASSERT_FALSE(schemas.empty()) << "the EPDDL domain declares no actions";
  EXPECT_NE(
    std::find(schemas.begin(), schemas.end(), "goto-junction"), schemas.end());
  EXPECT_NE(std::find(schemas.begin(), schemas.end(), "report-blocked"), schemas.end());

  const auto draft = plansys2::draft_mapping(task, schemas, {});

  for (const auto & entry : draft.entries) {
    if (entry.grounded == "goto-junction_r1") {
      // The hyphenated run belongs to the name. Splitting at the first
      // underscore would read it the same way here, but the schema list is
      // what makes that a fact instead of a coincidence.
      EXPECT_EQ(entry.schema, "goto-junction");
      ASSERT_EQ(entry.arguments.size(), 1u);
      EXPECT_EQ(entry.arguments[0], "r1");
    }
  }
}

TEST(MappingDraftTest, APddlDomainResolvesWhatItCan)
{
  const auto task = load_task(task_path("robot-fleet.json"));
  const auto schemas = plansys2::read_epddl_schemas(example("robot-fleet-domain.epddl"));
  // test_5's domain is the one the corridor mission actually dispatches
  // against, and the one the shipped mapping was written for.
  const auto actions = plansys2::read_pddl_actions(
    std::string(EPISTEMIC_MISSION_PDDL_DIR) + "/test_5.pddl");

  ASSERT_FALSE(actions.empty()) << "the PDDL domain declares no actions";

  const auto draft = plansys2::draft_mapping(task, schemas, actions);

  // `goto-junction` and `goto_junction` are one name written for two parsers,
  // so this one resolves without help.
  bool saw_resolved = false;
  for (const auto & entry : draft.entries) {
    if (entry.grounded == "goto-junction_r1") {
      EXPECT_EQ(entry.pddl_action, "goto_junction");
      EXPECT_EQ(entry.proposal, "(goto_junction r1)");
      EXPECT_TRUE(entry.problem.empty()) << entry.problem;
      saw_resolved = true;
    }
  }
  EXPECT_TRUE(saw_resolved) << "goto-junction_r1 is not in the task";
}

TEST(MappingDraftTest, AnArityMismatchIsReportedBeforeItReachesTheExecutor)
{
  // The grounder's example PDDL domain declares goto_junction over three
  // parameters, and this task grounds it over one. A mapping written against
  // it would dispatch an expression the domain cannot match, and the executor
  // is where that would surface. Saying it here is the point of checking.
  const auto task = load_task(task_path("robot-fleet.json"));
  const auto schemas = plansys2::read_epddl_schemas(example("robot-fleet-domain.epddl"));
  const auto actions = plansys2::read_pddl_actions(example("robot-fleet-domain.pddl"));

  const auto draft = plansys2::draft_mapping(task, schemas, actions);

  bool saw_mismatch = false;
  for (const auto & entry : draft.entries) {
    if (entry.grounded == "goto-junction_r1") {
      EXPECT_EQ(entry.pddl_action, "goto_junction");
      EXPECT_NE(entry.problem.find("3 parameters"), std::string::npos) << entry.problem;
      EXPECT_NE(entry.problem.find("supplies 1"), std::string::npos) << entry.problem;
      saw_mismatch = true;
    }
  }
  EXPECT_TRUE(saw_mismatch);
}

TEST(MappingDraftTest, AModellingDecisionIsReportedAndNotGuessedAt)
{
  const auto task = load_task(task_path("robot-fleet.json"));
  const auto schemas = plansys2::read_epddl_schemas(example("robot-fleet-domain.epddl"));
  const auto actions = plansys2::read_pddl_actions(example("robot-fleet-domain.pddl"));

  const auto draft = plansys2::draft_mapping(task, schemas, actions);

  // `inspect` is `inspect_corridor` in the PDDL domain, and nothing in either
  // file says so. The draft has to say it does not know.
  bool saw_unresolved = false;
  for (const auto & entry : draft.entries) {
    if (entry.grounded.rfind("inspect", 0) == 0) {
      EXPECT_FALSE(entry.problem.empty())
        << "the draft claims to know a correspondence nothing states";
      saw_unresolved = true;
    }
  }
  EXPECT_TRUE(saw_unresolved);

  EXPECT_FALSE(draft.complete());
  EXPECT_FALSE(draft.warnings().empty());

  // The unresolved entry is still written, so the file is the complete list of
  // what has to be decided.
  const auto json = nlohmann::json::parse(draft.to_json());
  EXPECT_TRUE(json.contains("inspect_r1"));
  EXPECT_TRUE(json["inspect_r1"].contains("_check"));
}

TEST(MappingDraftTest, TheDraftIsWhatTheSolverReads)
{
  // The point of the format check: a draft the solver cannot load would be a
  // file a person edits and only then discovers is the wrong shape.
  const auto task = load_task(task_path("robot-fleet.json"));
  const auto draft = plansys2::draft_mapping(task, {}, {});

  const auto json = nlohmann::json::parse(draft.to_json(2.5));
  for (const auto & [key, value] : json.items()) {
    ASSERT_TRUE(value.contains("action")) << key;
    EXPECT_EQ(value.at("action").get<std::string>().front(), '(') << key;
    ASSERT_TRUE(value.contains("duration"));
    EXPECT_DOUBLE_EQ(value.at("duration").get<double>(), 2.5);
  }
}

TEST(MappingDraftTest, TheEpddlDeclarationSaysHowManyArgumentsAnActionTakes)
{
  // EPDDL is the side this system plans over, so its own declaration is what
  // says how many arguments a grounded name should carry. Reading it here
  // means an arity question can be answered without consulting any PDDL.
  const auto arities =
    plansys2::read_epddl_schema_arities(example("robot-fleet-domain.epddl"));

  ASSERT_FALSE(arities.empty());

  bool saw = false;
  for (const auto & [name, params] : arities) {
    if (name == "goto-junction") {
      EXPECT_EQ(params, 1u) << "the schema declares (?i - agent)";
      saw = true;
    }
  }
  EXPECT_TRUE(saw) << "goto-junction is not declared in the EPDDL domain";

  // And it agrees with what the grounding produced, which is the check that
  // matters: a disagreement means the task and the domain have drifted.
  const auto task = load_task(task_path("robot-fleet.json"));
  const auto schemas = plansys2::read_epddl_schemas(example("robot-fleet-domain.epddl"));
  const auto draft = plansys2::draft_mapping(task, schemas, {});

  for (const auto & entry : draft.entries) {
    for (const auto & [name, params] : arities) {
      if (entry.schema == name) {
        EXPECT_EQ(entry.arguments.size(), params)
          << entry.grounded << " carries " << entry.arguments.size()
          << " arguments but " << name << " declares " << params;
      }
    }
  }
}

TEST(MappingDraftTest, AMissingFileIsEmptyAndNotAnError)
{
  EXPECT_TRUE(plansys2::read_epddl_schemas("/no/such/domain.epddl").empty());
  EXPECT_TRUE(plansys2::read_epddl_schema_arities("/no/such/domain.epddl").empty());
  EXPECT_TRUE(plansys2::read_pddl_actions("/no/such/domain.pddl").empty());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
