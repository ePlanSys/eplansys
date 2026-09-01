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

#ifndef PLANSYS2_EPISTEMIC_PLANNER__STATE_JSON_HPP_
#define PLANSYS2_EPISTEMIC_PLANNER__STATE_JSON_HPP_

#include <string>

#include "plansys2_epistemic_planner/state.hpp"
#include "plansys2_epistemic_planner/task.hpp"

namespace plansys2
{

/**
 * @brief Render a pointed Kripke model in the task format's `initial-state`
 *        shape.
 *
 * The planner reads a task from a file and never had to write one back, so a
 * model could only ever be the one grounding produced. Replanning needs the
 * other direction: the belief state a mission actually reached is the state the
 * next plan has to start from, and it exists only in the running node.
 *
 * Worlds are named `w0 .. w{n-1}` in index order, which is the order the parser
 * assigns indices in, so a model written here and read back yields the same
 * indices and the same fingerprint. Atom and agent names come from `task`.
 *
 * @param[in] task Supplies the atom and agent vocabularies.
 * @param[in] state The model to write.
 * @return The `initial-state` object, as JSON.
 */
std::string state_to_json(const PlanningTask & task, const EpistemicState & state);

/**
 * @brief Read a model back, resolving names against a task.
 *
 * Every atom, agent and designated world named by the JSON must exist in
 * `task`. A name it does not have is an error and not a new symbol: it means
 * whoever wrote the model and whoever is reading it hold different problems,
 * and planning from it would search a space the policy cannot be executed in.
 *
 * @param[in] task The vocabulary to resolve against.
 * @param[in] json An `initial-state` object, as written by state_to_json.
 * @param[out] out The model read.
 * @param[out] error What was wrong, when the result is false.
 * @return True when the model was read.
 */
bool state_from_json(
  const PlanningTask & task, const std::string & json,
  EpistemicState & out, std::string & error);

}  // namespace plansys2

#endif  // PLANSYS2_EPISTEMIC_PLANNER__STATE_JSON_HPP_
