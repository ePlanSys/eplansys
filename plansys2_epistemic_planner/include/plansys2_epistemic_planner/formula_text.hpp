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

#ifndef PLANSYS2_EPISTEMIC_PLANNER__FORMULA_TEXT_HPP_
#define PLANSYS2_EPISTEMIC_PLANNER__FORMULA_TEXT_HPP_

#include <string>

#include "plansys2_epistemic_planner/task.hpp"
#include "plansys2_epistemic_planner/types.hpp"

namespace plansys2
{

/**
 * @brief Read a formula written the way a policy carries it.
 *
 * The counterpart of plansys2::render_formula. Knowledge preconditions and
 * epistemic goals travel as text — they have to cross a service boundary, and
 * the interned formula identifiers they were built from are process-local —
 * so the state that checks them has to read them back.
 *
 * The grammar is what render_formula writes:
 *
 *     formula := atom
 *              | "(true)" | "(false)"
 *              | "(not" formula ")"
 *              | "(and" formula+ ")"   | "(or" formula+ ")"
 *              | "(K" agent formula ")" | "(B" agent formula ")"
 *              | "(Kw" agent formula ")"
 *              | "(C (" agent+ ")" formula ")"
 *
 * K and B are the same modality under different frames — an agent's box — and
 * both are accepted whatever the task's frame is. Rejecting "(K ...)" on a
 * doxastic task would only mean the goal a policy carries stops parsing the
 * moment the frame changes, which tells the operator nothing useful.
 *
 * Names are resolved against the task, so a formula naming an agent or atom
 * the task does not have is an error rather than a fresh symbol: it means the
 * policy and the state disagree about which problem is being solved.
 *
 * @param[in] task Supplies the vocabulary.
 * @param[in] text The formula.
 * @param[out] error What was wrong, when the result is null.
 * @return The interned formula, or null.
 */
FormulaPtr parse_formula(
  const PlanningTask & task, const std::string & text, std::string & error);

}  // namespace plansys2

#endif  // PLANSYS2_EPISTEMIC_PLANNER__FORMULA_TEXT_HPP_
