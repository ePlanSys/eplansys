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

#ifndef PLANSYS2_EPISTEMIC_PLANNER__MAPPING_DRAFT_HPP_
#define PLANSYS2_EPISTEMIC_PLANNER__MAPPING_DRAFT_HPP_

#include <string>
#include <vector>

#include "plansys2_epistemic_planner/task.hpp"

namespace plansys2
{

/// One grounded action, split into the parts a mapping entry is written from.
struct DraftEntry
{
  /// The grounded name, which is the key a mapping is written under.
  std::string grounded;

  /// The EPDDL schema the name was ground from, when a declared schema matches
  /// it. Empty when none does, which is the case a person has to resolve.
  std::string schema;

  /// The arguments, in the order the grounded name carries them.
  std::vector<std::string> arguments;

  /// The PDDL action this was matched to, empty when nothing matched.
  std::string pddl_action;

  /// Why this entry needs a person, empty when it does not.
  std::string problem;

  /// The mapping value proposed, always populated: the matched PDDL action
  /// applied to the arguments, or the best available guess.
  std::string proposal;
};

/// A whole draft mapping, and what is wrong with it.
struct MappingDraft
{
  std::vector<DraftEntry> entries;

  /// True when every entry matched a PDDL action, so the draft can be used
  /// as it stands.
  bool complete() const;

  /// The draft as the JSON the `action_mapping` parameter reads. Entries that
  /// need a person are still written, so that the file is a complete list of
  /// what has to be decided and the planner's own error names a key that is
  /// present.
  std::string to_json(double default_duration = 1.0) const;

  /// One line per entry needing attention, for a person reading a terminal.
  std::vector<std::string> warnings() const;
};

/**
 * @brief Draft an action mapping for a grounded task.
 *
 * The two vocabularies cannot be reconciled automatically in general. plank
 * grounds an EPDDL action into a single token and the executor wants a PDDL
 * expression, and which PDDL action corresponds to which EPDDL schema is a
 * modelling decision: nothing in either file says that `inspect` is
 * `inspect_corridor`, because nothing has to.
 *
 * What can be done is everything else, and hand-writing the file meant doing
 * all of it by hand for the sake of the part that cannot be automated. This
 * produces every key the task will ever ask for, splits each grounded name
 * against the schemas actually declared instead of guessing at underscores,
 * matches what can be matched against the PDDL domain, and says which entries
 * are left. The remaining work is the modelling decision alone.
 *
 * @param[in] task The grounded task, which supplies the action names.
 * @param[in] schemas EPDDL schema names, longest match wins when splitting a
 *   grounded name. Empty falls back to splitting at the first underscore.
 * @param[in] pddl_actions Action names declared by the PDDL domain, with their
 *   parameter counts, used to match and to check arity.
 * @return The draft.
 */
MappingDraft draft_mapping(
  const PlanningTask & task,
  const std::vector<std::string> & schemas,
  const std::vector<std::pair<std::string, std::size_t>> & pddl_actions);

/// Action schema names declared by an EPDDL domain file. Reads `(:action x`
/// forms; returns an empty list when the file cannot be read.
std::vector<std::string> read_epddl_schemas(const std::string & domain_path);

/// The same, with the number of parameters each schema declares.
///
/// EPDDL is the side this system plans over, so its declaration is what says
/// how many arguments a grounded name should carry. A disagreement between
/// that and the name plank produced means the grounding and the domain have
/// drifted apart, which is worth saying before any PDDL is consulted.
std::vector<std::pair<std::string, std::size_t>> read_epddl_schema_arities(
  const std::string & domain_path);

/// Action names and parameter counts declared by a PDDL domain file.
std::vector<std::pair<std::string, std::size_t>> read_pddl_actions(
  const std::string & domain_path);

}  // namespace plansys2

#endif  // PLANSYS2_EPISTEMIC_PLANNER__MAPPING_DRAFT_HPP_
