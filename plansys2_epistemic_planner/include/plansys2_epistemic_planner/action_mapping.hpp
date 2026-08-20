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

#ifndef PLANSYS2_EPISTEMIC_PLANNER__ACTION_MAPPING_HPP_
#define PLANSYS2_EPISTEMIC_PLANNER__ACTION_MAPPING_HPP_

#include <optional>
#include <string>
#include <unordered_map>

namespace plansys2
{

/// One grounded epistemic action expressed the way the executor expects it.
struct MappedAction
{
  /// A PlanSys2 action expression: "(move r1 corridor kitchen)".
  std::string action;

  /// Seconds. The epistemic planner is untimed, so this is a property of the
  /// robot's action implementation, not of the plan; it is what the executor
  /// uses to schedule and to time out.
  float duration{1.0f};
};

/**
 * @class plansys2::ActionMapping
 * @brief Translates Aletheia's grounded action names into PlanSys2 actions.
 *
 * These are two different vocabularies, not two spellings of one. Aletheia
 * plans over an event model whose actions plank grounds into single tokens —
 * "pickup-A-hold_r2", "observe-private-A_r1", "ask_c1" — while the executor
 * splits "(pickup r2 A)" into a name and parameters and looks the name up in
 * the PDDL domain to find the BT that drives the hardware. Handing the
 * executor an untranslated name yields no match and therefore no motion, so
 * the translation has to happen before the plan leaves this plugin.
 *
 * Two sources, in order of preference:
 *
 *   load()          An explicit JSON map, which is the honest option: it
 *                   states the correspondence between the two vocabularies
 *                   instead of inferring it, and it survives plank changing
 *                   how it mangles names.
 *
 *   conventional()  A fallback for when no map is configured. It splits the
 *                   grounded name at its first underscore and reads what
 *                   follows as parameters, so "ask_c1" becomes "(ask c1)".
 *                   This is a guess about plank's naming and, more seriously,
 *                   about parameter order, which the grounded name does not
 *                   record. Fine for a domain whose actions take one agent
 *                   parameter; not something to rely on for a real robot.
 *
 * The map file is a JSON object keyed by grounded action name. A value may be
 * the action expression alone, or an object carrying a duration:
 *
 *   {
 *     "ask_c1":            "(ask c1)",
 *     "move-kitchen_r1":   {"action": "(move r1 corridor kitchen)",
 *                           "duration": 12.5}
 *   }
 */
class ActionMapping
{
public:
  /// Load a map file. Throws std::runtime_error if it cannot be read or does
  /// not have the documented shape.
  static ActionMapping load(const std::string & path);

  /// The underscore-splitting fallback described above.
  static ActionMapping conventional();

  /// Nullopt when a map is loaded and does not cover `grounded`. An unmapped
  /// action is a hole in the mapping, not something to paper over: the
  /// conventional guess would produce a name the domain does not have, and the
  /// executor would refuse the plan anyway, but later and less legibly.
  std::optional<MappedAction> translate(const std::string & grounded) const;

  /// True for conventional(), false for a loaded map.
  bool is_conventional() const {return conventional_;}

  /// Entries in a loaded map; zero for conventional().
  std::size_t size() const {return table_.size();}

private:
  std::unordered_map<std::string, MappedAction> table_;
  bool conventional_{false};
};

}  // namespace plansys2

#endif  // PLANSYS2_EPISTEMIC_PLANNER__ACTION_MAPPING_HPP_
