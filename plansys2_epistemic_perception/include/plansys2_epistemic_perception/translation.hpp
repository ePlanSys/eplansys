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

#ifndef PLANSYS2_EPISTEMIC_PERCEPTION__TRANSLATION_HPP_
#define PLANSYS2_EPISTEMIC_PERCEPTION__TRANSLATION_HPP_

#include <optional>
#include <string>

#include "plansys2_epistemic_perception/regions.hpp"

namespace plansys2
{

/// The sensing action a region's resolution answers.
///
/// A sensing action in a policy does not decide its own outcome: the planner
/// enumerated the outcomes and the robot has to say which one occurred. When
/// that observation is a look at the map, this is the binding that turns it
/// into the event name the state expects on ApplyAction.
struct SensingBinding
{
  /// The grounded action, as it travels on `PlanItem.epistemic_action`.
  std::string epistemic_action;

  /// The events the planner gave the action, one per outcome.
  std::string outcome_when_clear;
  std::string outcome_when_blocked;
};

/// What a region means in the vocabulary of the loaded task.
///
/// A grounded task has flat atom names -- `blocked`, `at-junction_r1` -- so
/// the region has to say which atom its occupancy decides, and in which
/// direction. `blocked` with `atom_true_when_clear` false is the corridor of
/// the fleet tutorial; `clear_corridor` with it true is the other way of
/// writing the same thing, and both occur in real domains.
struct RegionVocabulary
{
  std::string atom;
  bool atom_true_when_clear{true};
  std::optional<SensingBinding> sensing;
};

/// One thing perception has to say to the epistemic state, or nothing.
struct Emission
{
  enum class Kind
  {
    Nothing,      ///< the region is undecided; there is nothing to report
    Announce,     ///< everyone learns it: restrict the model to these worlds
    ApplyAction,  ///< a sensing action in the plan observed this outcome
  };

  Kind kind{Kind::Nothing};

  std::string formula;  ///< Announce: the formula that is now known
  std::string action;   ///< ApplyAction: the grounded action that was executed
  std::string outcome;  ///< ApplyAction: the event that fired
};

/// The atom a region is about by default: `<predicate>_<region>`.
///
/// This is plank's grounding convention -- a predicate and its arguments join
/// with underscores, which is why `at-junction` over `r1` becomes
/// `at-junction_r1`. A region that does not follow it names its atom itself.
std::string default_atom(const std::string & region, const std::string & predicate = "clear");

/// The formula asserting what the grid found, or nothing when it found
/// nothing determinate.
std::optional<std::string> formula_for(RegionClass region_class, const RegionVocabulary & about);

/// The translation of one region's classification into one call on the state.
///
/// A region bound to a sensing action reports through that action, because
/// what it observed is that action's outcome and the model has an event model
/// for it. A region without one is announced: information that arrived outside
/// the plan -- an operator, or two robots reconciling their maps -- and a
/// public announcement is what the state offers for that.
Emission emission_for(RegionClass region_class, const RegionVocabulary & about);

}  // namespace plansys2

#endif  // PLANSYS2_EPISTEMIC_PERCEPTION__TRANSLATION_HPP_
