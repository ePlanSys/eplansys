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

#include "plansys2_epistemic_perception/translation.hpp"

#include <optional>
#include <string>

namespace plansys2
{

std::string default_atom(const std::string & region, const std::string & predicate)
{
  return predicate + "_" + region;
}

std::optional<std::string> formula_for(RegionClass region_class, const RegionVocabulary & about)
{
  if (about.atom.empty() || region_class == RegionClass::Unknown) {
    return std::nullopt;
  }

  const bool holds = (region_class == RegionClass::Clear) == about.atom_true_when_clear;

  return holds ? about.atom : "(not " + about.atom + ")";
}

Emission emission_for(RegionClass region_class, const RegionVocabulary & about)
{
  Emission emission;

  if (region_class == RegionClass::Unknown) {
    return emission;
  }

  if (about.sensing.has_value()) {
    const auto & sensing = *about.sensing;
    const auto & outcome = region_class == RegionClass::Clear ?
      sensing.outcome_when_clear : sensing.outcome_when_blocked;

    // An action with no event for what was seen cannot be advanced by it. That
    // is a mis-configuration rather than an observation, and saying nothing is
    // better than applying the other outcome.
    if (sensing.epistemic_action.empty() || outcome.empty()) {
      return emission;
    }

    emission.kind = Emission::Kind::ApplyAction;
    emission.action = sensing.epistemic_action;
    emission.outcome = outcome;
    return emission;
  }

  const auto formula = formula_for(region_class, about);
  if (!formula.has_value()) {
    return emission;
  }

  emission.kind = Emission::Kind::Announce;
  emission.formula = *formula;
  return emission;
}

}  // namespace plansys2
