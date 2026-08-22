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

#include "gtest/gtest.h"
#include "plansys2_epistemic_perception/translation.hpp"

namespace
{

plansys2::RegionVocabulary clear_corridor()
{
  plansys2::RegionVocabulary about;
  about.atom = "clear_corridor";
  about.atom_true_when_clear = true;
  return about;
}

/// The fleet tutorial's way round: the atom names the obstruction.
plansys2::RegionVocabulary blocked()
{
  plansys2::RegionVocabulary about;
  about.atom = "blocked";
  about.atom_true_when_clear = false;
  return about;
}

}  // namespace

TEST(translation, an_atom_follows_the_grounding_convention)
{
  EXPECT_EQ(plansys2::default_atom("corridor"), "clear_corridor");
  EXPECT_EQ(plansys2::default_atom("corridor", "passable"), "passable_corridor");
}

TEST(translation, a_clear_region_asserts_its_atom)
{
  const auto formula = plansys2::formula_for(plansys2::RegionClass::Clear, clear_corridor());

  ASSERT_TRUE(formula.has_value());
  EXPECT_EQ(*formula, "clear_corridor");
}

TEST(translation, a_blocked_region_denies_its_atom)
{
  const auto formula = plansys2::formula_for(plansys2::RegionClass::Blocked, clear_corridor());

  ASSERT_TRUE(formula.has_value());
  EXPECT_EQ(*formula, "(not clear_corridor)");
}

TEST(translation, an_atom_that_names_the_obstruction_points_the_other_way)
{
  const auto when_clear = plansys2::formula_for(plansys2::RegionClass::Clear, blocked());
  const auto when_blocked = plansys2::formula_for(plansys2::RegionClass::Blocked, blocked());

  ASSERT_TRUE(when_clear.has_value());
  ASSERT_TRUE(when_blocked.has_value());
  EXPECT_EQ(*when_clear, "(not blocked)");
  EXPECT_EQ(*when_blocked, "blocked");
}

TEST(translation, an_undecided_region_asserts_nothing)
{
  EXPECT_FALSE(plansys2::formula_for(plansys2::RegionClass::Unknown, clear_corridor()));

  const auto emission = plansys2::emission_for(plansys2::RegionClass::Unknown, clear_corridor());
  EXPECT_EQ(emission.kind, plansys2::Emission::Kind::Nothing);
}

TEST(translation, a_region_without_a_sensing_action_is_announced)
{
  const auto emission = plansys2::emission_for(plansys2::RegionClass::Clear, clear_corridor());

  EXPECT_EQ(emission.kind, plansys2::Emission::Kind::Announce);
  EXPECT_EQ(emission.formula, "clear_corridor");
}

TEST(translation, a_region_bound_to_a_sensing_action_reports_through_it)
{
  auto about = blocked();
  about.sensing = plansys2::SensingBinding{"inspect_r1", "e-inspect-clear", "e-inspect-blocked"};

  const auto when_clear = plansys2::emission_for(plansys2::RegionClass::Clear, about);
  const auto when_blocked = plansys2::emission_for(plansys2::RegionClass::Blocked, about);

  EXPECT_EQ(when_clear.kind, plansys2::Emission::Kind::ApplyAction);
  EXPECT_EQ(when_clear.action, "inspect_r1");
  EXPECT_EQ(when_clear.outcome, "e-inspect-clear");

  EXPECT_EQ(when_blocked.kind, plansys2::Emission::Kind::ApplyAction);
  EXPECT_EQ(when_blocked.outcome, "e-inspect-blocked");
}

TEST(translation, a_sensing_action_missing_the_event_for_what_was_seen_says_nothing)
{
  auto about = blocked();
  about.sensing = plansys2::SensingBinding{"inspect_r1", "", "e-inspect-blocked"};

  const auto emission = plansys2::emission_for(plansys2::RegionClass::Clear, about);

  // Applying the other outcome would be a lie about what the robot saw.
  EXPECT_EQ(emission.kind, plansys2::Emission::Kind::Nothing);
}

TEST(translation, a_region_with_no_atom_at_all_says_nothing)
{
  plansys2::RegionVocabulary about;

  EXPECT_FALSE(plansys2::formula_for(plansys2::RegionClass::Clear, about));
  EXPECT_EQ(
    plansys2::emission_for(plansys2::RegionClass::Clear, about).kind,
    plansys2::Emission::Kind::Nothing);
}
