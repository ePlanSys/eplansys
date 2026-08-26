// Copyright 2026 Haniel Ulises Vasquez Morales
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
//
// Derived from the Aletheia epistemic planner, incorporated here as the
// in-process planning core of plansys2_epistemic_planner.
// Source: https://github.com/HanielUlises/Aletheia

#pragma once
#include "plansys2_epistemic_planner/state.hpp"

// Bisimulation contraction with canonical labelling.
//
// Returns the smallest state bisimilar to `s`, with its worlds numbered in an
// order that depends only on the model's structure — never on the numbering the
// caller happened to supply. Two bisimilar states therefore come back
// byte-identical, which is what makes fingerprint equality a sound test for
// "the planner has already seen this epistemic situation".
//
// Call after every product update. Takes by value so callers can std::move in.
[[nodiscard]] EpistemicState bisim_contract(EpistemicState s);
