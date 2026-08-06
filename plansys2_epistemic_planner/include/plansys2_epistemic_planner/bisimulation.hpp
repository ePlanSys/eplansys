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
