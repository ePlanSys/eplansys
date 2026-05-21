#pragma once
#include "plansys2_epistemic_planner/state.hpp"

// Compute the bisimulation contraction of s.
// Returns a smallest state bisimilar to s (same formula truth values).
// Call this after every product_update() to keep state sizes manageable.
// Takes s by value so callers can std::move into it without an extra copy.
EpistemicState bisim_contract(EpistemicState s);
