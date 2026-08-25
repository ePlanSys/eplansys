// Copyright 2026 Haniel Ulises Vasquez Morales
//
// Derived from the Aletheia epistemic planner, incorporated here as the
// in-process planning core of plansys2_epistemic_planner.
//
//     Source: https://github.com/HanielUlises/Aletheia
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

#include "plansys2_epistemic_planner/action.hpp"
#include "plansys2_epistemic_planner/state.hpp"

// Applicability reduces to two set tests once preconditions are evaluated as
// extensions over the whole model. sat(pre(e)) is computed once and memoised on
// the state, and the product update that follows reuses it; evaluating the
// precondition per designated world would instead re-descend the formula once
// for each world tested.

// Strong, conformant applicability.
//
// Ontic actions (|E_d| = 1) require W* ⊆ sat(pre(e)). An ontic action fired
// when only some designated worlds satisfy the precondition would silently drop
// the others from W'*, producing a state that conflates partial execution with
// full execution and can report spurious goal satisfaction.
//
// Sensing actions (|E_d| ≥ 2) require only W* ∩ sat(pre(e)) ≠ ∅ for some
// designated event. They branch through product_update_split, so worlds where
// different events fire land in separate subtrees and no conflation occurs.
bool Action::applicable(const EpistemicState& s) const {
    if (designated_events.empty()) return false;
    if (bits::empty(s.designated_bits())) return false;

    if (is_ontic()) {
        const EventIdx eid = *designated_events.begin();
        if (eid >= events.size()) return false;
        return bits::subset_of(s.designated_bits(),
                               s.sat(*events[eid].precondition));
    }

    return applicable_weak(s);
}

// Existential applicability, used only to shortlist actions for heuristic
// ranking. Never decides whether a successor is generated.
bool Action::applicable_weak(const EpistemicState& s) const {
    if (designated_events.empty()) return false;

    for (EventIdx eid : designated_events) {
        if (eid >= events.size()) continue;
        if (bits::intersects(s.designated_bits(),
                             s.sat(*events[eid].precondition)))
            return true;
    }
    return false;
}
