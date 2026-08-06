#include "plansys2_epistemic_planner/action.hpp"
#include "plansys2_epistemic_planner/state.hpp"

// Applicability reduces to two set tests once preconditions are evaluated as
// extensions over the whole model. The previous implementation looped over
// designated worlds calling holds_at per world, re-descending the precondition
// each time; here sat(pre(e)) is computed once and memoised on the state, so
// the product update that follows reuses it.

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
