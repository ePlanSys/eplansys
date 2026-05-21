#include "plansys2_epistemic_planner/action.hpp"
#include "plansys2_epistemic_planner/state.hpp"

// applicable: strong conformant-planning applicability.
//
// For ontic actions (single designated event):
//   Every designated world must satisfy the precondition of the designated event.
//   An ontic action applied when some designated worlds fail the precondition
//   would silently prune those worlds from the successor's designated set,
//   making the goal appear satisfied in a state that only represents the
//   subset of actual worlds where the action could execute — which is unsound.
//
// For sensing actions (multiple designated events):
//   At least one (designated world, designated event) pair must satisfy the
//   precondition. Sensing actions use product_update_split which partitions
//   the successor by event; worlds where a given event's precondition fails
//   simply don't appear in that event's branch — this is the intended semantics.
bool Action::applicable(const EpistemicState& s) const {
    if (designated_events.empty()) return false;

    if (is_ontic()) {
        // ∀ w ∈ W* : M,w ⊨ pre(e)  for the single designated event e
        EventIdx eid = *designated_events.begin();
        if (eid >= events.size()) return false;
        const FormulaPtr& pre = events[eid].precondition;
        for (WorldIdx w : s.designated)
            if (!s.holds_at(*pre, w)) return false;
        return !s.designated.empty();
    }

    // Sensing: ∃ (w,e) ∈ W* × E_d : M,w ⊨ pre(e)
    for (EventIdx eid : designated_events) {
        if (eid >= events.size()) continue;
        for (WorldIdx w : s.designated)
            if (s.holds_at(*events[eid].precondition, w))
                return true;
    }
    return false;
}

// applicable_weak: existential check used for heuristic ranking in AO* only.
// Never used to decide whether a successor is actually generated.
bool Action::applicable_weak(const EpistemicState& s) const {
    if (designated_events.empty()) return false;
    for (EventIdx eid : designated_events) {
        if (eid >= events.size()) continue;
        for (WorldIdx w : s.designated)
            if (s.holds_at(*events[eid].precondition, w))
                return true;
    }
    return false;
}