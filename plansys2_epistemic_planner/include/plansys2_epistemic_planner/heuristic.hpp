#pragma once
#include "plansys2_epistemic_planner/state.hpp"
#include "plansys2_epistemic_planner/task.hpp"

struct Heuristic {
    virtual ~Heuristic() = default;
    virtual float operator()(const EpistemicState& s,
                             const PlanningTask& task) const = 0;
};

// Returns true if `f` is, or has a top-level conjunct that is,
// a classical (non-epistemic) formula — i.e. Atom, Top, Bot, or a
// Not/And/Or that bottoms out in atoms without crossing a modal operator.
// A goal that returns false here is purely epistemic (KD45 belief operators
// with no bare atom conjunct) and should be routed to AO* + KnowledgeSpread.
static bool has_atom_conjunct(const Formula& f) {
    switch (f.kind) {
        case FormulaKind::Top:
        case FormulaKind::Bot:
        case FormulaKind::Atom:
            return true;

        case FormulaKind::Not:
            // ¬φ is classical iff φ is classical (no modal under negation)
            return !f.children.empty() && has_atom_conjunct(*f.children[0]);

        case FormulaKind::And:
            // Conjunction: classical if ANY conjunct is classical
            for (auto& c : f.children)
                if (has_atom_conjunct(*c)) return true;
            return false;

        case FormulaKind::Or:
            // Disjunction: classical only if ALL disjuncts are classical
            // (a disjunction of epistemic formulas is still epistemic)
            for (auto& c : f.children)
                if (!has_atom_conjunct(*c)) return false;
            return !f.children.empty();

        case FormulaKind::Belief:
        case FormulaKind::Common:
        case FormulaKind::Kw:
            // Modal operators — do not descend; this branch is epistemic
            return false;
    }
    return false;
}

// h1: number of designated worlds.
struct WorldCountHeuristic : Heuristic {
    float operator()(const EpistemicState& s,
                     const PlanningTask& task) const override;
};

// h2: number of goal conjuncts not yet satisfied.
struct UnsatisfiedGoalHeuristic : Heuristic {
    float operator()(const EpistemicState& s,
                     const PlanningTask& task) const override;
};

// h3: epistemic distance.
// For each unsatisfied belief conjunct [i]φ in the goal, counts the number
// of accessible worlds (from designated worlds) where φ fails.
// Gives a real gradient toward resolving epistemic uncertainty — unlike ug
// which only sees 0 or 1 per conjunct, ed sees how far each belief is from
// being true across the accessibility relation.
// For non-belief conjuncts falls back to ug (0 or 1).
// Combined: sum over all unsatisfied goal conjuncts.
struct EpistemicDistanceHeuristic : Heuristic {
    float operator()(const EpistemicState& s,
                     const PlanningTask& task) const override;
};

// h4: knowledge spread heuristic.
// Designed for domains where the goal is a conjunction of Kw formulas
// across multiple agents (e.g. Gossip). For each unsatisfied Kw conjunct,
// counts how many of the agent's accessible worlds still fail to resolve
// the formula. Sums across all conjuncts, giving a gradient as knowledge
// propagates through the agent graph.
struct KnowledgeSpreadHeuristic : Heuristic {
    float operator()(const EpistemicState& s,
                     const PlanningTask& task) const override;
};