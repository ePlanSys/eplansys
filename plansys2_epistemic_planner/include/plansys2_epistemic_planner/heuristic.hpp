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
[[nodiscard]] inline bool has_atom_conjunct(const Formula& f) {
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

// h5 / h6: relaxed announcement closure.
//
// The four heuristics above are all goal counting. None of them solves a
// relaxed problem, so none of them estimates a *distance*: they measure how
// wrong the current state is, not how much work remains. On any domain where
// several actions must be chained before the first goal conjunct becomes true,
// they are flat.
//
// The relaxation here is specific to epistemic planning. In classical planning
// one relaxes by ignoring delete effects, because progress is monotone growth
// of a fact set. In DEL, the actions that establish knowledge are announcements
// and sensing, which carry no ontic effect at all — they make progress by
// *eliminating worlds* an agent considers possible. The monotone quantity is
// therefore the world set, shrinking rather than growing, and the natural
// relaxation is to apply every eliminating action at once and ignore the
// interference between them:
//
//   W_0 = W
//   W_{k+1} = W_k ∩ ⋂ { sat_{M|W_k}(pre(e)) : a ∈ A, e ∈ E_d(a) }
//
// where an event is skipped if including it would empty W*. Each layer costs
// one action in the relaxed plan, so the layer at which a goal conjunct first
// becomes true is a lower bound on the number of steps needed to establish it —
// the epistemic analogue of a relaxed planning graph's fact levels.
//
// The closure ignores three things, and is a lower bound because of it: that
// announcements have preconditions which must themselves be established, that
// events which prune well in the relaxation may be inconsistent with the actual
// world, and that ontic effects can restore uncertainty. It costs
// O(L · |A| · |E| · |φ| · |W|²/64) with L ≤ |W| layers, which the bit-matrix
// representation makes cheap enough to run at every node.
//
// Two aggregations over the per-conjunct levels, as with h_max and h_add:
//
//   rpg   max over goal conjuncts   — lower bound, safe under-estimate
//   radd  sum over goal conjuncts   — assumes conjuncts are independent;
//                                     over-estimates when they share work, but
//                                     far better guided on conjunctive goals
//                                     such as Gossip and Grapevine
//
// Conjuncts the closure cannot resolve fall back to a residual epistemic
// distance offset past the last layer, so the estimate degrades to a gradient
// rather than to a constant.
enum class RelaxedAggregation { Max, Add };

struct RelaxedClosureHeuristic : Heuristic {
    explicit RelaxedClosureHeuristic(RelaxedAggregation agg) : agg_(agg) {}

    float operator()(const EpistemicState& s,
                     const PlanningTask& task) const override;

private:
    RelaxedAggregation agg_;
};