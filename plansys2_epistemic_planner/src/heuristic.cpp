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

#include "plansys2_epistemic_planner/heuristic.hpp"

#include <algorithm>
#include <vector>

// Heuristics over satisfaction sets.
//
// Every heuristic here is a goal-decomposition estimate: it measures how far
// each unsatisfied goal conjunct is from holding, either as a 0/1 flag or as a
// fraction of the accessible worlds that still act as counterexamples.
//
// Determinism is a required property, not an incidental one. Both `ed` and `ks`
// truncate their counterexample scan after a fixed number of accessible worlds,
// so the estimate depends on which worlds fall inside the sample; were the
// sample drawn in an unspecified order, the heuristic value — and therefore the
// plan returned — would vary between runs over an identical task. Traversal is
// over bit sets in ascending world index, which fixes the sample and makes the
// estimate a function of the state alone.
//
// Cost is one bottom-up evaluation of the goal per state, shared across
// conjuncts through the state's satisfaction cache, rather than one recursive
// descent per conjunct.

namespace {

// Number of accessible worlds examined before a counterexample count is
// truncated. Bounds the cost of `ed` and `ks` on wide models.
constexpr std::size_t kMaxSample = 64;

// Depth cap on projection through nested modalities.
constexpr std::size_t kMaxDepth = 4;

using WordVec = std::vector<bits::Word>;

// The worlds agent `ag` considers possible from anywhere in `designated`:
// ⋃ { R_ag(w) | w ∈ designated }.
WordVec project(const EpistemicState& s, bits::ConstWordSpan designated, AgentIdx ag) {
    WordVec out(s.rel_words, 0);
    if (ag >= s.num_agents) return out;
    bits::for_each(designated,
                   [&](std::uint32_t w) { bits::or_into(out, s.succ(ag, w)); });
    return out;
}

// Fraction of the worlds accessible from `designated` via `ag` at which `inner`
// fails, sampled in ascending world order and truncated at kMaxSample.
float counterexample_ratio(const EpistemicState& s, bits::ConstWordSpan designated,
                           AgentIdx ag, const Formula& inner) {
    if (ag >= s.num_agents) return 1.0f;

    const auto ext = s.sat(inner);

    std::size_t fails = 0, sampled = 0;
    bits::for_each_until(designated, [&](std::uint32_t w) {
        return bits::for_each_until(s.succ(ag, w), [&](std::uint32_t v) {
            if (!bits::test(ext, v)) ++fails;
            return ++sampled < kMaxSample;
        });
    });

    if (sampled == 0) return 0.0f;
    return static_cast<float>(fails) / static_cast<float>(sampled);
}

[[nodiscard]] bool holds_throughout(const EpistemicState& s,
                                    bits::ConstWordSpan designated,
                                    const Formula& f) {
    return bits::subset_of(designated, s.sat(f));
}

} // namespace

// h1 — number of designated worlds.
float WorldCountHeuristic::operator()(const EpistemicState& s,
                                      const PlanningTask&) const {
    return static_cast<float>(s.num_designated());
}

// h2 — number of unsatisfied top-level goal conjuncts.
float UnsatisfiedGoalHeuristic::operator()(const EpistemicState& s,
                                           const PlanningTask& task) const {
    const Formula& goal = *task.goal;
    const auto designated = s.designated_bits();

    if (goal.kind == FormulaKind::And) {
        float unsat = 0.0f;
        for (const auto& c : goal.children)
            if (!holds_throughout(s, designated, *c)) unsat += 1.0f;
        return unsat;
    }
    return holds_throughout(s, designated, goal) ? 0.0f : 1.0f;
}

// h3 — epistemic distance.
//
// For a belief conjunct [i]φ, instead of the 0/1 verdict `ug` gives, this counts
// what fraction of the worlds agent i considers possible are counterexamples to
// φ — a real gradient as uncertainty is resolved. Nested modalities are handled
// by projecting the designated set through the accessibility relation and
// recursing, up to kMaxDepth.
namespace {

float epistemic_distance(const EpistemicState& s, bits::ConstWordSpan designated,
                         const Formula& f, std::size_t depth) {
    if (holds_throughout(s, designated, f)) return 0.0f;

    switch (f.kind) {
    case FormulaKind::Belief: {
        if (f.agent >= s.num_agents) return 1.0f;
        const Formula& inner = *f.children[0];

        const bool nested = inner.kind == FormulaKind::Belief ||
                            inner.kind == FormulaKind::Common ||
                            inner.kind == FormulaKind::And    ||
                            inner.kind == FormulaKind::Or;

        if (depth < kMaxDepth && nested) {
            const WordVec projected = project(s, designated, f.agent);
            if (bits::empty(projected)) return 1.0f;
            return epistemic_distance(s, projected, inner, depth + 1);
        }

        return counterexample_ratio(s, designated, f.agent, inner);
    }

    case FormulaKind::Common: {
        if (f.children.empty()) return 1.0f;
        if (depth >= kMaxDepth) return 1.0f;

        // Worst agent in the group: common knowledge is no closer than its
        // furthest constituent.
        float worst = 0.0f;
        for (AgentIdx ag : f.group) {
            const WordVec projected = project(s, designated, ag);
            if (bits::empty(projected)) continue;
            worst = std::max(worst,
                             epistemic_distance(s, projected, *f.children[0], depth + 1));
        }
        return worst;
    }

    case FormulaKind::And: {
        float total = 0.0f;
        for (const auto& c : f.children)
            total += epistemic_distance(s, designated, *c, depth);
        return total;
    }

    case FormulaKind::Or: {
        // Covers Kw expanded as [i]φ ∨ [i]¬φ: credit the nearer disjunct.
        if (f.children.size() != 2) break;
        return std::min(epistemic_distance(s, designated, *f.children[0], depth),
                        epistemic_distance(s, designated, *f.children[1], depth));
    }

    case FormulaKind::Kw: {
        if (f.agent >= s.num_agents) return 1.0f;
        const Formula& inner = *f.children[0];
        // Distance to knowing φ, or to knowing ¬φ, whichever is nearer.
        const float to_true  = counterexample_ratio(s, designated, f.agent, inner);
        return std::min(to_true, 1.0f - to_true);
    }

    default:
        break;
    }

    return 1.0f;   // unsatisfied and structurally opaque
}

} // namespace

float EpistemicDistanceHeuristic::operator()(const EpistemicState& s,
                                             const PlanningTask& task) const {
    const Formula& goal = *task.goal;
    const auto designated = s.designated_bits();

    if (goal.kind == FormulaKind::And) {
        float total = 0.0f;
        for (const auto& c : goal.children)
            total += epistemic_distance(s, designated, *c, 0);
        return total;
    }
    return epistemic_distance(s, designated, goal, 0);
}

// h4 — knowledge spread.
//
// Aimed at goals that are conjunctions of Kw formulas across agents (Gossip,
// Grapevine). Each unsatisfied conjunct contributes the fraction of the agent's
// accessible worlds that still fail to resolve it, so the value falls smoothly
// as knowledge propagates through the agent graph rather than dropping in
// whole-conjunct steps.
namespace {

float knowledge_spread(const EpistemicState& s, const Formula& f) {
    const auto designated = s.designated_bits();
    if (holds_throughout(s, designated, f)) return 0.0f;

    switch (f.kind) {
    case FormulaKind::Or:
        // Kw.box expanded to [i]φ ∨ [i]¬φ: whichever direction is nearer.
        if (f.children.size() == 2)
            return std::min(knowledge_spread(s, *f.children[0]),
                            knowledge_spread(s, *f.children[1]));
        break;

    case FormulaKind::Kw: {
        const float to_true =
            counterexample_ratio(s, designated, f.agent, *f.children[0]);
        return std::min(to_true, 1.0f - to_true);
    }

    case FormulaKind::Belief:
        return counterexample_ratio(s, designated, f.agent, *f.children[0]);

    case FormulaKind::And: {
        float total = 0.0f;
        for (const auto& c : f.children) total += knowledge_spread(s, *c);
        return total;
    }

    default:
        break;
    }

    return 1.0f;
}

} // namespace

float KnowledgeSpreadHeuristic::operator()(const EpistemicState& s,
                                           const PlanningTask& task) const {
    const Formula& goal = *task.goal;

    if (goal.kind == FormulaKind::And) {
        float total = 0.0f;
        for (const auto& c : goal.children) total += knowledge_spread(s, *c);
        return total;
    }
    return knowledge_spread(s, goal);
}

// h5 / h6 — relaxed announcement closure
//
// See heuristic.hpp for the relaxation. The loop below is a fixpoint over a
// shrinking world set: at each layer every designated event of every action
// contributes its precondition extension as a pruning constraint, all of them
// are applied at once, and the layer at which each goal conjunct first becomes
// true is recorded.
//
// Cost is bounded by the fact that the world set strictly shrinks whenever a
// layer makes progress, so there are at most |W| layers; kMaxLayers caps it
// further for the pathological case.

namespace {

constexpr std::size_t kMaxLayers = 32;

// Top-level conjuncts of the goal, which is how every heuristic here decomposes
// it. A non-conjunctive goal is treated as a single conjunct.
std::vector<const Formula*> goal_conjuncts(const Formula& goal) {
    std::vector<const Formula*> out;
    if (goal.kind == FormulaKind::And) {
        out.reserve(goal.children.size());
        for (const auto& c : goal.children) out.push_back(c.get());
    } else {
        out.push_back(&goal);
    }
    return out;
}

// Does agent `ag` tell events e and f apart under action `a`?
//
// Optimistically: true if *any* observability case distinguishes them. The real
// update picks the first case whose guard holds at the source world, which is
// world-dependent; taking the union over cases can only make knowledge easier
// to acquire, which is the direction a relaxation must err in.
//
// An agent with no observability cases falls back, in the real update, to full
// uncertainty (R^E_i(e) = E), and so distinguishes nothing.
bool distinguishes(const Action& a, AgentIdx ag, EventIdx e, EventIdx f) {
    if (ag >= a.obs_cases.size()) return false;
    for (const ObsCase& c : a.obs_cases[ag]) {
        if (e >= c.relation.size()) continue;
        if (!c.relation[e].count(f)) return true;
    }
    return false;
}

// One relaxed step, applied in place. Returns true if anything changed.
//
// Two monotone effects are modelled, both of which only ever remove structure:
//
//   Worlds. Each designated event contributes its precondition extension as a
//   pruning constraint — the worlds an announcement of that event eliminates.
//   The event is skipped when it is inconsistent with W* (it could not have
//   fired), when it prunes nothing, or when applying it would empty W*: the
//   relaxation may discard possible worlds freely, but a model with no actual
//   world represents no situation at all.
//
//   Edges. This is what the world-elimination-only version missed, and why it
//   was flat on private announcements. When agent i can tell event e from event
//   f, the product update leaves no R_i edge from a world where pre(e) held to
//   one where pre(f) held: i has observed which of the two occurred, so those
//   worlds are no longer mutually accessible for i. Relaxed, that is
//
//       R_i ← R_i \ ( sat(pre(e)) × sat(pre(f)) )
//
//   applied for every distinguishing agent and every ordered pair of designated
//   events. Gossip and Grapevine make progress entirely through this term —
//   their announcements have trivial preconditions and eliminate no worlds at
//   all, so a relaxation that only prunes worlds reaches its fixpoint at layer
//   zero and reports nothing.
bool relaxed_step(EpistemicState& m, const PlanningTask& task) {
    const std::uint32_t nw = m.num_worlds;

    std::vector<bits::Word> keep(m.rel_words, 0);
    bits::fill_all(keep, nw);

    bool progress = false;

    // Extensions are copied out because several are held live at once and the
    // model is mutated below, which invalidates the state's satisfaction cache.
    std::vector<bits::Word> ext_e, ext_f;

    for (const Action& a : task.actions) {
        std::vector<EventIdx> events(a.designated_events.begin(),
                                     a.designated_events.end());
        std::sort(events.begin(), events.end());

        // Edge cuts.
        for (EventIdx e : events) {
            if (e >= a.events.size()) continue;
            m.sat_copy(*a.events[e].precondition, ext_e);
            if (bits::empty(ext_e)) continue;

            for (EventIdx f : events) {
                if (f == e || f >= a.events.size()) continue;
                m.sat_copy(*a.events[f].precondition, ext_f);
                if (bits::empty(ext_f)) continue;

                for (AgentIdx ag = 0; ag < m.num_agents; ++ag) {
                    if (!distinguishes(a, ag, e, f)) continue;

                    bits::for_each(ext_e, [&](std::uint32_t w) {
                        auto row = m.succ(ag, w);
                        if (!bits::intersects(row, ext_f)) return;
                        bits::andnot_into(row, ext_f);
                        progress = true;
                    });
                }
            }
        }
    }

    if (progress) m.invalidate();

    // World prunes.
    const auto designated = m.designated_bits();
    for (const Action& a : task.actions) {
        for (EventIdx e : a.designated_events) {
            if (e >= a.events.size()) continue;

            m.sat_copy(*a.events[e].precondition, ext_e);

            if (!bits::intersects(ext_e, designated)) continue;
            if (bits::subset_of(keep, ext_e))          continue;

            std::vector<bits::Word> cand = keep;
            bits::and_into(cand, ext_e);
            if (!bits::intersects(cand, designated)) continue;

            keep     = std::move(cand);
            progress = true;
        }
    }

    if (bits::count(keep) != nw) {
        std::vector<WorldIdx> remap;
        m = restrict_state(m, keep, remap);
    }

    return progress;
}

} // namespace

float RelaxedClosureHeuristic::operator()(const EpistemicState& s,
                                          const PlanningTask& task) const {
    const std::vector<const Formula*> conjuncts = goal_conjuncts(*task.goal);

    // level[i] = layer at which conjunct i first held, or npos if never.
    constexpr std::size_t npos = static_cast<std::size_t>(-1);
    std::vector<std::size_t> level(conjuncts.size(), npos);
    std::size_t remaining = conjuncts.size();

    EpistemicState model = s;
    std::size_t    layer = 0;

    for (;;) {
        for (std::size_t i = 0; i < conjuncts.size(); ++i) {
            if (level[i] != npos) continue;
            if (model.satisfies(*conjuncts[i])) { level[i] = layer; --remaining; }
        }
        if (remaining == 0) break;
        if (layer >= kMaxLayers) break;

        if (!relaxed_step(model, task)) break;   // fixpoint: nothing more to remove
        ++layer;
    }

    // Conjuncts the closure never resolved sit one layer past the horizon, with
    // a residual in [0,1) from the current state so the estimate still has a
    // gradient rather than collapsing to a constant.
    const auto score = [&](std::size_t i) -> float {
        if (level[i] != npos) return static_cast<float>(level[i]);
        const float residual =
            std::min(1.0f, epistemic_distance(s, s.designated_bits(), *conjuncts[i], 0));
        return static_cast<float>(layer + 1) + residual;
    };

    if (agg_ == RelaxedAggregation::Max) {
        float worst = 0.0f;
        for (std::size_t i = 0; i < conjuncts.size(); ++i)
            worst = std::max(worst, score(i));
        return worst;
    }

    float total = 0.0f;
    for (std::size_t i = 0; i < conjuncts.size(); ++i) total += score(i);
    return total;
}
