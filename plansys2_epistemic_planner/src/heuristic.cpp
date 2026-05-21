#include "plansys2_epistemic_planner/heuristic.hpp"
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <queue>

float WorldCountHeuristic::operator()(const EpistemicState& s,
                                       const PlanningTask&) const {
    return static_cast<float>(s.designated.size());
}

static float count_unsatisfied(const EpistemicState& s, const Formula& f) {
    if (f.kind == FormulaKind::And) {
        float unsat = 0.0f;
        for (auto& c : f.children)
            if (!s.satisfies(*c)) unsat += 1.0f;
        return unsat;
    }
    return s.satisfies(f) ? 0.0f : 1.0f;
}

float UnsatisfiedGoalHeuristic::operator()(const EpistemicState& s,
                                            const PlanningTask& task) const {
    return count_unsatisfied(s, *task.goal);
}

static constexpr size_t MAX_SAMPLE = 64;
static constexpr size_t MAX_DEPTH  = 4;

struct HoldsKey {
    const Formula* formula;
    WorldIdx world;

    bool operator==(const HoldsKey& o) const {
        return formula == o.formula &&
               world == o.world;
    }
};

struct HoldsKeyHash {
    size_t operator()(const HoldsKey& k) const {
        size_t h1 =
            std::hash<const void*>()(
                static_cast<const void*>(k.formula)
            );

        size_t h2 =
            std::hash<size_t>()(k.world);

        return h1 ^ (h2 << 1);
    }
};

static bool cached_holds_at(
    const EpistemicState& s,
    const Formula& f,
    WorldIdx w,
    std::unordered_map<HoldsKey, bool, HoldsKeyHash>& cache
) {
    HoldsKey key{&f, w};

    auto it = cache.find(key);

    if (it != cache.end())
        return it->second;

    bool result = s.holds_at(f, w);

    cache.emplace(key, result);

    return result;
}

static std::unordered_set<WorldIdx>
project_designated(const EpistemicState& s,
                   const std::unordered_set<WorldIdx>& designated,
                   AgentIdx ag) {
    std::unordered_set<WorldIdx> reachable;

    if (ag >= s.accessibility.size())
        return reachable;

    for (WorldIdx w : designated) {
        if (w >= s.accessibility[ag].size())
            continue;

        for (WorldIdx v : s.accessibility[ag][w])
            reachable.insert(v);
    }

    return reachable;
}

// Forward declaration
static float epistemic_distance_for_conjunct(
    const EpistemicState& s,
    const std::unordered_set<WorldIdx>& designated,
    const Formula& f,
    size_t depth,
    std::unordered_map<HoldsKey, bool, HoldsKeyHash>& cache
);

// Build a projected epistemic state from agent ag's perspective:
// designated worlds = all worlds accessible from s.designated via R_ag.
// This lets us recurse into nested belief formulas.

static float epistemic_distance_for_conjunct(
    const EpistemicState& s,
    const std::unordered_set<WorldIdx>& designated,
    const Formula& f,
    size_t depth,
    std::unordered_map<HoldsKey, bool, HoldsKeyHash>& cache
) {
    bool satisfied = true;

    for (WorldIdx w : designated) {
        if (!cached_holds_at(s, f, w, cache)) {
            satisfied = false;
            break;
        }
    }

    if (satisfied) return 0.0f;

    if (f.kind == FormulaKind::Belief) {
        AgentIdx ag = f.agent;
        if (ag >= s.accessibility.size()) return 1.0f;

        const Formula& inner = *f.children[0];

        // If inner is itself a belief formula and we haven't hit the depth cap,
        // recurse by projecting the state through agent ag's accessibility.
        if (depth < MAX_DEPTH &&
            (inner.kind == FormulaKind::Belief ||
             inner.kind == FormulaKind::Common  ||
             inner.kind == FormulaKind::And     ||
             inner.kind == FormulaKind::Or)) {

            auto projected =
                project_designated(
                    s,
                    designated,
                    ag
                );

            if (projected.empty())
                return 1.0f;

            return epistemic_distance_for_conjunct(
                s,
                projected,
                inner,
                depth + 1,
                cache
            );
        }

        // Leaf belief formula: count counterexample worlds with sampling cap
        size_t counterexamples = 0;
        size_t sampled = 0;

        for (WorldIdx w : designated) {

            if (w >= s.accessibility[ag].size())
                continue;

            for (WorldIdx v : s.accessibility[ag][w]) {

                if (!cached_holds_at(s, inner, v, cache))
                    counterexamples++;

                sampled++;

                if (sampled >= MAX_SAMPLE) goto done;
            }
        }

        done:
        if (sampled == 0) return 0.0f;

        return static_cast<float>(counterexamples) /
               static_cast<float>(sampled);
    }

    // Common knowledge: project through union of group relations
    if (f.kind == FormulaKind::Common && !f.children.empty()) {
        if (depth >= MAX_DEPTH) return satisfied ? 0.0f : 1.0f;

        // Project through each agent in the group, take max distance
        float worst = 0.0f;

        for (AgentIdx ag : f.group) {

            auto projected =
                project_designated(
                    s,
                    designated,
                    ag
                );

            if (projected.empty())
                continue;

            float d =
                epistemic_distance_for_conjunct(
                    s,
                    projected,
                    *f.children[0],
                    depth + 1,
                    cache
                );

            worst = std::max(worst, d);
        }

        return worst;
    }

    // Conjunction: sum distances of unsatisfied conjuncts
    if (f.kind == FormulaKind::And) {
        float total = 0.0f;

        for (auto& c : f.children) {
            total += epistemic_distance_for_conjunct(
                s,
                designated,
                *c,
                depth,
                cache
            );
        }

        return total;
    }

    // Kw: [i]φ ∨ [i]¬φ — take the closer branch
    if (f.kind == FormulaKind::Or && f.children.size() == 2) {

        float d0 =
            epistemic_distance_for_conjunct(
                s,
                designated,
                *f.children[0],
                depth,
                cache
            );

        float d1 =
            epistemic_distance_for_conjunct(
                s,
                designated,
                *f.children[1],
                depth,
                cache
            );

        return std::min(d0, d1);
    }

    return satisfied ? 0.0f : 1.0f;
}

float EpistemicDistanceHeuristic::operator()(const EpistemicState& s,
                                              const PlanningTask& task) const {

    std::unordered_map<
        HoldsKey,
        bool,
        HoldsKeyHash
    > cache;

    const Formula& goal = *task.goal;

    if (goal.kind == FormulaKind::And) {
        float total = 0.0f;

        for (auto& c : goal.children) {
            total += epistemic_distance_for_conjunct(
                s,
                s.designated,
                *c,
                0,
                cache
            );
        }

        return total;
    }

    return epistemic_distance_for_conjunct(
        s,
        s.designated,
        goal,
        0,
        cache
    );
}

// Count worlds accessible from designated via agent ag where formula f fails.
// Returns value in [0, 1] — fraction of accessible worlds that are counterexamples.
static float kw_distance(const EpistemicState& s, AgentIdx ag, const Formula& f) {
    if (ag >= s.accessibility.size()) return 1.0f;

    size_t fails = 0, total = 0;

    for (WorldIdx w : s.designated) {

        if (w >= s.accessibility[ag].size())
            continue;

        for (WorldIdx v : s.accessibility[ag][w]) {

            if (!s.holds_at(f, v))
                fails++;

            total++;

            if (total >= MAX_SAMPLE) goto kw_done;
        }
    }

    kw_done:

    if (total == 0) return 0.0f;

    return static_cast<float>(fails) /
           static_cast<float>(total);
}

// Recursively flatten a Kw formula [i]φ ∨ [i]¬φ into its two belief branches
// and return the distance of the closer one — whichever direction
// the agent is closer to knowing.
static float kw_spread_conjunct(const EpistemicState& s, const Formula& f) {
    if (s.satisfies(f)) return 0.0f;

    // Kw.box expanded as Or{Belief, Belief}: take min branch distance
    if (f.kind == FormulaKind::Or && f.children.size() == 2) {
        float d0 = kw_spread_conjunct(s, *f.children[0]);
        float d1 = kw_spread_conjunct(s, *f.children[1]);
        return std::min(d0, d1);
    }

    if (f.kind == FormulaKind::Belief) {
        AgentIdx ag = f.agent;
        const Formula& inner = *f.children[0];
        return kw_distance(s, ag, inner);
    }

    // Conjunction of Kw formulas (group Kw.box expands to And)
    if (f.kind == FormulaKind::And) {
        float total = 0.0f;
        for (auto& c : f.children)
            total += kw_spread_conjunct(s, *c);
        return total;
    }

    return s.satisfies(f) ? 0.0f : 1.0f;
}

float KnowledgeSpreadHeuristic::operator()(const EpistemicState& s,
                                            const PlanningTask& task) const {
    const Formula& goal = *task.goal;

    if (goal.kind == FormulaKind::And) {
        float total = 0.0f;

        for (auto& c : goal.children)
            total += kw_spread_conjunct(s, *c);

        return total;
    }

    return kw_spread_conjunct(s, goal);
}