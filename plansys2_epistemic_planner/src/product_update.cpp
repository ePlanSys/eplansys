#include "plansys2_epistemic_planner/product_update.hpp"

#include <algorithm>
#include <vector>

// ─────────────────────────────────────────────────────────────────────────────
// DEL product update  M ⊗ A
//
// Given M = (W, {R_i}, V, W*) and an event model A = (E, {R^E_i}, pre, post, E_d):
//
//   W'          = { (w,e) | w ∈ W, e ∈ E, M,w ⊨ pre(e) }
//   R'_i        = { ((w,e),(v,f)) | (w,v) ∈ R_i ∧ (e,f) ∈ R^E_i }
//   V'(w,e)(p)  = post(e)(p) if p ∈ dom(post(e)), else V(w)(p)
//   W'*         = { (w,e) | w ∈ W*, e ∈ E_d }
//
// Postconditions are conditional: p becomes true at (w,e) iff post_true[p] holds
// at w *in the pre-update model* (symmetrically for post_false). Atoms in
// neither map are inherited.
//
// R^E_i is not fixed: each agent carries an ordered list of (condition, event
// relation) cases, and the first case whose condition holds at w supplies that
// agent's event relation there. If no case matches the agent is treated as
// fully observing, R^E_i(e) = E, which over-approximates uncertainty and is
// therefore safe.
//
// ── What changed ────────────────────────────────────────────────────────────
//
// Three costs dominated the previous implementation, all of them removed here.
//
// 1. Preconditions, postcondition guards and observability conditions were
//    evaluated pointwise with holds_at(φ, w), once per (world, event) or per
//    (world, event, agent) triple. They are now evaluated once each as
//    extensions over the whole source model, and every later test is a bit
//    lookup. Observability conditions in particular depend only on w, never on
//    e, so the old placement inside the pair loop repeated each evaluation |E|
//    times over.
//
// 2. The (w,e) → world table was a hash map probed from the innermost loop of
//    the relation construction. It is now a flat array.
//
// 3. Iteration ran over the hash map, visiting source worlds in essentially
//    random order. It now runs in index order, so the source model's
//    accessibility rows are walked sequentially.
// ─────────────────────────────────────────────────────────────────────────────

namespace {

// Precomputed, model-wide extensions of everything the update needs to test.
struct EventPrecomputation {
    std::vector<std::vector<bits::Word>> pre;          // [event]      → sat(pre(e))
    std::vector<std::vector<std::pair<AtomIdx, std::vector<bits::Word>>>> post_true;
    std::vector<std::vector<std::pair<AtomIdx, std::vector<bits::Word>>>> post_false;

    // obs_case[agent][world] = index of the first matching observability case,
    // or -1 for the fully-observant fallback.
    std::vector<std::int32_t> obs_case;
};

EventPrecomputation precompute(const EpistemicState& s, const Action& a) {
    const std::uint32_t ne = static_cast<std::uint32_t>(a.events.size());
    const std::uint32_t nw = s.num_worlds;
    const std::uint32_t na = s.num_agents;

    EventPrecomputation p;
    p.pre.resize(ne);
    p.post_true.resize(ne);
    p.post_false.resize(ne);

    for (std::uint32_t e = 0; e < ne; ++e) {
        const Event& ev = a.events[e];
        s.sat_copy(*ev.precondition, p.pre[e]);

        p.post_true[e].reserve(ev.post_true.size());
        for (const auto& [atom, cond] : ev.post_true) {
            std::vector<bits::Word> ext;
            s.sat_copy(*cond, ext);
            p.post_true[e].emplace_back(atom, std::move(ext));
        }
        p.post_false[e].reserve(ev.post_false.size());
        for (const auto& [atom, cond] : ev.post_false) {
            std::vector<bits::Word> ext;
            s.sat_copy(*cond, ext);
            p.post_false[e].emplace_back(atom, std::move(ext));
        }
    }

    // One extension per observability condition, then a single pass to pick the
    // first match per world. Conditions repeat heavily across agents and
    // actions, and formula interning makes every repeat a memo hit.
    p.obs_case.assign(std::size_t(na) * nw, -1);
    for (AgentIdx ag = 0; ag < na && ag < a.obs_cases.size(); ++ag) {
        const auto& cases = a.obs_cases[ag];
        if (cases.empty()) continue;

        std::vector<std::vector<bits::Word>> ext(cases.size());
        for (std::size_t c = 0; c < cases.size(); ++c)
            s.sat_copy(*cases[c].condition, ext[c]);

        for (WorldIdx w = 0; w < nw; ++w) {
            for (std::size_t c = 0; c < cases.size(); ++c) {
                if (bits::test(ext[c], w)) {
                    p.obs_case[std::size_t(ag) * nw + w] = static_cast<std::int32_t>(c);
                    break;
                }
            }
        }
    }

    return p;
}

// ── KD45 seriality repair ───────────────────────────────────────────────────
//
// KD45 requires every R_i to be serial: ∀w ∃v. w R_i v. The product does not
// preserve seriality — (w,e) is non-serial for agent i whenever R_i(w) = ∅ or
// R^E_i(e) = ∅ — and removal cascades, because a removed world may have been
// some other world's only successor. The set of serial worlds is the greatest
// fixpoint of "every agent's row, restricted to survivors, is non-empty", which
// this computes by repeated sweeps.
//
// Returns the surviving set, or an empty set if repair emptied W*.
std::vector<bits::Word> serial_core(const EpistemicState& s) {
    std::vector<bits::Word> alive(s.rel_words, 0);
    bits::fill_all(alive, s.num_worlds);

    for (bool changed = true; changed;) {
        changed = false;
        for (WorldIdx w = 0; w < s.num_worlds; ++w) {
            if (!bits::test(alive, w)) continue;
            for (AgentIdx ag = 0; ag < s.num_agents; ++ag) {
                if (!bits::intersects(s.succ(ag, w), alive)) {
                    bits::reset(alive, w);
                    changed = true;
                    break;
                }
            }
        }
    }
    return alive;
}

} // namespace

Outcome<ProductUpdateResult>
product_update_with_map(const EpistemicState& s, const Action& a,
                        bool enforce_kd45, const WorldCapPolicy& cap) {
    const std::uint32_t nw = s.num_worlds;
    const std::uint32_t na = s.num_agents;
    const std::uint32_t ne = static_cast<std::uint32_t>(a.events.size());

    // Pessimistic bound on |W'|, checked before any allocation.
    if (!cap.allows(nw, ne))
        return pruned<ProductUpdateResult>(PruneReason::WorldCapExceeded);

    const EventPrecomputation p = precompute(s, a);

    // ── W' ──────────────────────────────────────────────────────────────────
    ProductUpdateResult out;
    out.num_events = ne;
    out.pair_to_idx.assign(std::size_t(nw) * ne, kNoWorld);

    std::vector<std::uint64_t> surviving;    // packed (w,e), in construction order
    surviving.reserve(std::size_t(nw) * ne / 2 + 1);

    for (WorldIdx w = 0; w < nw; ++w) {
        for (EventIdx e = 0; e < ne; ++e) {
            if (!bits::test(p.pre[e], w)) continue;
            out.pair_to_idx[std::size_t(w) * ne + e] =
                static_cast<WorldIdx>(surviving.size());
            surviving.push_back((std::uint64_t(w) << 32) | e);
        }
    }

    if (surviving.empty())
        return pruned<ProductUpdateResult>(PruneReason::Inapplicable);

    EpistemicState result;
    result.allocate(static_cast<std::uint32_t>(surviving.size()),
                    s.num_atoms, na);

    for (std::size_t i = 0; i < surviving.size(); ++i) {
        const auto w = static_cast<WorldIdx>(surviving[i] >> 32);
        const auto e = static_cast<EventIdx>(surviving[i] & 0xFFFFFFFFu);

        auto dst = result.val(static_cast<WorldIdx>(i));
        bits::copy_from(dst, s.val(w));

        for (const auto& [atom, ext] : p.post_true[e])
            if (bits::test(ext, w)) bits::set(dst, atom);
        for (const auto& [atom, ext] : p.post_false[e])
            if (bits::test(ext, w)) bits::reset(dst, atom);
    }

    // ── W'* ─────────────────────────────────────────────────────────────────
    {
        auto des = result.designated_bits();
        bits::for_each(s.designated_bits(), [&](std::uint32_t w) {
            for (EventIdx e : a.designated_events) {
                if (e >= ne) continue;
                const WorldIdx idx = out.pair_to_idx[std::size_t(w) * ne + e];
                if (idx != kNoWorld) bits::set(des, idx);
            }
        });
        if (bits::empty(des))
            return pruned<ProductUpdateResult>(PruneReason::Inapplicable);
    }

    // ── R'_i ────────────────────────────────────────────────────────────────
    //
    //   R'_i((w,e)) = { (v,f) | v ∈ R_i(w), f ∈ R^E_i(e) }
    //
    // Walked in source-world order so that s.succ(ag, w) is read once per
    // (agent, world) and stays in cache across that world's events.
    for (AgentIdx ag = 0; ag < na; ++ag) {
        const auto* agent_cases =
            (ag < a.obs_cases.size()) ? &a.obs_cases[ag] : nullptr;

        for (WorldIdx w = 0; w < nw; ++w) {
            const auto world_row = s.succ(ag, w);
            if (bits::empty(world_row)) continue;

            const std::int32_t ci = p.obs_case[std::size_t(ag) * nw + w];
            const std::vector<std::unordered_set<EventIdx>>* event_rel =
                (ci >= 0 && agent_cases) ? &(*agent_cases)[ci].relation : nullptr;

            for (EventIdx e = 0; e < ne; ++e) {
                const WorldIdx new_w = out.pair_to_idx[std::size_t(w) * ne + e];
                if (new_w == kNoWorld) continue;

                auto dst = result.succ(ag, new_w);

                if (event_rel) {
                    const auto& event_row = (*event_rel)[e];
                    bits::for_each(world_row, [&](std::uint32_t v) {
                        for (EventIdx f : event_row) {
                            if (f >= ne) continue;
                            const WorldIdx t = out.pair_to_idx[std::size_t(v) * ne + f];
                            if (t != kNoWorld) bits::set(dst, t);
                        }
                    });
                } else {
                    // Fully-observant fallback: cross with every event. Pairs
                    // that failed their precondition are absent from the table,
                    // so they drop out here without a separate check.
                    bits::for_each(world_row, [&](std::uint32_t v) {
                        for (EventIdx f = 0; f < ne; ++f) {
                            const WorldIdx t = out.pair_to_idx[std::size_t(v) * ne + f];
                            if (t != kNoWorld) bits::set(dst, t);
                        }
                    });
                }
            }
        }
    }

    result.invalidate();

    // ── KD45 repair ─────────────────────────────────────────────────────────
    if (enforce_kd45) {
        const auto alive = serial_core(result);

        if (bits::count(alive) != result.num_worlds) {
            std::vector<WorldIdx> remap;
            EpistemicState repaired = restrict_state(result, alive, remap);

            if (bits::empty(repaired.designated_bits()))
                return pruned<ProductUpdateResult>(PruneReason::NonSerial);

            for (WorldIdx& idx : out.pair_to_idx)
                if (idx != kNoWorld) idx = remap[idx];

            result = std::move(repaired);
        } else if (bits::empty(result.designated_bits())) {
            return pruned<ProductUpdateResult>(PruneReason::NonSerial);
        }
    }

    out.state = std::move(result);
    return ok(std::move(out));
}

Outcome<EpistemicState>
product_update(const EpistemicState& s, const Action& a,
               bool enforce_kd45, const WorldCapPolicy& cap) {
    auto res = product_update_with_map(s, a, enforce_kd45, cap);
    if (!res) return pruned<EpistemicState>(res.error());
    return ok(std::move(res->state));
}

// ── Sensing ─────────────────────────────────────────────────────────────────
//
// For a sensing action with E_d = {e₁, …}, the branch for e_k is the epistemic
// state given that e_k fired. All branches share the product model W' and R';
// only W'*_k = { (w, e_k) | w ∈ W* } ∩ W' differs. The single shared update also
// keeps world ids coherent between branches — running the update once per event
// would compact ids independently and make the designated sets refer to
// different worlds.
std::vector<std::pair<EventIdx, EpistemicState>>
product_update_split(const EpistemicState& s, const Action& a,
                     bool enforce_kd45, const WorldCapPolicy& cap) {
    auto full = product_update_with_map(s, a, enforce_kd45, cap);
    if (!full) return {};

    const EpistemicState& model = full->state;
    const std::uint32_t   ne    = full->num_events;

    std::vector<std::pair<EventIdx, EpistemicState>> results;
    results.reserve(a.designated_events.size());

    // Deterministic branch order: designated_events is an unordered_set, and the
    // order it yields would otherwise leak into the conditional plan's branch
    // order and into AO*'s search order.
    std::vector<EventIdx> events(a.designated_events.begin(),
                                 a.designated_events.end());
    std::sort(events.begin(), events.end());

    for (EventIdx eid : events) {
        if (eid >= ne) continue;

        std::vector<bits::Word> des(model.rel_words, 0);
        bits::for_each(s.designated_bits(), [&](std::uint32_t w) {
            const WorldIdx idx = full->pair_to_idx[std::size_t(w) * ne + eid];
            if (idx != kNoWorld) bits::set(des, idx);
        });

        // An empty designated set means this outcome is inconsistent with the
        // current state: the event could not have fired in any actual world.
        if (bits::empty(des)) continue;

        EpistemicState branch = model;
        bits::copy_from(branch.designated_bits(), des);
        branch.invalidate();

        results.emplace_back(eid, std::move(branch));
    }

    return results;
}
