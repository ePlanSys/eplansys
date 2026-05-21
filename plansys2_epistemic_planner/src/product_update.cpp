#include "plansys2_epistemic_planner/product_update.hpp"
#include <algorithm>
#include <iostream>

// DEL Product Update  s ⊗ a
//
// New worlds  : W' = { (w,e) | w ∈ W, e ∈ E, M,w ⊨ pre(e) }
// New relation: R'_i = { ((w,e),(v,f)) | w R_i v  ∧  e R^E_i f }
// New valuation: V'(w,e)(p) = post(e,p) evaluated at w
//                             (unchanged if p not in postconditions)
// New designated: W'* = { (w,e) | w ∈ W*, e ∈ E_d }
//

static uint64_t encode_pair(WorldIdx w, EventIdx e) {
    return (static_cast<uint64_t>(w) << 32) | static_cast<uint64_t>(e);
}

// KD45 seriality repair.
//
// In KD45, every accessibility relation must be serial: ∀w ∃v: w R_i v.
// The standard DEL product update does not guarantee this — a world (w,e)
// survives in the result if pre(e) holds at w, but its R_i row in the result
// is the cross-product of R_i(w) and R^E_i(e). If either set is empty, the
// new world has no i-successors, violating seriality.
//
// A world with an empty R_i row makes [i]φ vacuously true there, which
// corrupts belief-formula evaluation and can produce spurious goal satisfaction.
//
// This function iteratively removes all worlds that are non-serial for any
// agent, then removes worlds that became non-serial because their only
// successors were removed, until a fixpoint is reached.
//
// IMPORTANT: after pruning we compact world IDs so that worlds[i].id == i
// and all accessibility / designated sets use the new compacted IDs.
// The pair_to_idx map passed in is updated in place to reflect the new IDs.
// Callers that hold a reference to pair_to_idx will see the updated values.
//
// If any designated world is removed the function returns false (action not
// applicable); the caller returns nullopt.
//
static bool enforce_seriality(EpistemicState& s,
                               std::unordered_map<uint64_t, WorldIdx>& pair_to_idx) {
    size_t na = s.num_agents;
    bool changed = true;

    while (changed) {
        changed = false;

        std::unordered_set<WorldIdx> non_serial;
        for (auto& world : s.worlds) {
            WorldIdx w = world.id;
            for (AgentIdx ag = 0; ag < na; ag++) {
                // world id == vector position (invariant maintained below)
                if (s.accessibility[ag][w].empty()) {
                    non_serial.insert(w);
                    break;
                }
            }
        }

        if (non_serial.empty()) break;

        std::unordered_set<WorldIdx> keep;
        for (auto& world : s.worlds)
            if (!non_serial.count(world.id))
                keep.insert(world.id);

        for (AgentIdx ag = 0; ag < na; ag++) {
            for (auto& world : s.worlds) {
                WorldIdx w = world.id;
                auto& row = s.accessibility[ag][w];
                std::unordered_set<WorldIdx> pruned;
                for (WorldIdx v : row)
                    if (keep.count(v))
                        pruned.insert(v);
                if (pruned.size() != row.size()) {
                    row = std::move(pruned);
                    changed = true;
                }
            }
        }

        s.worlds.erase(
            std::remove_if(s.worlds.begin(), s.worlds.end(),
                [&](const World& w){ return non_serial.count(w.id) > 0; }),
            s.worlds.end());

        for (WorldIdx w : non_serial)
            s.designated.erase(w);

        if (s.designated.empty()) return false;
    }

    // Compact world IDs so that worlds[i].id == i.
    //
    // Build old->new id remapping in the order worlds now appear in the vector.
    // Then patch worlds, accessibility rows, designated set, and pair_to_idx.
    {
        std::unordered_map<WorldIdx, WorldIdx> remap;
        remap.reserve(s.worlds.size());
        for (WorldIdx i = 0; i < static_cast<WorldIdx>(s.worlds.size()); i++)
            remap[s.worlds[i].id] = i;

        // Check whether compaction is actually needed.
        bool needs_compact = false;
        for (auto& [old_id, new_id] : remap)
            if (old_id != new_id) { needs_compact = true; break; }

        if (needs_compact) {
            // Patch world ids
            for (WorldIdx i = 0; i < static_cast<WorldIdx>(s.worlds.size()); i++)
                s.worlds[i].id = i;

            // Rebuild accessibility with compacted ids.
            // The outer vector is sized to old nw; rebuild it to new size.
            size_t new_nw = s.worlds.size();
            std::vector<Relation> new_acc(na, Relation(new_nw));
            for (AgentIdx ag = 0; ag < na; ag++) {
                for (auto& [old_id, new_id] : remap) {
                    auto& old_row = s.accessibility[ag][old_id];
                    auto& new_row = new_acc[ag][new_id];
                    for (WorldIdx v : old_row) {
                        auto it = remap.find(v);
                        if (it != remap.end())
                            new_row.insert(it->second);
                    }
                }
            }
            s.accessibility = std::move(new_acc);

            // Patch designated set
            std::unordered_set<WorldIdx> new_des;
            for (WorldIdx w : s.designated) {
                auto it = remap.find(w);
                if (it != remap.end())
                    new_des.insert(it->second);
            }
            s.designated = std::move(new_des);

            // Patch pair_to_idx so callers see the compacted ids.
            for (auto& [key, old_wid] : pair_to_idx) {
                auto it = remap.find(old_wid);
                if (it != remap.end())
                    old_wid = it->second;
                // Entries whose world was removed are left with stale ids;
                // the split logic filters them via survived_worlds below.
            }
        }
    }

    return !s.designated.empty();
}

// Core product update that returns both the state and the pair_to_idx map.
std::optional<ProductUpdateResult>
product_update_with_map(const EpistemicState& s, const Action& a,
                        bool enforce_kd45) {
    size_t na = s.num_agents;

    ProductUpdateResult out;
    EpistemicState& result = out.state;
    auto& pair_to_idx = out.pair_to_idx;

    result.num_agents = na;

    for (auto& world : s.worlds) {
        for (auto& event : a.events) {
            if (!s.holds_at(*event.precondition, world.id))
                continue;

            WorldIdx new_id = static_cast<WorldIdx>(result.worlds.size());
            pair_to_idx[encode_pair(world.id, event.id)] = new_id;

            World new_world;
            new_world.id    = new_id;
            new_world.atoms = world.atoms;

            for (auto& [atom, cond] : event.post_true) {
                if (s.holds_at(*cond, world.id))
                    new_world.atoms.insert(atom);
            }
            for (auto& [atom, cond] : event.post_false) {
                if (s.holds_at(*cond, world.id))
                    new_world.atoms.erase(atom);
            }

            result.worlds.push_back(std::move(new_world));
        }
    }

    // designated worlds: (w,e) designated iff w ∈ W* AND e ∈ E_d
    for (WorldIdx w : s.designated) {
        for (EventIdx e : a.designated_events) {
            auto key = encode_pair(w, e);
            auto it = pair_to_idx.find(key);
            if (it != pair_to_idx.end())
                result.designated.insert(it->second);
        }
    }

    if (result.designated.empty()) return std::nullopt;

    // Build accessibility relations
    size_t nw_new = result.worlds.size();
    result.accessibility.resize(na, Relation(nw_new));

    for (AgentIdx ag = 0; ag < na; ag++) {
        for (auto& [pair_we, new_w] : pair_to_idx) {
            WorldIdx w = static_cast<WorldIdx>(pair_we >> 32);
            EventIdx e = static_cast<EventIdx>(pair_we & 0xFFFFFFFF);

            const RelRow& world_row = s.accessibility[ag][w];

            const std::vector<std::unordered_set<EventIdx>>* event_rel = nullptr;
            if (ag < a.obs_cases.size()) {
                for (auto& oc : a.obs_cases[ag]) {
                    if (s.holds_at(*oc.condition, w)) {
                        event_rel = &oc.relation;
                        break;
                    }
                }
            }

            if (event_rel) {
                const auto& event_row = (*event_rel)[e];
                for (WorldIdx v : world_row) {
                    for (EventIdx f : event_row) {
                        auto key2 = encode_pair(v, f);
                        auto it2 = pair_to_idx.find(key2);
                        if (it2 != pair_to_idx.end())
                            result.accessibility[ag][new_w].insert(it2->second);
                    }
                }
            } else {
                // No matching obs case: fully observable fallback
                for (WorldIdx v : world_row) {
                    for (auto& event : a.events) {
                        auto key2 = encode_pair(v, event.id);
                        auto it2 = pair_to_idx.find(key2);
                        if (it2 != pair_to_idx.end())
                            result.accessibility[ag][new_w].insert(it2->second);
                    }
                }
            }
        }
    }

    // KD45 seriality repair.  enforce_seriality patches pair_to_idx in place
    // so that all world ids it contains reflect the compacted numbering.
    if (enforce_kd45) {
        if (!enforce_seriality(result, pair_to_idx))
            return std::nullopt;
    }

    return out;
}

std::optional<EpistemicState> product_update(const EpistemicState& s,
                                              const Action& a,
                                              bool enforce_kd45) {
    auto res = product_update_with_map(s, a, enforce_kd45);
    if (!res) return std::nullopt;
    return std::move(res->state);
}

// Sensing product update: one EpistemicState per designated event.
std::vector<std::pair<EventIdx, EpistemicState>>
product_update_split(const EpistemicState& s, const Action& a, bool enforce_kd45) {
    // Run the full product update through the shared path so we get both the
    // updated state and the exact pair_to_idx map that was used to build it.
    // This is the only authoritative mapping; we must not re-derive it.
    auto maybe_full = product_update_with_map(s, a, enforce_kd45);
    if (!maybe_full) return {};

    const EpistemicState& full   = maybe_full->state;
    const auto& pair_to_idx      = maybe_full->pair_to_idx;

    // Collect the set of world ids that survived seriality repair.
    std::unordered_set<WorldIdx> survived;
    survived.reserve(full.worlds.size());
    for (auto& fw : full.worlds)
        survived.insert(fw.id);

    std::vector<std::pair<EventIdx, EpistemicState>> results;

    for (EventIdx eid : a.designated_events) {
        std::unordered_set<WorldIdx> branch_designated;

        for (WorldIdx w : s.designated) {
            auto key = encode_pair(w, eid);
            auto it  = pair_to_idx.find(key);

            // (w, eid) pair did not pass the precondition check — skip.
            if (it == pair_to_idx.end()) continue;

            // World was removed by KD45 seriality repair — skip.
            if (!survived.count(it->second)) continue;

            branch_designated.insert(it->second);
        }

        if (branch_designated.empty()) continue;

        // Sub-state: same worlds and accessibility as full update,
        // but only the designated worlds for this event.
        EpistemicState branch;
        branch.num_agents    = full.num_agents;
        branch.worlds        = full.worlds;
        branch.accessibility = full.accessibility;
        branch.designated    = std::move(branch_designated);

        results.emplace_back(eid, std::move(branch));
    }

    return results;
}