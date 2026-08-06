#include "plansys2_epistemic_planner/bisimulation.hpp"

#include <algorithm>
#include <vector>

// ─────────────────────────────────────────────────────────────────────────────
// Contraction = reachability restriction + ordered partition refinement.
//
// Two worlds w, v of a multi-pointed model are bisimilar when
//
//   (i)   V(w) = V(v),
//   (ii)  w ∈ W* ⇔ v ∈ W*,
//   (iii) for every agent i, every R_i-successor of w has a bisimilar
//         R_i-successor of v, and symmetrically.
//
// Condition (ii) is what makes the quotient a *pointed* invariant. Dropping it
// (as the previous implementation did) still preserves formula truth, because
// bisimilar worlds satisfy the same formulas — but it lets a designated world
// merge with a non-designated one, so the quotient no longer determines W*, and
// the canonical form below would identify states that are genuinely different
// planning situations. Keeping it costs at most a coarser contraction and buys
// a sound structural identity.
//
// ── Why ordered refinement rather than Paige–Tarjan ─────────────────────────
//
// Paige–Tarjan refines in O(m log n), asymptotically better than the O(r·(m +
// n log n)) loop below (r = number of rounds, bounded by n but in practice
// small). It does not, however, produce a *canonical* numbering of the
// resulting classes, and the planner needs one: the closed list identifies
// states by fingerprint, so bisimilar models must serialise identically.
//
// This implementation gets canonicity for free from the refinement itself. Each
// round sorts worlds by a key and assigns class ids in sorted order:
//
//   round 0:  key(w) = ( [w ∈ W*], V(w) )
//   round k:  key(w) = ( class_{k-1}(w), ⟨sorted class_{k-1} of R_i(w)⟩_{i∈Ag} )
//
// Round 0's key is a function of the model alone. By induction each later key
// is too, so the class ids at the fixpoint are determined by the isomorphism
// class of the model and nothing else. Renumbering worlds by final class id is
// then a canonical form.
//
// The fixpoint test is exact: since the round-k key begins with the round-(k-1)
// class, sorting is order-preserving on the previous partition, so ids are
// stable and "no class split" is exactly "assignment unchanged".
// ─────────────────────────────────────────────────────────────────────────────

namespace {

// Worlds unreachable from W* cannot affect the truth of any formula evaluated
// at a designated world, so they are dropped before refinement rather than
// carried through it. The product update materialises the full W × E cross
// product, which regularly leaves such worlds behind.
EpistemicState restrict_to_reachable(const EpistemicState& s) {
    const std::uint32_t nw = s.num_worlds;

    std::vector<bits::Word> reach(s.rel_words, 0);
    bits::copy_from(reach, s.designated_bits());

    std::vector<WorldIdx> frontier;
    frontier.reserve(nw);
    bits::for_each(s.designated_bits(),
                   [&](std::uint32_t w) { frontier.push_back(w); });

    while (!frontier.empty()) {
        const WorldIdx w = frontier.back();
        frontier.pop_back();
        for (AgentIdx ag = 0; ag < s.num_agents; ++ag) {
            bits::for_each(s.succ(ag, w), [&](std::uint32_t v) {
                if (!bits::test(reach, v)) {
                    bits::set(reach, v);
                    frontier.push_back(v);
                }
            });
        }
    }

    if (bits::count(reach) == nw) return s;

    std::vector<WorldIdx> remap;
    return restrict_state(s, reach, remap);
}

} // namespace

EpistemicState bisim_contract(EpistemicState s) {
    if (s.num_worlds == 0) return s;

    EpistemicState m  = restrict_to_reachable(s);
    const std::uint32_t nw = m.num_worlds;
    const std::uint32_t na = m.num_agents;
    if (nw == 0) return m;

    std::vector<std::int32_t> class_of(nw, 0);
    std::vector<std::int32_t> next_class(nw, 0);

    // Keys are variable-length int32 runs in one flat buffer; `key_at` slices
    // them. Both buffers are reused across rounds, so refinement performs no
    // per-world allocation — the previous implementation built a fresh
    // vector<vector<int>> for every world on every round.
    std::vector<std::int32_t> key_data;
    std::vector<std::uint32_t> key_begin(nw + 1, 0);
    std::vector<WorldIdx>      order(nw);
    std::vector<std::int32_t>  nbr;

    const auto key_at = [&](WorldIdx w) {
        return std::span<const std::int32_t>(key_data.data() + key_begin[w],
                                             key_begin[w + 1] - key_begin[w]);
    };

    const auto lex_less = [&](WorldIdx a, WorldIdx b) {
        const auto ka = key_at(a), kb = key_at(b);
        return std::lexicographical_compare(ka.begin(), ka.end(),
                                            kb.begin(), kb.end());
    };
    const auto lex_equal = [&](WorldIdx a, WorldIdx b) {
        const auto ka = key_at(a), kb = key_at(b);
        return ka.size() == kb.size() &&
               std::equal(ka.begin(), ka.end(), kb.begin());
    };

    // Sorts `order` by key and writes class ids in that order into next_class.
    const auto assign_classes = [&]() -> std::int32_t {
        for (WorldIdx w = 0; w < nw; ++w) order[w] = w;
        std::sort(order.begin(), order.end(), lex_less);

        std::int32_t id = 0;
        next_class[order[0]] = 0;
        for (std::size_t i = 1; i < order.size(); ++i) {
            if (!lex_equal(order[i - 1], order[i])) ++id;
            next_class[order[i]] = id;
        }
        return id + 1;
    };

    // ── Round 0: valuation and designation ──────────────────────────────────
    {
        key_data.clear();
        for (WorldIdx w = 0; w < nw; ++w) {
            key_begin[w] = static_cast<std::uint32_t>(key_data.size());
            key_data.push_back(m.is_designated(w) ? 1 : 0);
            // Valuation words, halved into int32 so the whole key is one type.
            for (bits::Word word : m.val(w)) {
                key_data.push_back(static_cast<std::int32_t>(word & 0xFFFFFFFFu));
                key_data.push_back(static_cast<std::int32_t>(word >> 32));
            }
        }
        key_begin[nw] = static_cast<std::uint32_t>(key_data.size());
        assign_classes();
        class_of.swap(next_class);
    }

    // ── Rounds 1..: split on neighbour classes ──────────────────────────────
    std::int32_t num_classes = *std::max_element(class_of.begin(), class_of.end()) + 1;

    for (;;) {
        key_data.clear();
        for (WorldIdx w = 0; w < nw; ++w) {
            key_begin[w] = static_cast<std::uint32_t>(key_data.size());
            key_data.push_back(class_of[w]);

            for (AgentIdx ag = 0; ag < na; ++ag) {
                nbr.clear();
                bits::for_each(m.succ(ag, w),
                               [&](std::uint32_t v) { nbr.push_back(class_of[v]); });
                std::sort(nbr.begin(), nbr.end());
                nbr.erase(std::unique(nbr.begin(), nbr.end()), nbr.end());

                // Length prefix keeps the concatenation unambiguous, so plain
                // lexicographic comparison of the flat key is exact.
                key_data.push_back(static_cast<std::int32_t>(nbr.size()));
                key_data.insert(key_data.end(), nbr.begin(), nbr.end());
            }
        }
        key_begin[nw] = static_cast<std::uint32_t>(key_data.size());

        const std::int32_t count = assign_classes();
        if (next_class == class_of) break;

        class_of.swap(next_class);
        num_classes = count;
    }

    // ── Quotient ────────────────────────────────────────────────────────────
    //
    // Class ids are already canonical, so world c of the result is class c.
    std::vector<WorldIdx> repr(num_classes, kNoWorld);
    for (WorldIdx w = 0; w < nw; ++w)
        if (repr[class_of[w]] == kNoWorld) repr[class_of[w]] = w;

    EpistemicState out;
    out.allocate(static_cast<std::uint32_t>(num_classes), m.num_atoms, na);

    for (std::int32_t c = 0; c < num_classes; ++c)
        bits::copy_from(out.val(static_cast<WorldIdx>(c)), m.val(repr[c]));

    for (AgentIdx ag = 0; ag < na; ++ag) {
        for (std::int32_t c = 0; c < num_classes; ++c) {
            auto dst = out.succ(ag, static_cast<WorldIdx>(c));
            bits::for_each(m.succ(ag, repr[c]), [&](std::uint32_t v) {
                bits::set(dst, static_cast<WorldIdx>(class_of[v]));
            });
        }
    }

    // Designation is constant within a class by construction (round 0 split on
    // it), so testing the representative is exact.
    auto des = out.designated_bits();
    for (std::int32_t c = 0; c < num_classes; ++c)
        if (m.is_designated(repr[c])) bits::set(des, static_cast<WorldIdx>(c));

    out.invalidate();
    return out;
}
