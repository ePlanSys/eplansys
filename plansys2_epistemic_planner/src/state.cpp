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

#include "plansys2_epistemic_planner/state.hpp"

#include <algorithm>
#include <cassert>
#include <iostream>

// Satisfaction-set model checking
//
// Model checking proceeds by extension rather than by per-world descent. A
// per-world evaluator holds_at(φ, w) walks φ once for every world it is asked
// about, so [i]φ over |W| worlds re-descends into φ once per (source,
// successor) pair and C_G φ requires an independent breadth-first search from
// every world, with φ re-checked at every node visited; nested modalities
// multiply those factors.
//
// This evaluator instead computes, for each subformula, its extension
//
//     sat(φ) = { w ∈ W : M, w ⊨ φ }
//
// bottom-up, as a bit set over W. The modal cases become set operations:
//
//     sat(¬φ)     = W \ sat(φ)
//     sat(φ ∧ ψ)  = sat(φ) ∩ sat(ψ)
//     sat([i]φ)   = { w : R_i(w) ⊆ sat(φ) }           -- one ANDNOT test/world
//     sat(Kw_i φ) = sat([i]φ) ∪ sat([i]¬φ)            -- reuses sat(φ) once
//     sat(C_G φ)  = νX. sat(φ) ∩ { w : R_G(w) ⊆ X }   -- one fixpoint, not one
//                                                        BFS per world
//
// Each subformula is evaluated exactly once per model, and because formulas are
// hash-consed the memo is shared across every occurrence of a subformula
// anywhere in the task: a precondition that also appears as an observability
// guard is computed once.

class SatCache {
public:
    explicit SatCache(const EpistemicState& s)
        : s_(s), nw_(s.num_worlds), rw_(s.rel_words) {}

    bits::ConstWordSpan sat(const Formula& f) { return cat(resolve(f)); }

private:
    const EpistemicState& s_;
    std::uint32_t nw_;
    std::uint32_t rw_;

    // One slot per computed extension. The outer vector may reallocate as slots
    // are added, but that only moves the inner vector *objects* — their heap
    // buffers stay put, so a span handed out by sat() remains valid for the
    // lifetime of the cache. Callers may therefore hold several extensions at
    // once, which the product update relies on.
    std::vector<std::vector<bits::Word>> slots_;
    std::vector<std::int64_t>            offset_;   // formula id → slot, -1 unknown

    [[nodiscard]] std::uint32_t alloc() {
        slots_.emplace_back(rw_, 0);
        return static_cast<std::uint32_t>(slots_.size() - 1);
    }

    [[nodiscard]] bits::WordSpan at(std::uint32_t i) noexcept {
        return {slots_[i].data(), rw_};
    }
    [[nodiscard]] bits::ConstWordSpan cat(std::uint32_t i) const noexcept {
        return {slots_[i].data(), rw_};
    }

    std::uint32_t resolve(const Formula& f) {
        if (offset_.size() <= f.id)
            offset_.resize(std::max<std::size_t>(f.id + 1, formula_universe_size()), -1);
        if (offset_[f.id] >= 0)
            return static_cast<std::uint32_t>(offset_[f.id]);

        const std::uint32_t out = compute(f);
        offset_[f.id] = static_cast<std::int64_t>(out);
        return out;
    }

    std::uint32_t compute(const Formula& f) {
        switch (f.kind) {
        case FormulaKind::Top: {
            const std::uint32_t out = alloc();
            bits::fill_all(at(out), nw_);
            return out;
        }

        case FormulaKind::Bot:
            return alloc();   // zero-initialised

        case FormulaKind::Atom: {
            // Column extraction: gather bit `atom` from every world's valuation.
            const std::uint32_t out = alloc();
            auto dst = at(out);
            for (WorldIdx w = 0; w < nw_; ++w)
                if (s_.has_atom(w, f.atom)) bits::set(dst, w);
            return out;
        }

        case FormulaKind::Not: {
            const std::uint32_t c   = resolve(*f.children[0]);
            const std::uint32_t out = alloc();
            bits::complement_into(at(out), cat(c), nw_);
            return out;
        }

        case FormulaKind::And: {
            std::vector<std::uint32_t> cs;
            cs.reserve(f.children.size());
            for (const auto& c : f.children) cs.push_back(resolve(*c));

            const std::uint32_t out = alloc();
            bits::fill_all(at(out), nw_);
            for (std::uint32_t c : cs) bits::and_into(at(out), cat(c));
            return out;
        }

        case FormulaKind::Or: {
            std::vector<std::uint32_t> cs;
            cs.reserve(f.children.size());
            for (const auto& c : f.children) cs.push_back(resolve(*c));

            const std::uint32_t out = alloc();
            for (std::uint32_t c : cs) bits::or_into(at(out), cat(c));
            return out;
        }

        case FormulaKind::Belief: {
            const std::uint32_t c   = resolve(*f.children[0]);
            const std::uint32_t out = alloc();
            if (f.agent < s_.num_agents) box_into(at(out), cat(c), f.agent);
            return out;
        }

        case FormulaKind::Kw: {
            // [i]φ ∨ [i]¬φ from a single extension of φ. The old evaluator
            // called holds_at(φ, v) twice per accessible world here.
            const std::uint32_t c = resolve(*f.children[0]);

            const std::uint32_t neg = alloc();
            bits::complement_into(at(neg), cat(c), nw_);

            const std::uint32_t out = alloc();
            if (f.agent < s_.num_agents) {
                box_into(at(out), cat(c), f.agent);

                std::vector<bits::Word> other(rw_, 0);
                box_into(other, cat(neg), f.agent);
                bits::or_into(at(out), other);
            }
            return out;
        }

        case FormulaKind::Common: {
            // Greatest fixpoint of X ↦ sat(φ) ∩ { w : ∀i∈G. R_i(w) ⊆ X }.
            //
            // w belongs to the fixpoint exactly when φ holds at every world
            // reachable from w by the reflexive-transitive closure of ⋃_{i∈G} R_i
            // — the semantics the per-world BFS implemented, but converging in
            // at most |W| passes over the whole model rather than running one
            // search per world.
            const std::uint32_t c   = resolve(*f.children[0]);
            const std::uint32_t out = alloc();

            bits::fill_all(at(out), nw_);

            std::vector<bits::Word> next(rw_, 0);
            for (;;) {
                bits::copy_from(next, cat(c));
                for (WorldIdx w = 0; w < nw_; ++w) {
                    if (!bits::test(next, w)) continue;
                    for (AgentIdx ag : f.group) {
                        if (ag >= s_.num_agents) continue;
                        if (!bits::subset_of(s_.succ(ag, w), cat(out))) {
                            bits::reset(next, w);
                            break;
                        }
                    }
                }
                if (bits::equal(next, cat(out))) break;
                bits::copy_from(at(out), next);
            }
            return out;
        }
        }
        return alloc();   // unreachable
    }

    // dst := { w : R_ag(w) ⊆ src }
    void box_into(bits::WordSpan dst, bits::ConstWordSpan src, AgentIdx ag) const {
        for (WorldIdx w = 0; w < nw_; ++w) {
            const auto row = s_.succ(ag, w);
            bool all = true;
            for (std::uint32_t i = 0; i < rw_; ++i) {
                if (row[i] & ~src[i]) { all = false; break; }
            }
            if (all) bits::set(dst, w);
        }
    }
};

// EpistemicState

EpistemicState::EpistemicState() = default;

EpistemicState::EpistemicState(const EpistemicState& o)
    : num_worlds(o.num_worlds), num_atoms(o.num_atoms), num_agents(o.num_agents),
      val_words(o.val_words), rel_words(o.rel_words),
      valuation(o.valuation), relation(o.relation), designated(o.designated),
      fp_(o.fp_) {}

EpistemicState& EpistemicState::operator=(const EpistemicState& o) {
    if (this == &o) return *this;
    num_worlds = o.num_worlds; num_atoms = o.num_atoms; num_agents = o.num_agents;
    val_words  = o.val_words;  rel_words = o.rel_words;
    valuation  = o.valuation;  relation  = o.relation;  designated = o.designated;
    cache_.reset();
    fp_ = o.fp_;
    return *this;
}

// A SatCache holds a back-reference to the state it was built from, so it can
// never be carried across a move — the moved-from object's address is what it
// captured. Both moves therefore drop the cache on each side and let it be
// rebuilt lazily. The fingerprint has no such dependency and is carried over.
EpistemicState::EpistemicState(EpistemicState&& o) noexcept
    : num_worlds(o.num_worlds), num_atoms(o.num_atoms), num_agents(o.num_agents),
      val_words(o.val_words), rel_words(o.rel_words),
      valuation(std::move(o.valuation)), relation(std::move(o.relation)),
      designated(std::move(o.designated)), fp_(o.fp_) {
    o.cache_.reset();
}

EpistemicState& EpistemicState::operator=(EpistemicState&& o) noexcept {
    if (this == &o) return *this;
    num_worlds = o.num_worlds; num_atoms = o.num_atoms; num_agents = o.num_agents;
    val_words  = o.val_words;  rel_words = o.rel_words;
    valuation  = std::move(o.valuation);
    relation   = std::move(o.relation);
    designated = std::move(o.designated);
    cache_.reset();
    o.cache_.reset();
    fp_ = o.fp_;
    return *this;
}

EpistemicState::~EpistemicState() = default;

void EpistemicState::allocate(std::uint32_t worlds, std::uint32_t atoms,
                              std::uint32_t agents) {
    num_worlds = worlds;
    num_atoms  = atoms;
    num_agents = agents;
    val_words  = static_cast<std::uint32_t>(bits::words_for(atoms));
    rel_words  = static_cast<std::uint32_t>(bits::words_for(worlds));

    valuation.assign(std::size_t(worlds) * val_words, 0);
    relation.assign(std::size_t(agents) * worlds * rel_words, 0);
    designated.assign(rel_words, 0);
    invalidate();
}

void EpistemicState::invalidate() const noexcept {
    cache_.reset();
    fp_.reset();
}

bits::ConstWordSpan EpistemicState::sat(const Formula& f) const {
    if (!cache_) cache_ = std::make_unique<SatCache>(*this);
    return cache_->sat(f);
}

void EpistemicState::sat_copy(const Formula& f, std::vector<bits::Word>& out) const {
    const auto s = sat(f);
    out.assign(s.begin(), s.end());
}

bool EpistemicState::holds_at(const Formula& f, WorldIdx w) const {
    assert(w < num_worlds);
    return bits::test(sat(f), w);
}

bool EpistemicState::satisfies(const Formula& f) const {
    if (num_worlds == 0) return true;   // vacuous: W* is empty
    return bits::subset_of(designated_bits(), sat(f));
}

// Identity.

Fingerprint EpistemicState::fingerprint() const {
    if (fp_) return *fp_;

    // Two independent 64-bit streams over the same word sequence, each seeded
    // and finalised with splitmix64, so one flipped bit anywhere in the model
    // changes roughly half the bits of both halves.
    bits::Word h1 = bits::mix64(0x243F6A8885A308D3ULL ^ num_worlds);
    bits::Word h2 = bits::mix64(0x13198A2E03707344ULL ^
                                ((std::uint64_t(num_agents) << 32) | num_atoms));

    const auto absorb = [&](bits::Word w) noexcept {
        h1 = bits::mix64(h1 ^ w);
        h2 = bits::mix64((h2 + w) * 0x9E3779B97F4A7C15ULL);
    };

    for (bits::Word w : valuation)  absorb(w);
    absorb(0xA5A5A5A5A5A5A5A5ULL);          // domain separator between arrays
    for (bits::Word w : relation)   absorb(w);
    absorb(0x5A5A5A5A5A5A5A5AULL);
    for (bits::Word w : designated) absorb(w);

    fp_ = Fingerprint{h1, h2};
    return *fp_;
}

std::size_t EpistemicState::hash() const {
    return static_cast<std::size_t>(fingerprint().lo);
}

bool EpistemicState::operator==(const EpistemicState& o) const noexcept {
    return num_worlds == o.num_worlds && num_atoms == o.num_atoms &&
           num_agents == o.num_agents &&
           valuation  == o.valuation  && relation == o.relation &&
           designated == o.designated;
}

// Restriction.

EpistemicState restrict_state(const EpistemicState& s,
                              bits::ConstWordSpan keep,
                              std::vector<WorldIdx>& remap) {
    remap.assign(s.num_worlds, kNoWorld);

    std::vector<WorldIdx> survivors;
    survivors.reserve(bits::count(keep));
    bits::for_each(keep, [&](std::uint32_t w) {
        remap[w] = static_cast<WorldIdx>(survivors.size());
        survivors.push_back(w);
    });

    EpistemicState out;
    out.allocate(static_cast<std::uint32_t>(survivors.size()),
                 s.num_atoms, s.num_agents);

    for (std::size_t nw = 0; nw < survivors.size(); ++nw)
        bits::copy_from(out.val(static_cast<WorldIdx>(nw)), s.val(survivors[nw]));

    for (AgentIdx ag = 0; ag < s.num_agents; ++ag) {
        for (std::size_t nw = 0; nw < survivors.size(); ++nw) {
            auto dst = out.succ(ag, static_cast<WorldIdx>(nw));
            bits::for_each(s.succ(ag, survivors[nw]), [&](std::uint32_t v) {
                if (remap[v] != kNoWorld) bits::set(dst, remap[v]);
            });
        }
    }

    auto des = out.designated_bits();
    bits::for_each(s.designated_bits(), [&](std::uint32_t w) {
        if (remap[w] != kNoWorld) bits::set(des, remap[w]);
    });

    out.invalidate();
    return out;
}

// Debug output.

void EpistemicState::print(const std::vector<std::string>& atom_names,
                           const std::vector<std::string>& agent_names) const {
    std::cout << "Worlds: " << num_worlds << "  Designated: {";
    bits::for_each(designated_bits(), [&](std::uint32_t w) { std::cout << w << " "; });
    std::cout << "}\n";

    for (WorldIdx w = 0; w < num_worlds; ++w) {
        std::cout << "  w" << w;
        if (is_designated(w)) std::cout << "*";
        std::cout << ": {";
        for (AtomIdx a = 0; a < num_atoms; ++a) {
            if (!has_atom(w, a)) continue;
            if (a < atom_names.size()) std::cout << atom_names[a] << " ";
            else                       std::cout << a << " ";
        }
        std::cout << "}\n";
    }

    for (AgentIdx ag = 0; ag < num_agents; ++ag) {
        const std::string aname =
            (ag < agent_names.size()) ? agent_names[ag] : std::to_string(ag);
        std::cout << "  R_" << aname << ": ";
        for (WorldIdx w = 0; w < num_worlds; ++w)
            bits::for_each(succ(ag, w), [&](std::uint32_t v) {
                std::cout << w << "->" << v << " ";
            });
        std::cout << "\n";
    }
}
