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

#pragma once
#include "plansys2_epistemic_planner/bitset.hpp"
#include "plansys2_epistemic_planner/formula.hpp"
#include "plansys2_epistemic_planner/types.hpp"

#include <memory>
#include <span>

class SatCache;   // defined in state.cpp

// A 128-bit structural digest of a canonically-labelled epistemic state.
//
// Closed lists store fingerprints instead of states. A contracted state costs
// a few kilobytes; a fingerprint costs sixteen bytes, and comparison is two
// integer tests rather than a graph walk. At 128 bits the probability of a
// collision anywhere in a search of 10^9 states is below 10^-20, far below the
// probability of any other failure mode in the system.
struct Fingerprint {
    std::uint64_t lo{0};
    std::uint64_t hi{0};

    friend bool operator==(const Fingerprint&, const Fingerprint&) = default;
};

struct FingerprintHash {
    std::size_t operator()(const Fingerprint& f) const noexcept {
        return static_cast<std::size_t>(f.lo ^ (f.hi * 0x9E3779B97F4A7C15ULL));
    }
};

// Epistemic state = multi-pointed Kripke model (W, {R_i}_{i∈Ag}, V, W*).
//
// Storage is three flat word arrays and nothing else:
//
//   valuation   num_worlds × val_words     V : W → 2^P as a bit matrix
//   relation    num_agents × num_worlds × rel_words
//                                          each R_i ⊆ W × W as a bit matrix,
//                                          row-major by source world
//   designated  rel_words                  W* ⊆ W
//
// Two consequences drive the rest of the planner. Modal operators become
// word-parallel: [i]φ holds at w exactly when R_i(w) ∧ ¬sat(φ) is empty, which
// costs ⌈|W|/64⌉ ANDNOT tests regardless of how many successors w has. And the
// whole model is three contiguous allocations, so copying a state is three
// memcpys and hashing it is a linear scan with no pointer chasing.
struct EpistemicState {
    std::uint32_t num_worlds{0};
    std::uint32_t num_atoms{0};
    std::uint32_t num_agents{0};

    std::uint32_t val_words{0};   // = words_for(num_atoms)
    std::uint32_t rel_words{0};   // = words_for(num_worlds)

    std::vector<bits::Word> valuation;
    std::vector<bits::Word> relation;
    std::vector<bits::Word> designated;

    // All five are defined out of line: SatCache is incomplete here, and
    // std::unique_ptr needs the complete type to destroy it.
    EpistemicState();

    // The satisfaction cache and the fingerprint are derived data. They are
    // rebuilt lazily rather than copied, so copy construction stays a plain
    // copy of the three arrays.
    EpistemicState(const EpistemicState& o);
    EpistemicState& operator=(const EpistemicState& o);
    EpistemicState(EpistemicState&&) noexcept;
    EpistemicState& operator=(EpistemicState&&) noexcept;
    ~EpistemicState();

    void allocate(std::uint32_t worlds, std::uint32_t atoms, std::uint32_t agents);

    // Element access.
    [[nodiscard]] bits::WordSpan val(WorldIdx w) noexcept
        { return {valuation.data() + std::size_t(w) * val_words, val_words}; }
    [[nodiscard]] bits::ConstWordSpan val(WorldIdx w) const noexcept
        { return {valuation.data() + std::size_t(w) * val_words, val_words}; }

    [[nodiscard]] bits::WordSpan succ(AgentIdx ag, WorldIdx w) noexcept
        { return {relation.data() + row_offset(ag, w), rel_words}; }
    [[nodiscard]] bits::ConstWordSpan succ(AgentIdx ag, WorldIdx w) const noexcept
        { return {relation.data() + row_offset(ag, w), rel_words}; }

    [[nodiscard]] bits::WordSpan designated_bits() noexcept
        { return {designated.data(), rel_words}; }
    [[nodiscard]] bits::ConstWordSpan designated_bits() const noexcept
        { return {designated.data(), rel_words}; }

    [[nodiscard]] bool has_atom(WorldIdx w, AtomIdx a) const noexcept
        { return bits::test(val(w), a); }
    [[nodiscard]] bool is_designated(WorldIdx w) const noexcept
        { return bits::test(designated_bits(), w); }
    [[nodiscard]] std::size_t num_designated() const noexcept
        { return bits::count(designated_bits()); }

    // Mutators. These invalidate the derived caches; use them while building a
    // state, not on one that has already been evaluated.
    void set_atom(WorldIdx w, AtomIdx a)  { bits::set(val(w), a);            invalidate(); }
    void set_designated(WorldIdx w)       { bits::set(designated_bits(), w); invalidate(); }
    void add_edge(AgentIdx ag, WorldIdx from, WorldIdx to)
                                          { bits::set(succ(ag, from), to);   invalidate(); }

    // Model checking.
    //
    // sat(φ) is the extension of φ: the set of worlds at which φ holds,
    // computed bottom-up over the whole model and memoised by formula id.
    // The returned span is owned by this state's cache and remains valid until
    // the next mutation *or the next sat() call*, since the cache arena may
    // reallocate; callers holding several extensions at once must copy them.
    [[nodiscard]] bits::ConstWordSpan sat(const Formula& f) const;

    // Convenience wrapper that copies the extension out of the cache arena.
    void sat_copy(const Formula& f, std::vector<bits::Word>& out) const;

    [[nodiscard]] bool holds_at(const Formula& f, WorldIdx w) const;

    // M ⊨ φ, i.e. φ holds at every designated world: W* ⊆ sat(φ).
    [[nodiscard]] bool satisfies(const Formula& f) const;

    void invalidate() const noexcept;

    // Identity.
    //
    // These compare the *labelled* structure. They are exact up to isomorphism
    // only for states produced by bisim_contract, which assigns a canonical
    // world numbering; every state the search stores has been through it.
    [[nodiscard]] Fingerprint fingerprint() const;
    [[nodiscard]] std::size_t hash() const;
    [[nodiscard]] bool operator==(const EpistemicState& o) const noexcept;

    void print(const std::vector<std::string>& atom_names,
               const std::vector<std::string>& agent_names) const;

    // Bytes of model storage, excluding derived caches.
    [[nodiscard]] std::size_t footprint() const noexcept {
        return (valuation.size() + relation.size() + designated.size()) * sizeof(bits::Word);
    }

private:
    [[nodiscard]] std::size_t row_offset(AgentIdx ag, WorldIdx w) const noexcept {
        return (std::size_t(ag) * num_worlds + w) * rel_words;
    }

    mutable std::unique_ptr<SatCache>  cache_;
    mutable std::optional<Fingerprint> fp_;
};

// Restrict a state to `keep`, compacting world indices to 0..|keep|-1.
//
// Used by KD45 seriality repair (drop non-serial worlds) and by bisimulation
// contraction (drop worlds unreachable from W*). `remap` is filled with the
// old→new index of every retained world and kNoWorld elsewhere; callers holding
// world indices into the source — notably the product update's (w,e) table —
// patch them through it.
[[nodiscard]] EpistemicState restrict_state(const EpistemicState& s,
                                            bits::ConstWordSpan keep,
                                            std::vector<WorldIdx>& remap);
