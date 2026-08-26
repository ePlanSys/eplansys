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
#include "plansys2_epistemic_planner/types.hpp"

enum class FormulaKind : std::uint8_t {
    Top,        // ⊤
    Bot,        // ⊥
    Atom,       // p
    Not,        // ¬φ
    And,        // φ ∧ ψ
    Or,         // φ ∨ ψ
    Belief,     // [i]φ   (B_i φ)
    Common,     // [C.G]φ (C_G φ)
    Kw,         // <Kw.i>φ  ≡  [i]φ ∨ [i]¬φ
};

// Formulas are hash-consed: the make_* factories intern into a process-wide
// registry, so two structurally identical formulas are the *same* object and
// carry the same dense `id`.
//
// This matters for more than memory. Satisfaction-set evaluation memoises by
// formula id, and a planning task's action preconditions, postcondition guards,
// observability conditions and goal conjuncts share a great many subformulas.
// Interning turns that sharing into cache hits: one bottom-up pass computes
// each distinct subformula's extension once per model, no matter how many
// syntactic occurrences it has.
//
// Ids are dense and stable for the life of the process, which lets the
// evaluator use a flat vector rather than a hash map for its memo table.
struct Formula {
    FormulaKind kind{FormulaKind::Top};

    // Dense index into the intern registry. Assigned by the factories.
    std::uint32_t id{0};

    AtomIdx  atom{0};    // Atom
    AgentIdx agent{0};   // Belief / Kw

    std::vector<AgentIdx>   group;      // Common
    std::vector<FormulaPtr> children;

    // Each factory returns the canonical interned representative.
    [[nodiscard]] static FormulaPtr make_top();
    [[nodiscard]] static FormulaPtr make_bot();
    [[nodiscard]] static FormulaPtr make_atom(AtomIdx a);
    [[nodiscard]] static FormulaPtr make_not(FormulaPtr f);
    [[nodiscard]] static FormulaPtr make_and(std::vector<FormulaPtr> fs);
    [[nodiscard]] static FormulaPtr make_or(std::vector<FormulaPtr> fs);
    [[nodiscard]] static FormulaPtr make_belief(AgentIdx ag, FormulaPtr f);
    [[nodiscard]] static FormulaPtr make_common(std::vector<AgentIdx> grp, FormulaPtr f);
    [[nodiscard]] static FormulaPtr make_kw(AgentIdx ag, FormulaPtr f);
};

// Number of distinct interned formulas. Evaluators size their memo tables from
// this; it only ever grows, and stops growing once parsing is complete.
[[nodiscard]] std::size_t formula_universe_size() noexcept;

// Drop every interned formula and restart identifier numbering at zero.
//
// A one-shot process never needs this: the registry is sized by one task and
// dies with it. A long-lived host that solves many tasks in-process does — the
// registry is global and otherwise accumulates every formula of every task
// solved, for the life of the process.
//
// Safe only when no FormulaPtr from a previous task is still in use. Formulas
// are shared_ptr, so surviving references stay valid, but identifier numbering
// restarts and any memo table indexed by formula id would silently alias.
// Call it after the PlanningTask and every derived state have been destroyed.
//
// Not thread-safe, like the rest of the registry: no two solves may overlap.
void formula_registry_reset() noexcept;
