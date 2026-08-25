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

#include "plansys2_epistemic_planner/formula.hpp"
#include "plansys2_epistemic_planner/bitset.hpp"

#include <unordered_map>
#include <vector>

// Hash-consing registry.
//
// A formula is identified by (kind, atom, agent, group, children-ids). Because
// children are themselves interned, comparing children by pointer identity is
// exact — no deep structural walk is needed at intern time, so interning a
// formula of size n costs O(arity), not O(n).

namespace {

struct InternKey {
    FormulaKind kind;
    AtomIdx     atom;
    AgentIdx    agent;
    std::vector<AgentIdx>       group;
    std::vector<const Formula*> children;   // interned representatives

    bool operator==(const InternKey& o) const noexcept {
        return kind == o.kind && atom == o.atom && agent == o.agent &&
               group == o.group && children == o.children;
    }
};

struct InternKeyHash {
    std::size_t operator()(const InternKey& k) const noexcept {
        bits::Word h = bits::mix64(static_cast<bits::Word>(k.kind));
        h = bits::mix64(h ^ (static_cast<bits::Word>(k.atom) << 1));
        h = bits::mix64(h ^ (static_cast<bits::Word>(k.agent) << 33));
        for (AgentIdx g : k.group)
            h = bits::mix64(h ^ static_cast<bits::Word>(g));
        for (const Formula* c : k.children)
            h = bits::mix64(h ^ static_cast<bits::Word>(c->id));
        return static_cast<std::size_t>(h);
    }
};

// Interned formulas are immortal for the life of the process. The planner
// builds them once during parsing and then reads them from every hot loop; the
// registry keeps ids dense so evaluators can index memo tables directly.
struct Registry {
    std::unordered_map<InternKey, FormulaPtr, InternKeyHash> table;
    std::vector<FormulaPtr>                                  by_id;
};

Registry& registry() {
    static Registry r;
    return r;
}

InternKey key_of(FormulaKind kind, AtomIdx atom, AgentIdx agent,
                 const std::vector<AgentIdx>& group,
                 const std::vector<FormulaPtr>& children) {
    InternKey k{kind, atom, agent, group, {}};
    k.children.reserve(children.size());
    for (const FormulaPtr& c : children) k.children.push_back(c.get());
    return k;
}

FormulaPtr intern(FormulaKind kind, AtomIdx atom, AgentIdx agent,
                  std::vector<AgentIdx> group, std::vector<FormulaPtr> children) {
    Registry& r  = registry();
    InternKey key = key_of(kind, atom, agent, group, children);

    if (auto it = r.table.find(key); it != r.table.end())
        return it->second;

    auto f      = std::make_shared<Formula>();
    f->kind     = kind;
    f->atom     = atom;
    f->agent    = agent;
    f->group    = std::move(group);
    f->children = std::move(children);
    f->id       = static_cast<std::uint32_t>(r.by_id.size());

    r.by_id.push_back(f);
    r.table.emplace(std::move(key), f);
    return f;
}

} // namespace

std::size_t formula_universe_size() noexcept {
    return registry().by_id.size();
}

void formula_registry_reset() noexcept {
    Registry& r = registry();
    r.table.clear();
    r.by_id.clear();
}

FormulaPtr Formula::make_top()  { return intern(FormulaKind::Top, 0, 0, {}, {}); }
FormulaPtr Formula::make_bot()  { return intern(FormulaKind::Bot, 0, 0, {}, {}); }

FormulaPtr Formula::make_atom(AtomIdx a) {
    return intern(FormulaKind::Atom, a, 0, {}, {});
}

FormulaPtr Formula::make_not(FormulaPtr child) {
    return intern(FormulaKind::Not, 0, 0, {}, {std::move(child)});
}

FormulaPtr Formula::make_and(std::vector<FormulaPtr> fs) {
    return intern(FormulaKind::And, 0, 0, {}, std::move(fs));
}

FormulaPtr Formula::make_or(std::vector<FormulaPtr> fs) {
    return intern(FormulaKind::Or, 0, 0, {}, std::move(fs));
}

FormulaPtr Formula::make_belief(AgentIdx ag, FormulaPtr child) {
    return intern(FormulaKind::Belief, 0, ag, {}, {std::move(child)});
}

FormulaPtr Formula::make_common(std::vector<AgentIdx> grp, FormulaPtr child) {
    return intern(FormulaKind::Common, 0, 0, std::move(grp), {std::move(child)});
}

// <Kw.i>φ ≡ [i]φ ∨ [i]¬φ, kept as a primitive so the evaluator can derive both
// modal tests from a single extension of φ rather than expanding the syntax.
FormulaPtr Formula::make_kw(AgentIdx ag, FormulaPtr child) {
    return intern(FormulaKind::Kw, 0, ag, {}, {std::move(child)});
}
