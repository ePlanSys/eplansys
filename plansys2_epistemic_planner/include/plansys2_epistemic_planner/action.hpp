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
#include "plansys2_epistemic_planner/types.hpp"
#include "plansys2_epistemic_planner/formula.hpp"

// One event inside an action's event model 
struct Event {
    EventIdx id;
    std::string name;

    FormulaPtr precondition;   // must hold in world for (world,event) to exist

    // postconditions: atom -> formula that gives new truth value
    // if atom not in map -> unchanged
    std::unordered_map<AtomIdx, FormulaPtr> post_true;   // atom becomes true if formula holds
    std::unordered_map<AtomIdx, FormulaPtr> post_false;  // atom becomes false if formula holds

    bool is_nil{false};        // trivial nil event (no effects)
};

// One conditional observability case for an agent:
// if condition holds at world w, use this event relation.
// Cases are evaluated in order; first match wins.
struct ObsCase {
    FormulaPtr condition;
    std::vector<std::unordered_set<EventIdx>> relation; // [event_id] -> reachable event ids
};

// Abstract epistemic action = event model + observability
struct Action {
    std::string name;

    std::vector<Event> events;
    std::unordered_set<EventIdx> designated_events;   // E_d

    // obs_cases[agent_idx] = ordered list of (condition, event_relation) pairs.
    // Evaluated per world during product update; first matching case is used.
    // If no case matches, agent is treated as fully observable.
    std::vector<std::vector<ObsCase>> obs_cases;

    size_t num_agents{0};

    // True iff the action has exactly one designated event and is therefore
    // an ontic action (no sensing branches). Sensing actions have |E_d| >= 2
    // (the second designated event is typically the nil / no-op branch).
    bool is_ontic() const { return designated_events.size() == 1; }

    // Strong applicability (conformant semantics):
    //   - For ontic actions: ALL designated worlds must satisfy the precondition
    //     of every designated event. An ontic action that fires in only some
    //     actual worlds would silently drop the non-firing worlds from the
    //     designated set, producing a state that conflates partial execution
    //     with full execution and gives spurious goal satisfaction.
    //   - For sensing actions: at least one designated world satisfies the
    //     precondition of at least one designated event (existential). Sensing
    //     actions branch via product_update_split, so worlds where one event
    //     fires and worlds where the other fires land in separate subtrees —
    //     no conflation occurs.
    bool applicable(const EpistemicState& s) const;

    // Weak applicability: existential check used only for AO* action ranking
    // (heuristic ordering). Never used to decide whether to generate a successor.
    bool applicable_weak(const EpistemicState& s) const;
};