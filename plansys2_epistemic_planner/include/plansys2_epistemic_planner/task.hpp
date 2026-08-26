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
#include "plansys2_epistemic_planner/state.hpp"
#include "plansys2_epistemic_planner/action.hpp"

struct PlanningTask {
    // Language
    std::vector<std::string> atom_names;
    std::vector<std::string> agent_names;

    // Task components
    EpistemicState            init;
    std::vector<Action>       actions;
    FormulaPtr                goal;

    // Frame semantics: true = KD45 (belief/doxastic), false = S5 (knowledge).
    bool kd45 = false;

    // True iff at least one action has agents with heterogeneous observability
    // (some Fully, some Oblivious or conditional). Set by the parser after all
    // actions are loaded. Gossip, Grapevine, and AMC are the canonical cases.
    // Used by the heuristic selector to prefer knowledge spread over epistemic distance
    // and by strategy selector to avoid routing partial-obs domains to GBFS.
    bool partial_obs = false;

    // True iff the goal is a pure conjunction of Kw (knowing-whether) formulas
    // with no bare atom conjuncts. Set by the parser.
    // Kw-only goals have a specific gradient structure that KnowledgeSpreadHeuristic
    // is designed to exploit; EpistemicDistance wastes cycles on the wrong projection.
    bool goal_kw_only = false;

    std::unordered_map<std::string, AtomIdx>   atom_index;
    std::unordered_map<std::string, AgentIdx>  agent_index;
    std::unordered_map<std::string, ActionIdx> action_index;

    size_t num_atoms()   const { return atom_names.size(); }
    size_t num_agents()  const { return agent_names.size(); }
    size_t num_actions() const { return actions.size(); }
};