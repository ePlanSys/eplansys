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

    // Frame semantics: true = KD45 (belief/doxastic), false = S5 (knowledge)
    bool kd45 = false;

    // Convenience lookups
    std::unordered_map<std::string, AtomIdx>  atom_index;
    std::unordered_map<std::string, AgentIdx> agent_index;
    std::unordered_map<std::string, ActionIdx> action_index;

    size_t num_atoms()   const { return atom_names.size(); }
    size_t num_agents()  const { return agent_names.size(); }
    size_t num_actions() const { return actions.size(); }
};