#pragma once
#include "plansys2_epistemic_planner/types.hpp"
#include "plansys2_epistemic_planner/formula.hpp"

// One possible world
struct World {
    WorldIdx id;
    std::unordered_set<AtomIdx> atoms;   // true propositions
};

//  Epistemic state = multi-pointed Kripke model ─
struct EpistemicState {
    std::vector<World>    worlds;
    std::unordered_set<WorldIdx> designated;   // actual worlds W*

    // accessibility[agent_idx][world_id] = set of reachable world ids
    // outer vector indexed by AgentIdx
    std::vector<Relation> accessibility;

    size_t num_agents{0};

    // Formula evaluation 
    // Returns true iff formula holds at world w in this model
    bool holds_at(const Formula& f, WorldIdx w) const;

    // Returns true iff formula holds in the multi-pointed state
    // (i.e. at all designated worlds)
    bool satisfies(const Formula& f) const;

    // Hashing (for visited-state detection in search) 
    size_t hash() const;
    bool operator==(const EpistemicState& o) const;

    void print(const std::vector<std::string>& atom_names,
               const std::vector<std::string>& agent_names) const;
};
