#include "plansys2_epistemic_planner/bisimulation.hpp"
#include <algorithm>
#include <numeric>
#include <map>

// Bisimulation contraction via partition refinement 
//
// Two worlds w, v are bisimilar iff:
//   1. V(w) = V(v)                            (same atom valuation)
//   2. For every agent i and every u accessible from w via R_i,
//      there exists u' accessible from v via R_i that is bisimilar to u,
//      and vice-versa.
//
// Algorithm: start with partition P = {atoms-based equivalence classes},
// then refine by splitting classes that violate condition 2, until stable.
// Result: merge worlds in the same class.
//
EpistemicState bisim_contract(EpistemicState s) {
    size_t nw = s.worlds.size();
    size_t na = s.num_agents;

    if (nw == 0) return s;

    // Initial partition: group worlds by atom set 
    // class_of[w] = class index
    std::vector<int> class_of(nw);
    {
        std::map<std::vector<AtomIdx>, int> sig_to_class;
        for (WorldIdx w = 0; w < nw; w++) {
            std::vector<AtomIdx> sig(s.worlds[w].atoms.begin(),
                                     s.worlds[w].atoms.end());
            std::sort(sig.begin(), sig.end());
            auto [it, inserted] = sig_to_class.emplace(sig, (int)sig_to_class.size());
            class_of[w] = it->second;
        }
    }

    //  Refinement loop 
    bool changed = true;
    while (changed) {
        changed = false;
        // Build new partition: split by (current_class, {class sets of neighbours})
        // signature of w = (class_of[w],
        //                   for each agent i: sorted set of classes of R_i(w))
        std::map<std::vector<int>, int> sig_to_new;
        std::vector<int> new_class(nw);

        for (WorldIdx w = 0; w < nw; w++) {
            std::vector<int> sig;
            sig.push_back(class_of[w]);
            for (AgentIdx ag = 0; ag < na; ag++) {
                // sorted set of class indices of neighbours
                std::vector<int> nbr_classes;
                for (WorldIdx v : s.accessibility[ag][w])
                    nbr_classes.push_back(class_of[v]);
                std::sort(nbr_classes.begin(), nbr_classes.end());
                nbr_classes.erase(std::unique(nbr_classes.begin(),
                                              nbr_classes.end()),
                                  nbr_classes.end());
                sig.insert(sig.end(), nbr_classes.begin(), nbr_classes.end());
                sig.push_back(-1);
            }
            auto [it, inserted] = sig_to_new.emplace(sig, (int)sig_to_new.size());
            new_class[w] = it->second;
        }

        if (new_class != class_of) {
            class_of = new_class;
            changed  = true;
        }
    }

    //  Build contracted state
    // One representative world per class
    int num_classes = *std::max_element(class_of.begin(), class_of.end()) + 1;
    std::vector<WorldIdx> repr(num_classes, nw);    // nw = invalid sentinel
    for (WorldIdx w = 0; w < nw; w++) {
        int c = class_of[w];
        if (repr[c] == nw) repr[c] = w;             // first world of class = representative
    }

    EpistemicState result;
    result.num_agents = na;

    // New worlds
    for (int c = 0; c < num_classes; c++) {
        WorldIdx rw = repr[c];
        World w;
        w.id    = static_cast<WorldIdx>(c);
        w.atoms = s.worlds[rw].atoms;
        result.worlds.push_back(std::move(w));
    }

    // Designated worlds: classes that contain at least one designated world
    for (WorldIdx w : s.designated)
        result.designated.insert(static_cast<WorldIdx>(class_of[w]));

    // Accessibility: R'_i(c1) contains c2 iff R_i(repr[c1]) contains repr[c2]
    result.accessibility.resize(na, Relation(num_classes));
    for (AgentIdx ag = 0; ag < na; ag++) {
        for (int c = 0; c < num_classes; c++) {
            WorldIdx rw = repr[c];
            for (WorldIdx v : s.accessibility[ag][rw]) {
                result.accessibility[ag][c].insert(
                    static_cast<WorldIdx>(class_of[v]));
            }
        }
    }

    return result;
}