#pragma once
#include "plansys2_epistemic_planner/task.hpp"
#include "plansys2_epistemic_planner/heuristic.hpp"
#include <optional>
#include <vector>
#include <string>
#include <memory>
#include <chrono>

// Shared planner instrumentation and runtime statistics.
// Tracks search effort, heuristic behavior, frontier growth,
// plateau escapes, and total runtime.
struct PlannerStats {
    size_t nodes_expanded{0};
    size_t nodes_generated{0};

    size_t dead_ends{0};

    size_t heuristic_calls{0};
    size_t heuristic_improvements{0};
    size_t heuristic_stalls{0};

    size_t plateau_escapes{0};

    size_t max_frontier_size{0};

    float initial_h{0.f};
    float best_h{0.f};
    float final_h{0.f};

    double elapsed_sec{0.0};

    std::chrono::steady_clock::time_point start_time;

    void start_timer() {
        start_time = std::chrono::steady_clock::now();
    }

    void stop_timer() {
        elapsed_sec =
            std::chrono::duration<double>(
                std::chrono::steady_clock::now() - start_time
            ).count();
    }
};

// Linear-plan search result.
struct SearchResult {
    std::vector<std::string> plan;
    PlannerStats stats;
};

namespace gbfs {

// Greedy Best-First Search.
// Returns the plan (action name sequence) on success, or nullopt if no
// plan exists within the given limits.
//
// max_nodes: expansion limit (0 = unlimited)
std::optional<SearchResult> search(const PlanningTask& task,
                                   const Heuristic& h,
                                   size_t max_nodes = 0);

} // namespace gbfs

// Conditional plan (AND-OR search)

// A node in a conditional plan tree.
// - action: the action to execute at this point
// - branches: one entry per sensing outcome (EventIdx tags which event fired).
//   For ontic actions there is exactly one branch with EventIdx = the single
//   designated event. For sensing actions there is one branch per designated
//   event whose precondition was satisfiable.
// - A null PlanNode pointer in a branch means that branch is already at goal.
struct PlanNode {
    std::string action;
    std::vector<std::pair<EventIdx, std::shared_ptr<PlanNode>>> branches;
};

// Conditional-plan search result.
struct ConditionalSearchResult {
    std::shared_ptr<PlanNode> plan_tree;   // null = already at goal
    PlannerStats stats;
};

namespace aostar {

using Deadline = std::chrono::steady_clock::time_point;

// Iterative-deepening AND-OR search.
// Returns a conditional plan tree on success, or nullopt if no plan exists
// within the given depth limit or before the deadline is reached.
//
// max_depth: depth limit (0 = unlimited, use with care)
// deadline:  wall-clock deadline; defaults to max (no timeout)
std::optional<ConditionalSearchResult>
search(const PlanningTask& task,
       const Heuristic& h,
       size_t max_depth = 0,
       Deadline deadline = Deadline::max());

} // namespace aostar

// Enforced Hill Climbing.
// Greedily follows any h-improving successor. When stuck on a plateau,
// runs a BFS to escape to the nearest state with strictly lower h.
// Complete on solvable problems. Faster than GBFS on well-guided domains.
//
// max_nodes: expansion limit across both greedy and BFS phases (0 = unlimited)
namespace ehc {

std::optional<SearchResult> search(const PlanningTask& task,
                                   const Heuristic& h,
                                   size_t max_nodes = 0);

} // namespace ehc