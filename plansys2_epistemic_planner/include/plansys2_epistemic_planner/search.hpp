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
#include "plansys2_epistemic_planner/heuristic.hpp"
#include "plansys2_epistemic_planner/outcome.hpp"
#include "plansys2_epistemic_planner/task.hpp"

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>

// Shared planner instrumentation.
//
// Beyond effort counters, this records why branches were discarded. The product
// update can decline to produce a successor for three distinct reasons — the
// action was inapplicable, the pre-contraction world bound fired, or KD45
// seriality repair emptied the designated set — and only the first is a
// property of the domain. Collapsing them (as a bare nullopt did) made it
// impossible to tell a genuinely dead branch from one the planner chose to
// prune, which matters when reading a failed run.
struct PlannerStats {
    std::size_t nodes_expanded{0};
    std::size_t nodes_generated{0};

    std::size_t dead_ends{0};
    std::size_t duplicates_pruned{0};

    std::size_t pruned_world_cap{0};
    std::size_t pruned_non_serial{0};
    std::size_t pruned_inapplicable{0};

    std::size_t heuristic_calls{0};
    std::size_t heuristic_improvements{0};
    std::size_t heuristic_stalls{0};

    std::size_t plateau_escapes{0};

    std::size_t max_frontier_size{0};
    std::size_t closed_size{0};

    // Bytes of Kripke-model storage held live by the search at its peak. With
    // the bit-matrix representation this is the planner's dominant allocation,
    // so it is the number worth reporting.
    std::size_t peak_state_bytes{0};

    float initial_h{0.f};
    float best_h{0.f};
    float final_h{0.f};

    double elapsed_sec{0.0};

    std::chrono::steady_clock::time_point start_time;

    void start_timer() { start_time = std::chrono::steady_clock::now(); }

    void stop_timer() {
        elapsed_sec = std::chrono::duration<double>(
                          std::chrono::steady_clock::now() - start_time).count();
    }

    void record_prune(PruneReason r) noexcept {
        switch (r) {
            case PruneReason::WorldCapExceeded: ++pruned_world_cap;    break;
            case PruneReason::NonSerial:        ++pruned_non_serial;   break;
            case PruneReason::Inapplicable:     ++pruned_inapplicable; break;
            case PruneReason::None:                                    break;
        }
    }
};

// Linear-plan search result.
struct SearchResult {
    std::vector<std::string> plan;
    PlannerStats             stats;
};

using Deadline = std::chrono::steady_clock::time_point;

namespace gbfs {

// Greedy best-first search over bisimulation-contracted, canonically-labelled
// states. Duplicate detection is by 128-bit fingerprint, which is exact up to
// bisimilarity: two states representing the same epistemic situation under
// different world numberings are recognised as one.
//
// max_nodes: expansion limit (0 = unlimited)
// deadline:  wall-clock deadline, checked once per expansion
[[nodiscard]] std::optional<SearchResult>
search(const PlanningTask& task, const Heuristic& h,
       std::size_t max_nodes = 0, Deadline deadline = Deadline::max());

} // namespace gbfs

// Conditional plans (AND-OR search).

// A node in a conditional plan.
//
// `branches` holds one entry per sensing outcome, tagged by the event that
// fired. Ontic actions have exactly one branch. A null subtree pointer means
// that branch is already at the goal.
struct PlanNode {
    std::string action;
    std::vector<std::pair<EventIdx, std::shared_ptr<PlanNode>>> branches;
};

struct ConditionalSearchResult {
    std::shared_ptr<PlanNode> plan_tree;   // null = already at goal
    PlannerStats              stats;
};

namespace aostar {

// Iterative-deepening AND-OR search with a memo that persists across the
// deepening iterations.
//
// max_depth: depth limit (0 = unlimited)
// deadline:  wall-clock deadline
[[nodiscard]] std::optional<ConditionalSearchResult>
search(const PlanningTask& task, const Heuristic& h,
       std::size_t max_depth = 0, Deadline deadline = Deadline::max());

} // namespace aostar

namespace ehc {

// Enforced hill climbing. Descends to any h-improving successor; on a plateau,
// runs a breadth-first search for the nearest strictly better state, sharing
// its visited set with the descent so the two phases cannot cycle against each
// other.
//
// max_nodes: expansion limit across both phases (0 = unlimited)
[[nodiscard]] std::optional<SearchResult>
search(const PlanningTask& task, const Heuristic& h,
       std::size_t max_nodes = 0, Deadline deadline = Deadline::max());

} // namespace ehc
