#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <iostream>
#include <algorithm>
#include <climits>
#include <chrono>

#include "plansys2_epistemic_planner/search.hpp"
#include "plansys2_epistemic_planner/product_update.hpp"
#include "plansys2_epistemic_planner/bisimulation.hpp"

namespace gbfs {

struct Node {
    EpistemicState state;
    std::vector<std::string> plan;
    float h;

    bool operator>(const Node& o) const { return h > o.h; }
};

// Greedy Best-First Search over bisimulation-contracted epistemic states.
// The closed list is keyed by state hash with collision buckets for structural
// equality, since bisimulation contraction does not guarantee hash uniqueness.
// Returns the first plan found, or nullopt if the reachable space is exhausted
// or the node limit is exceeded.
std::optional<SearchResult> search(const PlanningTask& task,
                                   const Heuristic& h,
                                   size_t max_nodes) {
    SearchResult result;
    result.stats.start_timer();

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;
    std::unordered_map<size_t, std::vector<EpistemicState>> closed;

    EpistemicState init = bisim_contract(task.init);
    if (init.satisfies(*task.goal)) {
        result.plan = {};
        result.stats.initial_h = 0.f;
        result.stats.best_h    = 0.f;
        result.stats.final_h   = 0.f;
        result.stats.stop_timer();
        return result;
    }

    result.stats.heuristic_calls++;
    float init_h = h(init, task);
    result.stats.initial_h = init_h;
    result.stats.best_h    = init_h;
    result.stats.final_h   = init_h;

    open.push({std::move(init), {}, init_h});
    result.stats.max_frontier_size =
        std::max(result.stats.max_frontier_size, open.size());

    while (!open.empty()) {
        Node node = open.top();
        open.pop();

        result.stats.nodes_expanded++;

        if (max_nodes > 0 && result.stats.nodes_expanded > max_nodes) {
            std::cerr << "[gbfs] Node limit reached (" << max_nodes << ").\n";
            result.stats.stop_timer();
            return std::nullopt;
        }

        size_t hsh = node.state.hash();
        {
            auto& bucket = closed[hsh];
            bool already_seen = std::any_of(bucket.begin(), bucket.end(),
                [&](const EpistemicState& s){ return s == node.state; });
            if (already_seen) continue;
            bucket.push_back(node.state);
        }

        bool generated_successor = false;

        for (auto& action : task.actions) {
            if (!action.applicable(node.state)) continue;

            auto maybe_next = product_update(node.state, action, task.kd45);
            if (!maybe_next) continue;

            EpistemicState next = bisim_contract(std::move(*maybe_next));
            generated_successor = true;
            result.stats.nodes_generated++;

            std::vector<std::string> new_plan = node.plan;
            new_plan.push_back(action.name);

            if (next.satisfies(*task.goal)) {
                result.plan = std::move(new_plan);
                result.stats.final_h = 0.f;

                std::cerr << "[gbfs] Solution found! Length=" << result.plan.size()
                          << "  Expanded=" << result.stats.nodes_expanded
                          << "  Generated=" << result.stats.nodes_generated
                          << "  HeuristicCalls=" << result.stats.heuristic_calls
                          << "  BestH=" << result.stats.best_h
                          << "  FrontierMax=" << result.stats.max_frontier_size
                          << "\n";

                result.stats.stop_timer();
                return result;
            }

            result.stats.heuristic_calls++;
            float hval = h(next, task);
            result.stats.final_h = hval;

            if (hval < result.stats.best_h) {
                result.stats.best_h = hval;
                result.stats.heuristic_improvements++;
            } else {
                result.stats.heuristic_stalls++;
            }

            size_t nhsh = next.hash();
            auto it = closed.find(nhsh);
            bool seen = it != closed.end() &&
                std::any_of(it->second.begin(), it->second.end(),
                    [&](const EpistemicState& s){ return s == next; });

            if (!seen) {
                open.push({std::move(next), std::move(new_plan), hval});
                result.stats.max_frontier_size =
                    std::max(result.stats.max_frontier_size, open.size());
            }
        }

        if (!generated_successor)
            result.stats.dead_ends++;
    }

    std::cerr << "[gbfs] Search exhausted  no solution.\n";
    result.stats.stop_timer();
    return std::nullopt;
}

} // namespace gbfs

namespace aostar {

// Memo table maps (state_hash, depth_remaining) -> bool.
// A false entry records a proven failure: no solution exists from that state
// within that depth bound, so re-expansion is skipped on future encounters.
using MemoKey = std::pair<size_t, size_t>;
struct MemoKeyHash {
    size_t operator()(const MemoKey& k) const {
        return k.first ^ (k.second * 0x9e3779b97f4a7c15ULL);
    }
};
using MemoTable = std::unordered_map<MemoKey, bool, MemoKeyHash>;

// Ranks applicable actions by heuristic value of their raw successor state.
// Uses product_update (not product_update_split) for ranking since we only
// need a single aggregate heuristic estimate per action, not per-branch states.
static std::vector<const Action*>
rank_actions(const EpistemicState& s,
             const PlanningTask& task,
             const Heuristic& h,
             PlannerStats& stats) {
    std::vector<std::pair<float, const Action*>> ranked;
    for (auto& a : task.actions) {
        if (!a.applicable_weak(s)) continue;
        auto maybe = product_update(s, a, task.kd45);
        if (!maybe) continue;

        stats.heuristic_calls++;
        float hv = h(*maybe, task);
        ranked.emplace_back(hv, &a);
        stats.best_h = std::min(stats.best_h, hv);
    }

    std::sort(ranked.begin(), ranked.end(),
              [](auto& x, auto& y){ return x.first < y.first; });

    std::vector<const Action*> out;
    out.reserve(ranked.size());
    for (auto& [_, a] : ranked) out.push_back(a);
    return out;
}

// Iterative-deepening AND-OR DFS for conditional epistemic plans.
//
// Return value encoding:
//   nullopt             — failure (no solution within depth / pruned)
//   optional(nullptr)   — success: this state already satisfies the goal
//   optional(node_ptr)  — success: node_ptr is the plan subtree
//
// The distinction between nullopt and optional(nullptr) is critical.
// The caller checks result.has_value() to detect success, then checks
// whether *result is non-null to detect the goal-leaf case.
static std::optional<std::shared_ptr<PlanNode>>
and_or_dfs(const EpistemicState& s,
           size_t depth,
           const PlanningTask& task,
           const Heuristic& h,
           PlannerStats& stats,
           MemoTable& memo,
           std::unordered_set<size_t>& ancestors,
           std::chrono::steady_clock::time_point deadline) {

    stats.nodes_expanded++;

    // Wall-clock budget check — avoids blocking on deep subtrees.
    if (std::chrono::steady_clock::now() >= deadline)
        return std::nullopt;

    // Prune if this state hash already appears on the current DFS path.
    // Hash collisions may cause rare false positives (valid states pruned),
    // but this is acceptable given the correctness guarantee from the memo table.
    size_t shash = s.hash();
    if (ancestors.count(shash)) return std::nullopt;

    // Goal reached: return optional(nullptr) to signal success without a subtree.
    // This is distinct from nullopt (failure) — the parent checks has_value().
    if (s.satisfies(*task.goal))
        return std::optional<std::shared_ptr<PlanNode>>{nullptr};

    if (depth == 0)
        return std::nullopt;

    MemoKey key{shash, depth};
    auto mit = memo.find(key);
    if (mit != memo.end()) {
        if (!mit->second) return std::nullopt;
    }

    auto candidates = rank_actions(s, task, h, stats);

    bool generated_any = false;

    for (const Action* action : candidates) {
        // Strong applicability check: ontic actions require ∀ designated worlds
        // to satisfy the precondition. rank_actions used applicable_weak (∃)
        // for speed; we re-check with the conformant criterion here before
        // committing to a branch.
        if (!action->applicable(s)) continue;

        // product_update_split reuses the same pair_to_idx produced internally
        // by product_update, so the branch world IDs are consistent with the
        // full update and with each other.
        auto branches = product_update_split(s, *action, task.kd45);
        if (branches.empty()) continue;

        generated_any = true;
        stats.nodes_generated += branches.size();

        auto node = std::make_shared<PlanNode>();
        node->action = action->name;
        bool all_ok = true;

        for (auto& [eid, branch_state] : branches) {
            EpistemicState contracted = bisim_contract(branch_state);

            // Insert current hash into the ancestor set before recursing and
            // remove it on return, maintaining the invariant that ancestors
            // reflects exactly the states on the active DFS path.
            ancestors.insert(shash);
            auto child = and_or_dfs(contracted, depth - 1, task, h, stats,
                                    memo, ancestors, deadline);
            ancestors.erase(shash);

            // child.has_value() == true  means the branch is solved.
            // child.has_value() == false means failure on this branch.
            // *child == nullptr          means the branch reached the goal
            //                            immediately (leaf); store nullptr.
            if (!child.has_value()) {
                all_ok = false;
                break;
            }
            node->branches.emplace_back(eid, *child);
        }

        if (all_ok) {
            memo[key] = true;
            return node;
        }
    }

    if (!generated_any)
        stats.dead_ends++;

    memo[key] = false;
    return std::nullopt;
}

std::optional<ConditionalSearchResult>
search(const PlanningTask& task,
       const Heuristic& h,
       size_t max_depth,
       std::chrono::steady_clock::time_point deadline) {

    EpistemicState init = bisim_contract(task.init);

    ConditionalSearchResult out;
    out.stats.start_timer();

    if (init.satisfies(*task.goal)) {
        out.plan_tree = nullptr;
        out.stats.initial_h = 0.f;
        out.stats.best_h    = 0.f;
        out.stats.final_h   = 0.f;
        out.stats.stop_timer();
        return out;
    }

    out.stats.heuristic_calls++;
    float init_h = h(init, task);
    out.stats.initial_h = init_h;
    out.stats.best_h    = init_h;
    out.stats.final_h   = init_h;

    size_t depth_limit = (max_depth == 0) ? SIZE_MAX : max_depth;

    // Early termination: if no new nodes were expanded beyond the root at
    // depth d, no action was applicable and deeper iterations are redundant.
    size_t last_expanded = 0;

    for (size_t depth = 0; depth <= depth_limit; depth++) {
        if (std::chrono::steady_clock::now() >= deadline) {
            std::cerr << "[aostar] Timeout at depth " << depth << " — wrote null.\n";
            out.stats.stop_timer();
            return std::nullopt;
        }

        std::cerr << "[aostar] Trying depth " << depth << "\n";
        MemoTable memo;
        std::unordered_set<size_t> ancestors;

        auto result = and_or_dfs(init, depth, task, h, out.stats,
                                 memo, ancestors, deadline);

        if (result.has_value()) {
            out.plan_tree = *result;
            std::cerr << "[aostar] Solution found at depth " << depth
                      << "  Expanded=" << out.stats.nodes_expanded
                      << "  Generated=" << out.stats.nodes_generated << "\n";
            out.stats.stop_timer();
            return out;
        }

        if (depth > 0 && out.stats.nodes_expanded == last_expanded + 1) {
            std::cerr << "[aostar] Search space exhausted at depth " << depth << ".\n";
            out.stats.stop_timer();
            return std::nullopt;
        }

        last_expanded = out.stats.nodes_expanded;
    }

    std::cerr << "[aostar] No solution within depth " << depth_limit << ".\n";
    out.stats.stop_timer();
    return std::nullopt;
}

} // namespace aostar

namespace ehc {

// Enforced Hill Climbing with BFS plateau escape.
//
// Greedy descent: at each step, all applicable successors are generated,
// ranked by heuristic value, and the best strictly improving state is
// committed to. Goal checks are performed before heuristic evaluation.
//
// Plateau escape: when no improving successor exists, a BFS is launched from
// the current state to find the nearest state with a strictly lower heuristic
// value. The BFS shares the visited map with the greedy phase to prevent
// re-expansion of already-committed states and avoid oscillation between
// plateau regions.
//
// Falls back to nullopt on exhaustion; main() then retries with GBFS.
std::optional<SearchResult> search(const PlanningTask& task,
                                   const Heuristic& h,
                                   size_t max_nodes) {
    SearchResult result;
    result.stats.start_timer();

    EpistemicState cur = bisim_contract(task.init);

    if (cur.satisfies(*task.goal)) {
        result.plan = {};
        result.stats.initial_h = 0.f;
        result.stats.best_h    = 0.f;
        result.stats.final_h   = 0.f;
        result.stats.stop_timer();
        return result;
    }

    std::vector<std::string> plan;

    // Single visited map shared between greedy descent and BFS escape.
    // States already committed to by the greedy phase are not re-entered
    // during BFS, preventing cycles across phase boundaries.
    std::unordered_map<size_t, std::vector<EpistemicState>> visited;
    visited[cur.hash()].push_back(cur);

    result.stats.heuristic_calls++;
    float cur_h = h(cur, task);
    result.stats.initial_h = cur_h;
    result.stats.best_h    = cur_h;
    result.stats.final_h   = cur_h;

    while (true) {
        if (max_nodes > 0 && result.stats.nodes_expanded > max_nodes) {
            std::cerr << "[ehc] Node limit reached (" << max_nodes << ").\n";
            result.stats.stop_timer();
            return std::nullopt;
        }

        struct Succ {
            EpistemicState state;
            std::string    action_name;
            float          hval;
        };
        std::vector<Succ> succs;

        for (auto& action : task.actions) {
            if (!action.applicable(cur)) continue;

            auto maybe_next = product_update(cur, action, task.kd45);
            if (!maybe_next) continue;

            EpistemicState next = bisim_contract(std::move(*maybe_next));
            result.stats.nodes_generated++;

            if (next.satisfies(*task.goal)) {
                plan.push_back(action.name);
                result.plan = std::move(plan);
                result.stats.nodes_expanded++;
                result.stats.final_h = 0.f;

                std::cerr << "[ehc] Solution found! Length=" << result.plan.size()
                          << "  Expanded=" << result.stats.nodes_expanded
                          << "  Generated=" << result.stats.nodes_generated << "\n";

                result.stats.stop_timer();
                return result;
            }

            result.stats.heuristic_calls++;
            float hval = h(next, task);
            result.stats.final_h = hval;
            if (hval < result.stats.best_h) {
                result.stats.best_h = hval;
                result.stats.heuristic_improvements++;
            } else {
                result.stats.heuristic_stalls++;
            }

            succs.push_back({std::move(next), action.name, hval});
        }

        // Sort ascending: the first entry strictly below cur_h is the best
        // available improvement. Entries at or above cur_h are skipped.
        std::sort(succs.begin(), succs.end(),
                  [](const Succ& a, const Succ& b){ return a.hval < b.hval; });

        bool improved = false;
        for (auto& s : succs) {
            if (s.hval >= cur_h) break;

            size_t nhsh = s.state.hash();
            auto it = visited.find(nhsh);
            bool seen = it != visited.end() &&
                std::any_of(it->second.begin(), it->second.end(),
                    [&](const EpistemicState& vs){ return vs == s.state; });
            if (seen) continue;

            visited[nhsh].push_back(s.state);
            plan.push_back(s.action_name);
            cur   = std::move(s.state);
            cur_h = s.hval;
            improved = true;
            result.stats.nodes_expanded++;
            break;
        }

        if (improved) continue;

        std::cerr << "[ehc] Plateau at h=" << cur_h << "  BFS escape...\n";

        struct BFSNode {
            EpistemicState state;
            std::vector<std::string> suffix;
        };

        std::queue<BFSNode> bfs;
        bfs.push({cur, {}});
        result.stats.max_frontier_size =
            std::max(result.stats.max_frontier_size, bfs.size());

        bool escaped = false;
        auto run_bfs = [&]() -> bool {
            while (!bfs.empty()) {
                auto node = std::move(bfs.front());
                bfs.pop();

                result.stats.nodes_expanded++;
                if (max_nodes > 0 && result.stats.nodes_expanded > max_nodes) {
                    std::cerr << "[ehc] Node limit reached during BFS escape.\n";
                    return false;
                }

                for (auto& action : task.actions) {
                    if (!action.applicable(node.state)) continue;

                    auto maybe_next = product_update(node.state, action, task.kd45);
                    if (!maybe_next) continue;

                    EpistemicState next = bisim_contract(std::move(*maybe_next));
                    result.stats.nodes_generated++;

                    if (next.satisfies(*task.goal)) {
                        for (auto& a : node.suffix) plan.push_back(a);
                        plan.push_back(action.name);
                        result.plan = std::move(plan);
                        result.stats.plateau_escapes++;
                        result.stats.final_h = 0.f;

                        std::cerr << "[ehc] Solution found during BFS escape! Length="
                                  << result.plan.size()
                                  << "  Expanded=" << result.stats.nodes_expanded
                                  << "  Generated=" << result.stats.nodes_generated << "\n";

                        result.stats.stop_timer();
                        return true;
                    }

                    result.stats.heuristic_calls++;
                    float nh = h(next, task);
                    result.stats.final_h = nh;

                    if (nh < result.stats.best_h) {
                        result.stats.best_h = nh;
                        result.stats.heuristic_improvements++;
                    } else {
                        result.stats.heuristic_stalls++;
                    }

                    size_t nhsh = next.hash();

                    auto it = visited.find(nhsh);
                    bool seen = it != visited.end() &&
                        std::any_of(it->second.begin(), it->second.end(),
                            [&](const EpistemicState& vs){ return vs == next; });
                    if (seen) continue;

                    // Register in the shared visited map before deciding whether
                    // this is the escape state, so the greedy phase that follows
                    // will not re-enter it.
                    visited[nhsh].push_back(next);

                    if (nh < cur_h) {
                        for (auto& a : node.suffix) plan.push_back(a);
                        plan.push_back(action.name);
                        cur   = std::move(next);
                        cur_h = nh;
                        result.stats.plateau_escapes++;
                        return true;
                    }

                    std::vector<std::string> new_suffix = node.suffix;
                    new_suffix.push_back(action.name);
                    bfs.push({std::move(next), std::move(new_suffix)});
                    result.stats.max_frontier_size =
                        std::max(result.stats.max_frontier_size, bfs.size());
                }
            }
            return false;
        };

        escaped = run_bfs();

        if (!escaped) {
            std::cerr << "[ehc] BFS escape exhausted  no solution.\n";
            result.stats.stop_timer();
            return std::nullopt;
        }
    }
}

} // namespace ehc