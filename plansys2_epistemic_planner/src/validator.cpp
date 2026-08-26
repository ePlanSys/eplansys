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

#include "plansys2_epistemic_planner/validator.hpp"
#include "plansys2_epistemic_planner/product_update.hpp"
#include "plansys2_epistemic_planner/bisimulation.hpp"
#include <sstream>

static void replay(const EpistemicState& s,
                   const std::shared_ptr<PlanNode>& node,
                   const PlanningTask& task,
                   ValidationResult& result) {

    // Null node means this branch is at goal
    if (!node) {
        result.leaves_reached++;
        if (!s.satisfies(*task.goal)) {
            result.valid = false;
            std::ostringstream oss;
            oss << "Leaf reached but goal not satisfied ("
                << result.leaves_reached << " leaves so far)";
            result.error = oss.str();
        }
        return;
    }

    // Find the action by name
    const Action* action = nullptr;
    for (auto& a : task.actions) {
        if (a.name == node->action) { action = &a; break; }
    }
    if (!action) {
        result.valid = false;
        result.error = "Action not found in task: " + node->action;
        return;
    }

    //   - ontic actions: ∀ designated worlds must satisfy precondition
    //   - sensing actions: ∃ (world, event) pair satisfies precondition
    // This ensures the validator rejects plans the planner should never have produced.
    if (!action->applicable(s)) {
        result.valid = false;
        result.error = "Action not applicable (conformant check failed): " + node->action;
        return;
    }

    // Split product update — one branch per designated event
    auto branches = product_update_split(s, *action);
    if (branches.empty()) {
        result.valid = false;
        result.error = "product_update_split returned empty for: " + node->action;
        return;
    }

    result.branches_checked++;

    // Match each plan branch to a product update branch by EventIdx
    for (auto& [plan_eid, subtree] : node->branches) {
        bool found = false;
        for (auto& [actual_eid, branch_state] : branches) {
            if (actual_eid != plan_eid) continue;
            found = true;
            EpistemicState contracted = bisim_contract(branch_state);
            replay(contracted, subtree, task, result);
            if (!result.valid) return;
            break;
        }
        if (!found) {
            result.valid = false;
            std::ostringstream oss;
            oss << "Plan branch event " << plan_eid
                << " not produced by action " << node->action;
            result.error = oss.str();
            return;
        }
    }
}

ValidationResult validate(const PlanningTask& task,
                          const std::shared_ptr<PlanNode>& plan_tree) {
    ValidationResult result;
    result.valid = true;

    EpistemicState init = bisim_contract(task.init);

    if (!plan_tree) {
        // Empty plan — goal must hold in initial state
        result.leaves_reached = 1;
        if (!init.satisfies(*task.goal)) {
            result.valid = false;
            result.error = "Empty plan but initial state does not satisfy goal";
        }
        return result;
    }

    replay(init, plan_tree, task, result);
    return result;
}