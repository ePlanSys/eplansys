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
#include "plansys2_epistemic_planner/search.hpp"
#include "plansys2_epistemic_planner/task.hpp"
#include <string>
#include <vector>

struct ValidationResult {
    bool valid{false};
    std::string error;
    size_t branches_checked{0};
    size_t leaves_reached{0};
};

// Replay a conditional plan tree against the task.
// Checks that:
//   - every action is applicable in the state it is applied to
//   - every sensing branch matches an event that fires in that state
//   - every leaf (null subtree) satisfies the goal
ValidationResult validate(const PlanningTask& task,
                          const std::shared_ptr<PlanNode>& plan_tree);
