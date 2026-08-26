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
#include "plansys2_epistemic_planner/action.hpp"
#include "plansys2_epistemic_planner/outcome.hpp"
#include "plansys2_epistemic_planner/state.hpp"
#include "plansys2_epistemic_planner/world_cap_policy.hpp"

#include <vector>

// Result of a DEL product update: the updated state together with the dense
// (world, event) → new-world table used to build it.
//
// The table was previously an unordered_map keyed on a packed uint64. It is
// probed in the innermost loop of the relation construction — once per
// (agent, source world, source event, successor world, successor event) — so a
// hash lookup there dominated the update. The product index space is dense and
// small (|W| · |E|), so a flat vector with a sentinel is both smaller and O(1)
// with no hashing.
//
// After KD45 seriality repair the surviving worlds are compacted so that world
// ids are again 0..|W'|-1; the table is patched in place, and entries whose
// world did not survive are reset to kNoWorld.
struct ProductUpdateResult {
    EpistemicState        state;
    std::vector<WorldIdx> pair_to_idx;    // size |W| · num_events
    std::uint32_t         num_events{0};

    [[nodiscard]] WorldIdx at(WorldIdx w, EventIdx e) const noexcept {
        return pair_to_idx[std::size_t(w) * num_events + e];
    }
};

// Compute s ⊗ a (DEL product update).
[[nodiscard]] Outcome<ProductUpdateResult>
product_update_with_map(const EpistemicState& s, const Action& a,
                        bool enforce_kd45 = false,
                        const WorldCapPolicy& cap = make_world_cap_policy(false));

[[nodiscard]] Outcome<EpistemicState>
product_update(const EpistemicState& s, const Action& a,
               bool enforce_kd45 = false,
               const WorldCapPolicy& cap = make_world_cap_policy(false));

// Sensing update: one state per designated event, all sharing the same product
// model and differing only in which worlds are designated.
[[nodiscard]] std::vector<std::pair<EventIdx, EpistemicState>>
product_update_split(const EpistemicState& s, const Action& a,
                     bool enforce_kd45 = false,
                     const WorldCapPolicy& cap = make_world_cap_policy(false));
