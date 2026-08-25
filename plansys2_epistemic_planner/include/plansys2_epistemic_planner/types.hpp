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
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// Atom / agent / world / event indices.
using AtomIdx   = std::uint32_t;
using AgentIdx  = std::uint32_t;
using WorldIdx  = std::uint32_t;
using EventIdx  = std::uint32_t;
using ActionIdx = std::uint32_t;

// Sentinel for "no such world", used by the dense (world, event) product table.
inline constexpr WorldIdx kNoWorld = std::numeric_limits<WorldIdx>::max();

// Forward declarations
struct Formula;
struct Action;
struct EpistemicState;
using FormulaPtr = std::shared_ptr<Formula>;
