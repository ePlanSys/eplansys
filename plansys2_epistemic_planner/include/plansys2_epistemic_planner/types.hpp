#pragma once
#include <vector>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <memory>
#include <optional>
#include <variant>
#include <cstdint>

// Atom / agent indices
using AtomIdx  = uint32_t;
using AgentIdx = uint32_t;
using WorldIdx = uint32_t;
using EventIdx = uint32_t;
using ActionIdx= uint32_t;

// Adjacency list: world -> set of reachable worlds 
using RelRow   = std::unordered_set<WorldIdx>;
using Relation = std::vector<RelRow>;     // [world_idx] -> reachable

// Forward declarations 
struct Formula;
struct Action;
struct EpistemicState;
using FormulaPtr = std::shared_ptr<Formula>;
