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
