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
#include "plansys2_epistemic_planner/task.hpp"

#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

// Automatic selection of search strategy and heuristic, expressed as data
// rather than as a chain of `if` statements over magic numbers.
//
// The thresholds in the built-in policy are tuned to one 15-instance benchmark
// suite. Encoding them as control flow made three things impossible: retuning
// without a rebuild, seeing *which* condition decided a run, and reporting the
// policy alongside the results it produced. A rule table fixes all three, and
// costs nothing at runtime — selection happens once, before search starts.
//
// Evaluation is first-match-wins over an ordered list. Each rule is a
// conjunction of comparisons against named task features; a rule with no
// conditions always matches and so acts as a terminal default. Disjunction is
// expressed by listing two rules with the same outcome, which keeps the
// condition language small enough to validate exhaustively at load time.

// Numeric view of a task's structure. Every quantity a rule may test lives
// here; booleans are 0/1 so a single comparison form covers them too.
struct TaskFeatures {
    double sensing               = 0;  // any action with |E_d| > 1
    double max_designated_events = 0;  // max |E_d| over actions
    double worlds                = 0;  // |W| in the initial state
    double designated            = 0;  // |W*| in the initial state
    double actions               = 0;  // number of ground actions
    double agents                = 0;
    double atoms                 = 0;
    double goal_modal_depth      = 0;  // deepest nesting of [i] / C_G / Kw
    double goal_kw_only          = 0;
    double goal_has_atom_conjunct= 0;
    double kd45                  = 0;  // 0 = S5 frame, 1 = KD45 frame
    double partial_obs           = 0;

    static TaskFeatures extract(const PlanningTask& task);

    // Nullopt for an unknown name — the caller turns that into a load error
    // naming the offending feature, never a silent false.
    [[nodiscard]] std::optional<double> lookup(std::string_view name) const;

    // Every legal feature name, for validation and for --print-policy.
    [[nodiscard]] static std::span<const std::string_view> names();
};

enum class Comparison { Le, Lt, Ge, Gt, Eq, Ne };

[[nodiscard]] std::optional<Comparison> parse_comparison(std::string_view op);
[[nodiscard]] std::string_view          comparison_name(Comparison op);

struct Condition {
    std::string feature;
    Comparison  op = Comparison::Le;
    double      value = 0;

    [[nodiscard]] bool holds(const TaskFeatures& f) const;
    [[nodiscard]] std::string describe() const;
};

struct SelectionRule {
    std::string            name;     // shown in the decision trace
    std::string            outcome;  // strategy or heuristic label
    std::vector<Condition> all;      // conjunction; empty ⇒ always matches

    [[nodiscard]] bool matches(const TaskFeatures& f) const;
};

// Which rule fired, and what it chose. `rule` is empty only when a rule list
// is itself empty, which load-time validation rejects.
struct Decision {
    std::string outcome;
    std::string rule;
};

[[nodiscard]] Decision select(const std::vector<SelectionRule>& rules,
                              const TaskFeatures&               features);

struct SelectionPolicy {
    std::vector<SelectionRule> strategy_rules;
    std::vector<SelectionRule> heuristic_rules;

    // The defaults reproduce the hand-written selectors exactly; see
    // src/selection_policy.cpp for the per-rule rationale.
    [[nodiscard]] static SelectionPolicy builtin();

    // Throws std::runtime_error with a message naming the offending rule and
    // field. A policy that does not validate must not fall back to the
    // built-in one: silently planning under a different policy than the file
    // asked for is worse than refusing to start.
    [[nodiscard]] static SelectionPolicy load(const std::string& path);

    [[nodiscard]] std::string to_json() const;
};

// Legal outcome labels, also accepted by --strategy and --heuristic.
[[nodiscard]] std::span<const std::string_view> strategy_labels();
[[nodiscard]] std::span<const std::string_view> heuristic_labels();
