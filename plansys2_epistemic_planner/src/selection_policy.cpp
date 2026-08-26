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

#include "plansys2_epistemic_planner/selection_policy.hpp"
#include "plansys2_epistemic_planner/heuristic.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <fstream>
#include <sstream>
#include <stdexcept>

using nlohmann::json;

namespace {

int modal_depth(const Formula& f) {
    switch (f.kind) {
        case FormulaKind::Belief:
        case FormulaKind::Common:
        case FormulaKind::Kw:
            return f.children.empty() ? 1 : 1 + modal_depth(*f.children[0]);
        case FormulaKind::And:
        case FormulaKind::Or: {
            int d = 0;
            for (auto& c : f.children)
                d = std::max(d, modal_depth(*c));
            return d;
        }
        default:
            return 0;
    }
}

constexpr std::array<std::string_view, 12> kFeatureNames{
    "sensing", "max_designated_events", "worlds", "designated",
    "actions", "agents", "atoms", "goal_modal_depth",
    "goal_kw_only", "goal_has_atom_conjunct", "kd45", "partial_obs",
};

constexpr std::array<std::string_view, 3> kStrategyLabels{"gbfs", "ehc", "aostar"};

constexpr std::array<std::string_view, 6> kHeuristicLabels{
    "ug", "ed", "ks", "wc", "rpg", "radd"};

// A rule with no conditions is a terminal default; anything after it is dead.
// Building rules by hand makes that easy to get wrong, so both the built-in
// table and every loaded file are checked for it.
void check_reachable(const std::vector<SelectionRule>& rules, const char* what) {
    for (size_t i = 0; i + 1 < rules.size(); i++)
        if (rules[i].all.empty())
            throw std::runtime_error(
                std::string(what) + " rule '" + rules[i].name +
                "' has no conditions, so it always matches and the " +
                std::to_string(rules.size() - i - 1) +
                " rule(s) after it can never fire");
}

void check_terminal(const std::vector<SelectionRule>& rules, const char* what) {
    if (rules.empty())
        throw std::runtime_error(std::string(what) + " rule list is empty");
    if (!rules.back().all.empty())
        throw std::runtime_error(
            std::string(what) + " rule list does not end in an unconditional "
            "default rule, so some tasks would select nothing");
}

void check_labels(const std::vector<SelectionRule>& rules,
                  std::span<const std::string_view> legal,
                  const char* what) {
    for (auto& r : rules) {
        if (std::find(legal.begin(), legal.end(), r.outcome) == legal.end()) {
            std::ostringstream msg;
            msg << what << " rule '" << r.name << "' has unknown outcome '"
                << r.outcome << "'; expected one of:";
            for (auto& l : legal) msg << ' ' << l;
            throw std::runtime_error(msg.str());
        }
        for (auto& c : r.all) {
            if (!TaskFeatures{}.lookup(c.feature)) {
                std::ostringstream msg;
                msg << what << " rule '" << r.name << "' tests unknown feature '"
                    << c.feature << "'; expected one of:";
                for (auto& n : TaskFeatures::names()) msg << ' ' << n;
                throw std::runtime_error(msg.str());
            }
        }
    }
}

void validate(const std::vector<SelectionRule>& rules,
              std::span<const std::string_view> legal,
              const char* what) {
    check_labels(rules, legal, what);
    check_reachable(rules, what);
    check_terminal(rules, what);
}

SelectionRule rule(std::string name, std::string outcome,
                   std::vector<Condition> all) {
    return SelectionRule{std::move(name), std::move(outcome), std::move(all)};
}

Condition cond(std::string feature, Comparison op, double value) {
    return Condition{std::move(feature), op, value};
}

std::vector<SelectionRule> parse_rules(const json& j, const char* what) {
    if (!j.is_array())
        throw std::runtime_error(std::string(what) + " must be an array of rules");

    std::vector<SelectionRule> rules;
    rules.reserve(j.size());

    for (size_t i = 0; i < j.size(); i++) {
        const json& jr = j[i];
        const std::string where =
            std::string(what) + " rule #" + std::to_string(i);

        if (!jr.is_object())
            throw std::runtime_error(where + " must be an object");
        if (!jr.contains("outcome") || !jr["outcome"].is_string())
            throw std::runtime_error(where + " needs a string \"outcome\"");

        SelectionRule r;
        r.outcome = jr["outcome"].get<std::string>();
        r.name    = jr.value("name", where);

        if (jr.contains("when")) {
            const json& jw = jr["when"];
            if (!jw.is_array())
                throw std::runtime_error("rule '" + r.name +
                                         "': \"when\" must be an array");

            for (const json& jc : jw) {
                if (!jc.is_object() || !jc.contains("feature") ||
                    !jc.contains("op") || !jc.contains("value"))
                    throw std::runtime_error(
                        "rule '" + r.name +
                        "': each condition needs \"feature\", \"op\", \"value\"");

                auto op = parse_comparison(jc["op"].get<std::string>());
                if (!op)
                    throw std::runtime_error(
                        "rule '" + r.name + "': unknown operator '" +
                        jc["op"].get<std::string>() +
                        "'; expected <=, <, >=, >, ==, !=");

                if (!jc["value"].is_number())
                    throw std::runtime_error("rule '" + r.name +
                                             "': \"value\" must be a number "
                                             "(booleans are 0 and 1)");

                r.all.push_back(cond(jc["feature"].get<std::string>(), *op,
                                     jc["value"].get<double>()));
            }
        }
        rules.push_back(std::move(r));
    }
    return rules;
}

json dump_rules(const std::vector<SelectionRule>& rules) {
    json out = json::array();
    for (auto& r : rules) {
        json jr;
        jr["name"]    = r.name;
        jr["outcome"] = r.outcome;
        jr["when"]    = json::array();
        for (auto& c : r.all)
            jr["when"].push_back({{"feature", c.feature},
                                  {"op", std::string(comparison_name(c.op))},
                                  {"value", c.value}});
        out.push_back(std::move(jr));
    }
    return out;
}

}  // namespace

TaskFeatures TaskFeatures::extract(const PlanningTask& task) {
    TaskFeatures f;

    size_t max_ed = 0;
    for (auto& a : task.actions)
        max_ed = std::max(max_ed, a.designated_events.size());

    f.max_designated_events = static_cast<double>(max_ed);
    f.sensing               = max_ed > 1 ? 1 : 0;
    f.worlds                = static_cast<double>(task.init.num_worlds);
    f.designated            = static_cast<double>(task.init.num_designated());
    f.actions               = static_cast<double>(task.num_actions());
    f.agents                = static_cast<double>(task.num_agents());
    f.atoms                 = static_cast<double>(task.num_atoms());
    f.goal_modal_depth      = task.goal ? modal_depth(*task.goal) : 0;
    f.goal_kw_only          = task.goal_kw_only ? 1 : 0;
    f.goal_has_atom_conjunct =
        (task.goal && has_atom_conjunct(*task.goal)) ? 1 : 0;
    f.kd45        = task.kd45 ? 1 : 0;
    f.partial_obs = task.partial_obs ? 1 : 0;

    return f;
}

std::optional<double> TaskFeatures::lookup(std::string_view name) const {
    if (name == "sensing")                return sensing;
    if (name == "max_designated_events")  return max_designated_events;
    if (name == "worlds")                 return worlds;
    if (name == "designated")             return designated;
    if (name == "actions")                return actions;
    if (name == "agents")                 return agents;
    if (name == "atoms")                  return atoms;
    if (name == "goal_modal_depth")       return goal_modal_depth;
    if (name == "goal_kw_only")           return goal_kw_only;
    if (name == "goal_has_atom_conjunct") return goal_has_atom_conjunct;
    if (name == "kd45")                   return kd45;
    if (name == "partial_obs")            return partial_obs;
    return std::nullopt;
}

std::span<const std::string_view> TaskFeatures::names() {
    return kFeatureNames;
}

std::span<const std::string_view> strategy_labels()  { return kStrategyLabels; }
std::span<const std::string_view> heuristic_labels() { return kHeuristicLabels; }

std::optional<Comparison> parse_comparison(std::string_view op) {
    if (op == "<=") return Comparison::Le;
    if (op == "<")  return Comparison::Lt;
    if (op == ">=") return Comparison::Ge;
    if (op == ">")  return Comparison::Gt;
    if (op == "==") return Comparison::Eq;
    if (op == "!=") return Comparison::Ne;
    return std::nullopt;
}

std::string_view comparison_name(Comparison op) {
    switch (op) {
        case Comparison::Le: return "<=";
        case Comparison::Lt: return "<";
        case Comparison::Ge: return ">=";
        case Comparison::Gt: return ">";
        case Comparison::Eq: return "==";
        case Comparison::Ne: return "!=";
    }
    return "?";
}

bool Condition::holds(const TaskFeatures& f) const {
    auto v = f.lookup(feature);
    if (!v) return false;  // unreachable: validated at construction/load

    switch (op) {
        case Comparison::Le: return *v <= value;
        case Comparison::Lt: return *v <  value;
        case Comparison::Ge: return *v >= value;
        case Comparison::Gt: return *v >  value;
        case Comparison::Eq: return *v == value;
        case Comparison::Ne: return *v != value;
    }
    return false;
}

std::string Condition::describe() const {
    std::ostringstream os;
    os << feature << ' ' << comparison_name(op) << ' ' << value;
    return os.str();
}

bool SelectionRule::matches(const TaskFeatures& f) const {
    return std::all_of(all.begin(), all.end(),
                       [&](const Condition& c) { return c.holds(f); });
}

Decision select(const std::vector<SelectionRule>& rules,
                const TaskFeatures&               features) {
    for (auto& r : rules)
        if (r.matches(features))
            return Decision{r.outcome, r.name};
    return Decision{};
}

// The built-in policy. Each rule below is one branch of the selectors this
// replaced, in the same order, so the default behaviour is unchanged.
SelectionPolicy SelectionPolicy::builtin() {
    SelectionPolicy p;

    // Strategy. AO* is the only algorithm that can represent a contingent
    // plan, so every sensing task prefers it while the branching stays
    // tractable; the three thresholds below are where the suite stopped
    // paying for it.
    p.strategy_rules = {
        rule("sensing-small-designated", "aostar",
             {cond("sensing", Comparison::Eq, 1),
              cond("designated", Comparison::Le, 16)}),

        // Deep modal goals justify a wider designated set: the branching is in
        // the goal structure, not in the world count.
        rule("sensing-deep-goal", "aostar",
             {cond("sensing", Comparison::Eq, 1),
              cond("goal_modal_depth", Comparison::Ge, 2),
              cond("designated", Comparison::Le, 32)}),

        rule("sensing-few-actions", "aostar",
             {cond("sensing", Comparison::Eq, 1),
              cond("designated", Comparison::Le, 8),
              cond("actions", Comparison::Le, 32)}),

        rule("sensing-too-wide", "gbfs",
             {cond("sensing", Comparison::Eq, 1)}),

        // Private-announcement S5 domains (gossip, grapevine) grow in worlds
        // but keep a linear plan, so the world-count thresholds below would
        // misroute them.
        rule("s5-partial-obs", "gbfs",
             {cond("partial_obs", Comparison::Eq, 1),
              cond("kd45", Comparison::Eq, 0)}),

        // KD45 seriality repair leaves conditional structure even without
        // sensing events, which AO* handles and EHC does not.
        rule("kd45-small", "aostar",
             {cond("kd45", Comparison::Eq, 1),
              cond("worlds", Comparison::Le, 8),
              cond("designated", Comparison::Le, 4),
              cond("goal_modal_depth", Comparison::Ge, 1)}),

        // EHC's plateau escape is a BFS over full models: cheap only while the
        // model is small and the goal shallow.
        rule("s5-shallow-small", "ehc",
             {cond("kd45", Comparison::Eq, 0),
              cond("designated", Comparison::Le, 4),
              cond("worlds", Comparison::Le, 16),
              cond("actions", Comparison::Le, 12),
              cond("goal_modal_depth", Comparison::Le, 1)}),

        rule("wide-model", "gbfs", {cond("worlds", Comparison::Gt, 16)}),
        rule("many-actions", "gbfs", {cond("actions", Comparison::Gt, 12)}),
        rule("default", "ehc", {}),
    };

    // Heuristic. The first two rules share an outcome because the original
    // condition was a disjunction; first-match ordering makes that faithful.
    p.heuristic_rules = {
        // ks counts unresolved worlds per Kw conjunct — the right gradient
        // when every conjunct is Kw-shaped. ed would project through the
        // Belief operators Kw expands into and double-count.
        rule("kw-only-goal", "ks", {cond("goal_kw_only", Comparison::Eq, 1)}),

        rule("sensing", "ed", {cond("sensing", Comparison::Eq, 1)}),

        // A goal with no bare atom conjunct is purely epistemic; ug would see
        // only 0 or 1 per conjunct.
        rule("purely-epistemic-goal", "ed",
             {cond("goal_has_atom_conjunct", Comparison::Eq, 0)}),

        rule("default", "ug", {}),
    };

    validate(p.strategy_rules,  strategy_labels(),  "strategy");
    validate(p.heuristic_rules, heuristic_labels(), "heuristic");
    return p;
}

SelectionPolicy SelectionPolicy::load(const std::string& path) {
    std::ifstream in(path);
    if (!in.is_open())
        throw std::runtime_error("cannot open policy file: " + path);

    json j;
    try {
        in >> j;
    } catch (const json::exception& e) {
        throw std::runtime_error("policy file " + path + " is not valid JSON: " +
                                 e.what());
    }

    if (!j.is_object())
        throw std::runtime_error("policy file " + path +
                                 " must be a JSON object with \"strategy\" "
                                 "and/or \"heuristic\" keys");

    // A file may override one selector and inherit the other, which is the
    // common case when tuning: strategy thresholds move far more often than
    // the heuristic mapping.
    SelectionPolicy p = builtin();

    if (j.contains("strategy")) {
        p.strategy_rules = parse_rules(j["strategy"], "strategy");
        validate(p.strategy_rules, strategy_labels(), "strategy");
    }
    if (j.contains("heuristic")) {
        p.heuristic_rules = parse_rules(j["heuristic"], "heuristic");
        validate(p.heuristic_rules, heuristic_labels(), "heuristic");
    }
    if (!j.contains("strategy") && !j.contains("heuristic"))
        throw std::runtime_error("policy file " + path +
                                 " has neither a \"strategy\" nor a "
                                 "\"heuristic\" section");

    return p;
}

std::string SelectionPolicy::to_json() const {
    json j;
    j["features"] = json::array();
    for (auto& n : TaskFeatures::names())
        j["features"].push_back(std::string(n));
    j["strategy"]  = dump_rules(strategy_rules);
    j["heuristic"] = dump_rules(heuristic_rules);
    return j.dump(2);
}
