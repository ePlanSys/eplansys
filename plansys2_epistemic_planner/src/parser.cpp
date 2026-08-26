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

#include "plansys2_epistemic_planner/parser.hpp"
#include "plansys2_epistemic_planner/formula.hpp"
#include "plansys2_epistemic_planner/heuristic.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <stdexcept>
#include <iostream>

using json = nlohmann::json;

/**
 * @brief Parse a logical formula from its JSON representation.
 *
 * Supports atomic propositions, boolean constants, logical connectives
 * (negation, conjunction, disjunction), and modal operators (box, diamond).
 *
 * @param j JSON object encoding the formula.
 * @param atom_idx Mapping from atom names to indices.
 * @param agent_idx Mapping from agent names to indices.
 * @return Parsed formula as a FormulaPtr.
 *
 * @throws std::runtime_error if the formula is malformed or references unknown symbols.
 */
static FormulaPtr parse_formula(
    const json& j,
    const std::unordered_map<std::string,AtomIdx>&  atom_idx,
    const std::unordered_map<std::string,AgentIdx>& agent_idx)
{
    if (j.is_string()) {
        std::string s = j.get<std::string>();
        if (s == "true")  return Formula::make_top();
        if (s == "false") return Formula::make_bot();

        auto it = atom_idx.find(s);
        if (it == atom_idx.end())
            throw std::runtime_error("Unknown atom: " + s);

        return Formula::make_atom(it->second);
    }

    if (!j.is_object())
        throw std::runtime_error("Expected formula object or string");

    if (j.contains("connective")) {
        std::string conn = j.at("connective").get<std::string>();

        if (conn == "not") {
            return Formula::make_not(
                parse_formula(j.at("formula"), atom_idx, agent_idx));
        }

        if (conn == "and") {
            std::vector<FormulaPtr> children;
            for (auto& c : j.at("formulas"))
                children.push_back(parse_formula(c, atom_idx, agent_idx));
            return Formula::make_and(std::move(children));
        }

        if (conn == "or") {
            std::vector<FormulaPtr> children;
            for (auto& c : j.at("formulas"))
                children.push_back(parse_formula(c, atom_idx, agent_idx));
            return Formula::make_or(std::move(children));
        }

        // imply: p -> q  ≡  ¬p ∨ q
        if (conn == "imply") {
            auto lhs = parse_formula(j.at("formulas")[0], atom_idx, agent_idx);
            auto rhs = parse_formula(j.at("formulas")[1], atom_idx, agent_idx);
            return Formula::make_or({Formula::make_not(lhs), rhs});
        }

        // forall over agents: universally quantified conjunction
        // plank expands these at ground time so this is a fallback for
        // any residual quantified formulas in the JSON
        if (conn == "forall") {
            if (j.contains("formulas")) {
                std::vector<FormulaPtr> children;
                for (auto& c : j.at("formulas"))
                    children.push_back(parse_formula(c, atom_idx, agent_idx));
                return Formula::make_and(std::move(children));
            }
            return parse_formula(j.at("formula"), atom_idx, agent_idx);
        }

        throw std::runtime_error("Unknown connective: " + conn);
    }

    if (j.contains("modality-name")) {
        std::string mname = j.at("modality-name").get<std::string>();
        auto& midx = j.at("modality-index");

        FormulaPtr child =
            parse_formula(j.at("formula"), atom_idx, agent_idx);

        if (mname == "box") {
            if (midx.size() == 1) {
                std::string aname = midx[0].get<std::string>();
                auto it = agent_idx.find(aname);
                if (it == agent_idx.end())
                    throw std::runtime_error("Unknown agent: " + aname);

                return Formula::make_belief(it->second, child);
            }

            std::vector<AgentIdx> grp;
            for (auto& a : midx) {
                std::string aname = a.get<std::string>();
                auto it = agent_idx.find(aname);
                if (it == agent_idx.end())
                    throw std::runtime_error("Unknown agent in group: " + aname);
                grp.push_back(it->second);
            }

            return Formula::make_common(std::move(grp), child);
        }

        if (mname == "diamond") {
            if (midx.size() == 1) {
                std::string aname = midx[0].get<std::string>();
                auto it = agent_idx.find(aname);
                if (it == agent_idx.end())
                    throw std::runtime_error("Unknown agent: " + aname);

                return Formula::make_not(
                    Formula::make_belief(it->second,
                        Formula::make_not(child)));
            }
        }

        // [Kw.i]φ  ≡  [i]φ ∨ [i]¬φ  (knowing-whether)
        // [Kw.G]φ  ≡  ∧_{i∈G} ([i]φ ∨ [i]¬φ)  (group knowing-whether)
        if (mname == "Kw.box") {
            if (midx.size() == 1) {
                std::string aname = midx[0].get<std::string>();
                auto it = agent_idx.find(aname);
                if (it == agent_idx.end())
                    throw std::runtime_error("Unknown agent: " + aname);
                return Formula::make_kw(it->second, child);
            }
            if (midx.size() > 1) {
                std::vector<FormulaPtr> conjuncts;
                for (auto& a : midx) {
                    std::string aname = a.get<std::string>();
                    auto it = agent_idx.find(aname);
                    if (it == agent_idx.end())
                        throw std::runtime_error("Unknown agent in Kw.box group: " + aname);
                    conjuncts.push_back(Formula::make_kw(it->second, child));
                }
                return Formula::make_and(std::move(conjuncts));
            }
        }

        // <Kw.i>φ  ≡  ¬([i]φ ∨ [i]¬φ)  (not knowing-whether)
        // <Kw.G>φ  ≡  ∨_{i∈G} ¬([i]φ ∨ [i]¬φ)
        if (mname == "Kw.diamond") {
            if (midx.size() == 1) {
                std::string aname = midx[0].get<std::string>();
                auto it = agent_idx.find(aname);
                if (it == agent_idx.end())
                    throw std::runtime_error("Unknown agent: " + aname);
                return Formula::make_not(Formula::make_kw(it->second, child));
            }
            if (midx.size() > 1) {
                std::vector<FormulaPtr> disjuncts;
                for (auto& a : midx) {
                    std::string aname = a.get<std::string>();
                    auto it = agent_idx.find(aname);
                    if (it == agent_idx.end())
                        throw std::runtime_error("Unknown agent in Kw.diamond group: " + aname);
                    disjuncts.push_back(Formula::make_not(Formula::make_kw(it->second, child)));
                }
                return Formula::make_or(std::move(disjuncts));
            }
        }

        // C.box — common knowledge/belief over a group
        // plank emits "C.box" with modality-index listing all group agents
        if (mname == "C.box") {
            std::vector<AgentIdx> grp;
            for (auto& a : midx) {
                std::string aname = a.get<std::string>();
                auto it = agent_idx.find(aname);
                if (it == agent_idx.end())
                    throw std::runtime_error("Unknown agent in C.box group: " + aname);
                grp.push_back(it->second);
            }
            return Formula::make_common(std::move(grp), child);
        }

        // C.diamond — dual: ¬C.box¬φ
        if (mname == "C.diamond") {
            std::vector<AgentIdx> grp;
            for (auto& a : midx) {
                std::string aname = a.get<std::string>();
                auto it = agent_idx.find(aname);
                if (it == agent_idx.end())
                    throw std::runtime_error("Unknown agent in C.diamond group: " + aname);
                grp.push_back(it->second);
            }
            return Formula::make_not(
                Formula::make_common(std::move(grp), Formula::make_not(child)));
        }

        throw std::runtime_error("Unknown modality: " + mname);
    }

    throw std::runtime_error("Unrecognised formula shape");
}

/**
 * @brief Extract and parse a formula, unwrapping a "formula" field if present.
 *
 * Some JSON structures wrap formulas inside a "formula" key. This function
 * transparently handles both wrapped and direct representations.
 *
 * @param j JSON object containing the formula or wrapper.
 * @param atom_idx Mapping from atom names to indices.
 * @param agent_idx Mapping from agent names to indices.
 * @return Parsed formula as a FormulaPtr.
 */
static FormulaPtr unwrap_formula(
    const json& j,
    const std::unordered_map<std::string,AtomIdx>&  atom_idx,
    const std::unordered_map<std::string,AgentIdx>& agent_idx)
{
    if (j.contains("formula"))
        return parse_formula(j.at("formula"), atom_idx, agent_idx);

    return parse_formula(j, atom_idx, agent_idx);
}

/**
 * @brief Load a planning task from a JSON file.
 *
 * The input JSON is expected to follow a structured format including:
 * - Language definition (atoms and agents)
 * - Requirements (used to detect KD45 vs S5 frame constraints)
 * - Initial epistemic state (worlds, valuations, accessibility relations)
 * - Actions with events, preconditions, effects, and observability conditions
 * - Goal formula
 *
 * The kd45 flag is set when the requirements contain ":kd45" or ":belief",
 * indicating that all accessibility relations must be serial (KD45n frame).
 * Product update will enforce seriality on the resulting state when this flag
 * is true, preventing vacuous-truth errors from worlds with empty R_i rows.
 *
 * @param json_path Path to the JSON file.
 * @return Fully constructed PlanningTask instance.
 *
 * @throws std::runtime_error if the file cannot be read or parsing fails.
 */
PlanningTask load_task(const std::string& json_path) {
    std::ifstream f(json_path);
    if (!f.is_open())
        throw std::runtime_error("Cannot open JSON file: " + json_path);

    json j;
    f >> j;

    PlanningTask task;

    // Read requirements to detect KD45 (doxastic/belief) vs S5 (knowledge) frame.
    // plank exports requirements as a list of strings under "requirements".
    // KD45 is indicated by ":kd45" or ":belief"; S5 by ":s5" or ":knowledge".
    // If requirements are absent or neither flag is found, default to S5 (conservative).
    task.kd45 = false;
    if (j.contains("planning-task-info")) {
        auto& pti = j.at("planning-task-info");
        if (pti.contains("requirements")) {
            for (auto& req : pti.at("requirements")) {
                std::string r = req.get<std::string>();
                if (r == ":kd45" || r == ":KD45-frames" || r == ":belief" || r == ":doxastic") {
                    task.kd45 = true;
                    break;
                }
            }
        }
    }


    if (!task.kd45 && j.contains("requirements")) {
        for (auto& req : j.at("requirements")) {
            std::string r = req.get<std::string>();
            if (r == ":kd45" || r == ":KD45-frames" || r == ":belief" || r == ":doxastic") {
                task.kd45 = true;
                break;
            }
        }
    }

    std::cerr << "[parser] Frame: " << (task.kd45 ? "KD45 (belief)" : "S5 (knowledge)") << "\n";

    for (auto& a : j.at("language").at("atoms")) {
        std::string name = a.get<std::string>();
        task.atom_index[name] = static_cast<AtomIdx>(task.atom_names.size());
        task.atom_names.push_back(name);
    }

    for (auto& a : j.at("language").at("agents")) {
        std::string name = a.get<std::string>();
        task.agent_index[name] = static_cast<AgentIdx>(task.agent_names.size());
        task.agent_names.push_back(name);
    }

    size_t na = task.num_agents();

    auto& is = j.at("initial-state");

    std::unordered_map<std::string, WorldIdx> world_idx;

    {
        WorldIdx idx = 0;
        for (auto& w : is.at("worlds"))
            world_idx[w.get<std::string>()] = idx++;
    }

    size_t nw = world_idx.size();

    // The model is allocated up front as three flat bit arrays; every field
    // below writes bits into them rather than growing per-world containers.
    task.init.allocate(static_cast<std::uint32_t>(nw),
                       static_cast<std::uint32_t>(task.num_atoms()),
                       static_cast<std::uint32_t>(na));

    for (auto& [wname, atoms] : is.at("labels").items()) {
        auto it = world_idx.find(wname);
        if (it == world_idx.end()) continue;

        for (auto& a : atoms) {
            auto ait = task.atom_index.find(a.get<std::string>());
            if (ait != task.atom_index.end())
                task.init.set_atom(it->second, ait->second);
        }
    }

    for (auto& d : is.at("designated")) {
        auto it = world_idx.find(d.get<std::string>());
        if (it != world_idx.end())
            task.init.set_designated(it->second);
    }

    for (auto& [agent_name, rows] : is.at("relations").items()) {
        auto ait = task.agent_index.find(agent_name);
        if (ait == task.agent_index.end()) continue;

        AgentIdx ag = ait->second;

        for (auto& [src_wname, targets] : rows.items()) {
            auto sit = world_idx.find(src_wname);
            if (sit == world_idx.end()) continue;

            for (auto& t : targets) {
                auto tit = world_idx.find(t.get<std::string>());
                if (tit != world_idx.end())
                    task.init.add_edge(ag, sit->second, tit->second);
            }
        }
    }

    for (auto& [action_name, a_j] : j.at("actions").items()) {
        Action act;
        act.name       = action_name;
        act.num_agents = na;

        std::unordered_map<std::string, EventIdx> event_idx;

        for (auto& e : a_j.at("events")) {
            std::string ename = e.get<std::string>();
            EventIdx eid = static_cast<EventIdx>(act.events.size());

            event_idx[ename] = eid;

            Event ev;
            ev.id   = eid;
            ev.name = ename;
            ev.is_nil = (ename == "nil");
            ev.precondition = Formula::make_top();

            act.events.push_back(std::move(ev));
        }

        size_t ne = act.events.size();

        if (a_j.contains("preconditions")) {
            for (auto& [ename, pre_j] : a_j.at("preconditions").items()) {
                auto it = event_idx.find(ename);
                if (it == event_idx.end()) continue;

                act.events[it->second].precondition =
                    unwrap_formula(pre_j, task.atom_index, task.agent_index);
            }
        }

        if (a_j.contains("effects")) {
            for (auto& [ename, eff_j] : a_j.at("effects").items()) {
                auto it = event_idx.find(ename);
                if (it == event_idx.end() || eff_j.is_null()) continue;

                Event& ev = act.events[it->second];

                for (auto& [atom_name, val_j] : eff_j.items()) {
                    auto ait = task.atom_index.find(atom_name);
                    if (ait == task.atom_index.end()) continue;

                    AtomIdx atom = ait->second;

                    FormulaPtr cond =
                        unwrap_formula(val_j, task.atom_index, task.agent_index);

                    if (cond->kind == FormulaKind::Top) {
                        ev.post_true[atom] = Formula::make_top();
                    } else if (cond->kind == FormulaKind::Bot) {
                        ev.post_false[atom] = Formula::make_top();
                    } else {
                        ev.post_true[atom]  = cond;
                        ev.post_false[atom] = Formula::make_not(cond);
                    }
                }
            }
        }

        for (auto& d : a_j.at("designated")) {
            std::string ename = d.get<std::string>();
            auto it = event_idx.find(ename);
            if (it != event_idx.end())
                act.designated_events.insert(it->second);
        }

        // Build obs_type_rel: obs_type_name -> event relation
        std::unordered_map<std::string,
            std::vector<std::unordered_set<EventIdx>>> obs_type_rel;

        if (a_j.contains("relations")) {
            for (auto& [obs_type, rel_j] : a_j.at("relations").items()) {
                std::vector<std::unordered_set<EventIdx>> rel(ne);

                for (auto& [src_ename, targets] : rel_j.items()) {
                    auto sit = event_idx.find(src_ename);
                    if (sit == event_idx.end()) continue;

                    for (auto& t : targets) {
                        std::string tname = t.get<std::string>();
                        auto tit = event_idx.find(tname);
                        if (tit != event_idx.end())
                            rel[sit->second].insert(tit->second);
                    }
                }

                obs_type_rel[obs_type] = std::move(rel);
            }
        }

        // Collect all observability cases per agent
        act.obs_cases.resize(na);

        if (a_j.contains("observability-conditions")) {
            for (auto& [agent_name, obs_j] : a_j.at("observability-conditions").items()) {
                auto ait = task.agent_index.find(agent_name);
                if (ait == task.agent_index.end()) continue;

                AgentIdx ag = ait->second;

                for (auto& [obs_type, cond_j] : obs_j.items()) {
                    auto rit = obs_type_rel.find(obs_type);
                    if (rit == obs_type_rel.end()) continue;

                    ObsCase oc;
                    oc.condition = unwrap_formula(cond_j, task.atom_index, task.agent_index);
                    oc.relation  = rit->second;
                    act.obs_cases[ag].push_back(std::move(oc));
                }
            }
        }

        task.action_index[act.name] =
            static_cast<ActionIdx>(task.actions.size());

        task.actions.push_back(std::move(act));
    }

    task.goal =
        unwrap_formula(j.at("goal"), task.atom_index, task.agent_index);

    // Detect partial observability.
    //
    // A domain has partial observability iff at least one action has agents
    // with heterogeneous observability — some Fully observable, others
    // Oblivious or conditional. In the parsed representation this shows up as
    // obs_cases[ag][0].relation differing between agents: Fully has the
    // identity relation (e -> {e} for all events), Oblivious maps every event
    // to {nil} (the single non-designated event), and conditional cases have
    // world-dependent rows.
    //
    // Comparing obs_cases sizes across agents is insufficient — Gossip assigns
    // exactly one ObsCase per agent (all have size 1) but with structurally
    // different relations. We instead compare the actual relation vectors of
    // the first ObsCase across agents: if any two agents disagree the domain
    // has partial observability.
    task.partial_obs = false;
    for (auto& act : task.actions) {
        if (act.obs_cases.size() < 2) continue;

        size_t ref_ag = act.obs_cases.size();
        for (size_t ag = 0; ag < act.obs_cases.size(); ag++) {
            if (!act.obs_cases[ag].empty()) { ref_ag = ag; break; }
        }
        if (ref_ag == act.obs_cases.size()) continue;

        const auto& ref_rel = act.obs_cases[ref_ag][0].relation;
        for (size_t ag = ref_ag + 1; ag < act.obs_cases.size(); ag++) {
            if (act.obs_cases[ag].empty()) continue;
            const auto& ag_rel = act.obs_cases[ag][0].relation;
            if (ag_rel.size() != ref_rel.size()) {
                task.partial_obs = true;
                break;
            }
            bool differs = false;
            for (size_t ei = 0; ei < ref_rel.size() && !differs; ei++)
                if (ag_rel[ei] != ref_rel[ei])
                    differs = true;
            if (differs) {
                task.partial_obs = true;
                break;
            }
        }
        if (task.partial_obs) break;
    }

    // Detect Kw-only goal.
    //
    // A goal is Kw-only if it is a single Kw formula or a conjunction where
    // every top-level conjunct is a Kw formula (FormulaKind::Kw, or an Or of
    // two Belief formulas that the parser expands Kw into). We check the
    // top-level structure only — deeper nesting is handled by the heuristic.
    // has_atom_conjunct (defined in heuristic.hpp) returns true iff the formula
    // has any bare atom at the top level, so goal_kw_only = !has_atom_conjunct.
    task.goal_kw_only = task.goal && !has_atom_conjunct(*task.goal);

    std::cerr << "[parser] Loaded: "
              << task.num_atoms()   << " atoms, "
              << task.num_agents()  << " agents, "
              << task.init.num_worlds << " worlds ("
              << task.init.num_designated() << " designated), "
              << task.num_actions() << " actions"
              << "  partial_obs=" << task.partial_obs
              << "  goal_kw_only=" << task.goal_kw_only
              << "\n";

    return task;
}