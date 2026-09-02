// Copyright 2026 Intelligent Robotics Lab
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

#include "plansys2_epistemic_planner/state_json.hpp"

#include <string>
#include <unordered_map>
#include <vector>

#include <nlohmann/json.hpp>

namespace plansys2
{

namespace
{

/// The name a world index is written under. The parser assigns indices in the
/// order the `worlds` array lists them, so writing them in index order is what
/// makes the round trip preserve indices.
std::string world_name(std::uint32_t w)
{
  return "w" + std::to_string(w);
}

}  // namespace

std::string state_to_json(const PlanningTask & task, const EpistemicState & state)
{
  nlohmann::json out;

  auto & worlds = out["worlds"] = nlohmann::json::array();
  for (std::uint32_t w = 0; w < state.num_worlds; ++w) {
    worlds.push_back(world_name(w));
  }

  // Written for every world, including the ones with no atoms true. An absent
  // entry and an empty one mean the same thing to the parser, but a reader
  // comparing two models by eye should not have to work out which worlds were
  // omitted.
  auto & labels = out["labels"] = nlohmann::json::object();
  for (std::uint32_t w = 0; w < state.num_worlds; ++w) {
    auto & atoms = labels[world_name(w)] = nlohmann::json::array();
    for (std::uint32_t a = 0; a < task.num_atoms(); ++a) {
      if (state.has_atom(w, static_cast<AtomIdx>(a))) {
        atoms.push_back(task.atom_names[a]);
      }
    }
  }

  auto & designated = out["designated"] = nlohmann::json::array();
  for (std::uint32_t w = 0; w < state.num_worlds; ++w) {
    if (state.is_designated(static_cast<WorldIdx>(w))) {
      designated.push_back(world_name(w));
    }
  }

  auto & relations = out["relations"] = nlohmann::json::object();
  for (std::uint32_t ag = 0; ag < task.num_agents(); ++ag) {
    auto & rows = relations[task.agent_names[ag]] = nlohmann::json::object();
    for (std::uint32_t w = 0; w < state.num_worlds; ++w) {
      auto & targets = rows[world_name(w)] = nlohmann::json::array();
      const auto row = state.succ(static_cast<AgentIdx>(ag), static_cast<WorldIdx>(w));
      bits::for_each(
        row, [&](std::size_t to) {
          targets.push_back(world_name(static_cast<std::uint32_t>(to)));
        });
    }
  }

  return out.dump();
}

bool state_from_json(
  const PlanningTask & task, const std::string & json,
  EpistemicState & out, std::string & error)
{
  nlohmann::json j;
  try {
    j = nlohmann::json::parse(json);
  } catch (const std::exception & e) {
    error = std::string("the model is not JSON: ") + e.what();
    return false;
  }

  for (const auto * key : {"worlds", "labels", "designated", "relations"}) {
    if (!j.contains(key)) {
      error = std::string("the model has no \"") + key + "\" member";
      return false;
    }
  }

  std::unordered_map<std::string, WorldIdx> world_idx;
  try {
    WorldIdx idx = 0;
    for (const auto & w : j.at("worlds")) {
      world_idx[w.get<std::string>()] = idx++;
    }
  } catch (const std::exception & e) {
    error = std::string("the world list is malformed: ") + e.what();
    return false;
  }

  if (world_idx.empty()) {
    // A model with no worlds satisfies nothing and refutes nothing. Reading it
    // would replace a usable state with one that cannot answer a question.
    error = "the model has no worlds";
    return false;
  }

  EpistemicState state;
  state.allocate(
    static_cast<std::uint32_t>(world_idx.size()),
    static_cast<std::uint32_t>(task.num_atoms()),
    static_cast<std::uint32_t>(task.num_agents()));

  try {
    for (const auto & [wname, atoms] : j.at("labels").items()) {
      const auto wit = world_idx.find(wname);
      if (wit == world_idx.end()) {
        error = "the labels name world '" + wname + "', which the model does not list";
        return false;
      }
      for (const auto & a : atoms) {
        const auto name = a.get<std::string>();
        const auto ait = task.atom_index.find(name);
        if (ait == task.atom_index.end()) {
          error = "the model names atom '" + name + "', which the task does not have";
          return false;
        }
        state.set_atom(wit->second, ait->second);
      }
    }

    for (const auto & d : j.at("designated")) {
      const auto name = d.get<std::string>();
      const auto wit = world_idx.find(name);
      if (wit == world_idx.end()) {
        error = "world '" + name + "' is designated but not listed";
        return false;
      }
      state.set_designated(wit->second);
    }

    for (const auto & [agent_name, rows] : j.at("relations").items()) {
      const auto agit = task.agent_index.find(agent_name);
      if (agit == task.agent_index.end()) {
        error = "the model names agent '" + agent_name + "', which the task does not have";
        return false;
      }
      for (const auto & [src, targets] : rows.items()) {
        const auto sit = world_idx.find(src);
        if (sit == world_idx.end()) {
          error = "the relation of '" + agent_name + "' leaves unlisted world '" + src + "'";
          return false;
        }
        for (const auto & t : targets) {
          const auto tname = t.get<std::string>();
          const auto tit = world_idx.find(tname);
          if (tit == world_idx.end()) {
            error = "the relation of '" + agent_name + "' reaches unlisted world '" + tname + "'";
            return false;
          }
          state.add_edge(agit->second, sit->second, tit->second);
        }
      }
    }
  } catch (const std::exception & e) {
    error = std::string("the model is malformed: ") + e.what();
    return false;
  }

  if (state.num_designated() == 0) {
    // Nothing is held possible, so every formula holds vacuously and the goal
    // check would report success from a model that says nothing at all.
    error = "the model designates no world";
    return false;
  }

  out = std::move(state);
  return true;
}

EpistemicState agent_perspective(const EpistemicState & state, AgentIdx agent)
{
  if (agent >= state.num_agents) {
    return state;
  }

  EpistemicState out = state;

  // Everything the agent holds possible from a world that is actually the
  // case. Accumulated before it is written back, since the designated set is
  // what is being read while it is being replaced.
  std::vector<bits::Word> reachable(state.rel_words, 0);
  for (std::uint32_t w = 0; w < state.num_worlds; ++w) {
    if (!state.is_designated(static_cast<WorldIdx>(w))) {
      continue;
    }
    const auto row = state.succ(agent, static_cast<WorldIdx>(w));
    for (std::uint32_t i = 0; i < state.rel_words; ++i) {
      reachable[i] |= row[i];
    }
  }

  bool any = false;
  for (const auto word : reachable) {
    if (word != 0) {
      any = true;
      break;
    }
  }
  if (!any) {
    // An agent with no accessible world at all. Every formula would hold in
    // the result, so the model as it stands is the more honest answer.
    return state;
  }

  out.designated.assign(reachable.begin(), reachable.end());
  out.invalidate();
  return out;
}

}  // namespace plansys2
