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

#include "plansys2_epistemic_planner/mapping_draft.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

namespace plansys2
{

namespace
{

/// The grounded name split at the schema that matches it.
///
/// Longest match wins. `pickup-A-hold_r2` is the case that makes it matter: a
/// split at the first underscore reads the whole hyphenated run as the action
/// name, and only the declared schemas say where the name stops and a bound
/// argument begins.
bool split_at_schema(
  const std::string & grounded, const std::vector<std::string> & schemas,
  std::string & schema, std::vector<std::string> & arguments)
{
  const std::string * best = nullptr;
  for (const auto & candidate : schemas) {
    if (grounded.rfind(candidate, 0) != 0) {
      continue;         // not a prefix
    }
    if (grounded.size() > candidate.size() &&
      grounded[candidate.size()] != '_' && grounded[candidate.size()] != '-')
    {
      continue;         // a prefix of a longer word, not of a bound name
    }
    if (!best || candidate.size() > best->size()) {
      best = &candidate;
    }
  }
  if (!best) {
    return false;
  }

  schema = *best;
  arguments.clear();

  std::string rest = grounded.substr(best->size());
  if (!rest.empty() && (rest.front() == '_' || rest.front() == '-')) {
    rest.erase(0, 1);
  }
  std::string current;
  for (const char c : rest) {
    if (c == '_') {
      if (!current.empty()) {
        arguments.push_back(current);
      }
      current.clear();
    } else {
      current += c;
    }
  }
  if (!current.empty()) {
    arguments.push_back(current);
  }
  return true;
}

/// The split the solver falls back to when no schema matches: first underscore
/// separates the name from the arguments.
void split_at_underscore(
  const std::string & grounded, std::string & schema,
  std::vector<std::string> & arguments)
{
  arguments.clear();
  const auto first = grounded.find('_');
  if (first == std::string::npos) {
    schema = grounded;
    return;
  }
  schema = grounded.substr(0, first);

  std::string current;
  for (std::size_t i = first + 1; i <= grounded.size(); ++i) {
    if (i == grounded.size() || grounded[i] == '_') {
      if (!current.empty()) {
        arguments.push_back(current);
      }
      current.clear();
    } else {
      current += grounded[i];
    }
  }
}

/// `goto-junction` and `goto_junction` are the same name written for two
/// parsers. Comparing them this way is what lets a schema match a PDDL action
/// whose only difference is the separator.
std::string normalise(const std::string & name)
{
  std::string out;
  out.reserve(name.size());
  for (const char c : name) {
    out += (c == '-') ? '_' : static_cast<char>(std::tolower(c));
  }
  return out;
}

std::string expression(const std::string & name, const std::vector<std::string> & arguments)
{
  std::string out = "(" + name;
  for (const auto & argument : arguments) {
    out += " " + argument;
  }
  return out + ")";
}

}  // namespace

bool MappingDraft::complete() const
{
  return std::all_of(
    entries.begin(), entries.end(),
    [](const DraftEntry & e) {return e.problem.empty();});
}

std::vector<std::string> MappingDraft::warnings() const
{
  std::vector<std::string> out;
  for (const auto & entry : entries) {
    if (!entry.problem.empty()) {
      out.push_back(entry.grounded + ": " + entry.problem);
    }
  }
  return out;
}

std::string MappingDraft::to_json(double default_duration) const
{
  nlohmann::ordered_json out;
  for (const auto & entry : entries) {
    nlohmann::ordered_json value;
    value["action"] = entry.proposal;
    value["duration"] = default_duration;
    if (!entry.problem.empty()) {
      // Carried in the file itself, since the file is what a person opens to
      // finish the job and the terminal output is gone by then.
      value["_check"] = entry.problem;
    }
    out[entry.grounded] = value;
  }
  return out.dump(2);
}

MappingDraft draft_mapping(
  const PlanningTask & task,
  const std::vector<std::string> & schemas,
  const std::vector<std::pair<std::string, std::size_t>> & pddl_actions)
{
  MappingDraft draft;

  for (const auto & action : task.actions) {
    DraftEntry entry;
    entry.grounded = action.name;

    if (!split_at_schema(entry.grounded, schemas, entry.schema, entry.arguments)) {
      split_at_underscore(entry.grounded, entry.schema, entry.arguments);
      if (!schemas.empty()) {
        entry.problem =
          "no declared EPDDL schema is a prefix of this name, so the split into "
          "name and arguments is a guess";
      }
    }

    // A PDDL action of the same name, allowing for the separator. Arity has to
    // agree too: a name that matches with the wrong number of parameters would
    // reach the executor and fail there, which is the failure this is here to
    // move forward in time.
    const auto wanted = normalise(entry.schema);
    const std::pair<std::string, std::size_t> * match = nullptr;
    const std::pair<std::string, std::size_t> * name_only = nullptr;
    for (const auto & candidate : pddl_actions) {
      if (normalise(candidate.first) != wanted) {
        continue;
      }
      name_only = &candidate;
      if (candidate.second == entry.arguments.size()) {
        match = &candidate;
        break;
      }
    }

    if (match) {
      entry.pddl_action = match->first;
      entry.proposal = expression(match->first, entry.arguments);
    } else if (name_only) {
      entry.pddl_action = name_only->first;
      entry.proposal = expression(name_only->first, entry.arguments);
      entry.problem =
        "the PDDL action '" + name_only->first + "' takes " +
        std::to_string(name_only->second) + " parameters and this grounding "
        "supplies " + std::to_string(entry.arguments.size()) +
        "; the order and the missing ones have to be written by hand";
    } else {
      entry.proposal = expression(entry.schema, entry.arguments);
      entry.problem =
        pddl_actions.empty() ?
        "no PDDL domain was given, so nothing checked that this action exists" :
        "the PDDL domain declares no action named '" + entry.schema +
        "'; the correspondence is a modelling decision and has to be written by hand";
    }

    draft.entries.push_back(std::move(entry));
  }

  return draft;
}

std::vector<std::string> read_epddl_schemas(const std::string & domain_path)
{
  std::vector<std::string> out;
  std::ifstream file(domain_path);
  if (!file) {
    return out;
  }

  std::string text((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

  const std::string marker = "(:action";
  std::size_t at = 0;
  while ((at = text.find(marker, at)) != std::string::npos) {
    std::size_t i = at + marker.size();
    // `(:action-type-libraries` is not an action declaration; the character
    // after the keyword is what tells them apart.
    if (i < text.size() && (text[i] == '-' || text[i] == 't')) {
      at = i;
      continue;
    }
    while (i < text.size() && std::isspace(static_cast<unsigned char>(text[i]))) {
      ++i;
    }
    std::string name;
    while (i < text.size() && !std::isspace(static_cast<unsigned char>(text[i])) &&
      text[i] != '(' && text[i] != ')')
    {
      name += text[i++];
    }
    if (!name.empty()) {
      out.push_back(name);
    }
    at = i;
  }
  return out;
}

std::vector<std::pair<std::string, std::size_t>> read_epddl_schema_arities(
  const std::string & domain_path)
{
  std::vector<std::pair<std::string, std::size_t>> out;
  std::ifstream file(domain_path);
  if (!file) {
    return out;
  }

  std::string text((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

  const std::string marker = "(:action";
  std::size_t at = 0;
  while ((at = text.find(marker, at)) != std::string::npos) {
    std::size_t i = at + marker.size();
    if (i < text.size() && (text[i] == '-' || text[i] == 't')) {
      at = i;           // (:action-type-libraries, not a schema
      continue;
    }
    while (i < text.size() && std::isspace(static_cast<unsigned char>(text[i]))) {
      ++i;
    }
    std::string name;
    while (i < text.size() && !std::isspace(static_cast<unsigned char>(text[i])) &&
      text[i] != '(' && text[i] != ')')
    {
      name += text[i++];
    }
    if (name.empty()) {
      at = i;
      continue;
    }

    std::size_t params = 0;
    const auto plist = text.find(":parameters", i);
    if (plist != std::string::npos) {
      const auto open = text.find('(', plist);
      const auto close = text.find(')', open == std::string::npos ? plist : open);
      if (open != std::string::npos && close != std::string::npos) {
        for (std::size_t k = open; k < close; ++k) {
          if (text[k] == '?') {
            ++params;
          }
        }
      }
    }
    out.emplace_back(name, params);
    at = i;
  }
  return out;
}

std::vector<std::pair<std::string, std::size_t>> read_pddl_actions(
  const std::string & domain_path)
{
  std::vector<std::pair<std::string, std::size_t>> out;
  std::ifstream file(domain_path);
  if (!file) {
    return out;
  }

  std::string text((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

  for (const std::string marker : {":action", ":durative-action"}) {
    std::size_t at = 0;
    while ((at = text.find(marker, at)) != std::string::npos) {
      std::size_t i = at + marker.size();
      if (marker == ":action" && i < text.size() && text[i] == '-') {
        at = i;         // ":action-type", not an action
        continue;
      }
      while (i < text.size() && std::isspace(static_cast<unsigned char>(text[i]))) {
        ++i;
      }
      std::string name;
      while (i < text.size() && !std::isspace(static_cast<unsigned char>(text[i])) &&
        text[i] != '(' && text[i] != ')')
      {
        name += text[i++];
      }
      if (name.empty()) {
        at = i;
        continue;
      }

      // Count the parameters by counting the variables in the :parameters
      // list, which is the next one after the name.
      std::size_t params = 0;
      const auto plist = text.find(":parameters", i);
      if (plist != std::string::npos) {
        const auto open = text.find('(', plist);
        const auto close = text.find(')', open == std::string::npos ? plist : open);
        if (open != std::string::npos && close != std::string::npos) {
          for (std::size_t k = open; k < close; ++k) {
            if (text[k] == '?') {
              ++params;
            }
          }
        }
      }
      out.emplace_back(name, params);
      at = i;
    }
  }
  return out;
}

}  // namespace plansys2
