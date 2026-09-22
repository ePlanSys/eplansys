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

#include "plansys2_epistemic_executor/policy_parallel.hpp"

#include <algorithm>
#include <cctype>
#include <set>
#include <string>
#include <vector>

namespace plansys2
{

namespace
{

/// The parts of a grounded epistemic name after the schema, which are its
/// arguments and, in EPDDL, are agents: `relay-north-dirty_north_relay` names
/// `north` and `relay`.
void grounded_arguments(const std::string & name, std::vector<std::string> & out)
{
  std::size_t at = name.find('_');
  while (at != std::string::npos) {
    const auto start = at + 1;
    at = name.find('_', start);
    const auto part = name.substr(start, at == std::string::npos ? at : at - start);
    if (!part.empty()) {
      out.push_back(part);
    }
  }
}

/// The objects of a PDDL action expression: `(relay_north north relay)`.
void expression_arguments(const std::string & expression, std::vector<std::string> & out)
{
  std::string token;
  bool first = true;
  for (const char c : expression) {
    if (c == '(' || c == ')' || std::isspace(static_cast<unsigned char>(c))) {
      if (!token.empty()) {
        if (!first) {            // the first token is the action's name
          out.push_back(token);
        }
        first = false;
        token.clear();
      }
      continue;
    }
    token += c;
  }
  if (!token.empty() && !first) {
    out.push_back(token);
  }
}

/// Whether `text` contains `name` as a word rather than inside a longer one,
/// so that an agent called `north` is not found in `north-west`.
bool mentions(const std::string & text, const std::string & name)
{
  const auto word = [](char c) {
      return std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-';
    };

  std::size_t at = text.find(name);
  while (at != std::string::npos) {
    const bool before = at == 0 || !word(text[at - 1]);
    const auto end = at + name.size();
    const bool after = end >= text.size() || !word(text[end]);
    if (before && after) {
      return true;
    }
    at = text.find(name, at + 1);
  }
  return false;
}

/// The epistemic half of the test. See the header for what it does and does
/// not establish.
bool epistemically_independent(
  const plansys2_msgs::msg::PlanItem & a, const plansys2_msgs::msg::PlanItem & b)
{
  const auto agents_a = item_agents(a);
  const auto agents_b = item_agents(b);

  for (const auto & one : agents_a) {
    if (std::find(agents_b.begin(), agents_b.end(), one) != agents_b.end()) {
      return false;
    }
    for (const auto & requirement : b.knowledge_requirements) {
      if (mentions(requirement, one)) {
        return false;
      }
    }
  }

  for (const auto & other : agents_b) {
    for (const auto & requirement : a.knowledge_requirements) {
      if (mentions(requirement, other)) {
        return false;
      }
    }
  }

  return true;
}

}  // namespace

std::vector<std::string> item_agents(const plansys2_msgs::msg::PlanItem & item)
{
  std::vector<std::string> names;
  grounded_arguments(item.epistemic_action, names);
  expression_arguments(item.action, names);

  std::sort(names.begin(), names.end());
  names.erase(std::unique(names.begin(), names.end()), names.end());
  return names;
}

ParallelGroups parallel_groups(const Policy & policy, const Independence & independent)
{
  ParallelGroups groups;
  if (policy.empty() || policy.sequential()) {
    // A plan that names no continuations is PlanSys2's own sequence, and
    // PlanSys2 has a builder that parallelises those already.
    return groups;
  }

  std::set<std::uint32_t> taken;

  for (const auto index : policy.preorder()) {
    if (taken.count(index)) {
      continue;
    }

    ParallelGroup group{index};
    auto current = index;

    while (const auto next = policy.only_successor(current)) {
      if (*next == plansys2_msgs::msg::PlanItem::POLICY_DONE || taken.count(*next)) {
        break;
      }

      const auto & candidate = policy.item(*next);
      const bool joins = std::all_of(
        group.begin(), group.end(),
        [&](std::uint32_t member) {
          const auto & held = policy.item(member);
          if (!epistemically_independent(held, candidate)) {
            return false;
          }
          return !independent || independent(held, candidate);
        });

      if (!joins) {
        break;
      }

      group.push_back(*next);
      current = *next;
    }

    if (group.size() > 1) {
      for (const auto member : group) {
        taken.insert(member);
      }
      groups.push_back(group);
    }
  }

  return groups;
}

}  // namespace plansys2
