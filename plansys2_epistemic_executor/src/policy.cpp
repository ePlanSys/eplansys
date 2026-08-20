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

#include "plansys2_epistemic_executor/policy.hpp"

#include <algorithm>
#include <set>
#include <string>
#include <vector>

namespace plansys2
{

namespace
{
constexpr std::uint32_t kDone = plansys2_msgs::msg::PlanItem::POLICY_DONE;
}  // namespace

std::string Policy::validate(const plansys2_msgs::msg::Plan & plan)
{
  if (plan.items.empty()) {
    return "";   // an empty policy is the answer when the goal already holds
  }

  const auto size = static_cast<std::uint32_t>(plan.items.size());

  for (std::uint32_t i = 0; i < size; ++i) {
    const auto & item = plan.items[i];

    if (item.children.size() != item.outcomes.size()) {
      return "item " + std::to_string(i) + " has " +
             std::to_string(item.children.size()) + " continuations but " +
             std::to_string(item.outcomes.size()) +
             " outcomes; each continuation must say which observation selects it";
    }

    // Duplicated outcomes would make the choice depend on which entry is
    // looked at first, which is not a choice at all.
    const std::set<std::string> distinct(item.outcomes.begin(), item.outcomes.end());
    if (distinct.size() != item.outcomes.size()) {
      return "item " + std::to_string(i) + " names one outcome twice";
    }

    if (item.children.size() > 1 && !item.sensing) {
      return "item " + std::to_string(i) +
             " branches but is not a sensing action; nothing would ever observe "
             "which continuation applies";
    }

    for (const auto child : item.children) {
      if (child == kDone) {
        continue;
      }
      if (child >= size) {
        return "item " + std::to_string(i) + " continues to " +
               std::to_string(child) + ", which is past the end of the plan";
      }
      if (child <= i) {
        return "item " + std::to_string(i) + " continues to " +
               std::to_string(child) +
               "; a policy is a tree, so a continuation cannot point backwards";
      }
    }
  }

  // A plan that names no continuations is a PlanSys2 sequence, and every item
  // is reachable by construction. Checking reachability against links that
  // were never meant to be there would reject every classical plan.
  const bool linked = std::any_of(
    plan.items.begin(), plan.items.end(),
    [](const plansys2_msgs::msg::PlanItem & item) {return !item.children.empty();});
  if (!linked) {
    return "";
  }

  // Every node must be reachable, and reachable once: a node reached twice is
  // a shared subtree, which the executor would run as one action from two
  // branches.
  std::vector<int> arrivals(size, 0);
  for (const auto & item : plan.items) {
    for (const auto child : item.children) {
      if (child != kDone) {
        ++arrivals[child];
      }
    }
  }
  for (std::uint32_t i = 1; i < size; ++i) {
    if (arrivals[i] == 0) {
      return "item " + std::to_string(i) +
             " is unreachable from the root, so the executor could never run it";
    }
    if (arrivals[i] > 1) {
      return "item " + std::to_string(i) + " is reached from " +
             std::to_string(arrivals[i]) + " places; a policy is a tree";
    }
  }
  if (arrivals[0] != 0) {
    return "item 0 is the root and cannot be a continuation of another item";
  }

  return "";
}

bool Policy::branches() const
{
  return std::any_of(
    plan_.items.begin(), plan_.items.end(),
    [](const plansys2_msgs::msg::PlanItem & item) {return item.children.size() > 1;});
}

bool Policy::sequential() const
{
  return std::none_of(
    plan_.items.begin(), plan_.items.end(),
    [](const plansys2_msgs::msg::PlanItem & item) {return !item.children.empty();});
}

std::optional<std::uint32_t> Policy::successor(
  std::uint32_t index, const std::string & outcome) const
{
  if (sequential()) {
    // Nothing was observed and nothing chosen: the sequence continues.
    (void)outcome;
    return only_successor(index);
  }
  const auto & item = plan_.items[index];
  for (std::size_t i = 0; i < item.outcomes.size(); ++i) {
    if (item.outcomes[i] == outcome) {
      return item.children[i];
    }
  }
  return std::nullopt;
}

std::optional<std::uint32_t> Policy::only_successor(std::uint32_t index) const
{
  if (sequential()) {
    const auto next = index + 1;
    return next < plan_.items.size() ? std::optional<std::uint32_t>(next) : std::nullopt;
  }
  const auto & item = plan_.items[index];
  if (item.children.size() != 1 || item.children[0] == kDone) {
    return std::nullopt;
  }
  return item.children[0];
}

std::vector<std::uint32_t> Policy::preorder() const
{
  std::vector<std::uint32_t> order;
  if (plan_.items.empty()) {
    return order;
  }
  order.reserve(plan_.items.size());

  if (sequential()) {
    for (std::uint32_t i = 0; i < plan_.items.size(); ++i) {
      order.push_back(i);
    }
    return order;
  }

  const auto visit = [&](std::uint32_t index, const auto & self) -> void {
      order.push_back(index);
      for (const auto child : plan_.items[index].children) {
        if (child != kDone) {
          self(child, self);
        }
      }
    };
  visit(root(), visit);
  return order;
}

}  // namespace plansys2
