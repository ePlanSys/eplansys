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

#include "plansys2_epistemic_executor/policy_schedule.hpp"

#include <algorithm>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <utility>
#include <vector>

namespace plansys2
{

namespace
{

using Node = ScheduledNode;

Schedule written(
  const Policy & policy, std::uint32_t index, std::map<std::uint32_t, Schedule> & made)
{
  const auto found = made.find(index);
  if (found != made.end()) {
    return found->second;
  }

  auto node = std::make_shared<Node>();
  node->item = index;
  made[index] = node;

  // The three readings policy_to_bt has always made of a node: one
  // continuation, several chosen by outcome, or none.
  const auto & item = policy.item(index);
  if (const auto only = policy.only_successor(index)) {
    node->next = {written(policy, *only, made)};
  } else if (item.children.size() > 1) {
    for (const auto child : item.children) {
      node->next.push_back(
        child == plansys2_msgs::msg::PlanItem::POLICY_DONE ?
        nullptr : written(policy, child, made));
    }
  }
  return node;
}

/// Past this many nodes the rearrangement is abandoned. Moving a node above a
/// branch copies what it passed once per outcome, and a policy that branches
/// on many independent observations would multiply without bound; one that
/// does is dispatched as `parallel_groups` would dispatch it.
constexpr std::size_t kMaxNodes = 20000;

class Scheduler
{
public:
  Scheduler(const Policy & policy, const Independence & independent)
  : policy_(policy), independent_(independent) {}

  /// An action's earliest start below `above`, and what it depends on there.
  using Above = std::vector<std::pair<std::uint32_t, double>>;

  /// The subtree with every action moved as high as it may go.
  Schedule settle(const Schedule & node, const Above & above)
  {
    if (!node || too_big()) {
      return node;
    }

    const double start = earliest(node->item, above);
    auto copy = fresh(*node);

    auto below = above;
    below.emplace_back(node->item, start);
    for (auto & next : copy->next) {
      next = settle(next, below);
    }

    if (auto lifted = lift(copy, above, start)) {
      return lifted;
    }
    return copy;
  }

  /// Mark the chains that start together, heads only.
  void group(const Schedule & node)
  {
    if (!node || !seen_.insert(node.get()).second) {
      return;
    }

    std::vector<std::uint32_t> members{node->item};
    std::vector<Node *> frontier{node.get()};

    while (true) {
      std::vector<Node *> following;
      bool whole = true;
      for (const auto * at : frontier) {
        if (at->next.empty()) {
          whole = false;
          break;
        }
        for (const auto & next : at->next) {
          if (!next) {
            whole = false;
            break;
          }
          following.push_back(next.get());
        }
      }
      if (!whole || following.empty()) {
        break;
      }

      const auto candidate = following.front()->item;
      const bool twins = std::all_of(
        following.begin(), following.end(),
        [&](const Node * other) {return same_action(other->item, candidate);});
      const bool free = std::all_of(
        members.begin(), members.end(),
        [&](std::uint32_t member) {return overlap(member, candidate);});
      if (!twins || !free) {
        break;
      }

      members.push_back(candidate);
      frontier = std::move(following);
    }

    node->group = members.size();
    for (auto * at : frontier) {
      for (const auto & next : at->next) {
        group(next);
      }
    }
  }

  bool too_big() const {return made_ > kMaxNodes;}

private:
  Schedule fresh(const Node & from)
  {
    ++made_;
    return std::make_shared<Node>(from);
  }

  bool same_action(std::uint32_t a, std::uint32_t b) const
  {
    const auto & first = policy_.item(a);
    const auto & second = policy_.item(b);
    return first.action == second.action &&
           first.epistemic_action == second.epistemic_action &&
           first.outcomes == second.outcomes;
  }

  bool overlap(std::uint32_t a, std::uint32_t b)
  {
    const auto key = std::minmax(a, b);
    const auto found = overlap_.find(key);
    if (found != overlap_.end()) {
      return found->second;
    }
    const bool answer = may_overlap(policy_.item(a), policy_.item(b), independent_);
    overlap_[key] = answer;
    return answer;
  }

  /// When the action could start at the soonest: once everything above it
  /// that it depends on has finished, by declared duration.
  double earliest(std::uint32_t item, const Above & above)
  {
    double start = 0.0;
    for (const auto & [ancestor, ancestor_start] : above) {
      if (!overlap(ancestor, item)) {
        const auto took = std::max(0.0, static_cast<double>(policy_.item(ancestor).duration));
        start = std::max(start, ancestor_start + took);
      }
    }
    return start;
  }

  /// Move what follows `parent` above it, when that is the same action on
  /// every branch, independent of the parent, and able to start sooner. The
  /// parent is copied under each of the moved action's own continuations,
  /// with what used to follow the moved action there below it.
  Schedule lift(const Schedule & parent, const Above & above, double parent_start)
  {
    if (parent->next.empty()) {
      return nullptr;
    }
    for (const auto & next : parent->next) {
      if (!next) {
        return nullptr;     // a branch that ends has nothing to move
      }
    }

    const auto & first = parent->next.front();
    for (const auto & next : parent->next) {
      if (!same_action(next->item, first->item)) {
        return nullptr;
      }
    }
    if (!overlap(parent->item, first->item)) {
      return nullptr;
    }

    const double start = earliest(first->item, above);
    if (!(start < parent_start)) {
      return nullptr;
    }

    // A moved action that ended the policy leaves one continuation, the
    // parent, whose branches then end where the moved action did.
    const std::size_t slots = std::max<std::size_t>(first->next.size(), 1);

    auto moved = fresh(*first);
    moved->next.clear();
    auto below = above;
    below.emplace_back(first->item, start);

    for (std::size_t slot = 0; slot < slots; ++slot) {
      auto copy = fresh(*parent);
      for (std::size_t branch = 0; branch < copy->next.size(); ++branch) {
        const auto & twin = parent->next[branch];
        copy->next[branch] = twin->next.empty() ? nullptr : twin->next[slot];
      }
      moved->next.push_back(settle(copy, below));
    }
    return moved;
  }

  const Policy & policy_;
  const Independence & independent_;
  std::map<std::pair<std::uint32_t, std::uint32_t>, bool> overlap_;
  std::set<const Node *> seen_;
  std::size_t made_{0};
};

}  // namespace

Schedule policy_schedule(const Policy & policy)
{
  return policy_schedule(policy, ParallelGroups{});
}

Schedule policy_schedule(const Policy & policy, const ParallelGroups & groups)
{
  if (policy.empty()) {
    return nullptr;
  }

  std::map<std::uint32_t, Schedule> made;
  auto root = written(policy, Policy::root(), made);

  for (const auto & group : groups) {
    const auto head = made.find(group.front());
    if (group.size() > 1 && head != made.end()) {
      head->second->group = group.size();
    }
  }
  return root;
}

Schedule schedule_policy(const Policy & policy, const Independence & independent)
{
  if (policy.empty() || policy.sequential()) {
    return policy_schedule(policy);
  }

  Scheduler scheduler(policy, independent);
  auto root = scheduler.settle(policy_schedule(policy), {});
  if (scheduler.too_big()) {
    return policy_schedule(policy, parallel_groups(policy, independent));
  }
  scheduler.group(root);
  return root;
}

std::vector<std::vector<std::uint32_t>> scheduled_groups(const Schedule & schedule)
{
  std::vector<std::vector<std::uint32_t>> groups;
  std::set<const ScheduledNode *> seen;

  const auto walk = [&](const Schedule & node, const auto & self) -> void {
      if (!node || !seen.insert(node.get()).second) {
        return;
      }
      if (node->group > 1) {
        std::vector<std::uint32_t> members;
        const ScheduledNode * at = node.get();
        for (std::size_t i = 0; i < node->group && at; ++i) {
          members.push_back(at->item);
          at = i + 1 < node->group ? at->next.front().get() : nullptr;
        }
        groups.push_back(members);
      }
      for (const auto & next : node->next) {
        self(next, self);
      }
    };
  walk(schedule, walk);
  return groups;
}

}  // namespace plansys2
