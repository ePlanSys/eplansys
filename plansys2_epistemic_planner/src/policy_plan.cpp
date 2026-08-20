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

#include "plansys2_epistemic_planner/policy_plan.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <set>
#include <utility>
#include <string>
#include <vector>

#include "plansys2_epistemic_planner/action.hpp"
#include "plansys2_epistemic_planner/formula.hpp"

namespace plansys2
{

std::string render_formula(const PlanningTask & task, const Formula & f)
{
  const auto agent = [&task](AgentIdx a) -> std::string {
      return a < task.agent_names.size() ? task.agent_names[a] : "?agent";
    };
  const auto atom = [&task](AtomIdx a) -> std::string {
      return a < task.atom_names.size() ? task.atom_names[a] : "?atom";
    };
  // The modality an agent's box denotes differs by frame: S5 is knowledge,
  // KD45 is belief, and calling both "K" would misreport what the plan needs.
  const char * box = task.kd45 ? "B" : "K";

  switch (f.kind) {
    case FormulaKind::Top:
      return "(true)";
    case FormulaKind::Bot:
      return "(false)";
    case FormulaKind::Atom:
      return atom(f.atom);
    case FormulaKind::Not:
      return "(not " + (f.children.empty() ? "?" : render_formula(task, *f.children[0])) + ")";
    case FormulaKind::And:
    case FormulaKind::Or: {
        std::string out = f.kind == FormulaKind::And ? "(and" : "(or";
        for (const auto & child : f.children) {
          out += " " + render_formula(task, *child);
        }
        return out + ")";
      }
    case FormulaKind::Belief:
      return std::string("(") + box + " " + agent(f.agent) + " " +
             (f.children.empty() ? "?" : render_formula(task, *f.children[0])) + ")";
    case FormulaKind::Kw:
      return "(Kw " + agent(f.agent) + " " +
             (f.children.empty() ? "?" : render_formula(task, *f.children[0])) + ")";
    case FormulaKind::Common: {
        std::string out = "(C (";
        for (std::size_t i = 0; i < f.group.size(); ++i) {
          out += (i ? " " : "") + agent(f.group[i]);
        }
        out += ") " + (f.children.empty() ? "?" : render_formula(task, *f.children[0])) + ")";
        return out;
      }
  }
  return "?";
}

namespace
{

/// The epistemic conditions an action needs, one string per designated event.
///
/// Only the modal ones are reported. An action's precondition may mix ordinary
/// facts with knowledge, and the ordinary part is already the PDDL action's
/// own precondition, checked by the executor against the problem expert;
/// repeating it here would have two components checking the same thing against
/// two different stores.
std::vector<std::string> knowledge_requirements(
  const PlanningTask & task, const Action & action)
{
  const auto is_modal = [](const Formula & f) {
      return f.kind == FormulaKind::Belief || f.kind == FormulaKind::Kw ||
             f.kind == FormulaKind::Common;
    };

  // A conjunction is split, so that a failing conjunct can be named on its own
  // rather than reported as part of one long formula.
  std::vector<std::string> out;
  const auto collect = [&](const Formula & f, const auto & self) -> void {
      if (f.kind == FormulaKind::And) {
        for (const auto & child : f.children) {
          self(*child, self);
        }
        return;
      }
      if (is_modal(f)) {
        out.push_back(render_formula(task, f));
      }
    };

  for (const auto event_id : action.designated_events) {
    if (event_id < action.events.size() && action.events[event_id].precondition) {
      collect(*action.events[event_id].precondition, collect);
    }
  }

  std::sort(out.begin(), out.end());
  out.erase(std::unique(out.begin(), out.end()), out.end());
  return out;
}

std::string event_name(const Action & action, EventIdx event)
{
  if (event < action.events.size() && !action.events[event].name.empty()) {
    return action.events[event].name;
  }
  return "e" + std::to_string(event);
}

/// Give every node a distinct start time, keeping each strictly after its
/// parent ends.
///
/// The executor keys its action map by action expression and start time in
/// milliseconds, so two nodes on different branches running the same action at
/// the same time would collide onto one entry and drive one action executor
/// from two places. Branches are alternatives that never both run, so the
/// collision is invisible until the day the wrong branch is taken.
///
/// Nodes are visited parents-first, which is the order they were emitted in,
/// so a parent's time is already final when its children are adjusted.
void assign_unique_times(plansys2_msgs::msg::Plan & plan)
{
  const auto to_ms = [](float t) {return static_cast<std::int64_t>(std::llround(t * 1000.0));};

  std::set<std::pair<std::string, std::int64_t>> taken;

  for (std::size_t i = 0; i < plan.items.size(); ++i) {
    auto & item = plan.items[i];
    std::int64_t ms = to_ms(item.time);

    while (!taken.insert({item.action, ms}).second) {
      ++ms;   // the next free millisecond for this action
    }
    item.time = static_cast<float>(ms) / 1000.0f;

    const std::int64_t ends = ms + to_ms(item.duration);
    for (const auto child : item.children) {
      if (child != plansys2_msgs::msg::PlanItem::POLICY_DONE) {
        auto & successor = plan.items[child];
        successor.time = std::max(successor.time, static_cast<float>(ends) / 1000.0f);
      }
    }
  }
}

}  // namespace

bool policy_branches(const std::shared_ptr<PlanNode> & tree)
{
  if (!tree) {
    return false;
  }
  if (tree->branches.size() > 1) {
    return true;
  }
  for (const auto & [event, child] : tree->branches) {
    (void)event;
    if (policy_branches(child)) {
      return true;
    }
  }
  return false;
}

std::optional<plansys2_msgs::msg::Plan> to_policy_plan(
  const PlanningTask & task,
  const std::shared_ptr<PlanNode> & tree,
  const ActionMapping & mapping,
  std::string & error)
{
  plansys2_msgs::msg::Plan plan;
  if (task.goal) {
    plan.epistemic_goal = render_formula(task, *task.goal);
  }

  if (!tree) {
    return plan;   // the goal already held: a valid, empty policy
  }

  // Depth-first, appending each node as it is reached, so items[0] is the root
  // and a parent is always written before the children that name it. The
  // recursion returns the index it wrote, which is what the parent records.
  const auto emit =
    [&](const std::shared_ptr<PlanNode> & node, float start, const auto & self)
    -> std::optional<std::uint32_t> {
      const auto mapped = mapping.translate(node->action);
      if (!mapped) {
        error = "no mapping for grounded action '" + node->action + "'";
        return std::nullopt;
      }

      const auto action_it = task.action_index.find(node->action);
      if (action_it == task.action_index.end()) {
        error = "the policy names an action absent from the task: " + node->action;
        return std::nullopt;
      }
      const Action & action = task.actions[action_it->second];

      plansys2_msgs::msg::PlanItem item;
      item.time = start;
      item.action = mapped->action;
      item.duration = mapped->duration;
      item.epistemic_action = node->action;
      item.sensing = !action.is_ontic();
      item.knowledge_requirements = knowledge_requirements(task, action);

      const std::uint32_t index = static_cast<std::uint32_t>(plan.items.size());
      plan.items.push_back(item);

      // Branches are emitted in event order so that the policy reads the same
      // way twice, and so that a flattening consumer taking the first child
      // takes the same one the solver's own flatten does.
      auto branches = node->branches;
      std::sort(
        branches.begin(), branches.end(),
        [](const auto & a, const auto & b) {return a.first < b.first;});

      const float child_start = start + mapped->duration;
      for (const auto & [event, child] : branches) {
        std::uint32_t child_index = plansys2_msgs::msg::PlanItem::POLICY_DONE;
        if (child) {
          const auto emitted = self(child, child_start, self);
          if (!emitted) {
            return std::nullopt;
          }
          child_index = *emitted;
        }
        // The vector may have grown behind us during the recursion, so the
        // parent is addressed by index rather than by a reference taken above.
        plan.items[index].children.push_back(child_index);
        plan.items[index].outcomes.push_back(event_name(action, event));
      }

      return index;
    };

  if (!emit(tree, 0.0f, emit)) {
    return std::nullopt;
  }

  assign_unique_times(plan);
  return plan;
}

}  // namespace plansys2
