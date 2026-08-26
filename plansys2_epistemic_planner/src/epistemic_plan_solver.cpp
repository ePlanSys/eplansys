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

#include "plansys2_epistemic_planner/epistemic_plan_solver.hpp"

#include <unistd.h>

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "plansys2_epistemic_planner/action_mapping.hpp"
#include "plansys2_epistemic_planner/formula.hpp"
#include "plansys2_epistemic_planner/formula_text.hpp"
#include "plansys2_epistemic_planner/heuristic.hpp"
#include "plansys2_epistemic_planner/parser.hpp"
#include "plansys2_epistemic_planner/policy_plan.hpp"
#include "plansys2_epistemic_planner/selection_policy.hpp"
#include "plansys2_epistemic_planner/validator.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace plansys2
{

namespace
{

/// Aletheia's parser reads from a path. Writing the JSON to a temporary file
/// keeps the vendored parser untouched; it costs one small write per call,
/// which is immaterial next to the search itself.
class TempTask
{
public:
  explicit TempTask(const std::string & contents)
  {
    path_ = std::filesystem::temp_directory_path() /
      ("eplansys-task-" + std::to_string(::getpid()) + "-" +
      std::to_string(counter_++) + ".json");
    std::ofstream out(path_);
    out << contents;
  }

  ~TempTask()
  {
    std::error_code ec;
    std::filesystem::remove(path_, ec);
  }

  TempTask(const TempTask &) = delete;
  TempTask & operator=(const TempTask &) = delete;

  std::string path() const {return path_.string();}

private:
  std::filesystem::path path_;
  static inline unsigned counter_ = 0;
};

/// True when the string parses as an object carrying the grounded-task key.
bool is_epistemic_task_json(const std::string & s)
{
  if (s.find("planning-task-info") == std::string::npos) {
    return false;   // cheap reject before paying for a parse
  }
  try {
    const auto j = nlohmann::json::parse(s);
    return j.is_object() && j.contains("planning-task-info");
  } catch (const nlohmann::json::exception &) {
    return false;
  }
}

/// A policy tree that never offers a choice is a linear plan wearing a tree's
/// clothes; flattening it discards nothing and needs no warning.
bool branches_anywhere(const std::shared_ptr<PlanNode> & node)
{
  if (!node) {
    return false;
  }
  if (node->branches.size() > 1) {
    return true;
  }
  for (const auto & [event, child] : node->branches) {
    (void)event;
    if (branches_anywhere(child)) {
      return true;
    }
  }
  return false;
}

/// Follow the lowest event index at each node — the same rule serialize.py
/// uses for competition output.
void flatten(const std::shared_ptr<PlanNode> & node, std::vector<std::string> & out)
{
  if (!node) {
    return;
  }
  out.push_back(node->action);
  if (node->branches.empty()) {
    return;
  }
  const auto lowest = std::min_element(
    node->branches.begin(), node->branches.end(),
    [](const auto & a, const auto & b) {return a.first < b.first;});
  flatten(lowest->second, out);
}

std::unique_ptr<Heuristic> make_heuristic(const std::string & label)
{
  if (label == "ug") {return std::make_unique<UnsatisfiedGoalHeuristic>();}
  if (label == "ed") {return std::make_unique<EpistemicDistanceHeuristic>();}
  if (label == "ks") {return std::make_unique<KnowledgeSpreadHeuristic>();}
  if (label == "wc") {return std::make_unique<WorldCountHeuristic>();}
  if (label == "rpg") {
    return std::make_unique<RelaxedClosureHeuristic>(RelaxedAggregation::Max);
  }
  if (label == "radd") {
    return std::make_unique<RelaxedClosureHeuristic>(RelaxedAggregation::Add);
  }
  return nullptr;
}

/// Translate a grounded plan into a Plan message, or report the first action
/// the mapping does not cover. Aletheia is a classical-time planner: actions
/// are sequential and untimed, so they are laid out back to back, each
/// starting when the previous one ends.
std::optional<plansys2_msgs::msg::Plan> to_plan_msg(
  const std::vector<std::string> & actions,
  const ActionMapping & mapping,
  std::string & error)
{
  plansys2_msgs::msg::Plan plan;
  plan.items.reserve(actions.size());

  float t = 0.0f;
  for (const auto & a : actions) {
    const auto mapped = mapping.translate(a);
    if (!mapped) {
      error = "no mapping for grounded action '" + a + "'";
      return std::nullopt;
    }

    plansys2_msgs::msg::PlanItem item;
    item.time = t;
    item.action = mapped->action;
    item.duration = mapped->duration;
    plan.items.push_back(item);
    t += mapped->duration;
  }
  return plan;
}

}  // namespace

EpistemicPlanSolver::EpistemicPlanSolver()
{
}

void EpistemicPlanSolver::configure(
  rclcpp_lifecycle::LifecycleNode::SharedPtr lc_node,
  const std::string & plugin_name)
{
  lc_node_ = lc_node;

  task_file_parameter_name_ = plugin_name + ".task_file";
  heuristic_parameter_name_ = plugin_name + ".heuristic";
  strategy_parameter_name_ = plugin_name + ".strategy";
  policy_file_parameter_name_ = plugin_name + ".policy_file";
  conditional_parameter_name_ = plugin_name + ".conditional_plan";
  action_mapping_parameter_name_ = plugin_name + ".action_mapping";
  epddl_parameter_names_ = EpddlParameterNames(plugin_name);
  goal_from_state_parameter_name_ = plugin_name + ".goal_from_state";

  const auto declare = [&](const std::string & name, const std::string & def) {
      if (!lc_node_->has_parameter(name)) {
        lc_node_->declare_parameter<std::string>(name, def);
      }
    };

  declare(task_file_parameter_name_, "");
  declare(heuristic_parameter_name_, "");     // empty: leave it to the policy
  declare(strategy_parameter_name_, "");      // empty: leave it to the policy
  declare(policy_file_parameter_name_, "");
  declare(conditional_parameter_name_, "flatten");
  declare(action_mapping_parameter_name_, "");   // empty: naming convention

  if (!lc_node_->has_parameter(goal_from_state_parameter_name_)) {
    lc_node_->declare_parameter<bool>(goal_from_state_parameter_name_, true);
  }

  declare_epddl_parameters(lc_node_, epddl_parameter_names_);

  // Built once, so that its cache survives between calls: the planner is
  // asked for a task on every get_plan, and unchanged sources must not mean
  // another fork of plank.
  grounder_ = EpddlGrounder(read_plank_command(lc_node_, epddl_parameter_names_));

  // The epistemic state latches its state, so subscribing here is enough to
  // have the current goal before the first planning request arrives. A goal
  // set later arrives the same way. Nothing is asked of the state: getPlan
  // runs inside the planner's own service callback, and a service call from
  // there can deadlock on a single-threaded executor.
  if (lc_node_->get_parameter(goal_from_state_parameter_name_).as_bool()) {
    state_sub_ = lc_node_->create_subscription<std_msgs::msg::String>(
      "epistemic_state/state", rclcpp::QoS(1).transient_local(),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        try {
          const auto j = nlohmann::json::parse(msg->data);
          state_goal_ = j.value("goal", "");
        } catch (const nlohmann::json::exception &) {
          state_goal_.clear();   // an unreadable state is no goal, not a stale one
        }
      });
  }
}

std::string EpistemicPlanSolver::parameter(const std::string & name) const
{
  if (!lc_node_ || !lc_node_->has_parameter(name)) {
    return "";
  }
  return lc_node_->get_parameter(name).as_string();
}

std::optional<PlanningTask> EpistemicPlanSolver::resolve_task(
  const std::string & problem, std::string & error)
{
  const auto spec = read_epddl_spec(lc_node_, epddl_parameter_names_);
  const std::string task_file = parameter(task_file_parameter_name_);

  try {
    // A task arriving in the request describes this one call and outranks
    // anything configured, which is what makes a one-off task possible
    // without reconfiguring the planner node.
    if (is_epistemic_task_json(problem)) {
      TempTask tmp(problem);
      return load_task(tmp.path());
    }

    if (!spec.empty()) {
      if (!task_file.empty()) {
        RCLCPP_WARN(
          lc_node_->get_logger(),
          "[epistemic] both EPDDL sources and a task_file are set; grounding "
          "the sources and ignoring %s. Two descriptions of one problem drift "
          "apart, so name only one.", task_file.c_str());
      }

      const auto ground = grounder_.ground(spec);
      if (!ground.ok) {
        error = ground.error;
        return std::nullopt;
      }
      TempTask tmp(ground.task_json);
      return load_task(tmp.path());
    }

    if (!task_file.empty()) {
      if (!std::filesystem::exists(task_file)) {
        error = "task_file does not exist: " + task_file;
        return std::nullopt;
      }
      return load_task(task_file);
    }
  } catch (const std::exception & e) {
    error = std::string("could not load epistemic task: ") + e.what();
    return std::nullopt;
  }

  error =
    "no epistemic task: the problem string is not a grounded task, and "
    "neither the epddl_domain/epddl_problem pair nor task_file is set. This "
    "planner reads EPDDL, ground through plank, or the grounded IePC JSON "
    "directly; PDDL cannot express event models or per-agent observability, "
    "so no translation from the PDDL problem is attempted.";
  return std::nullopt;
}

bool EpistemicPlanSolver::apply_state_goal(PlanningTask & task, std::string & error) const
{
  if (state_goal_.empty()) {
    return true;      // no state, or a state with no goal: plan for the task's
  }

  const auto rendered = task.goal ? render_formula(task, *task.goal) : std::string();
  if (rendered == state_goal_) {
    return true;      // the state is reporting the task's own goal back
  }

  const auto goal = parse_formula(task, state_goal_, error);
  if (!goal) {
    error =
      "the epistemic state's goal '" + state_goal_ + "' does not parse against "
      "this task: " + error + ". The state and the planner are holding "
      "different problems.";
    return false;
  }

  task.goal = goal;
  // Derived from the goal by the parser, and now stale: it decides whether the
  // knowledge-spread heuristic is preferred, so a goal swap that left it
  // behind would keep selecting for the goal that was replaced.
  task.goal_kw_only = !has_atom_conjunct(*task.goal);

  RCLCPP_INFO(
    lc_node_->get_logger(), "[epistemic] planning for the goal set on the state: %s",
    state_goal_.c_str());
  return true;
}

std::optional<plansys2_msgs::msg::Plan> EpistemicPlanSolver::getPlan(
  const std::string & domain, const std::string & problem,
  const std::string & node_namespace, const rclcpp::Duration solver_timeout)
{
  (void)domain;          // the epistemic task is self-contained
  (void)node_namespace;

  cancel_requested_ = false;

  // The interned-formula registry is global and outlives any one task, so a
  // long-lived planner node would otherwise accumulate every formula of every
  // task it has ever solved. Clear it once nothing from the previous solve is
  // alive, which is here, before the next task is parsed.
  formula_registry_reset();

  std::string error;
  auto task_opt = resolve_task(problem, error);
  if (!task_opt) {
    RCLCPP_ERROR(lc_node_->get_logger(), "[epistemic] %s", error.c_str());
    return std::nullopt;
  }
  if (!apply_state_goal(*task_opt, error)) {
    RCLCPP_ERROR(lc_node_->get_logger(), "[epistemic] %s", error.c_str());
    return std::nullopt;
  }
  const PlanningTask & task = *task_opt;

  // Selection policy: explicit parameters win, otherwise the rule table
  // decides from the task's structure.
  SelectionPolicy policy;
  try {
    const std::string policy_file = parameter(policy_file_parameter_name_);
    policy = policy_file.empty() ? SelectionPolicy::builtin() :
      SelectionPolicy::load(policy_file);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(lc_node_->get_logger(), "[epistemic] selection policy: %s", e.what());
    return std::nullopt;
  }

  // Load the mapping before searching: a mapping file that cannot be read is
  // worth reporting immediately rather than after the search has spent the
  // whole timeout producing a plan that cannot be translated.
  ActionMapping mapping = ActionMapping::conventional();
  const std::string mapping_file = parameter(action_mapping_parameter_name_);
  if (!mapping_file.empty()) {
    try {
      mapping = ActionMapping::load(mapping_file);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(lc_node_->get_logger(), "[epistemic] action mapping: %s", e.what());
      return std::nullopt;
    }
    RCLCPP_INFO(
      lc_node_->get_logger(), "[epistemic] action mapping: %zu entries from %s",
      mapping.size(), mapping_file.c_str());
  } else {
    RCLCPP_WARN(
      lc_node_->get_logger(),
      "[epistemic] no action_mapping set; falling back to the naming "
      "convention, which guesses parameter order from the grounded name. Set "
      "action_mapping before dispatching to real actions.");
  }

  const TaskFeatures features = TaskFeatures::extract(task);

  std::string heuristic_label = parameter(heuristic_parameter_name_);
  if (heuristic_label.empty()) {
    heuristic_label = select(policy.heuristic_rules, features).outcome;
  }
  std::string strategy_label = parameter(strategy_parameter_name_);
  if (strategy_label.empty()) {
    strategy_label = select(policy.strategy_rules, features).outcome;
  }

  std::unique_ptr<Heuristic> h = make_heuristic(heuristic_label);
  if (!h) {
    RCLCPP_ERROR(
      lc_node_->get_logger(), "[epistemic] unknown heuristic '%s'",
      heuristic_label.c_str());
    return std::nullopt;
  }

  const Deadline deadline =
    std::chrono::steady_clock::now() +
    std::chrono::nanoseconds(solver_timeout.nanoseconds());

  RCLCPP_INFO(
    lc_node_->get_logger(),
    "[epistemic] strategy=%s heuristic=%s worlds=%zu designated=%zu actions=%zu",
    strategy_label.c_str(), heuristic_label.c_str(),
    static_cast<std::size_t>(task.init.num_worlds),
    static_cast<std::size_t>(task.init.num_designated()),
    static_cast<std::size_t>(task.num_actions()));

  std::vector<std::string> actions;

  if (strategy_label == "aostar") {
    auto result = aostar::search(task, *h, 0, deadline);
    if (!result) {
      RCLCPP_WARN(lc_node_->get_logger(), "[epistemic] no solution found");
      return std::nullopt;
    }

    // An empty tree means the goal already held: a valid, empty plan.
    if (!result->plan_tree) {
      std::string unused;
      return to_plan_msg({}, mapping, unused);
    }

    const auto vr = validate(task, result->plan_tree);
    if (!vr.valid) {
      RCLCPP_ERROR(
        lc_node_->get_logger(), "[epistemic] plan failed validation: %s",
        vr.error.c_str());
      return std::nullopt;
    }

    const std::string mode = parameter(conditional_parameter_name_);

    // "policy" keeps the branches. The Plan message carries them in the
    // epistemic fields of its items, and an executor with an epistemic BT
    // builder runs the branch the world turns out to take. The other two modes
    // predate that path and are kept for a plain PlanSys2 executor, which can
    // only run a sequence.
    if (mode == "policy") {
      std::string policy_error;
      auto policy = to_policy_plan(task, result->plan_tree, mapping, policy_error);
      if (!policy) {
        RCLCPP_ERROR(
          lc_node_->get_logger(),
          "[epistemic] %s. Add it to the action_mapping file.", policy_error.c_str());
        return std::nullopt;
      }
      RCLCPP_INFO(
        lc_node_->get_logger(), "[epistemic] policy with %zu nodes%s",
        policy->items.size(),
        policy_branches(result->plan_tree) ? ", branching" : ", linear");
      return policy;
    }

    if (branches_anywhere(result->plan_tree)) {
      if (mode == "reject") {
        RCLCPP_ERROR(
          lc_node_->get_logger(),
          "[epistemic] the solution is a branching policy, which a flat "
          "plansys2_msgs/Plan cannot represent, and conditional_plan is "
          "'reject'. Set conditional_plan to 'policy' to keep the branches, "
          "which needs an executor using the epistemic BT builder.");
        return std::nullopt;
      }
      RCLCPP_WARN(
        lc_node_->get_logger(),
        "[epistemic] the solution is a branching policy; returning only the "
        "lowest-event branch because plansys2_msgs/Plan is a flat sequence. "
        "This plan is valid only if execution takes that contingency.");
    }

    flatten(result->plan_tree, actions);

  } else if (strategy_label == "ehc" || strategy_label == "gbfs") {
    auto result = strategy_label == "ehc" ?
      ehc::search(task, *h, 0, deadline) :
      gbfs::search(task, *h, 0, deadline);

    if (!result && strategy_label == "ehc") {
      RCLCPP_INFO(lc_node_->get_logger(), "[epistemic] EHC failed, falling back to GBFS");
      result = gbfs::search(task, *h, 0, deadline);
    }
    if (!result) {
      RCLCPP_WARN(lc_node_->get_logger(), "[epistemic] no solution found");
      return std::nullopt;
    }
    actions = result->plan;

  } else {
    RCLCPP_ERROR(
      lc_node_->get_logger(), "[epistemic] unknown strategy '%s'",
      strategy_label.c_str());
    return std::nullopt;
  }

  RCLCPP_INFO(lc_node_->get_logger(), "[epistemic] plan length %zu", actions.size());

  std::string mapping_error;
  auto plan = to_plan_msg(actions, mapping, mapping_error);
  if (!plan) {
    // The plan is sound; it just cannot be expressed in the executor's
    // vocabulary. Returning it anyway would have the executor reject or, worse,
    // silently skip the action.
    RCLCPP_ERROR(
      lc_node_->get_logger(),
      "[epistemic] %s. Add it to the action_mapping file.", mapping_error.c_str());
    return std::nullopt;
  }
  return plan;
}

bool EpistemicPlanSolver::isDomainValid(
  const std::string & domain, const std::string & node_namespace)
{
  (void)domain;
  (void)node_namespace;

  // The epistemic task is self-contained and carries no separate domain, so
  // there is nothing here to validate. Reporting false would stop the planner
  // node from coming up; the task itself is checked when it is loaded.
  return true;
}

}  // namespace plansys2

PLUGINLIB_EXPORT_CLASS(plansys2::EpistemicPlanSolver, plansys2::PlanSolverBase)
