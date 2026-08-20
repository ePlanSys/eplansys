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

#include "plansys2_aletheia_plan_solver/aletheia_plan_solver.hpp"

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "plansys2_epistemic_planner/action_mapping.hpp"
#include "plansys2_epistemic_planner/formula.hpp"
#include "plansys2_epistemic_planner/parser.hpp"
#include "plansys2_epistemic_planner/policy_plan.hpp"
#include "plansys2_epistemic_planner/validator.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace plansys2
{

namespace
{

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

/// Follow the lowest event index at each node, which is the rule the planner's
/// own competition output uses, so both flattenings pick the same branch.
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

/// Lay a grounded sequence out back to back, each action starting when the
/// previous one ends. The planner is untimed, so the durations are the
/// mapping's, not the plan's.
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

/// Read one node of the tree form. Recursive, and bounded by the file: a
/// malformed branch is reported rather than skipped, because a policy missing
/// a branch would execute as a policy that simply never handles that outcome.
std::optional<std::shared_ptr<PlanNode>> read_plan_node(
  const nlohmann::json & j, std::string & error)
{
  if (j.is_null()) {
    return std::shared_ptr<PlanNode>{};   // a leaf: nothing more to do
  }
  if (!j.is_object()) {
    error = "a plan node is neither an object nor null";
    return std::nullopt;
  }
  if (!j.contains("action") || !j["action"].is_string()) {
    error = "a plan node has no \"action\" string";
    return std::nullopt;
  }

  auto node = std::make_shared<PlanNode>();
  node->action = j["action"].get<std::string>();

  if (!j.contains("branches")) {
    return node;
  }
  if (!j["branches"].is_array()) {
    error = "the \"branches\" of node '" + node->action + "' is not an array";
    return std::nullopt;
  }

  for (const auto & branch : j["branches"]) {
    if (!branch.is_object() || !branch.contains("event") ||
      !branch["event"].is_number_unsigned())
    {
      error = "a branch of node '" + node->action + "' has no unsigned \"event\"";
      return std::nullopt;
    }
    if (!branch.contains("subtree")) {
      error = "a branch of node '" + node->action + "' has no \"subtree\"";
      return std::nullopt;
    }

    auto child = read_plan_node(branch["subtree"], error);
    if (!child) {
      return std::nullopt;
    }
    node->branches.emplace_back(
      static_cast<EventIdx>(branch["event"].get<std::uint32_t>()), std::move(*child));
  }

  return node;
}

}  // namespace

std::optional<AletheiaPlan> read_plan_file(const std::string & path, std::string & error)
{
  std::ifstream in(path);
  if (!in.is_open()) {
    // The planner reports "no solution found" by exiting zero and writing no
    // plan file, so a missing file is an ordinary outcome and says so.
    error = "no plan file at " + path + ": the planner found no solution, or "
      "never ran";
    return std::nullopt;
  }

  nlohmann::json j;
  try {
    in >> j;
  } catch (const nlohmann::json::exception & e) {
    error = std::string("the plan file is not valid JSON: ") + e.what();
    return std::nullopt;
  }

  AletheiaPlan plan;

  if (j.is_array()) {
    for (const auto & entry : j) {
      if (!entry.is_string()) {
        error = "a linear plan holds an entry that is not an action name";
        return std::nullopt;
      }
      plan.linear.push_back(entry.get<std::string>());
    }
    return plan;
  }

  plan.is_tree = true;
  auto tree = read_plan_node(j, error);
  if (!tree) {
    return std::nullopt;
  }
  plan.tree = std::move(*tree);
  return plan;
}

AletheiaPlanSolver::AletheiaPlanSolver()
{
}

void AletheiaPlanSolver::configure(
  rclcpp_lifecycle::LifecycleNode::SharedPtr lc_node,
  const std::string & plugin_name)
{
  lc_node_ = lc_node;

  command_parameter_name_ = plugin_name + ".command";
  arguments_parameter_name_ = plugin_name + ".arguments";
  output_dir_parameter_name_ = plugin_name + ".output_dir";
  task_file_parameter_name_ = plugin_name + ".task_file";
  heuristic_parameter_name_ = plugin_name + ".heuristic";
  strategy_parameter_name_ = plugin_name + ".strategy";
  policy_file_parameter_name_ = plugin_name + ".policy_file";
  action_mapping_parameter_name_ = plugin_name + ".action_mapping";
  conditional_parameter_name_ = plugin_name + ".conditional_plan";

  const auto declare = [&](const std::string & name, const std::string & def) {
      if (!lc_node_->has_parameter(name)) {
        lc_node_->declare_parameter<std::string>(name, def);
      }
    };

  declare(command_parameter_name_, "epistemic_planner");
  declare(arguments_parameter_name_, "");
  declare(output_dir_parameter_name_, std::filesystem::temp_directory_path().string());
  declare(task_file_parameter_name_, "");
  declare(heuristic_parameter_name_, "");     // empty: leave it to the planner
  declare(strategy_parameter_name_, "");      // empty: leave it to the planner
  declare(policy_file_parameter_name_, "");
  declare(action_mapping_parameter_name_, "");   // empty: naming convention
  declare(conditional_parameter_name_, "flatten");
}

std::string AletheiaPlanSolver::parameter(const std::string & name) const
{
  if (!lc_node_ || !lc_node_->has_parameter(name)) {
    return "";
  }
  return lc_node_->get_parameter(name).as_string();
}

std::optional<std::filesystem::path>
AletheiaPlanSolver::create_folders(const std::string & node_namespace)
{
  auto output_dir = parameter(output_dir_parameter_name_);
  if (output_dir.empty()) {
    output_dir = std::filesystem::temp_directory_path().string();
  }

  const char * home_dir = std::getenv("HOME");
  if (!output_dir.empty() && output_dir[0] == '~') {
    if (!home_dir) {
      RCLCPP_ERROR(
        lc_node_->get_logger(), "[aletheia] cannot expand ~ in output_dir: HOME is unset");
      return std::nullopt;
    }
    output_dir.replace(0, 1, home_dir);
  }

  auto output_path = std::filesystem::path(output_dir);
  if (!node_namespace.empty()) {
    for (const auto & p : std::filesystem::path(node_namespace)) {
      if (p != std::filesystem::current_path().root_directory()) {
        output_path /= p;
      }
    }
  }

  try {
    std::filesystem::create_directories(output_path);
  } catch (const std::filesystem::filesystem_error & err) {
    RCLCPP_ERROR(
      lc_node_->get_logger(), "[aletheia] could not create %s: %s",
      output_path.string().c_str(), err.what());
    return std::nullopt;
  }

  return output_path;
}

std::optional<std::filesystem::path> AletheiaPlanSolver::resolve_task_path(
  const std::string & problem, const std::filesystem::path & output_dir,
  std::string & error) const
{
  if (is_epistemic_task_json(problem)) {
    // The planner reads a path, so an inline task is written out. It stays on
    // disk next to the plan and the log, which is what makes a failed run
    // reproducible by hand with the same command line.
    const auto path = output_dir / "task.json";
    std::ofstream out(path);
    if (!out.is_open()) {
      error = "could not write the task to " + path.string();
      return std::nullopt;
    }
    out << problem;
    return path;
  }

  const std::string task_file = parameter(task_file_parameter_name_);
  if (!task_file.empty()) {
    if (!std::filesystem::exists(task_file)) {
      error = "task_file does not exist: " + task_file;
      return std::nullopt;
    }
    return std::filesystem::path(task_file);
  }

  error =
    "the problem string is not a grounded epistemic task and no task_file "
    "parameter is set. This planner reads the IePC epistemic JSON format; "
    "PDDL cannot express event models or per-agent observability, so no "
    "translation from the PDDL problem is attempted.";
  return std::nullopt;
}

std::string AletheiaPlanSolver::build_command(
  const std::filesystem::path & task_path,
  const std::filesystem::path & plan_path,
  const rclcpp::Duration & solver_timeout) const
{
  // PlanSolverBase::tokenize splits on whitespace with std::quoted, so every
  // path is quoted: an output directory under a home directory with a space in
  // it would otherwise arrive as two arguments.
  const auto quoted = [](const std::string & s) {return "\"" + s + "\"";};

  std::ostringstream cmd;
  cmd << parameter(command_parameter_name_);
  cmd << " --task " << quoted(task_path.string());
  cmd << " --plan " << quoted(plan_path.string());

  const auto heuristic = parameter(heuristic_parameter_name_);
  if (!heuristic.empty()) {
    cmd << " --heuristic " << heuristic;
  }
  const auto strategy = parameter(strategy_parameter_name_);
  if (!strategy.empty()) {
    cmd << " --strategy " << strategy;
  }
  const auto policy_file = parameter(policy_file_parameter_name_);
  if (!policy_file.empty()) {
    cmd << " --policy " << quoted(policy_file);
  }

  // The planner's own timeout applies to AO* only, and is a second line of
  // defence: PlanSolverBase kills the process at the same deadline. Asking the
  // planner to stop itself is worth doing anyway, since it exits reporting no
  // solution rather than being killed mid-write.
  cmd << " --timeout " << static_cast<std::int64_t>(solver_timeout.seconds());

  const auto arguments = parameter(arguments_parameter_name_);
  if (!arguments.empty()) {
    cmd << " " << arguments;
  }

  return cmd.str();
}

std::optional<plansys2_msgs::msg::Plan> AletheiaPlanSolver::getPlan(
  const std::string & domain, const std::string & problem,
  const std::string & node_namespace, const rclcpp::Duration solver_timeout)
{
  (void)domain;          // the epistemic task is self-contained

  const auto output_dir = create_folders(node_namespace);
  if (!output_dir) {
    return std::nullopt;
  }

  std::string error;
  const auto task_path = resolve_task_path(problem, *output_dir, error);
  if (!task_path) {
    RCLCPP_ERROR(lc_node_->get_logger(), "[aletheia] %s", error.c_str());
    return std::nullopt;
  }

  // The task is parsed here as well as in the subprocess. It is what names the
  // events a branch is taken on, the epistemic conditions each action needs
  // and the goal a leaf is checked against, none of which the plan file
  // carries: it names actions and event indices only.
  //
  // The interned-formula registry is global and outlives any one task, so it
  // is cleared before this task's formulas are built.
  formula_registry_reset();

  PlanningTask task;
  try {
    task = load_task(task_path->string());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      lc_node_->get_logger(), "[aletheia] could not load %s: %s",
      task_path->string().c_str(), e.what());
    return std::nullopt;
  }

  ActionMapping mapping = ActionMapping::conventional();
  const auto mapping_file = parameter(action_mapping_parameter_name_);
  if (!mapping_file.empty()) {
    try {
      mapping = ActionMapping::load(mapping_file);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        lc_node_->get_logger(), "[aletheia] could not load the action mapping: %s", e.what());
      return std::nullopt;
    }
  }

  const auto plan_path = *output_dir / "plan.json";
  const auto log_path = *output_dir / "aletheia.log";

  // A plan file left by an earlier request would otherwise be read as this
  // request's answer when the planner fails before writing one.
  std::error_code ec;
  std::filesystem::remove(plan_path, ec);

  const auto command = build_command(*task_path, plan_path, solver_timeout);
  RCLCPP_DEBUG(
    lc_node_->get_logger(), "[aletheia] running: %s (timeout %.1fs)",
    command.c_str(), solver_timeout.seconds());

  // The third argument is where the child's stdout is captured. POPF prints
  // its plan there; Aletheia writes the plan to --plan and prints its search
  // trace instead, so this is a log rather than the result. Its stderr is not
  // redirected and reaches the planner node's own console.
  if (!execute_planner(command, solver_timeout, log_path.string())) {
    RCLCPP_ERROR(
      lc_node_->get_logger(),
      "[aletheia] the planner failed, was cancelled, or timed out. Command: %s. Output: %s",
      command.c_str(), log_path.string().c_str());
    return std::nullopt;
  }

  const auto plan = read_plan_file(plan_path.string(), error);
  if (!plan) {
    RCLCPP_ERROR(
      lc_node_->get_logger(), "[aletheia] %s. Planner output: %s",
      error.c_str(), log_path.string().c_str());
    return std::nullopt;
  }

  if (!plan->is_tree) {
    // GBFS and EHC write a sequence, which has no branches to preserve and no
    // events to name, so it converts directly.
    auto msg = to_plan_msg(plan->linear, mapping, error);
    if (!msg) {
      RCLCPP_ERROR(
        lc_node_->get_logger(),
        "[aletheia] %s. Add it to the action_mapping file.", error.c_str());
      return std::nullopt;
    }
    RCLCPP_INFO(
      lc_node_->get_logger(), "[aletheia] plan with %zu actions", msg->items.size());
    return msg;
  }

  if (!plan->tree) {
    return plansys2_msgs::msg::Plan();   // the goal already held: an empty plan
  }

  // The planner validates its own solution, and this validates it again
  // against the task as parsed here. The two parses could disagree only if the
  // binary and this workspace were built from different sources, which is
  // exactly the failure a separately built planner introduces and exactly what
  // this catches.
  const auto vr = validate(task, plan->tree);
  if (!vr.valid) {
    RCLCPP_ERROR(
      lc_node_->get_logger(), "[aletheia] the plan failed validation: %s", vr.error.c_str());
    return std::nullopt;
  }

  const std::string mode = parameter(conditional_parameter_name_);

  if (mode == "policy") {
    auto policy = to_policy_plan(task, plan->tree, mapping, error);
    if (!policy) {
      RCLCPP_ERROR(
        lc_node_->get_logger(),
        "[aletheia] %s. Add it to the action_mapping file.", error.c_str());
      return std::nullopt;
    }
    RCLCPP_INFO(
      lc_node_->get_logger(), "[aletheia] policy with %zu nodes%s",
      policy->items.size(), policy_branches(plan->tree) ? ", branching" : ", linear");
    return policy;
  }

  if (branches_anywhere(plan->tree)) {
    if (mode == "reject") {
      RCLCPP_ERROR(
        lc_node_->get_logger(),
        "[aletheia] the solution is a branching policy, which a flat "
        "plansys2_msgs/Plan cannot represent, and conditional_plan is "
        "'reject'. Set conditional_plan to 'policy' to keep the branches, "
        "which needs an executor using the epistemic BT builder.");
      return std::nullopt;
    }
    RCLCPP_WARN(
      lc_node_->get_logger(),
      "[aletheia] the solution is a branching policy; returning only the "
      "lowest-event branch because plansys2_msgs/Plan is a flat sequence. "
      "This plan is valid only if execution takes that contingency.");
  }

  std::vector<std::string> actions;
  flatten(plan->tree, actions);

  auto msg = to_plan_msg(actions, mapping, error);
  if (!msg) {
    RCLCPP_ERROR(
      lc_node_->get_logger(),
      "[aletheia] %s. Add it to the action_mapping file.", error.c_str());
    return std::nullopt;
  }
  RCLCPP_INFO(
    lc_node_->get_logger(), "[aletheia] plan with %zu actions", msg->items.size());
  return msg;
}

bool AletheiaPlanSolver::isDomainValid(
  const std::string & domain, const std::string & node_namespace)
{
  (void)domain;
  (void)node_namespace;
  return true;
}

}  // namespace plansys2

PLUGINLIB_EXPORT_CLASS(plansys2::AletheiaPlanSolver, plansys2::PlanSolverBase)
