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

#ifndef PLANSYS2_ALETHEIA_PLAN_SOLVER__ALETHEIA_PLAN_SOLVER_HPP_
#define PLANSYS2_ALETHEIA_PLAN_SOLVER__ALETHEIA_PLAN_SOLVER_HPP_

#include <chrono>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "plansys2_core/PlanSolverBase.hpp"
#include "plansys2_epistemic_planner/search.hpp"
#include "plansys2_epistemic_planner/task.hpp"
#include "plansys2_msgs/msg/plan.hpp"

namespace plansys2
{

/// What the planner wrote to its plan file.
///
/// Aletheia emits one of two shapes, and which one it emits is decided by the
/// strategy it ran: AO* writes a policy tree, GBFS and EHC write a flat array
/// of grounded action names. Both are represented here so that the reader can
/// report which it found rather than guessing a conversion.
struct AletheiaPlan
{
  /// Set when the file held a policy tree. Null with `linear` empty means the
  /// planner reported an empty plan, which is what it writes when the goal
  /// already held.
  std::shared_ptr<PlanNode> tree;

  /// Set when the file held a flat array of grounded action names.
  std::vector<std::string> linear;

  bool is_tree{false};
};

/**
 * @brief Read a plan file written by the Aletheia planner.
 *
 * Free function rather than a member because it involves no ROS: it is the
 * one piece of this plugin that can be tested without a lifecycle node, and
 * it is where a change in the planner's output format would first show.
 *
 * The tree form is an object with an `action` string and a `branches` array,
 * each branch carrying an integer `event` and a `subtree` that is either
 * another such object or null. The linear form is an array of strings. A file
 * holding the bare token `null` is an empty plan.
 *
 * @param[in] path Path to the plan file.
 * @param[out] error What was wrong, when the result is nullopt.
 * @return The plan, or nullopt when the file is missing or malformed.
 */
std::optional<AletheiaPlan> read_plan_file(
  const std::string & path, std::string & error);

/**
 * @class plansys2::AletheiaPlanSolver
 * @brief PlanSolverBase plugin that runs Aletheia as an external process.
 *
 * The counterpart of POPFPlanSolver for epistemic planning, and structurally
 * the same: write the input to a file, run a planner binary under a timeout,
 * read its output back. The difference is what is written and what is read.
 * POPF is handed PDDL and returns a timed sequence; Aletheia is handed a
 * grounded epistemic task in the IePC JSON format and returns a policy tree.
 *
 * plansys2_epistemic_planner performs exactly this search in process, and is
 * the better default: it costs no fork, no serialisation, and no dependency on
 * a binary being installed. This plugin exists for the cases where that is not
 * what is wanted --- a planner built and versioned separately from the
 * workspace, one run under its own resource limits, or one being compared
 * against the in-process build. The two produce the same plan for the same
 * task, and both are selected by naming them in `plan_solver_plugins`, so
 * switching between them is a parameter change.
 *
 * The task and the plan are handled by plansys2_epistemic_planner's own
 * parser, validator and policy serialisation. This plugin adds the subprocess
 * and the reading of its output, and nothing else: a second implementation of
 * the epistemic-to-PlanSys2 translation would be a second thing to keep
 * correct.
 *
 * Parameters, all prefixed with the plugin name. The last six are the same as
 * the in-process plugin's, so a parameters file can move between them:
 *
 *   command          The planner binary. Default "epistemic_planner", found on
 *                    PATH. Give an absolute path when it is not installed
 *                    there.
 *   arguments        Extra arguments appended verbatim to the command line.
 *   output_dir       Where the task, plan and planner log are written.
 *                    Defaults to the system temporary directory, and a leading
 *                    `~` is expanded.
 *   task_file        Path to a grounded epistemic task JSON. Used when the
 *                    `problem` string is not itself epistemic JSON.
 *   heuristic        ug | ed | ks | wc | rpg | radd. Empty leaves the choice
 *                    to the planner's own selection policy.
 *   strategy         gbfs | ehc | aostar. Empty as above.
 *   policy_file      Selection-policy JSON overriding the planner's built-in
 *                    rules.
 *   action_mapping   Path to a JSON map from grounded epistemic action names
 *                    to PlanSys2 action expressions. Empty falls back to the
 *                    naming convention, which is not suitable for dispatching
 *                    to real actions.
 *   conditional_plan policy | flatten | reject, as in the in-process plugin.
 */
class AletheiaPlanSolver : public PlanSolverBase
{
public:
  AletheiaPlanSolver();

  void configure(
    rclcpp_lifecycle::LifecycleNode::SharedPtr lc_node,
    const std::string & plugin_name) override;

  std::optional<plansys2_msgs::msg::Plan> getPlan(
    const std::string & domain, const std::string & problem,
    const std::string & node_namespace = "",
    const rclcpp::Duration solver_timeout = std::chrono::seconds(15)) override;

  /// True unconditionally. The epistemic task is self-contained and carries no
  /// PDDL domain to validate; reporting false would stop the planner node from
  /// coming up, and the task itself is checked when it is loaded.
  bool isDomainValid(
    const std::string & domain, const std::string & node_namespace = "") override;

  /// Directory the task, plan and log are written to, creating it if needed.
  std::optional<std::filesystem::path> create_folders(const std::string & node_namespace);

protected:
  /// The command line, assembled from the parameters. Paths are quoted, since
  /// PlanSolverBase tokenises with std::quoted.
  std::string build_command(
    const std::filesystem::path & task_path,
    const std::filesystem::path & plan_path,
    const rclcpp::Duration & solver_timeout) const;

private:
  /// Resolve the grounded task to a path the planner can read: the `problem`
  /// string written out when it is epistemic JSON, otherwise the configured
  /// task file. Nullopt with `error` set on failure.
  std::optional<std::filesystem::path> resolve_task_path(
    const std::string & problem, const std::filesystem::path & output_dir,
    std::string & error) const;

  std::string parameter(const std::string & name) const;

  std::string command_parameter_name_;
  std::string arguments_parameter_name_;
  std::string output_dir_parameter_name_;
  std::string task_file_parameter_name_;
  std::string heuristic_parameter_name_;
  std::string strategy_parameter_name_;
  std::string policy_file_parameter_name_;
  std::string action_mapping_parameter_name_;
  std::string conditional_parameter_name_;
};

}  // namespace plansys2

#endif  // PLANSYS2_ALETHEIA_PLAN_SOLVER__ALETHEIA_PLAN_SOLVER_HPP_
