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

#ifndef PLANSYS2_EPDDL_GROUNDER__EPDDL_GROUNDER_HPP_
#define PLANSYS2_EPDDL_GROUNDER__EPDDL_GROUNDER_HPP_

#include <string>
#include <vector>

namespace plansys2
{

/// The three files an EPDDL specification is made of. Libraries are the
/// action-type libraries the domain names in its `:action-type-libraries`
/// clause; the grounder supplies the packaged `intermediate` library when
/// none are given, which is the one every domain in this repository uses.
struct EpddlSpec
{
  std::string domain;
  std::string problem;
  std::vector<std::string> libraries;

  /// Neither file named: no EPDDL source is configured at all.
  bool empty() const {return domain.empty() && problem.empty();}

  /// Both files named. A spec that is neither empty nor complete is a
  /// configuration mistake worth reporting rather than guessing at.
  bool complete() const {return !domain.empty() && !problem.empty();}
};

/// A grounded task, or the reason there is not one.
struct GroundResult
{
  bool ok = false;
  std::string task_json;   ///< contents, not a path, when ok
  std::string error;       ///< plank's own diagnostics when not
};

/**
 * @class plansys2::EpddlGrounder
 * @brief Turns EPDDL sources into the grounded task JSON the planner reads.
 *
 * Grounding EPDDL means building the initial Kripke model from a finitary S5
 * theory and instantiating every event model over the object universe. That
 * is `plank`'s job, not this package's: `plank export` is invoked as a
 * subprocess and its output is read back, the same arrangement
 * plansys2_popf_plan_solver has with POPF.
 *
 * The binary is located, in order: the command passed to the constructor, the
 * PLANK environment variable, then `plank` on PATH. Building plank in the
 * workspace (it is listed in dependency_repos.repos) puts it on PATH once the
 * workspace is sourced, so the default case needs no configuration.
 *
 * Grounding the same unchanged sources twice returns the cached result: the
 * planner asks for a task on every get_plan, and re-running a subprocess to
 * reproduce a file that has not changed would put a fork on the planning path.
 */
class EpddlGrounder
{
public:
  /// @param command Path to, or name of, the plank binary. Empty selects the
  ///   PLANK environment variable, then PATH.
  explicit EpddlGrounder(const std::string & command = "");

  /// Ground @p spec, or explain why it could not be ground.
  GroundResult ground(const EpddlSpec & spec);

  /// The resolved binary, or empty when none was found. Resolution happens on
  /// first use, so this is empty until ground() has been called once.
  const std::string & command() const {return resolved_command_;}

  /// Absolute path to the `intermediate` action-type library shipped with this
  /// package, or empty when the package share directory cannot be found.
  static std::string packaged_library();

private:
  std::string requested_command_;
  std::string resolved_command_;

  std::string cache_key_;
  std::string cache_json_;
};

}  // namespace plansys2

#endif  // PLANSYS2_EPDDL_GROUNDER__EPDDL_GROUNDER_HPP_
