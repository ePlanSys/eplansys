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

#ifndef PLANSYS2_EPDDL_GROUNDER__PARAMETERS_HPP_
#define PLANSYS2_EPDDL_GROUNDER__PARAMETERS_HPP_

#include <string>
#include <vector>

#include "plansys2_epddl_grounder/epddl_grounder.hpp"

namespace plansys2
{

/// The parameter names every node that grounds EPDDL declares, under an
/// optional prefix. A plan solver plugin prefixes them with its plugin name,
/// the way its other parameters are prefixed; a node passes no prefix.
///
/// Both halves of the epistemic system — the planner that searches for a
/// policy and the epistemic state that tracks what the agents know — need the
/// *same* task, and the point of naming EPDDL sources rather than a grounded
/// file is that they can be pointed at one set of sources instead of at a
/// JSON that had to be produced and copied by hand.
struct EpddlParameterNames
{
  explicit EpddlParameterNames(const std::string & prefix = "")
  : domain(qualify(prefix, "epddl_domain")),
    problem(qualify(prefix, "epddl_problem")),
    libraries(qualify(prefix, "epddl_libraries")),
    plank_command(qualify(prefix, "plank_command"))
  {
  }

  std::string domain;
  std::string problem;
  std::string libraries;
  std::string plank_command;

private:
  static std::string qualify(const std::string & prefix, const std::string & name)
  {
    return prefix.empty() ? name : prefix + "." + name;
  }
};

/// Declare the four parameters on @p node, leaving any already declared alone.
/// Templated on the node type because a plan solver holds a LifecycleNode and
/// nothing here needs more than declare_parameter.
template<typename NodeT>
void declare_epddl_parameters(NodeT node, const EpddlParameterNames & names)
{
  if (!node->has_parameter(names.domain)) {
    node->template declare_parameter<std::string>(names.domain, "");
  }
  if (!node->has_parameter(names.problem)) {
    node->template declare_parameter<std::string>(names.problem, "");
  }
  if (!node->has_parameter(names.libraries)) {
    // Empty: the packaged `intermediate` library, which is the one every
    // domain in this repository declares.
    node->template declare_parameter<std::vector<std::string>>(
      names.libraries, std::vector<std::string>{});
  }
  if (!node->has_parameter(names.plank_command)) {
    // Empty: the PLANK environment variable, then `plank` on PATH.
    node->template declare_parameter<std::string>(names.plank_command, "");
  }
}

/// Read the EPDDL sources back off @p node. An unset parameter reads as empty,
/// which EpddlSpec::empty() reports as "no EPDDL source configured".
template<typename NodeT>
EpddlSpec read_epddl_spec(NodeT node, const EpddlParameterNames & names)
{
  EpddlSpec spec;
  if (node->has_parameter(names.domain)) {
    spec.domain = node->get_parameter(names.domain).as_string();
  }
  if (node->has_parameter(names.problem)) {
    spec.problem = node->get_parameter(names.problem).as_string();
  }
  if (node->has_parameter(names.libraries)) {
    spec.libraries = node->get_parameter(names.libraries).as_string_array();
  }
  return spec;
}

template<typename NodeT>
std::string read_plank_command(NodeT node, const EpddlParameterNames & names)
{
  return node->has_parameter(names.plank_command) ?
         node->get_parameter(names.plank_command).as_string() :
         std::string();
}

}  // namespace plansys2

#endif  // PLANSYS2_EPDDL_GROUNDER__PARAMETERS_HPP_
