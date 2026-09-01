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

// Drafting the action mapping, so that only the modelling decision is left.

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "plansys2_epistemic_planner/mapping_draft.hpp"
#include "plansys2_epistemic_planner/parser.hpp"

namespace
{

int usage(const std::string & program, const std::string & error)
{
  if (!error.empty()) {
    std::cerr << program << ": " << error << "\n\n";
  }
  std::cerr
    << "usage: " << program << " -t <task.json> [-e <domain.epddl>]"
    << " [-p <domain.pddl>] [-o <mapping.json>] [--duration <seconds>]\n\n"
    << "Drafts the action mapping for a grounded epistemic task: one entry\n"
    << "per grounded action, ready to be finished by hand.\n\n"
    << "  -t  the grounded task, which supplies the action names\n"
    << "  -e  the EPDDL domain, whose declared schemas say where each\n"
    << "      grounded name stops and its arguments begin\n"
    << "  -p  the PDDL domain, against which names and arity are resolved\n"
    << "  -o  where to write; standard output by default\n\n"
    << "Which PDDL action corresponds to which EPDDL schema is a modelling\n"
    << "decision and is not derivable. Entries needing one are written with a\n"
    << "\"_check\" note saying so, and listed on standard error.\n";
  return error.empty() ? 0 : 2;
}

}  // namespace

int main(int argc, char ** argv)
{
  const std::string program = argc > 0 ? argv[0] : "draft_epistemic_mapping";

  std::string task_file, epddl_domain, pddl_domain, output_file;
  double duration = 1.0;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    const auto value = [&]() -> std::string {
        return i + 1 < argc ? argv[++i] : std::string();
      };

    if (arg == "-h" || arg == "--help") {
      return usage(program, "");
    } else if (arg == "-t") {
      task_file = value();
    } else if (arg == "-e") {
      epddl_domain = value();
    } else if (arg == "-p") {
      pddl_domain = value();
    } else if (arg == "-o") {
      output_file = value();
    } else if (arg == "--duration") {
      try {
        duration = std::stod(value());
      } catch (const std::exception &) {
        return usage(program, "--duration takes a number of seconds");
      }
    } else {
      return usage(program, "unrecognised argument '" + arg + "'");
    }
  }

  if (task_file.empty()) {
    return usage(program, "a grounded task is required (-t)");
  }
  if (duration <= 0.0) {
    return usage(program, "a duration has to be positive");
  }

  PlanningTask task;
  try {
    task = load_task(task_file);
  } catch (const std::exception & e) {
    std::cerr << program << ": could not read " << task_file << ": " << e.what() << "\n";
    return 1;
  }

  const auto schemas =
    epddl_domain.empty() ?
    std::vector<std::string>{} : plansys2::read_epddl_schemas(epddl_domain);
  const auto actions =
    pddl_domain.empty() ?
    std::vector<std::pair<std::string, std::size_t>>{} :
  plansys2::read_pddl_actions(pddl_domain);

  if (!epddl_domain.empty() && schemas.empty()) {
    std::cerr << program << ": " << epddl_domain << " declares no actions\n";
  }
  if (!pddl_domain.empty() && actions.empty()) {
    std::cerr << program << ": " << pddl_domain << " declares no actions\n";
  }

  const auto draft = plansys2::draft_mapping(task, schemas, actions);
  const auto text = draft.to_json(duration);

  if (output_file.empty()) {
    std::cout << text << "\n";
  } else {
    std::ofstream out(output_file);
    if (!out) {
      std::cerr << program << ": could not write " << output_file << "\n";
      return 1;
    }
    out << text << "\n";
  }

  const auto warnings = draft.warnings();
  for (const auto & warning : warnings) {
    std::cerr << program << ": " << warning << "\n";
  }

  if (!warnings.empty()) {
    std::cerr
      << program << ": " << warnings.size() << " of " << draft.entries.size()
      << " entries need a decision before this maps to real actions\n";
  }

  // A draft with open entries is still a useful file, so this is not a
  // failure. The exit status says whether anything is left to do, which is
  // what a script wants to branch on.
  return draft.complete() ? 0 : 3;
}
