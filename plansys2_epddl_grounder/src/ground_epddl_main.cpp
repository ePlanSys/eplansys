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

// Grounding by hand, for the times when what is wanted is the task file
// itself: to inspect it, to check it into a test, or to pass it to a node as
// a task_file. The planner and the epistemic state ground their own sources
// and never call this.

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include "plansys2_epddl_grounder/epddl_grounder.hpp"

namespace
{

int usage(const std::string & program, const std::string & error)
{
  if (!error.empty()) {
    std::cerr << program << ": " << error << "\n\n";
  }
  std::cerr
    << "usage: " << program << " -d <domain.epddl> -p <problem.epddl>"
    << " [-l <library.epddl> ...] [-o <task.json>]\n\n"
    << "Grounds an EPDDL specification into the task JSON the epistemic\n"
    << "planner reads, writing it to <task.json> or to standard output.\n\n"
    << "With no -l, the `intermediate` action-type library shipped with this\n"
    << "package is used, which is the one the example domains declare.\n";
  return error.empty() ? 0 : 2;
}

}  // namespace

int main(int argc, char ** argv)
{
  const std::string program = argc > 0 ? argv[0] : "ground_epddl";

  plansys2::EpddlSpec spec;
  std::string output_file;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    const auto value = [&]() -> std::string {
        return i + 1 < argc ? argv[++i] : std::string();
      };

    if (arg == "-h" || arg == "--help") {
      return usage(program, "");
    } else if (arg == "-d") {
      spec.domain = value();
    } else if (arg == "-p") {
      spec.problem = value();
    } else if (arg == "-o") {
      output_file = value();
    } else if (arg == "-l") {
      // Like plank's own -l: every path after the flag, until the next flag.
      while (i + 1 < argc && argv[i + 1][0] != '-') {
        spec.libraries.push_back(argv[++i]);
      }
    } else {
      return usage(program, "unknown argument '" + arg + "'");
    }
  }

  if (!spec.complete()) {
    return usage(program, "both -d and -p are required");
  }

  plansys2::EpddlGrounder grounder;
  const auto result = grounder.ground(spec);

  if (!result.ok) {
    std::cerr << program << ": " << result.error << "\n";
    return 1;
  }

  if (output_file.empty()) {
    std::cout << result.task_json;
    return 0;
  }

  std::ofstream out(output_file);
  if (!out) {
    std::cerr << program << ": could not write " << output_file << "\n";
    return 1;
  }
  out << result.task_json;
  std::cerr << "wrote " << output_file << "\n";
  return 0;
}
