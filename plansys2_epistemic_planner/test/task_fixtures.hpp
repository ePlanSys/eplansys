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

#ifndef TASK_FIXTURES_HPP_
#define TASK_FIXTURES_HPP_

#include <string>

// The fixtures are addressed through a compile definition rather than
// ament_index_cpp: these tests never need the installed share directory, and
// get_package_share_directory.hpp is one of the headers rolling dropped, so
// keeping the tests clear of it means one less distro difference to carry.
#ifndef EPISTEMIC_TEST_TASK_DIR
#error "EPISTEMIC_TEST_TASK_DIR must be defined by the build"
#endif

inline std::string task_path(const std::string & name)
{
  return std::string(EPISTEMIC_TEST_TASK_DIR) + "/" + name + ".json";
}

#ifdef EPISTEMIC_EXAMPLE_MAPPING_DIR
/// The action mappings shipped in examples/mappings, tested against the tasks
/// they translate so the two cannot drift apart unnoticed.
inline std::string mapping_path(const std::string & name)
{
  return std::string(EPISTEMIC_EXAMPLE_MAPPING_DIR) + "/" + name + ".json";
}
#endif

#endif  // TASK_FIXTURES_HPP_
