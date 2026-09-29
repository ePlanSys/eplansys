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

#ifndef LEAVE_HPP_
#define LEAVE_HPP_

#include <unistd.h>

#include <cstdio>

#include "rclcpp/rclcpp.hpp"

namespace eplansys_demo
{

/// End the process once its work is done, without destroying the performers.
///
/// A performer interrupted by the launch file at the end of a mission
/// segfaulted in about one run in five, after every action had finished. The
/// fault is in the performer's destructor, run from main after
/// rclcpp::shutdown(), while CascadeLifecycleNode frees its activators_state_
/// map through a pointer that is no longer valid; no thread of this process was
/// using the node by then. Nothing is left to do at that point, so the process
/// leaves through _exit and skips the teardown, as plansys2_tests does for its
/// own exit-time fault.
[[noreturn]] inline void leave(int status)
{
  rclcpp::shutdown();
  std::fflush(stdout);
  std::fflush(stderr);
  _exit(status);
}

}  // namespace eplansys_demo

#endif  // LEAVE_HPP_
