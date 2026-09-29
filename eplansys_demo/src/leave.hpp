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

#ifndef EPLANSYS_DEMO__LEAVE_HPP_
#define EPLANSYS_DEMO__LEAVE_HPP_

#include <unistd.h>

#include <cstdio>

#include "rclcpp/rclcpp.hpp"

namespace eplansys_demo
{

/// End the process once its work is done, without static destruction.
///
/// rclcpp::shutdown() does not finalise the global context; that happens in
/// static destruction, inside _dl_fini, while Fast DDS listener threads are
/// still running in libraries the loader is unmapping. A performer interrupted
/// by the launch file at the end of a mission segfaulted in about one run in
/// five that way, after every action had finished. plansys2_tests leaves
/// through _exit for the same reason.
[[noreturn]] inline void leave(int status)
{
  rclcpp::shutdown();
  std::fflush(stdout);
  std::fflush(stderr);
  _exit(status);
}

}  // namespace eplansys_demo

#endif  // EPLANSYS_DEMO__LEAVE_HPP_
