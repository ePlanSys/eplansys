// Copyright 2026 Haniel Vásquez Morales
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

#ifndef PLANSYS2_LIFECYCLE_MANAGER__EXECUTOR_HPP_
#define PLANSYS2_LIFECYCLE_MANAGER__EXECUTOR_HPP_

// The executor the lifecycle manager spins its clients on.
//
// EventsExecutor is what upstream changed to, and it is the better choice
// here: bringing a system up is a burst of service calls and transition
// notifications, which is the traffic a wait-set executor handles worst. It
// arrived in rclcpp with Iron, though, and this repository also builds on
// Humble, where the header does not exist at all -- the failure is not a
// missing symbol at link time but four "'exe' was not declared" errors, since
// the declaration itself is what fails to parse.
//
// So the header decides, rather than the distribution name: a distribution
// that has the events executor gets it, and one that does not falls back to
// the single-threaded executor, which spins the same nodes correctly and only
// less efficiently.

#if __has_include(<rclcpp/experimental/executors/events_executor/events_executor.hpp>)
#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>
#define PLANSYS2_HAS_EVENTS_EXECUTOR 1
#else
#include <rclcpp/executors/single_threaded_executor.hpp>
#define PLANSYS2_HAS_EVENTS_EXECUTOR 0
#endif

namespace plansys2
{

#if PLANSYS2_HAS_EVENTS_EXECUTOR
using ManagerExecutor = rclcpp::experimental::executors::EventsExecutor;
#else
using ManagerExecutor = rclcpp::executors::SingleThreadedExecutor;
#endif

}  // namespace plansys2

#endif  // PLANSYS2_LIFECYCLE_MANAGER__EXECUTOR_HPP_
