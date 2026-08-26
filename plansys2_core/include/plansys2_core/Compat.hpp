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

#ifndef PLANSYS2_CORE__COMPAT_HPP_
#define PLANSYS2_CORE__COMPAT_HPP_

// The two places where this fork's sources, which follow rolling, do not
// compile against Humble.
//
// The fork is maintained against rolling, but the epistemic packages are meant
// to be usable on a released distribution, and the CI builds part of the tree
// on Humble for that reason. Both differences below are spelled as a question
// about the API rather than about the distribution's name, so that a future
// distribution that changes back, or a backport, is handled without editing
// this file.

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/version.h>

#if __has_include(<rclcpp/experimental/executors/events_executor/events_executor.hpp>)
#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>
#define PLANSYS2_HAS_EVENTS_EXECUTOR 1
#else
#include <rclcpp/executors/single_threaded_executor.hpp>
#define PLANSYS2_HAS_EVENTS_EXECUTOR 0
#endif

namespace plansys2
{

// EventsExecutor is the better choice where it exists: bringing a system up is
// a burst of service calls and transition notifications, which is the traffic
// a wait-set executor handles worst. It arrived with Iron, and on Humble the
// header is absent entirely -- so the declaration itself fails to parse, and
// every later use of the variable is reported as undeclared, which is four
// confusing errors from one missing include.
#if PLANSYS2_HAS_EVENTS_EXECUTOR
using SpinExecutor = rclcpp::experimental::executors::EventsExecutor;
#else
using SpinExecutor = rclcpp::executors::SingleThreadedExecutor;
#endif

/// The QoS to create a service with.
///
/// Newer rclcpp takes an rclcpp::QoS here. Humble's create_service still takes
/// the rmw profile, and rclcpp::ServicesQoS does not convert to it, so the
/// call site cannot be written once for both. Humble is rclcpp 16 and is the
/// only distribution this fork targets that still wants the profile; anything
/// newer gets the QoS object the sources were written with.
#if RCLCPP_VERSION_GTE(17, 0, 0)
inline rclcpp::QoS service_qos() {return rclcpp::ServicesQoS();}
#else
inline const rmw_qos_profile_t & service_qos() {return rmw_qos_profile_services_default;}
#endif

}  // namespace plansys2

#endif  // PLANSYS2_CORE__COMPAT_HPP_
