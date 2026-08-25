// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <algorithm>
#include <chrono>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging_macros.h"

#include "plansys2_lifecycle_manager/lifecycle_manager.hpp"

namespace plansys2
{

LifecycleServiceClient::LifecycleServiceClient(
  const std::string & node_name, const std::string & managed_node)
: Node(node_name), managed_node_(managed_node)
{}

void
LifecycleServiceClient::init()
{
  std::string get_state_service_name = managed_node_ + "/get_state";
  std::string change_state_service_name = managed_node_ + "/change_state";
  RCLCPP_INFO(get_logger(), "Creating client for service [%s]", get_state_service_name.c_str());
  RCLCPP_INFO(
    get_logger(), "Creating client for service [%s]",
    change_state_service_name.c_str());
  client_get_state_ = this->create_client<lifecycle_msgs::srv::GetState>(get_state_service_name);
  client_change_state_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
    change_state_service_name);
}

unsigned int
LifecycleServiceClient::get_state(std::chrono::seconds time_out)
{
  auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
  if (!client_get_state_->wait_for_service(time_out)) {
    RCLCPP_ERROR(
      get_logger(),
      "Service %s is not available.",
      client_get_state_->get_service_name());
    return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
  }
  // We send the service request for asking the current
  // state of the lc_talker node.
  auto future_result = client_get_state_->async_send_request(request);
  // Let's wait until we have the answer from the node.
  // If the request times out, we return an unknown state.
  auto future_status = wait_for_result(future_result, time_out);
  auto state = future_result.get();

  if (future_status != std::future_status::ready) {
    RCLCPP_ERROR(
      get_logger(), "Server time out while getting current state for node %s",
      managed_node_.c_str());
    return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
  }
  // We have an successful answer. So let's print the current state.
  if (state != nullptr) {
    RCLCPP_INFO(
      get_logger(), "Node %s has current state %s.",
      get_name(), state->current_state.label.c_str());
    return state->current_state.id;
  } else {
    RCLCPP_ERROR(
      get_logger(), "Failed to get current state for node %s", managed_node_.c_str());
    return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
  }
}

bool
LifecycleServiceClient::change_state(std::uint8_t transition, std::chrono::seconds time_out)
{
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = transition;
  if (!client_change_state_->wait_for_service(time_out)) {
    RCLCPP_ERROR(
      get_logger(),
      "Service %s is not available.",
      client_change_state_->get_service_name());
    return false;
  }
  // We send the request with the transition we want to invoke.
  auto future_result = client_change_state_->async_send_request(request);
  // Let's wait until we have the answer from the node.
  // If the request times out, we return an unknown state.
  auto future_status = wait_for_result(future_result, time_out);
  if (future_status != std::future_status::ready) {
    RCLCPP_ERROR(
      get_logger(), "Server time out while getting current state for node %s",
      managed_node_.c_str());
    return false;
  }
  // We have an answer, let's print our success.
  if (future_result.get()->success) {
    RCLCPP_INFO(
      get_logger(), "Transition %d successfully triggered.", static_cast<int>(transition));
    return true;
  } else {
    RCLCPP_WARN(
      get_logger(), "Failed to trigger transition %u", static_cast<unsigned int>(transition));
    return false;
  }
}

namespace
{

/// The order the nodes are brought up in, and the order they are activated in.
/// They differ, and both are deliberate: the planner configures first because
/// loading a plan solver plugin is what fails when a plugin is missing, and
/// finding that out before the experts have read any PDDL keeps the failure
/// legible; the domain expert activates first because everything downstream
/// asks it questions.
///
/// Perception goes last in both, and the activate order is the reason. Once it
/// is active it starts reporting what the map says, and every route it has for
/// reporting is a call on the epistemic state; bringing it up before the state
/// it talks to would spend its first observations on a node that cannot answer
/// yet.
///
/// A name listed here that is not in the map is skipped rather than waited
/// for. That is what makes the epistemic nodes optional: a classical bringup
/// hands over four nodes, an epistemic one five or six, and this function does
/// not need to know which of them it is being used for.
const std::vector<std::string> & configure_order()
{
  static const std::vector<std::string> order{
    "planner", "domain_expert", "problem_expert", "executor", "epistemic_state",
    "epistemic_perception"};
  return order;
}

const std::vector<std::string> & activate_order()
{
  static const std::vector<std::string> order{
    "domain_expert", "problem_expert", "planner", "executor", "epistemic_state",
    "epistemic_perception"};
  return order;
}

/// The listed names that are present, followed by any name the caller supplied
/// that the lists do not mention. An unknown node is still managed — being
/// unrecognised is no reason to leave it unconfigured — it simply goes last.
std::vector<std::string> ordered_nodes(
  const std::map<std::string, std::shared_ptr<LifecycleServiceClient>> & manager_nodes,
  const std::vector<std::string> & order)
{
  std::vector<std::string> result;
  result.reserve(manager_nodes.size());

  for (const auto & name : order) {
    if (manager_nodes.find(name) != manager_nodes.end()) {
      result.push_back(name);
    }
  }
  for (const auto & [name, client] : manager_nodes) {
    (void)client;
    if (std::find(order.begin(), order.end(), name) == order.end()) {
      result.push_back(name);
    }
  }
  return result;
}

}  // namespace

bool
startup_function(
  std::map<std::string, std::shared_ptr<LifecycleServiceClient>> & manager_nodes,
  std::chrono::seconds timeout)
{
  for (const auto & name : ordered_nodes(manager_nodes, configure_order())) {
    if (!manager_nodes[name]->change_state(
        lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE,
        timeout))
    {
      return false;
    }

    // Ctrl-C during this wait used to hang: the loop had no way out but the
    // node reaching INACTIVE, so a node that never configures held the whole
    // bringup. Checking rclcpp::ok() lets shutdown end it.
    while (rclcpp::ok() &&
      manager_nodes[name]->get_state() !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
      std::cerr << "Waiting for inactive state for " << name << std::endl;
    }
    if (!rclcpp::ok()) {
      return false;
    }
  }

  for (const auto & name : ordered_nodes(manager_nodes, activate_order())) {
    if (!rclcpp::ok()) {
      return false;
    }
    if (!manager_nodes[name]->change_state(
        lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE,
        timeout))
    {
      return false;
    }
  }

  for (const auto & [name, client] : manager_nodes) {
    (void)name;
    if (!client->get_state()) {
      return false;
    }
  }

  return true;
}

}  // namespace plansys2
