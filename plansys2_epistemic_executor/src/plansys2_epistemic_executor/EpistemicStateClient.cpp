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

#include "plansys2_epistemic_executor/EpistemicStateClient.hpp"

#include <memory>
#include <string>

namespace plansys2
{

EpistemicStateClient::EpistemicStateClient(const std::string & node_name)
{
  node_ = rclcpp::Node::make_shared(node_name);

  load_task_client_ = node_->create_client<plansys2_epistemic_msgs::srv::LoadTask>(
    "epistemic_state/load_task");
  check_formula_client_ = node_->create_client<plansys2_epistemic_msgs::srv::CheckFormula>(
    "epistemic_state/check_formula");
  apply_action_client_ = node_->create_client<plansys2_epistemic_msgs::srv::ApplyAction>(
    "epistemic_state/apply_action");
}

template<typename ServiceT, typename RequestT>
typename ServiceT::Response::SharedPtr EpistemicStateClient::call(
  const typename rclcpp::Client<ServiceT>::SharedPtr & client,
  const RequestT & request,
  const std::chrono::nanoseconds & timeout,
  std::string & error)
{
  if (!client->wait_for_service(timeout)) {
    error = std::string(client->get_service_name()) + " is not available";
    return nullptr;
  }

  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node_, future, timeout) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    error = std::string(client->get_service_name()) + " did not answer in time";
    return nullptr;
  }
  return future.get();
}

EpistemicStateClient::Answer EpistemicStateClient::load_task_file(
  const std::string & path, const std::chrono::nanoseconds & timeout)
{
  Answer answer;
  auto request = std::make_shared<plansys2_epistemic_msgs::srv::LoadTask::Request>();
  request->task_file = path;

  const auto response = call<plansys2_epistemic_msgs::srv::LoadTask>(
    load_task_client_, request, timeout, answer.error);
  if (!response) {
    return answer;
  }

  answer.answered = true;
  answer.success = response->success;
  answer.error = response->error;
  return answer;
}

EpistemicStateClient::Answer EpistemicStateClient::check_formula(
  const std::string & formula, const std::chrono::nanoseconds & timeout)
{
  Answer answer;
  auto request = std::make_shared<plansys2_epistemic_msgs::srv::CheckFormula::Request>();
  request->formula = formula;

  const auto response = call<plansys2_epistemic_msgs::srv::CheckFormula>(
    check_formula_client_, request, timeout, answer.error);
  if (!response) {
    return answer;
  }

  answer.answered = true;
  answer.success = response->success;
  answer.holds = response->holds;
  answer.error = response->error;
  return answer;
}

EpistemicStateClient::Answer EpistemicStateClient::apply_action(
  const std::string & epistemic_action,
  const std::string & observed_outcome,
  const std::chrono::nanoseconds & timeout)
{
  Answer answer;
  auto request = std::make_shared<plansys2_epistemic_msgs::srv::ApplyAction::Request>();
  request->epistemic_action = epistemic_action;
  request->observed_outcome = observed_outcome;

  const auto response = call<plansys2_epistemic_msgs::srv::ApplyAction>(
    apply_action_client_, request, timeout, answer.error);
  if (!response) {
    return answer;
  }

  answer.answered = true;
  answer.success = response->success;
  answer.outcome = response->outcome;
  answer.error = response->error;
  return answer;
}

bool EpistemicStateClient::available(const std::chrono::nanoseconds & timeout)
{
  return check_formula_client_->wait_for_service(timeout);
}

}  // namespace plansys2
