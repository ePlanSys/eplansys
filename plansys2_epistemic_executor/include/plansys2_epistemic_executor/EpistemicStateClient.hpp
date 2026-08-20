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

#ifndef PLANSYS2_EPISTEMIC_EXECUTOR__EPISTEMICSTATECLIENT_HPP_
#define PLANSYS2_EPISTEMIC_EXECUTOR__EPISTEMICSTATECLIENT_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "plansys2_epistemic_msgs/srv/apply_action.hpp"
#include "plansys2_epistemic_msgs/srv/check_formula.hpp"
#include "plansys2_epistemic_msgs/srv/load_task.hpp"
#include "rclcpp/rclcpp.hpp"

namespace plansys2
{

/**
 * @class plansys2::EpistemicStateClient
 * @brief Talks to the epistemic state, the way ProblemExpertClient talks to
 *        the problem expert.
 *
 * Behavior tree nodes tick in the executor's thread and cannot block for long,
 * so every call here takes a timeout and reports what happened rather than
 * waiting indefinitely. A call that times out is reported as a failure with
 * the reason, not as a false answer: a knowledge guard that silently answers
 * "no" when the state is unreachable would look exactly like one whose
 * condition does not hold, and the two call for opposite responses.
 */
class EpistemicStateClient
{
public:
  using Ptr = std::shared_ptr<EpistemicStateClient>;

  /// The outcome of a call: whether it was answered at all, and what it said.
  struct Answer
  {
    bool answered{false};   ///< the service replied
    bool success{false};    ///< the reply was not itself an error
    std::string error;
    std::string outcome;    ///< apply_action: the observation that occurred
    bool holds{false};      ///< check_formula: whether the formula holds
  };

  explicit EpistemicStateClient(const std::string & node_name = "epistemic_state_client");

  /// Load a task by path into the state.
  Answer load_task_file(
    const std::string & path,
    const std::chrono::nanoseconds & timeout = std::chrono::seconds(5));

  /// Does this epistemic formula hold now?
  Answer check_formula(
    const std::string & formula,
    const std::chrono::nanoseconds & timeout = std::chrono::seconds(2));

  /// Advance the state by an executed action. `observed_outcome` may be empty,
  /// in which case the state decides when it can and reports an error when it
  /// cannot.
  Answer apply_action(
    const std::string & epistemic_action,
    const std::string & observed_outcome = "",
    const std::chrono::nanoseconds & timeout = std::chrono::seconds(5));

  /// True when the state node is up. Worth asking once before a plan starts,
  /// rather than discovering it mid-execution.
  bool available(const std::chrono::nanoseconds & timeout = std::chrono::seconds(1));

private:
  template<typename ServiceT, typename RequestT>
  typename ServiceT::Response::SharedPtr call(
    const typename rclcpp::Client<ServiceT>::SharedPtr & client,
    const RequestT & request,
    const std::chrono::nanoseconds & timeout,
    std::string & error);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<plansys2_epistemic_msgs::srv::LoadTask>::SharedPtr load_task_client_;
  rclcpp::Client<plansys2_epistemic_msgs::srv::CheckFormula>::SharedPtr check_formula_client_;
  rclcpp::Client<plansys2_epistemic_msgs::srv::ApplyAction>::SharedPtr apply_action_client_;
};

}  // namespace plansys2

#endif  // PLANSYS2_EPISTEMIC_EXECUTOR__EPISTEMICSTATECLIENT_HPP_
