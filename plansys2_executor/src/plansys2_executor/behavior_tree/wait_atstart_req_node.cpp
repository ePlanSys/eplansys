// Copyright 2020 Intelligent Robotics Lab
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

#include <string>
#include <map>
#include <memory>
#include <tuple>

#include "plansys2_executor/behavior_tree/wait_atstart_req_node.hpp"
#include "plansys2_msgs/msg/tree.hpp"

namespace plansys2
{

WaitAtStartReq::WaitAtStartReq(
  const std::string & xml_tag_name,
  const BT::NodeConfig & conf)
: ActionNodeBase(xml_tag_name, conf)
{
  action_map_ =
    config().blackboard->get<std::shared_ptr<std::map<std::string, ActionExecutionInfo>>>(
    "action_map");

  problem_client_ =
    config().blackboard->get<std::shared_ptr<plansys2::ProblemExpertClient>>(
    "problem_client");
}

BT::NodeStatus
WaitAtStartReq::tick()
{
  std::string action;
  getInput("action", action);

  if (action_map_ != nullptr && (*action_map_)[action].action_executor != nullptr &&
    (*action_map_)[action].action_executor->get_internal_status() == ActionExecutor::RUNNING)
  {
    return BT::NodeStatus::SUCCESS;
  }

  auto node = config().blackboard->get<rclcpp_lifecycle::LifecycleNode::SharedPtr>("node");

  auto reqs_as = (*action_map_)[action].action_info.get_at_start_requirements();
  auto reqs_oa = (*action_map_)[action].action_info.get_overall_requirements();

  bool check_as = check(reqs_as, problem_client_);
  if (!check_as) {
    (*action_map_)[action].execution_error_info = "Error checking at start reqs";

    RCLCPP_ERROR_STREAM(
      node->get_logger(),
      "[" << action << "]" << (*action_map_)[action].execution_error_info << ": " <<
        parser::pddl::toString(reqs_as));

    return keep_waiting(node, action);
  }

  bool check_oa = check(reqs_oa, problem_client_);
  if (!check_oa) {
    (*action_map_)[action].execution_error_info = "Error checking over all reqs";

    RCLCPP_ERROR_STREAM(
      node->get_logger(),
      "[" << action << "]" << (*action_map_)[action].execution_error_info << ": " <<
        parser::pddl::toString(reqs_oa));

    return keep_waiting(node, action);
  }

  waiting_ = false;
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus
WaitAtStartReq::keep_waiting(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & action)
{
  // In a concurrent plan another action may yet bring the requirements about,
  // which is why PlanSys2 waits. In a sequential policy nothing else is
  // running, and the wait never ends: the mission sits at its time limit with
  // nothing reported. A positive at_start_req_timeout bounds it, and the
  // action fails, so the failure reaches the client and can be replanned.
  // Zero, the default, keeps the unbounded wait.
  if (!waiting_) {
    waiting_ = true;
    waiting_since_ = node->now();
  }
  const double limit = node->get_parameter("at_start_req_timeout").as_double();
  if (limit > 0.0 && (node->now() - waiting_since_).seconds() > limit) {
    waiting_ = false;
    (*action_map_)[action].execution_error_info += " (gave up after the wait limit)";
    RCLCPP_ERROR_STREAM(
      node->get_logger(),
      "[" << action << "] requirements still unmet after " << limit << "s; failing it");
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

}  // namespace plansys2
