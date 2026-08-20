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

#include "plansys2_epistemic_executor/behavior_tree/epistemic_nodes.hpp"

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace plansys2
{

namespace
{

rclcpp::Logger logger()
{
  return rclcpp::get_logger("epistemic_bt");
}

/// Fetch a blackboard entry, or null when the host did not set it. Reporting
/// that as a node failure beats throwing out of a tick, which would take down
/// the executor for a configuration mistake.
template<typename T>
std::shared_ptr<T> from_blackboard(const BT::NodeConfig & conf, const char * key)
{
  std::shared_ptr<T> value;
  try {
    if (!conf.blackboard->get(key, value)) {
      return nullptr;
    }
  } catch (const std::exception &) {
    return nullptr;   // present under a different type: not ours
  }
  return value;
}

/// The policy these nodes are executing.
///
/// Taken from the blackboard when a builder put it there, and otherwise built
/// from the plan the executor already publishes on the blackboard. The second
/// path is what lets these nodes work in a stock PlanSys2 executor, which
/// knows nothing about policies but does know its own plan.
std::shared_ptr<Policy> policy_from(const BT::NodeConfig & conf)
{
  if (auto shared = from_blackboard<Policy>(conf, kEpistemicPolicyKey)) {
    return shared;
  }

  plansys2_msgs::msg::Plan plan;
  try {
    if (!conf.blackboard->get(kCurrentPlanKey, plan)) {
      return nullptr;
    }
  } catch (const std::exception &) {
    return nullptr;
  }

  const auto problem = Policy::validate(plan);
  if (!problem.empty()) {
    RCLCPP_ERROR(logger(), "[epistemic] the plan is not a policy: %s", problem.c_str());
    return nullptr;
  }
  return std::make_shared<Policy>(plan);
}

/// The client these nodes ask. One per tree: they are created as the tree is,
/// and a client owns a node and its service clients, which is not something to
/// build four times over.
EpistemicStateClient::Ptr client_from(const BT::NodeConfig & conf)
{
  if (auto shared = from_blackboard<EpistemicStateClient>(conf, kEpistemicClientKey)) {
    return shared;
  }

  auto created = std::make_shared<EpistemicStateClient>();
  conf.blackboard->set(kEpistemicClientKey, created);
  return created;
}

std::vector<std::string> split(const std::string & text, char separator)
{
  std::vector<std::string> parts;
  std::size_t start = 0;
  for (std::size_t i = 0; i <= text.size(); ++i) {
    if (i == text.size() || text[i] == separator) {
      parts.push_back(text.substr(start, i - start));
      start = i + 1;
    }
  }
  return parts;
}

/// Policy node index from a port, or nullopt when it is missing or not one.
std::optional<std::uint32_t> node_index(
  const BT::TreeNode & self, const std::shared_ptr<Policy> & policy)
{
  std::string text;
  if (!self.getInput("node", text)) {
    return std::nullopt;
  }
  try {
    const auto index = static_cast<std::uint32_t>(std::stoul(text));
    if (!policy || index >= policy->size()) {
      return std::nullopt;
    }
    return index;
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

}  // namespace

// ── CheckKnowledge ──────────────────────────────────────────────────────────

CheckKnowledge::CheckKnowledge(const std::string & xml_tag_name, const BT::NodeConfig & conf)
: ConditionNode(xml_tag_name, conf)
{
  client_ = client_from(conf);
  policy_ = policy_from(conf);
}

BT::NodeStatus CheckKnowledge::tick()
{
  std::string action;
  getInput("action", action);

  if (!client_ || !policy_) {
    RCLCPP_ERROR(
      logger(), "[%s] no epistemic state on the blackboard; the tree was built "
      "without one", action.c_str());
    return BT::NodeStatus::FAILURE;
  }

  const auto index = node_index(*this, policy_);
  if (!index) {
    RCLCPP_ERROR(logger(), "[%s] this node names no policy node", action.c_str());
    return BT::NodeStatus::FAILURE;
  }

  const auto & requirements = policy_->item(*index).knowledge_requirements;
  for (const auto & requirement : requirements) {
    const auto answer = client_->check_formula(requirement);

    if (!answer.answered || !answer.success) {
      // Not the same as the requirement failing, and reported differently: one
      // says the plan is wrong, the other says we cannot tell.
      RCLCPP_ERROR(
        logger(), "[%s] could not check %s: %s", action.c_str(),
        requirement.c_str(), answer.error.c_str());
      return BT::NodeStatus::FAILURE;
    }

    if (!answer.holds) {
      RCLCPP_ERROR(
        logger(), "[%s] knowledge requirement does not hold: %s",
        action.c_str(), requirement.c_str());
      return BT::NodeStatus::FAILURE;
    }
  }

  return BT::NodeStatus::SUCCESS;
}

// ── ApplyEpistemicUpdate ────────────────────────────────────────────────────

ApplyEpistemicUpdate::ApplyEpistemicUpdate(
  const std::string & xml_tag_name, const BT::NodeConfig & conf)
: ActionNodeBase(xml_tag_name, conf)
{
  client_ = client_from(conf);
  policy_ = policy_from(conf);
}

BT::NodeStatus ApplyEpistemicUpdate::tick()
{
  std::string action;
  getInput("action", action);

  if (!client_ || !policy_) {
    RCLCPP_ERROR(logger(), "[%s] no epistemic state on the blackboard", action.c_str());
    return BT::NodeStatus::FAILURE;
  }

  const auto index = node_index(*this, policy_);
  if (!index) {
    RCLCPP_ERROR(logger(), "[%s] this node names no policy node", action.c_str());
    return BT::NodeStatus::FAILURE;
  }

  const auto & item = policy_->item(*index);

  // The update must happen once per execution of this action. A parent that
  // re-ticks a completed child would otherwise apply it twice, and a product
  // update is not idempotent — applying an announcement twice claims the
  // agents were told twice.
  if (applied_) {
    return BT::NodeStatus::SUCCESS;
  }

  std::string observed;
  getInput("observed", observed);

  const auto answer = client_->apply_action(item.epistemic_action, observed);
  if (!answer.answered || !answer.success) {
    RCLCPP_ERROR(
      logger(), "[%s] the epistemic update failed: %s", action.c_str(),
      answer.error.c_str());
    return BT::NodeStatus::FAILURE;
  }

  setOutput("outcome", answer.outcome);
  applied_ = true;

  if (item.sensing) {
    RCLCPP_INFO(logger(), "[%s] observed %s", action.c_str(), answer.outcome.c_str());
  }
  return BT::NodeStatus::SUCCESS;
}

// ── EpistemicSwitch ─────────────────────────────────────────────────────────

EpistemicSwitch::EpistemicSwitch(const std::string & xml_tag_name, const BT::NodeConfig & conf)
: ControlNode(xml_tag_name, conf)
{
}

void EpistemicSwitch::halt()
{
  running_child_ = -1;
  ControlNode::halt();
}

BT::NodeStatus EpistemicSwitch::tick()
{
  std::string outcome;
  getInput("outcome", outcome);

  std::string outcome_list;
  getInput("outcomes", outcome_list);
  const auto outcomes = split(outcome_list, ';');

  if (outcomes.size() != children_nodes_.size()) {
    RCLCPP_ERROR(
      logger(),
      "[EpistemicSwitch] %zu outcomes but %zu branches; the tree does not match "
      "the policy it was built from", outcomes.size(), children_nodes_.size());
    return BT::NodeStatus::FAILURE;
  }

  int selected = -1;
  for (std::size_t i = 0; i < outcomes.size(); ++i) {
    if (outcomes[i] == outcome) {
      selected = static_cast<int>(i);
      break;
    }
  }

  if (selected < 0) {
    // The world produced something the policy has no continuation for. There
    // is no safe default here: every branch was built for a different belief,
    // and running one anyway is acting confidently on an unsupported one.
    RCLCPP_ERROR(
      logger(),
      "[EpistemicSwitch] observed '%s', which the policy does not plan for "
      "(it plans for: %s). Replanning from the state that actually resulted is "
      "the way forward.",
      outcome.c_str(), outcome_list.c_str());
    return BT::NodeStatus::FAILURE;
  }

  // A branch already running keeps running: re-selecting each tick would be
  // the same choice anyway, and halting a running action to start it again
  // would interrupt the robot mid-action.
  if (running_child_ >= 0 && running_child_ != selected) {
    haltChild(running_child_);
  }
  running_child_ = selected;

  setStatus(BT::NodeStatus::RUNNING);
  const auto status = children_nodes_[selected]->executeTick();

  if (status != BT::NodeStatus::RUNNING) {
    haltChild(selected);
    running_child_ = -1;
  }
  return status;
}

// ── CheckEpistemicGoal ──────────────────────────────────────────────────────

CheckEpistemicGoal::CheckEpistemicGoal(
  const std::string & xml_tag_name, const BT::NodeConfig & conf)
: ConditionNode(xml_tag_name, conf)
{
  client_ = client_from(conf);
}

BT::NodeStatus CheckEpistemicGoal::tick()
{
  std::string goal;
  getInput("goal", goal);

  if (goal.empty()) {
    return BT::NodeStatus::SUCCESS;   // a classical plan: the executor checks its own goal
  }

  if (!client_) {
    RCLCPP_ERROR(logger(), "[goal] no epistemic state on the blackboard");
    return BT::NodeStatus::FAILURE;
  }

  const auto answer = client_->check_formula(goal);
  if (!answer.answered || !answer.success) {
    RCLCPP_ERROR(logger(), "[goal] could not check %s: %s", goal.c_str(), answer.error.c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (!answer.holds) {
    RCLCPP_ERROR(
      logger(),
      "[goal] the policy ran to completion but %s does not hold; the plan was "
      "built against a model the execution diverged from", goal.c_str());
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(logger(), "[goal] %s holds", goal.c_str());
  return BT::NodeStatus::SUCCESS;
}

void register_epistemic_nodes(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<CheckKnowledge>("CheckKnowledge");
  factory.registerNodeType<ApplyEpistemicUpdate>("ApplyEpistemicUpdate");
  factory.registerNodeType<EpistemicSwitch>("EpistemicSwitch");
  factory.registerNodeType<CheckEpistemicGoal>("CheckEpistemicGoal");
}

}  // namespace plansys2
