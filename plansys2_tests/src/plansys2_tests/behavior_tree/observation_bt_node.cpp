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

// The piece a real deployment has to supply and a test has to stand in for:
// what the robot actually observed.
//
// ApplyEpistemicUpdate takes the outcome on its `observed` port, and the
// epistemic state needs it whenever the model designates more than one world —
// which is exactly the interesting case, since a model that already knows what
// happened had nothing to sense. Nothing in PlanSys2 carries an observation
// back from a performer, so the binding is left to a domain-specific tree; this
// node is the test's version of one, reading the outcome from a topic the fake
// performers publish on.
//
// The topic carries what the world turns out to be, as `action=outcome` pairs
// separated by ';' — one per sensing action, because a mission with two
// corridors in it has two things to find out and they need not agree. A node
// reports the entry whose action its own action id starts with.
//
// It is deliberately dumb: it reports what it was told and never blocks the
// tree. An observation that has not arrived leaves the port empty, which is
// the same as not having sensed yet, and the epistemic state says so rather
// than guessing.

#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace plansys2_tests
{

/**
 * @class plansys2_tests::ObservedOutcome
 * @brief Puts the outcome a performer reported on the blackboard.
 */
class ObservedOutcome : public BT::SyncActionNode
{
public:
  ObservedOutcome(const std::string & xml_tag_name, const BT::NodeConfig & conf)
  : BT::SyncActionNode(xml_tag_name, conf)
  {
    std::string topic = "/epistemic_observation";
    getInput("topic", topic);
    getInput("action", action_);

    // One node per tree node, so the names have to differ: the tree holds an
    // ObservedOutcome for every action in the policy, branches included.
    node_ = rclcpp::Node::make_shared(
      "observation_reader_" + std::to_string(++instances_));
    // Transient local, because the observation is published once, before the
    // tree that reads it exists. A volatile subscription would come up after
    // the fact and find nothing.
    subscription_ = node_->create_subscription<std_msgs::msg::String>(
      topic, rclcpp::QoS(1).transient_local(),
      [this](const std_msgs::msg::String::SharedPtr message) {
        observed_ = outcome_for(message->data);
      });
  }

  BT::NodeStatus tick() override
  {
    rclcpp::spin_some(node_);
    setOutput("outcome", observed_);
    return BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<std::string>(
          "topic", "/epistemic_observation", "Where the performer reports what it saw"),
        BT::InputPort<std::string>("action", "", "Action id, to pick out this node's entry"),
        BT::OutputPort<std::string>("outcome", "The outcome observed, empty for none"),
      });
  }

private:
  static int instances_;

  /// The outcome recorded for this node's action, empty when the world says
  /// nothing about it.
  std::string outcome_for(const std::string & report) const
  {
    std::size_t start = 0;
    while (start <= report.size()) {
      const auto end = report.find(';', start);
      const auto entry = report.substr(
        start, end == std::string::npos ? std::string::npos : end - start);
      const auto equals = entry.find('=');
      if (equals != std::string::npos) {
        const auto action = entry.substr(0, equals);
        if (!action.empty() && action_.rfind(action, 0) == 0) {
          return entry.substr(equals + 1);
        }
      }
      if (end == std::string::npos) {
        break;
      }
      start = end + 1;
    }
    return "";
  }

  std::string action_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  std::string observed_;
};

int ObservedOutcome::instances_ = 0;

}  // namespace plansys2_tests

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<plansys2_tests::ObservedOutcome>("ObservedOutcome");
}
