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

// The performers of the two-site survey.
//
// They stand in for hardware and do nothing but wait, as the one-site
// performers do. What is worth watching here is not any one of them but the
// order they run in: two of these drives are for different robots and
// different sites, and a policy dispatched a node at a time still runs them
// one after the other. The wall clock this process reports is what a parallel
// dispatch would have to beat.
//
// Two of them sense, one per site, and each names its own outcome.

#include <memory>
#include <string>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"

namespace eplansys_demo
{

using namespace std::chrono_literals;   // NOLINT (build/namespaces)

/// A performer that takes a while and then finishes.
///
/// `outcome` is empty for an ordinary action, and for a sensing one it is the
/// event the robot saw. Naming it on `finish` is the whole of what a performer
/// owes a branching policy.
class SiteAction : public plansys2::ActionExecutorClient
{
public:
  SiteAction(
    const std::string & node_name, const std::string & action,
    double duration, const std::string & outcome)
  : ActionExecutorClient(node_name), duration_(duration), outcome_(outcome)
  {
    set_parameter(rclcpp::Parameter("action_name", action));
    set_parameter(rclcpp::Parameter("rate", 4.0));
  }

private:
  void do_work() override
  {
    if (!started_) {
      started_ = true;
      begun_ = now();

      RCLCPP_INFO(
        get_logger(), "%s: starting (%.1fs)", get_action_name().c_str(), duration_);
    }

    const auto elapsed = (now() - begun_).seconds();
    const auto progress = duration_ > 0.0 ? elapsed / duration_ : 1.0;

    if (progress < 1.0) {
      send_feedback(static_cast<float>(progress), "working");
      return;
    }

    if (outcome_.empty()) {
      RCLCPP_INFO(get_logger(), "%s: done", get_action_name().c_str());
    } else {
      RCLCPP_INFO(
        get_logger(), "%s: observed %s", get_action_name().c_str(), outcome_.c_str());
    }

    finish(true, 1.0, "done", outcome_);
    started_ = false;
  }

  bool started_{false};
  rclcpp::Time begun_;
  double duration_{3.0};
  std::string outcome_;
};

}  // namespace eplansys_demo

int main(int argc, char ** argv)
{
  // What each site turns out to hold. Taken from the command line rather than
  // from parameters, because the performers share one process and a parameter
  // set from a launch file reaches the node it names and not its neighbours.
  std::string north = "e-scan-north-dirty";
  std::string south = "e-scan-south-clean";
  for (int i = 1; i + 1 < argc; ++i) {
    if (std::string(argv[i]) == "--north") {
      north = argv[i + 1];
    } else if (std::string(argv[i]) == "--south") {
      south = argv[i + 1];
    }
  }

  rclcpp::init(argc, argv);

  auto goto_north = std::make_shared<eplansys_demo::SiteAction>(
    "goto_north_node", "goto_north", 4.0, "");
  auto goto_south = std::make_shared<eplansys_demo::SiteAction>(
    "goto_south_node", "goto_south", 4.0, "");
  // The two that observe anything, one site each.
  auto scan_north = std::make_shared<eplansys_demo::SiteAction>(
    "scan_north_node", "scan_north", 3.0, north);
  auto scan_south = std::make_shared<eplansys_demo::SiteAction>(
    "scan_south_node", "scan_south", 3.0, south);
  // Two audiences, per site, indistinguishable from here.
  auto broadcast_north = std::make_shared<eplansys_demo::SiteAction>(
    "broadcast_north_node", "broadcast_north", 2.0, "");
  auto broadcast_south = std::make_shared<eplansys_demo::SiteAction>(
    "broadcast_south_node", "broadcast_south", 2.0, "");
  auto relay_north = std::make_shared<eplansys_demo::SiteAction>(
    "relay_north_node", "relay_north", 2.0, "");
  auto relay_south = std::make_shared<eplansys_demo::SiteAction>(
    "relay_south_node", "relay_south", 2.0, "");

  rclcpp::executors::MultiThreadedExecutor executor;
  for (const auto & action : {goto_north, goto_south, scan_north, scan_south,
      broadcast_north, broadcast_south, relay_north, relay_south})
  {
    action->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    executor.add_node(action->get_node_base_interface());
  }

  executor.spin();
  rclcpp::shutdown();
  return 0;
}
