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

// The four performers of the corridor mission.
//
// They stand in for hardware and do nothing but wait, which is the point: the
// mission's difficulty is epistemic, and none of it lives here. What a
// deployment would replace is the waiting, not the structure.
//
// One of them is not like the others. `inspect_corridor` is a sensing action,
// and a sensing action has to report what it found: it names the outcome on
// `finish`, and that is the whole of what a performer has to do for a policy
// to branch on it. Everything after that --- the blackboard entry, the
// epistemic update, the switch --- is the framework's business.

#include <memory>
#include <string>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"

namespace eplansys_demo
{

using namespace std::chrono_literals;   // NOLINT (build/namespaces)

/// A performer that takes a while and then finishes.
///
/// `outcome` is empty for an ordinary action. For the sensing one it is the
/// event the robot saw, which is what the corridor's actual state is set
/// through: run the demo twice with different values and the policy takes
/// different branches, which is the thing worth watching.
class CorridorAction : public plansys2::ActionExecutorClient
{
public:
  CorridorAction(
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
      // The sensing case. Naming the outcome here is what lets the policy
      // branch: the executor carries it to the epistemic state, the state
      // performs the DEL update for that event, and the switch runs the
      // continuation planned for it.
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
  // What the robot turns out to see, taken from the command line rather than
  // from a parameter: the four performers share one process, and a parameter
  // set from a launch file reaches the node the file names, not its
  // neighbours.
  std::string observed = "e-inspect-clear";
  for (int i = 1; i + 1 < argc; ++i) {
    if (std::string(argv[i]) == "--outcome") {
      observed = argv[i + 1];
    }
  }

  rclcpp::init(argc, argv);

  // One process for the four performers: a demo that needed four terminals
  // would be teaching the reader about terminals.
  auto goto_junction = std::make_shared<eplansys_demo::CorridorAction>(
    "goto_junction_node", "goto_junction", 4.0, "");
  // The only one that observes anything.
  auto inspect = std::make_shared<eplansys_demo::CorridorAction>(
    "inspect_corridor_node", "inspect_corridor", 3.0, observed);
  auto report_clear = std::make_shared<eplansys_demo::CorridorAction>(
    "report_clear_node", "report_clear", 2.0, "");
  auto report_blocked = std::make_shared<eplansys_demo::CorridorAction>(
    "report_blocked_node", "report_blocked", 2.0, "");

  rclcpp::executors::MultiThreadedExecutor executor;
  for (const auto & action : {goto_junction, inspect, report_clear, report_blocked}) {
    action->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    executor.add_node(action->get_node_base_interface());
  }

  executor.spin();
  rclcpp::shutdown();
  return 0;
}
