// Copyright 2026 Haniel Ulises
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

// A plan solver that takes its time, for the planner node's tests only. It is
// built into the test's own build-tree prefix and never installed.

#include <chrono>
#include <fstream>
#include <memory>
#include <optional>
#include <string>
#include <thread>

#include "plansys2_core/PlanSolverBase.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace plansys2_planner_test
{

class SlowPlanSolver : public plansys2::PlanSolverBase
{
public:
  void configure(
    rclcpp_lifecycle::LifecycleNode::SharedPtr lc_node,
    const std::string & plugin_name) override
  {
    const auto marker = plugin_name + ".started_marker";
    if (!lc_node->has_parameter(marker)) {
      lc_node->declare_parameter<std::string>(marker, "");
    }
    marker_ = lc_node->get_parameter(marker).as_string();
  }

  std::optional<plansys2_msgs::msg::Plan> getPlan(
    const std::string &, const std::string &, const std::string &,
    const rclcpp::Duration) override
  {
    // Said on disk rather than in memory, since this runs in a library the
    // test loads through pluginlib and shares no symbols with.
    std::ofstream(marker_) << "searching\n";
    std::this_thread::sleep_for(std::chrono::seconds(3));

    plansys2_msgs::msg::Plan plan;
    plansys2_msgs::msg::PlanItem item;
    item.action = "(wait)";
    item.duration = 1.0f;
    plan.items.push_back(item);
    return plan;
  }

  bool isDomainValid(const std::string &, const std::string &) override
  {
    return true;
  }

private:
  std::string marker_;
};

}  // namespace plansys2_planner_test

PLUGINLIB_EXPORT_CLASS(plansys2_planner_test::SlowPlanSolver, plansys2::PlanSolverBase)
