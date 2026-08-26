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

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "plansys2_lifecycle_manager/executor.hpp"
#include "plansys2_lifecycle_manager/lifecycle_manager.hpp"

int main(int argc, char ** argv)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  // Which nodes to manage is a parameter rather than a constant because the
  // epistemic nodes are optional: a classical system has the four below, and
  // an epistemic one adds "epistemic_state", and "epistemic_perception" when
  // what does the sensing is a map. Naming a node that is not running
  // makes startup fail, which is the right outcome — it is the same failure as
  // that node crashing — so the launch file decides, not this program.
  auto node = rclcpp::Node::make_shared("lifecycle_manager_node");
  const std::vector<std::string> default_nodes{
    "domain_expert", "problem_expert", "planner", "executor"};
  node->declare_parameter("managed_nodes", default_nodes);

  const auto managed = node->get_parameter("managed_nodes").as_string_array();

  std::map<std::string, std::shared_ptr<plansys2::LifecycleServiceClient>> manager_nodes;
  for (const auto & name : managed) {
    manager_nodes[name] =
      std::make_shared<plansys2::LifecycleServiceClient>(name + "_lc_mngr", name);
  }

  plansys2::ManagerExecutor exe;
  exe.add_node(node);
  for (auto & manager_node : manager_nodes) {
    manager_node.second->init();
    exe.add_node(manager_node.second);
  }

  std::shared_future<bool> startup_future = std::async(
    std::launch::async,
    std::bind(plansys2::startup_function, manager_nodes, std::chrono::seconds(5)));
  exe.spin_until_future_complete(startup_future);

  if (!startup_future.get()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("plansys2_lifecycle_manager"),
      "Failed to start plansys2!");
    rclcpp::shutdown();
    return -1;
  }

  rclcpp::shutdown();

  return 0;
}
