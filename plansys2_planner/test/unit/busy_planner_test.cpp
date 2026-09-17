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

// A search that takes seconds must not take the process with it.
//
// plansys2_bringup's monolithic node serves every PlanSys2 node from one
// executor, and the lifecycle manager brings them up with service calls that
// same executor answers. A planning request that arrives during bring-up used
// to be searched inside its service callback, so the executor answered nothing
// else until the search ended: the lifecycle calls timed out, bring-up reported
// "Failed to start plansys2!", and the plan went to a system that had already
// shut down. A classical domain returns before anyone notices. An epistemic
// one with three sites took seven seconds.

#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "plansys2_core/Compat.hpp"
#include "plansys2_core/Utils.hpp"
#include "plansys2_pddl_parser/AmentIndexCompat.hpp"
#include "plansys2_domain_expert/DomainExpertNode.hpp"
#include "plansys2_msgs/srv/get_plan.hpp"
#include "plansys2_planner/PlannerNode.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

TEST(planner_node, other_nodes_are_answered_while_a_plan_is_searched)
{
  const auto marker = std::filesystem::temp_directory_path() /
    ("plansys2-slow-solver-" + std::to_string(::getpid()));
  std::filesystem::remove(marker);

  {
    auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
    auto planner_node = std::make_shared<plansys2::PlannerNode>(
      rclcpp::NodeOptions().parameter_overrides(
    {
      {"plan_solver_plugins", std::vector<std::string>{"SLOW"}},
      {"SLOW.plugin", "plansys2_planner_test::SlowPlanSolver"},
      {"SLOW.started_marker", marker.string()},
    }));
    auto test_node = rclcpp::Node::make_shared("busy_planner_test");

    const auto pkgpath = plansys2::get_package_share_dir("plansys2_planner");
    domain_node->set_parameter(
      rclcpp::Parameter("model_file", pkgpath + "/pddl/domain_simple.pddl"));

    // One executor for both nodes, as in the monolithic bring-up.
    plansys2::SpinExecutor exe;
    exe.add_node(domain_node->get_node_base_interface());
    exe.add_node(planner_node->get_node_base_interface());

    std::atomic<bool> finish{false};
    std::thread spinner([&]() {
        while (!finish) {
          exe.spin_some();
        }
      });

    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

    rclcpp::executors::SingleThreadedExecutor client_exe;
    client_exe.add_node(test_node);

    auto plan_client = test_node->create_client<plansys2_msgs::srv::GetPlan>("planner/get_plan");
    auto state_client =
      test_node->create_client<lifecycle_msgs::srv::GetState>("domain_expert/get_state");
    ASSERT_TRUE(plan_client->wait_for_service(5s));
    ASSERT_TRUE(state_client->wait_for_service(5s));

    auto plan_request = std::make_shared<plansys2_msgs::srv::GetPlan::Request>();
    plan_request->domain = "(define (domain d))";
    plan_request->problem = "(define (problem p))";
    auto plan_future = plan_client->async_send_request(plan_request);

    // Ask only once the search is under way, so that the question is put to an
    // executor that has a search in hand.
    const auto give_up = std::chrono::steady_clock::now() + 5s;
    while (!std::filesystem::exists(marker) && std::chrono::steady_clock::now() < give_up) {
      client_exe.spin_some(10ms);
    }
    ASSERT_TRUE(std::filesystem::exists(marker)) << "the slow solver never started";

    auto state_future = state_client->async_send_request(
      std::make_shared<lifecycle_msgs::srv::GetState::Request>());
    EXPECT_EQ(
      client_exe.spin_until_future_complete(state_future, 1s),
      rclcpp::FutureReturnCode::SUCCESS)
      << "the domain expert went unanswered while the planner searched";

    // And the plan still arrives once the search is done.
    ASSERT_EQ(
      client_exe.spin_until_future_complete(plan_future, 10s),
      rclcpp::FutureReturnCode::SUCCESS);
    const auto response = plan_future.get();
    EXPECT_TRUE(response->success);
    ASSERT_EQ(response->plan.items.size(), 1u);
    EXPECT_EQ(response->plan.items.front().action, "(wait)");

    finish = true;
    spinner.join();
  }
  std::filesystem::remove(marker);
  plansys2::drain_ros(200ms);
}

// Leave through _exit, for the reason planner_test.cpp gives.
extern "C" void __gcov_dump(void) __attribute__((weak));

int main(int argc, char ** argv)
{
  // The slow solver lives in a prefix of this test's own build tree, which
  // pluginlib finds through the ament index like any installed plugin.
  const char * prefix_path = std::getenv("AMENT_PREFIX_PATH");
  const std::string prefixes = std::string(TEST_PLUGIN_PREFIX) +
    (prefix_path != nullptr ? std::string(":") + prefix_path : std::string());
  setenv("AMENT_PREFIX_PATH", prefixes.c_str(), 1);

  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();

  if (__gcov_dump) {
    __gcov_dump();
  }
  _exit(result);
}
