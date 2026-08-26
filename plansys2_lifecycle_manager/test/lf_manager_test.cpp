// Copyright 2019 Intelligent Robotics Lab
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

#include <unistd.h>

#include <memory>
#include <string>
#include <map>

#include "gtest/gtest.h"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "plansys2_core/Compat.hpp"
#include "plansys2_lifecycle_manager/lifecycle_manager.hpp"

using namespace std::chrono_literals;

TEST(lifecycle_manager, lf_client)
{
  auto test_node = rclcpp_lifecycle::LifecycleNode::make_shared("test");
  auto client_node = std::make_shared<plansys2::LifecycleServiceClient>("mng_client", "test");

  auto exe = plansys2::SpinExecutor::make_shared();
  exe->add_node(test_node->get_node_base_interface());
  exe->add_node(client_node->get_node_base_interface());

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe->spin_some();}
    });

  client_node->init();

  auto start = test_node->now();
  while ((test_node->now() - start).seconds() < 1.0) {}

  ASSERT_EQ(
    test_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  ASSERT_EQ(client_node->get_state(1s), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

  ASSERT_FALSE(client_node->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE));

  ASSERT_TRUE(client_node->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));
  ASSERT_EQ(client_node->get_state(1s), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  ASSERT_TRUE(client_node->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE));
  ASSERT_EQ(client_node->get_state(1s), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  ASSERT_TRUE(client_node->change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE));
  ASSERT_EQ(client_node->get_state(1s), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  finish = true;
  t.join();
}

TEST(lifecycle_manager, lf_startup)
{
  auto de_node = rclcpp_lifecycle::LifecycleNode::make_shared("domain_expert");
  auto pe_node = rclcpp_lifecycle::LifecycleNode::make_shared("problem_expert");
  auto pl_node = rclcpp_lifecycle::LifecycleNode::make_shared("planner");
  auto ex_node = rclcpp_lifecycle::LifecycleNode::make_shared("executor");

  ASSERT_EQ(
    de_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  ASSERT_EQ(
    pe_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  ASSERT_EQ(
    pl_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  ASSERT_EQ(
    ex_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

  std::map<std::string, std::shared_ptr<plansys2::LifecycleServiceClient>> manager_nodes;
  manager_nodes["domain_expert"] = std::make_shared<plansys2::LifecycleServiceClient>(
    "domain_expert_lc_mngr", "domain_expert");
  manager_nodes["problem_expert"] = std::make_shared<plansys2::LifecycleServiceClient>(
    "domain_expert_lc_mngr", "problem_expert");
  manager_nodes["planner"] = std::make_shared<plansys2::LifecycleServiceClient>(
    "domain_expert_lc_mngr", "planner");
  manager_nodes["executor"] = std::make_shared<plansys2::LifecycleServiceClient>(
    "domain_expert_lc_mngr", "executor");

  plansys2::SpinExecutor exe;
  for (auto & manager_node : manager_nodes) {
    manager_node.second->init();
    exe.add_node(manager_node.second);
  }

  exe.add_node(de_node->get_node_base_interface());
  exe.add_node(pe_node->get_node_base_interface());
  exe.add_node(pl_node->get_node_base_interface());
  exe.add_node(ex_node->get_node_base_interface());

  auto start = de_node->now();
  while ((de_node->now() - start).seconds() < 1.0) {}

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });

  std::shared_future<bool> startup_future = std::async(
    std::launch::async,
    std::bind(plansys2::startup_function, manager_nodes, std::chrono::seconds(3)));

  startup_future.wait();

  ASSERT_EQ(
    de_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  ASSERT_EQ(
    pe_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  ASSERT_EQ(
    pl_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  ASSERT_EQ(
    ex_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  start = de_node->now();
  while ((de_node->now() - start).seconds() < 1.0) {}

  finish = true;
  t.join();
}

// An epistemic bringup manages a fifth node. What is checked here is that the
// same startup_function brings up a map it was not written around: the classical
// four still reach ACTIVE, and so does the extra one.
TEST(lifecycle_manager, lf_startup_with_epistemic_state)
{
  auto de_node = rclcpp_lifecycle::LifecycleNode::make_shared("domain_expert");
  auto pe_node = rclcpp_lifecycle::LifecycleNode::make_shared("problem_expert");
  auto pl_node = rclcpp_lifecycle::LifecycleNode::make_shared("planner");
  auto ex_node = rclcpp_lifecycle::LifecycleNode::make_shared("executor");
  auto es_node = rclcpp_lifecycle::LifecycleNode::make_shared("epistemic_state");

  std::map<std::string, std::shared_ptr<plansys2::LifecycleServiceClient>> manager_nodes;
  for (const auto & name :
    {"domain_expert", "problem_expert", "planner", "executor", "epistemic_state"})
  {
    manager_nodes[name] = std::make_shared<plansys2::LifecycleServiceClient>(
      std::string(name) + "_epistemic_lc_mngr", name);
  }

  plansys2::SpinExecutor exe;
  for (auto & manager_node : manager_nodes) {
    manager_node.second->init();
    exe.add_node(manager_node.second);
  }

  exe.add_node(de_node->get_node_base_interface());
  exe.add_node(pe_node->get_node_base_interface());
  exe.add_node(pl_node->get_node_base_interface());
  exe.add_node(ex_node->get_node_base_interface());
  exe.add_node(es_node->get_node_base_interface());

  auto start = de_node->now();
  while ((de_node->now() - start).seconds() < 1.0) {}

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });

  std::shared_future<bool> startup_future = std::async(
    std::launch::async,
    std::bind(plansys2::startup_function, manager_nodes, std::chrono::seconds(3)));

  startup_future.wait();

  ASSERT_TRUE(startup_future.get());

  for (const auto & node :
    {de_node, pe_node, pl_node, ex_node, es_node})
  {
    ASSERT_EQ(
      node->get_current_state().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) << node->get_name();
  }

  start = de_node->now();
  while ((de_node->now() - start).seconds() < 1.0) {}

  finish = true;
  t.join();
}

// Leave through _exit, so the DDS threads never outlive the process.
//
// rclcpp::shutdown() does not finalise the global context; that happens in
// static destruction, inside _dl_fini, while Fast DDS listener threads are
// still running in libraries the loader is unmapping. Under --coverage, which
// is how the rolling job builds, that same exit path writes a .gcda for every
// object file, stretching the window until about one run in ten segfaults with
// every test already passed. Dumping the counters and leaving through _exit
// keeps the coverage data and skips static destruction. __gcov_dump is weak:
// it is null in an uninstrumented build.
extern "C" void __gcov_dump(void) __attribute__((weak));

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();

  if (__gcov_dump) {
    __gcov_dump();
  }
  _exit(result);
}
