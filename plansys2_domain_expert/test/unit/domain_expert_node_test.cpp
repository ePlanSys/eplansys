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

#include <string>
#include <vector>
#include <regex>
#include <iostream>
#include <memory>
#include <atomic>
#include <thread>

#include "plansys2_pddl_parser/AmentIndexCompat.hpp"

#include "gtest/gtest.h"
#include "plansys2_domain_expert/DomainExpertNode.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"

#include "plansys2_core/Utils.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"


class ROS2Environment : public ::testing::Environment
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

// Stops the spin thread and joins it however the test body is left.
//
// A failing ASSERT_* returns from the test body, which used to skip the
// trailing t.join(). ~std::thread on a still-joinable thread calls
// std::terminate, so a timing-sensitive assertion turned into
// "terminate called without an active exception" and the process aborted
// before gtest could report which assertion failed.
class SpinThreadGuard
{
public:
  SpinThreadGuard(std::atomic<bool> & finish, std::thread & t)
  : finish_(finish), t_(t) {}

  ~SpinThreadGuard()
  {
    finish_ = true;
    if (t_.joinable()) {
      t_.join();
    }
  }

private:
  std::atomic<bool> & finish_;
  std::thread & t_;
};

// Waits for a lifecycle transition to land instead of assuming it fits in a
// fixed sleep. The transitions here took longer than the half second the test
// used to allow whenever the machine was loaded, which is what made this test
// fail intermittently in CI.
template<class NodeT, class ClockNodeT>
bool wait_for_state(
  const NodeT & node, const ClockNodeT & clock_node, uint8_t state_id,
  double timeout_s = 5.0)
{
  rclcpp::Rate rate(20);
  auto start = clock_node->now();
  while ((clock_node->now() - start).seconds() < timeout_s) {
    if (node->get_current_state().id() == state_id) {
      return true;
    }
    rate.sleep();
  }
  return node->get_current_state().id() == state_id;
}

TEST(domain_expert, lifecycle)
{
  {
    auto test_node = rclcpp::Node::make_shared("get_action_from_string");
    auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
    auto domain_client = std::make_shared<plansys2::DomainExpertClient>();

    std::string pkgpath = plansys2::get_package_share_dir("plansys2_domain_expert");

    domain_node->set_parameter({"model_file", pkgpath + "/pddl/domain_simple.pddl"});
    rclcpp::experimental::executors::EventsExecutor exe;

    exe.add_node(domain_node->get_node_base_interface());

    std::atomic<bool> finish = false;
    std::thread t([&]() {
        while (!finish) {exe.spin_some();}
      });
    SpinThreadGuard thread_guard(finish, t);

    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

    wait_for_state(
      domain_node, test_node, lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

    ASSERT_EQ(
    domain_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

    wait_for_state(
      domain_node, test_node, lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    ASSERT_EQ(
    domain_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    ASSERT_EQ(domain_client->getDomain(), domain_client->getDomain(true));
    auto domain_str = domain_client->getDomain();

    {
      rclcpp::Rate rate(10);
      auto start = test_node->now();
      while ((test_node->now() - start).seconds() < 0.5) {
        rate.sleep();
      }
    }

    std::ifstream domain_ifs_p(pkgpath + "/pddl/domain_simple_processed.pddl");
    std::string domain_str_p((
        std::istreambuf_iterator<char>(domain_ifs_p)),
      std::istreambuf_iterator<char>());

    ASSERT_EQ(domain_str, domain_str_p);
  }
  plansys2::drain_ros(200ms);
}

TEST(domain_expert, lifecycle_error)
{
  {
    auto test_node = rclcpp::Node::make_shared("get_action_from_string");
    auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
    auto domain_client = std::make_shared<plansys2::DomainExpertClient>();

    std::string pkgpath = plansys2::get_package_share_dir("plansys2_domain_expert");

    domain_node->set_parameter({"model_file", pkgpath + "/pddl/domain_2_error.pddl"});
    rclcpp::experimental::executors::EventsExecutor exe;

    exe.add_node(domain_node->get_node_base_interface());

    std::atomic<bool> finish = false;
    std::thread t([&]() {
        while (!finish) {exe.spin_some();}
      });
    SpinThreadGuard thread_guard(finish, t);

    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

    {
      rclcpp::Rate rate(10);
      auto start = test_node->now();
      while ((test_node->now() - start).seconds() < 0.5) {
        rate.sleep();
      }
    }
    // The failed configure leaves the node back in UNCONFIGURED, so the settle
    // time above stays: what this adds is patience for the transition itself
    // finishing on a loaded machine, where the node is still CONFIGURING.
    wait_for_state(
      domain_node, test_node, lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

    ASSERT_EQ(
    domain_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  }
  plansys2::drain_ros(200ms);
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
  ::testing::AddGlobalTestEnvironment(new ROS2Environment);

  const int result = RUN_ALL_TESTS();

  if (__gcov_dump) {
    __gcov_dump();
  }
  _exit(result);
}
