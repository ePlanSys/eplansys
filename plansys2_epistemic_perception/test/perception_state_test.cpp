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

#include <unistd.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "plansys2_epistemic_executor/EpistemicStateClient.hpp"
#include "plansys2_epistemic_executor/EpistemicStateNode.hpp"
#include "plansys2_epistemic_perception/EpistemicPerceptionNode.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;  // NOLINT(build/namespaces)

namespace
{

/// Perception against the real epistemic state, over the real services.
///
/// perception_node_test.cpp puts a stub behind those services, which is the
/// right test for "does a grid become the right call, once". It cannot answer
/// the question this file exists for: whether the call perception makes is one
/// the state accepts against a grounded task. The stub says yes to anything,
/// including a formula naming an atom no task declares -- and the launch file's
/// own comment warns that such a region "resolves to a formula the state cannot
/// parse, and says so on the first observation".
///
/// The task is the fleet corridor, whose vocabulary is the worked example in
/// this package's README: atoms `at-junction_r1`, `at-junction_r2`, `blocked`,
/// agents `r1` and `r2`. Its initial model designates both a world where the
/// corridor is blocked and one where it is not, so nobody knows which -- which
/// is exactly the state a map is able to settle.
std::string task_file()
{
  return std::string(EPISTEMIC_TASK_DIR) + "/robot-fleet.json";
}

/// A 4x4 metre grid of 1 m cells. Every cell free unless one is named occupied.
nav_msgs::msg::OccupancyGrid grid_with(int obstacle_at = -1)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.info.resolution = 1.0;
  grid.info.width = 4;
  grid.info.height = 4;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(16, 0);
  if (obstacle_at >= 0) {
    grid.data[obstacle_at] = 100;
  }
  return grid;
}

/// The corridor of the fleet task, as this package's README configures it:
/// the atom names the obstruction, so a clear corridor denies it.
rclcpp::NodeOptions corridor()
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {
      rclcpp::Parameter("map_topic", "/map"),
      rclcpp::Parameter("regions", std::vector<std::string>{"corridor"}),
      rclcpp::Parameter("corridor.boxes", std::vector<double>{0.0, 0.0, 2.0, 2.0}),
      rclcpp::Parameter("corridor.atom", "blocked"),
      rclcpp::Parameter("corridor.atom_true_when_clear", false),
      rclcpp::Parameter("call_timeout", 5.0),
    });
  return options;
}

}  // namespace

class PerceptionAgainstTheState : public ::testing::Test
{
protected:
  void SetUp() override
  {
    state_ = std::make_shared<plansys2::EpistemicStateNode>();
    state_->set_parameter(rclcpp::Parameter("task_file", task_file()));

    ASSERT_EQ(
      state_->configure().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(
      state_->activate().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    publisher_node_ = rclcpp::Node::make_shared("perception_state_test_publisher");
    map_pub_ = publisher_node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(1).transient_local());

    // Two executors, as in perception_node_test: perception blocks its own
    // callback while it waits for the state to answer, so the state has to be
    // spun by a thread that is not the one waiting on it.
    state_executor_.add_node(state_->get_node_base_interface());
    state_executor_.add_node(publisher_node_);
    spinning_ = true;
    state_thread_ = std::thread(
      [this]() {
        while (spinning_) {
          state_executor_.spin_some();
          std::this_thread::sleep_for(1ms);
        }
      });

    client_ = std::make_shared<plansys2::EpistemicStateClient>("perception_state_test_client");
  }

  void TearDown() override
  {
    if (perception_) {
      perception_executor_.cancel();
      if (perception_thread_.joinable()) {
        perception_thread_.join();
      }
    }
    spinning_ = false;
    if (state_thread_.joinable()) {
      state_thread_.join();
    }
  }

  /// Bring perception up and start spinning it.
  void start_perception(const rclcpp::NodeOptions & options)
  {
    using lifecycle_msgs::msg::Transition;
    perception_ = std::make_shared<plansys2::EpistemicPerceptionNode>(options);

    ASSERT_EQ(
      perception_->trigger_transition(Transition::TRANSITION_CONFIGURE).id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    perception_->trigger_transition(Transition::TRANSITION_ACTIVATE);

    perception_executor_.add_node(perception_->get_node_base_interface());
    perception_thread_ = std::thread([this]() {perception_executor_.spin();});
  }

  /// Does this formula hold in the state now?
  bool holds(const std::string & formula)
  {
    const auto answer = client_->check_formula(formula, 5s);
    EXPECT_TRUE(answer.answered) << formula << ": " << answer.error;
    EXPECT_TRUE(answer.success) << formula << ": " << answer.error;
    return answer.answered && answer.success && answer.holds;
  }

  /// Wait for a predicate, or give up and let the assertion say what was missing.
  template<typename Predicate>
  bool wait_for(Predicate predicate, std::chrono::milliseconds limit = 15s)
  {
    const auto deadline = std::chrono::steady_clock::now() + limit;
    while (std::chrono::steady_clock::now() < deadline) {
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(100ms);
    }
    return predicate();
  }

  std::shared_ptr<plansys2::EpistemicStateNode> state_;
  std::shared_ptr<plansys2::EpistemicPerceptionNode> perception_;
  plansys2::EpistemicStateClient::Ptr client_;
  rclcpp::Node::SharedPtr publisher_node_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

  rclcpp::executors::SingleThreadedExecutor state_executor_;
  rclcpp::executors::SingleThreadedExecutor perception_executor_;
  std::thread state_thread_;
  std::thread perception_thread_;
  std::atomic<bool> spinning_{false};
};

TEST_F(PerceptionAgainstTheState, NobodyKnowsAboutTheCorridorBeforeAMapArrives)
{
  // The premise the rest of the file rests on. The task designates a blocked
  // world and a clear one, so neither the atom nor its negation holds: this is
  // ignorance, not a third truth value, and it is what perception resolves.
  EXPECT_FALSE(holds("blocked"));
  EXPECT_FALSE(holds("(not blocked)"));
}

TEST_F(PerceptionAgainstTheState, AClearMapIsAnnouncedAndTheStateTakesIt)
{
  ASSERT_FALSE(holds("(not blocked)"));

  start_perception(corridor());
  map_pub_->publish(grid_with());

  // The announcement restricts the model to the worlds where the formula
  // holds, which here leaves only the world in which the corridor is clear.
  ASSERT_TRUE(wait_for([&]() {return holds("(not blocked)");}))
    << "perception never got '(not blocked)' into the state";

  EXPECT_FALSE(holds("blocked"));
}

TEST_F(PerceptionAgainstTheState, ABlockedMapIsAnnouncedAndTheStateTakesIt)
{
  ASSERT_FALSE(holds("blocked"));

  start_perception(corridor());

  // One occupied cell inside the region is enough: blocked is existential.
  map_pub_->publish(grid_with(5));

  ASSERT_TRUE(wait_for([&]() {return holds("blocked");}))
    << "perception never got 'blocked' into the state";

  EXPECT_FALSE(holds("(not blocked)"));
}

TEST_F(PerceptionAgainstTheState, BothAgentsComeToKnowIt)
{
  // A public announcement is what the state offers for something everyone
  // witnessed, and this is the property that distinguishes it from a private
  // observation: the fleet task's goal is that both robots know, so an
  // announcement only one of them received would not discharge it.
  start_perception(corridor());
  map_pub_->publish(grid_with());

  ASSERT_TRUE(wait_for([&]() {return holds("(not blocked)");}));

  EXPECT_TRUE(holds("(K r1 (not blocked))"));
  EXPECT_TRUE(holds("(K r2 (not blocked))"));
}

TEST_F(PerceptionAgainstTheState, AnAtomTheTaskDoesNotDeclareIsRefused)
{
  // The failure the launch file warns about, against the real state rather
  // than a stub that would have accepted it. Perception reports the refusal
  // and does not retry, so the region stays reported and the map keeps
  // arriving without a second call.
  auto options = corridor();
  options.parameter_overrides(
  {
    rclcpp::Parameter("map_topic", "/map"),
    rclcpp::Parameter("regions", std::vector<std::string>{"corridor"}),
    rclcpp::Parameter("corridor.boxes", std::vector<double>{0.0, 0.0, 2.0, 2.0}),
    rclcpp::Parameter("corridor.atom", "no-such-atom"),
    rclcpp::Parameter("call_timeout", 5.0),
  });

  start_perception(options);
  map_pub_->publish(grid_with());

  // Nothing the task can express changed, and the node is still running.
  std::this_thread::sleep_for(2s);
  EXPECT_FALSE(holds("blocked"));
  EXPECT_FALSE(holds("(not blocked)"));
}

// Leave through _exit, so the DDS threads never outlive the process. See the
// note in perception_node_test.cpp.
extern "C" void __gcov_dump(void) __attribute__((weak));

class ROS2Environment : public ::testing::Environment
{
public:
  void SetUp() override {rclcpp::init(0, nullptr);}
  void TearDown() override {rclcpp::shutdown();}
};

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
