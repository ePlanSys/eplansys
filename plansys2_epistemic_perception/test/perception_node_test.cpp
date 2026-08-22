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
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "plansys2_epistemic_msgs/srv/announce.hpp"
#include "plansys2_epistemic_msgs/srv/apply_action.hpp"
#include "plansys2_epistemic_perception/EpistemicPerceptionNode.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;  // NOLINT(build/namespaces)

namespace
{

/// The epistemic state, reduced to the two calls perception makes.
///
/// What an announcement does to a Kripke model is the state node's own test.
/// What is being checked here is that a grid becomes the right call, once, and
/// a stub says that without needing a grounded task to say it against.
class StubState : public rclcpp::Node
{
public:
  StubState()
  : rclcpp::Node("stub_epistemic_state")
  {
    announce_service_ = create_service<plansys2_epistemic_msgs::srv::Announce>(
      "epistemic_state/announce",
      [this](
        const std::shared_ptr<plansys2_epistemic_msgs::srv::Announce::Request> request,
        std::shared_ptr<plansys2_epistemic_msgs::srv::Announce::Response> response) {
        {
          std::lock_guard<std::mutex> guard(mutex_);
          announced_.push_back(request->formula);
        }
        response->success = accept_;
        response->error = accept_ ? "" : "the stub was told to refuse";
      });

    apply_action_service_ = create_service<plansys2_epistemic_msgs::srv::ApplyAction>(
      "epistemic_state/apply_action",
      [this](
        const std::shared_ptr<plansys2_epistemic_msgs::srv::ApplyAction::Request> request,
        std::shared_ptr<plansys2_epistemic_msgs::srv::ApplyAction::Response> response) {
        {
          std::lock_guard<std::mutex> guard(mutex_);
          applied_.push_back({request->epistemic_action, request->observed_outcome});
        }
        response->success = accept_;
        response->outcome = request->observed_outcome;
        response->error = accept_ ? "" : "the stub was told to refuse";
      });
  }

  struct Applied
  {
    std::string action;
    std::string outcome;
  };

  std::vector<std::string> announced() const
  {
    std::lock_guard<std::mutex> guard(mutex_);
    return announced_;
  }

  std::vector<Applied> applied() const
  {
    std::lock_guard<std::mutex> guard(mutex_);
    return applied_;
  }

  void refuse() {accept_ = false;}

private:
  mutable std::mutex mutex_;
  std::atomic<bool> accept_{true};
  std::vector<std::string> announced_;
  std::vector<Applied> applied_;
  rclcpp::Service<plansys2_epistemic_msgs::srv::Announce>::SharedPtr announce_service_;
  rclcpp::Service<plansys2_epistemic_msgs::srv::ApplyAction>::SharedPtr apply_action_service_;
};

/// A 4x4 metre grid of 1 m cells, every cell free.
nav_msgs::msg::OccupancyGrid free_grid()
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.info.resolution = 1.0;
  grid.info.width = 4;
  grid.info.height = 4;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(16, 0);
  return grid;
}

/// The world, the two nodes that talk about it, and the threads that spin
/// them.
///
/// Two executors rather than one, because perception blocks its own callback
/// while it waits for the state to answer: with a single executor that wait
/// would be for a service reply nobody is left to produce.
class Graph
{
public:
  explicit Graph(const rclcpp::NodeOptions & options)
  : perception_(std::make_shared<plansys2::EpistemicPerceptionNode>(options)),
    state_(std::make_shared<StubState>()),
    publisher_node_(rclcpp::Node::make_shared("test_map_publisher"))
  {
    map_pub_ = publisher_node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(1).transient_local());

    perception_executor_.add_node(perception_->get_node_base_interface());
    state_executor_.add_node(state_);
    state_executor_.add_node(publisher_node_);

    perception_thread_ = std::thread([this]() {perception_executor_.spin();});
    state_thread_ = std::thread([this]() {state_executor_.spin();});
  }

  ~Graph()
  {
    perception_executor_.cancel();
    state_executor_.cancel();
    perception_thread_.join();
    state_thread_.join();
  }

  Graph(const Graph &) = delete;
  Graph & operator=(const Graph &) = delete;

  bool bring_up()
  {
    using lifecycle_msgs::msg::Transition;
    const auto configured = perception_->trigger_transition(Transition::TRANSITION_CONFIGURE);
    if (configured.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      return false;
    }
    perception_->trigger_transition(Transition::TRANSITION_ACTIVATE);
    return true;
  }

  void publish(const nav_msgs::msg::OccupancyGrid & grid) {map_pub_->publish(grid);}

  StubState & state() {return *state_;}
  plansys2::EpistemicPerceptionNode & perception() {return *perception_;}

private:
  std::shared_ptr<plansys2::EpistemicPerceptionNode> perception_;
  std::shared_ptr<StubState> state_;
  rclcpp::Node::SharedPtr publisher_node_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

  rclcpp::executors::SingleThreadedExecutor perception_executor_;
  rclcpp::executors::SingleThreadedExecutor state_executor_;
  std::thread perception_thread_;
  std::thread state_thread_;
};

/// Wait for something to become true, or give up and let the assertion say
/// what was missing.
template<typename Predicate>
bool wait_for(Predicate predicate, std::chrono::milliseconds limit = 10s)
{
  const auto deadline = std::chrono::steady_clock::now() + limit;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(20ms);
  }
  return predicate();
}

rclcpp::NodeOptions with_corridor(bool sensing)
{
  std::vector<rclcpp::Parameter> overrides{
    rclcpp::Parameter("map_topic", "/map"),
    rclcpp::Parameter("regions", std::vector<std::string>{"corridor"}),
    rclcpp::Parameter("corridor.boxes", std::vector<double>{0.0, 0.0, 2.0, 2.0}),
    rclcpp::Parameter("corridor.atom", "blocked"),
    rclcpp::Parameter("corridor.atom_true_when_clear", false),
    rclcpp::Parameter("call_timeout", 2.0),
  };

  if (sensing) {
    overrides.push_back(rclcpp::Parameter("corridor.sensing_action", "inspect_r1"));
    overrides.push_back(rclcpp::Parameter("corridor.outcome_when_clear", "e-inspect-clear"));
    overrides.push_back(rclcpp::Parameter("corridor.outcome_when_blocked", "e-inspect-blocked"));
  }

  rclcpp::NodeOptions options;
  options.parameter_overrides(overrides);
  return options;
}

}  // namespace

class ROS2Environment : public ::testing::Environment
{
public:
  void SetUp() override {rclcpp::init(0, nullptr);}
  void TearDown() override {rclcpp::shutdown();}
};

TEST(perception_node, a_region_without_a_sensing_action_is_announced)
{
  Graph graph(with_corridor(false));
  ASSERT_TRUE(graph.bring_up());

  graph.publish(free_grid());

  ASSERT_TRUE(wait_for([&]() {return !graph.state().announced().empty();}));

  // The atom names the obstruction, so a corridor that is clear denies it.
  EXPECT_EQ(graph.state().announced().front(), "(not blocked)");
  EXPECT_TRUE(graph.state().applied().empty());
}

TEST(perception_node, a_region_bound_to_a_sensing_action_reports_through_it)
{
  Graph graph(with_corridor(true));
  ASSERT_TRUE(graph.bring_up());

  graph.publish(free_grid());

  ASSERT_TRUE(wait_for([&]() {return !graph.state().applied().empty();}));

  const auto applied = graph.state().applied().front();
  EXPECT_EQ(applied.action, "inspect_r1");
  EXPECT_EQ(applied.outcome, "e-inspect-clear");
  EXPECT_TRUE(graph.state().announced().empty());
}

TEST(perception_node, the_same_map_again_says_nothing_again)
{
  Graph graph(with_corridor(false));
  ASSERT_TRUE(graph.bring_up());

  const auto grid = free_grid();
  graph.publish(grid);
  ASSERT_TRUE(wait_for([&]() {return !graph.state().announced().empty();}));

  for (int again = 0; again < 3; ++again) {
    graph.publish(grid);
  }

  // Nothing changed about the corridor, so nothing more is said about it. An
  // announcement repeated is harmless in the model; a sensing action applied
  // twice is not, and both routes follow the same rule.
  std::this_thread::sleep_for(500ms);
  EXPECT_EQ(graph.state().announced().size(), 1u);
}

TEST(perception_node, an_obstacle_appearing_is_reported_as_a_change)
{
  Graph graph(with_corridor(false));
  ASSERT_TRUE(graph.bring_up());

  graph.publish(free_grid());
  ASSERT_TRUE(wait_for([&]() {return !graph.state().announced().empty();}));

  auto blocked = free_grid();
  blocked.data[5] = 100;
  graph.publish(blocked);

  ASSERT_TRUE(wait_for([&]() {return graph.state().announced().size() == 2u;}));
  EXPECT_EQ(graph.state().announced().back(), "blocked");
}

TEST(perception_node, a_region_that_goes_back_to_undecided_is_reported_again_when_it_resolves)
{
  Graph graph(with_corridor(false));
  ASSERT_TRUE(graph.bring_up());

  graph.publish(free_grid());
  ASSERT_TRUE(wait_for([&]() {return graph.state().announced().size() == 1u;}));

  auto unobserved = free_grid();
  unobserved.data[5] = -1;
  graph.publish(unobserved);

  // The map is subscribed with a depth of one, the way a latched map is: a
  // grid published while the callback is busy replaces the one waiting rather
  // than queueing behind it. Publishing the two back to back would be a test
  // of that queue and not of the node, so the middle grid is given time to
  // arrive on its own.
  std::this_thread::sleep_for(500ms);

  graph.publish(free_grid());

  ASSERT_TRUE(wait_for([&]() {return graph.state().announced().size() == 2u;}));
  EXPECT_EQ(graph.state().announced().back(), "(not blocked)");
}

TEST(perception_node, a_region_the_state_did_not_take_is_offered_again)
{
  Graph graph(with_corridor(false));
  graph.state().refuse();
  ASSERT_TRUE(graph.bring_up());

  graph.publish(free_grid());
  ASSERT_TRUE(wait_for([&]() {return !graph.state().announced().empty();}));

  // A refusal is the state disagreeing with the map, which is a reason to
  // replan rather than to keep asking, so the region counts as reported and
  // the same grid does not produce a second call.
  const auto after_refusal = graph.state().announced().size();
  graph.publish(free_grid());
  std::this_thread::sleep_for(500ms);

  EXPECT_EQ(graph.state().announced().size(), after_refusal);
}

TEST(perception_node, a_region_with_boxes_that_do_not_come_in_fours_refuses_to_configure)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
  {
    rclcpp::Parameter("regions", std::vector<std::string>{"corridor"}),
    rclcpp::Parameter("corridor.boxes", std::vector<double>{0.0, 0.0, 2.0}),
  });

  auto node = std::make_shared<plansys2::EpistemicPerceptionNode>(options);

  EXPECT_EQ(
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE).id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST(perception_node, half_a_sensing_binding_refuses_to_configure)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
  {
    rclcpp::Parameter("regions", std::vector<std::string>{"corridor"}),
    rclcpp::Parameter("corridor.boxes", std::vector<double>{0.0, 0.0, 2.0, 2.0}),
    rclcpp::Parameter("corridor.sensing_action", "inspect_r1"),
    rclcpp::Parameter("corridor.outcome_when_clear", "e-inspect-clear"),
  });

  auto node = std::make_shared<plansys2::EpistemicPerceptionNode>(options);

  EXPECT_EQ(
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE).id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
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
