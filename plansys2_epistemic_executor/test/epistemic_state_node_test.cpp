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

// The epistemic state as the problem expert's counterpart: it holds a goal
// that can be asked for and replaced, and it can be told something publicly.
// These run against the real node over real services, because what is being
// checked is the conversation, not the model checking underneath it.

#include <memory>
#include <string>
#include <thread>

#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/state.hpp"
#include "plansys2_epistemic_executor/EpistemicStateNode.hpp"
#include "plansys2_epistemic_msgs/srv/announce.hpp"
#include "plansys2_epistemic_msgs/srv/check_formula.hpp"
#include "plansys2_epistemic_msgs/srv/get_goal.hpp"
#include "plansys2_epistemic_msgs/srv/set_goal.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace
{

/// muddy-children-2: two children, both muddy, neither knowing it of itself.
/// Four worlds, one designated, and a goal that does not hold to begin with —
/// which is what makes it worth asking questions of.
std::string task_file()
{
  return std::string(EPISTEMIC_TASK_DIR) + "/muddy-children-2.json";
}

}  // namespace

class EpistemicStateNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<plansys2::EpistemicStateNode>();
    node_->set_parameter(rclcpp::Parameter("task_file", task_file()));

    ASSERT_EQ(
      node_->configure().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(
      node_->activate().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    client_node_ = rclcpp::Node::make_shared("epistemic_state_test_client");

    executor_.add_node(node_->get_node_base_interface());
    executor_.add_node(client_node_);

    spinning_ = true;
    spin_thread_ = std::thread(
      [this]() {
        while (spinning_) {
          executor_.spin_some();
          std::this_thread::sleep_for(1ms);
        }
      });
  }

  void TearDown() override
  {
    spinning_ = false;
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
  }

  /// Call a service and return the response, or null when it never answered.
  template<typename ServiceT>
  typename ServiceT::Response::SharedPtr call(
    const std::string & name, typename ServiceT::Request::SharedPtr request)
  {
    auto client = client_node_->create_client<ServiceT>(name);
    if (!client->wait_for_service(3s)) {
      return nullptr;
    }
    auto future = client->async_send_request(request);
    if (future.wait_for(3s) != std::future_status::ready) {
      return nullptr;
    }
    return future.get();
  }

  std::shared_ptr<plansys2::EpistemicStateNode> node_;
  rclcpp::Node::SharedPtr client_node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;
  std::atomic<bool> spinning_{false};
};

TEST_F(EpistemicStateNodeTest, ReportsTheTasksOwnGoalUntilOneIsSet)
{
  using GetGoal = plansys2_epistemic_msgs::srv::GetGoal;

  const auto response = call<GetGoal>(
    "epistemic_state/get_goal", std::make_shared<GetGoal::Request>());

  ASSERT_NE(response, nullptr);
  ASSERT_TRUE(response->success) << response->error;
  EXPECT_TRUE(response->from_task);
  EXPECT_FALSE(response->goal.empty());
  // Both children must come to know whether they are muddy. Nobody has asked
  // anything yet, so they do not.
  EXPECT_FALSE(response->holds);
}

TEST_F(EpistemicStateNodeTest, AGoalCanBeReplacedAndPutBack)
{
  using GetGoal = plansys2_epistemic_msgs::srv::GetGoal;
  using SetGoal = plansys2_epistemic_msgs::srv::SetGoal;

  const auto original = call<GetGoal>(
    "epistemic_state/get_goal", std::make_shared<GetGoal::Request>());
  ASSERT_NE(original, nullptr);

  auto set = std::make_shared<SetGoal::Request>();
  set->goal = "(Kw c1 muddy_c1)";
  const auto set_response = call<SetGoal>("epistemic_state/set_goal", set);

  ASSERT_NE(set_response, nullptr);
  ASSERT_TRUE(set_response->success) << set_response->error;
  EXPECT_FALSE(set_response->holds);

  auto after = call<GetGoal>(
    "epistemic_state/get_goal", std::make_shared<GetGoal::Request>());
  ASSERT_NE(after, nullptr);
  EXPECT_FALSE(after->from_task);
  EXPECT_NE(after->goal, original->goal);

  // Empty restores the task's own, rather than leaving the state with none.
  auto restore = std::make_shared<SetGoal::Request>();
  const auto restored = call<SetGoal>("epistemic_state/set_goal", restore);
  ASSERT_NE(restored, nullptr);
  ASSERT_TRUE(restored->success) << restored->error;

  after = call<GetGoal>(
    "epistemic_state/get_goal", std::make_shared<GetGoal::Request>());
  ASSERT_NE(after, nullptr);
  EXPECT_TRUE(after->from_task);
  EXPECT_EQ(after->goal, original->goal);
}

TEST_F(EpistemicStateNodeTest, AGoalTheTaskCannotExpressIsRefused)
{
  using SetGoal = plansys2_epistemic_msgs::srv::SetGoal;

  auto request = std::make_shared<SetGoal::Request>();
  request->goal = "(Kw nobody muddy_c1)";

  const auto response = call<SetGoal>("epistemic_state/set_goal", request);

  ASSERT_NE(response, nullptr);
  // Catching it here is the point of the goal living with the vocabulary: the
  // alternative is a planning request that fails later with nothing to point
  // at.
  EXPECT_FALSE(response->success);
  EXPECT_FALSE(response->error.empty());
}

TEST_F(EpistemicStateNodeTest, AnnouncingMakesItKnown)
{
  using Announce = plansys2_epistemic_msgs::srv::Announce;
  using CheckFormula = plansys2_epistemic_msgs::srv::CheckFormula;

  // c1 does not know it is muddy: it cannot tell the world where it is from
  // the one where it is not.
  auto before = std::make_shared<CheckFormula::Request>();
  before->formula = "(K c1 muddy_c1)";
  const auto unknown = call<CheckFormula>("epistemic_state/check_formula", before);
  ASSERT_NE(unknown, nullptr);
  ASSERT_TRUE(unknown->success) << unknown->error;
  EXPECT_FALSE(unknown->holds);

  auto request = std::make_shared<Announce::Request>();
  request->formula = "muddy_c1";
  const auto response = call<Announce>("epistemic_state/announce", request);

  ASSERT_NE(response, nullptr);
  ASSERT_TRUE(response->success) << response->error;
  // Four worlds, two of them muddy for c1: announcing rules the other two out.
  EXPECT_EQ(response->num_worlds, 2u);
  EXPECT_GT(response->num_designated, 0u);

  // And now it does know, which is what an announcement is for.
  const auto known = call<CheckFormula>("epistemic_state/check_formula", before);
  ASSERT_NE(known, nullptr);
  ASSERT_TRUE(known->success) << known->error;
  EXPECT_TRUE(known->holds);
}

TEST_F(EpistemicStateNodeTest, AnnouncingSomethingImpossibleIsRefused)
{
  using Announce = plansys2_epistemic_msgs::srv::Announce;

  auto request = std::make_shared<Announce::Request>();
  request->formula = "muddy_c1";
  ASSERT_TRUE(call<Announce>("epistemic_state/announce", request)->success);

  // Both children are muddy in the designated world, so after the first
  // announcement there is no world left where c1 is clean. Emptying the model
  // would leave the state believing nothing at all.
  request->formula = "(not muddy_c1)";
  const auto response = call<Announce>("epistemic_state/announce", request);

  ASSERT_NE(response, nullptr);
  EXPECT_FALSE(response->success);
  EXPECT_NE(response->error.find("no possible world"), std::string::npos)
    << response->error;
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
