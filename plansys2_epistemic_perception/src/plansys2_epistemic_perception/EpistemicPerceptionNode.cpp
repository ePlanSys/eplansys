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

#include "plansys2_epistemic_perception/EpistemicPerceptionNode.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace plansys2
{

using namespace std::chrono_literals;  // NOLINT(build/namespaces)

EpistemicPerceptionNode::EpistemicPerceptionNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("epistemic_perception", options)
{
  declare_parameter<std::string>("map_topic", "/map");
  declare_parameter<std::vector<std::string>>("regions", std::vector<std::string>{});
  declare_parameter<int>("free_below", Thresholds{}.free_below);
  declare_parameter<int>("occupied_above", Thresholds{}.occupied_above);
  declare_parameter<double>("call_timeout", 5.0);
}

bool EpistemicPerceptionNode::read_regions()
{
  watched_.clear();

  const auto names = get_parameter("regions").as_string_array();

  for (const auto & name : names) {
    // Declared here rather than in the constructor because there is no list of
    // regions until the parameters are read, and a region's own settings are
    // named after it.
    declare_parameter<std::vector<double>>(name + ".boxes", std::vector<double>{});
    declare_parameter<std::string>(name + ".atom", "");
    declare_parameter<std::string>(name + ".predicate", "clear");
    declare_parameter<bool>(name + ".atom_true_when_clear", true);
    declare_parameter<std::string>(name + ".sensing_action", "");
    declare_parameter<std::string>(name + ".outcome_when_clear", "");
    declare_parameter<std::string>(name + ".outcome_when_blocked", "");

    const auto boxes = get_parameter(name + ".boxes").as_double_array();
    if (boxes.empty() || boxes.size() % 4 != 0) {
      RCLCPP_ERROR(
        get_logger(),
        "[epistemic_perception] region '%s' needs boxes as [min_x, min_y, max_x, max_y] "
        "per box, and has %zu numbers.", name.c_str(), boxes.size());
      return false;
    }

    Watched watched;
    watched.region.name = name;
    for (std::size_t at = 0; at < boxes.size(); at += 4) {
      watched.region.boxes.push_back(
        Box{boxes[at], boxes[at + 1], boxes[at + 2], boxes[at + 3]});
    }

    const auto atom = get_parameter(name + ".atom").as_string();
    const auto predicate = get_parameter(name + ".predicate").as_string();
    watched.about.atom = atom.empty() ? default_atom(name, predicate) : atom;
    watched.about.atom_true_when_clear =
      get_parameter(name + ".atom_true_when_clear").as_bool();

    const auto action = get_parameter(name + ".sensing_action").as_string();
    if (!action.empty()) {
      SensingBinding sensing;
      sensing.epistemic_action = action;
      sensing.outcome_when_clear = get_parameter(name + ".outcome_when_clear").as_string();
      sensing.outcome_when_blocked = get_parameter(name + ".outcome_when_blocked").as_string();

      // A sensing action needs an event per outcome. Half a binding would send
      // one observation and silently drop the other, which is worse than
      // refusing to start.
      if (sensing.outcome_when_clear.empty() || sensing.outcome_when_blocked.empty()) {
        RCLCPP_ERROR(
          get_logger(),
          "[epistemic_perception] region '%s' binds sensing action '%s' without an outcome "
          "for both cases.", name.c_str(), action.c_str());
        return false;
      }

      watched.about.sensing = sensing;
    }

    const std::string reports_by = watched.about.sensing.has_value() ?
      "sensing " + watched.about.sensing->epistemic_action :
      "announcing " + watched.about.atom;

    RCLCPP_INFO(
      get_logger(), "[epistemic_perception] watching '%s': %zu box(es), %s",
      name.c_str(), watched.region.boxes.size(), reports_by.c_str());

    watched_.push_back(std::move(watched));
  }

  return true;
}

EpistemicPerceptionNode::CallbackReturnT
EpistemicPerceptionNode::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;

  if (!read_regions()) {
    return CallbackReturnT::FAILURE;
  }

  thresholds_.free_below = static_cast<std::int8_t>(get_parameter("free_below").as_int());
  thresholds_.occupied_above = static_cast<std::int8_t>(get_parameter("occupied_above").as_int());
  if (thresholds_.free_below > thresholds_.occupied_above) {
    RCLCPP_ERROR(
      get_logger(),
      "[epistemic_perception] free_below (%d) is above occupied_above (%d), which leaves no "
      "band for a cell that has been seen without being settled.",
      thresholds_.free_below, thresholds_.occupied_above);
    return CallbackReturnT::FAILURE;
  }

  call_timeout_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(get_parameter("call_timeout").as_double()));

  state_ = std::make_shared<EpistemicStateClient>("epistemic_perception_state_client");

  // The map is latched: a grid published before this node came up is still the
  // current one, and a perception layer that misses it waits for a robot to
  // move before it can say anything.
  const auto map_topic = get_parameter("map_topic").as_string();
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic, rclcpp::QoS(1).transient_local(),
    std::bind(&EpistemicPerceptionNode::map_callback, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(), "[%s] Configured: %zu region(s) on %s",
    get_name(), watched_.size(), map_topic.c_str());
  return CallbackReturnT::SUCCESS;
}

EpistemicPerceptionNode::CallbackReturnT
EpistemicPerceptionNode::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  active_ = true;

  RCLCPP_INFO(get_logger(), "[%s] Activated", get_name());
  return CallbackReturnT::SUCCESS;
}

EpistemicPerceptionNode::CallbackReturnT
EpistemicPerceptionNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  active_ = false;

  RCLCPP_INFO(get_logger(), "[%s] Deactivated", get_name());
  return CallbackReturnT::SUCCESS;
}

void EpistemicPerceptionNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  // Deactivated is not the same as unsubscribed: the grid keeps arriving and
  // is kept out of the model here, so that activating again starts from what
  // the map says now rather than from what it said when the node last ran.
  if (!active_) {
    return;
  }

  report(*msg);
}

void EpistemicPerceptionNode::report(const nav_msgs::msg::OccupancyGrid & grid)
{
  for (auto & watched : watched_) {
    const auto now = classify(watched.region, grid, thresholds_);

    // Same answer as last time, and last time got through: nothing happened.
    if (now == watched.last && watched.reported) {
      continue;
    }

    watched.last = now;

    if (now == RegionClass::Unknown) {
      // A region can go back to undecided -- the map grows, and the part that
      // arrived has never been seen. There is nothing to say about that, and
      // nothing said earlier is retracted: the model has no operation for
      // taking knowledge back.
      watched.reported = false;
      continue;
    }

    const auto emission = emission_for(now, watched.about);
    watched.reported = tell(watched, emission);
  }
}

bool EpistemicPerceptionNode::tell(const Watched & watched, const Emission & emission)
{
  // This blocks the callback until the state answers or the timeout runs out.
  // A map arrives several times a second and a region resolves once, so the
  // cost is paid on the transition rather than on every grid; the alternative,
  // an asynchronous call, would need the answer to be matched back to a region
  // that may have changed class in the meantime.
  EpistemicStateClient::Answer answer;

  switch (emission.kind) {
    case Emission::Kind::Announce:
      answer = state_->announce(emission.formula, call_timeout_);
      break;

    case Emission::Kind::ApplyAction:
      answer = state_->apply_action(emission.action, emission.outcome, call_timeout_);
      break;

    case Emission::Kind::Nothing:
    default:
      return false;
  }

  if (!answer.answered) {
    RCLCPP_WARN(
      get_logger(), "[epistemic_perception] '%s' is %s, and the state did not answer: %s",
      watched.region.name.c_str(), to_string(watched.last), answer.error.c_str());
    return false;
  }

  if (!answer.success) {
    // The state took the call and refused it: an announcement that holds
    // nowhere, or an outcome the model cannot account for. Both mean the model
    // and the map disagree, which is a reason to replan and not a reason to
    // keep trying, so the region counts as reported.
    RCLCPP_ERROR(
      get_logger(), "[epistemic_perception] '%s' is %s, and the state refused it: %s",
      watched.region.name.c_str(), to_string(watched.last), answer.error.c_str());
    return true;
  }

  if (emission.kind == Emission::Kind::Announce) {
    RCLCPP_INFO(
      get_logger(), "[epistemic_perception] '%s' is %s: announced %s",
      watched.region.name.c_str(), to_string(watched.last), emission.formula.c_str());
  } else {
    RCLCPP_INFO(
      get_logger(), "[epistemic_perception] '%s' is %s: %s observed %s",
      watched.region.name.c_str(), to_string(watched.last),
      emission.action.c_str(), emission.outcome.c_str());
  }

  return true;
}

}  // namespace plansys2
