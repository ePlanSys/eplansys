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

#ifndef PLANSYS2_EPISTEMIC_PERCEPTION__EPISTEMICPERCEPTIONNODE_HPP_
#define PLANSYS2_EPISTEMIC_PERCEPTION__EPISTEMICPERCEPTIONNODE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "plansys2_epistemic_executor/EpistemicStateClient.hpp"
#include "plansys2_epistemic_perception/regions.hpp"
#include "plansys2_epistemic_perception/translation.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace plansys2
{

/**
 * @class plansys2::EpistemicPerceptionNode
 * @brief Turns what a robot sees into something the epistemic state can hold.
 *
 * The epistemic state is advanced by executed actions, not by observing the
 * world, and that is deliberate: it is a model of what the agents know, and
 * knowledge changes by events with an event model, not by a topic. But
 * something has to close the loop with the map, or a sensing action in a
 * policy has no way to report which outcome it actually saw.
 *
 * That is this node. It holds the region definitions, watches an occupancy
 * grid, and when a region resolves it says so in the task's own vocabulary.
 *
 * Two things stand between a grid and a formula, and both live here:
 *
 *   the quantifier   A cell is free or occupied; a region is Clear when every
 *                    cell in it is free and Blocked when any one is occupied.
 *                    Which quantifier goes with which class is a modelling
 *                    decision, taken in classify() and written down there.
 *
 *   the vocabulary   A region is a place on a map; an atom is a name in a
 *                    grounded task. `corridor` becomes `blocked` or
 *                    `clear_corridor` by configuration, because only the
 *                    domain knows which way its own atom points.
 *
 * How it reports depends on why it is reporting, which is the difference a
 * public announcement makes:
 *
 *   ApplyAction  The region is bound to a sensing action. The observation
 *                belongs to that action, the state has its event model, and
 *                the update is the one the planner accounted for.
 *
 *   Announce     No sensing action. The information arrived outside the plan
 *                -- an operator, or two robots reconciling their maps when a
 *                link came back -- and everyone gets it at once.
 *
 * It reports on change and not on every map. A grid arrives several times a
 * second and says the same thing each time; announcing the same formula twice
 * is not wrong in the model, since restricting to worlds where it already
 * holds changes nothing, but applying a sensing action twice is, and the two
 * routes had better follow the same rule.
 */
class EpistemicPerceptionNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /// Options carry the parameter overrides a test or a launch file sets. A
  /// region's own settings are declared once the list of regions is known,
  /// which is at configure time, and an override waiting in the options is
  /// what gives them a value then.
  explicit EpistemicPerceptionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);

  /// A region as configured, with what it is about and what it last said.
  struct Watched
  {
    Region region;
    RegionVocabulary about;
    RegionClass last{RegionClass::Unknown};
    bool reported{false};

    /// Attempts spent on a refusal that reads as a race rather than as a
    /// disagreement. See the note in tell().
    int retries{0};
  };

  /// The configured value of `applicability_retries`.
  int applicability_retries_{kApplicabilityRetries};

  /// How many grids to keep offering an observation the state was not yet
  /// ready for, as a default. Long enough for the executor to apply the action
  /// before it, short enough not to mask a real conflict.
  ///
  /// The right number is a property of the scenario, not of this node: it has
  /// to exceed the gap between a region resolving and the model believing the
  /// robot is where it can resolve it, and that gap is however long the drive
  /// before the sensing action takes. Forty grids is a few seconds of race in
  /// a small building; a warehouse aisle a robot needs a minute to reach wants
  /// more, and would otherwise give up on a reading that was never in dispute.
  /// Hence a parameter, `applicability_retries`, with this as its default.
  static constexpr int kApplicabilityRetries = 40;

  /// The configured regions, for tests and for a node that wants to report
  /// what it is watching.
  const std::vector<Watched> & watched() const {return watched_;}

private:
  /// Declare one of a region's settings, unless it is already declared.
  ///
  /// Configure is reachable more than once over a node's life, and
  /// declare_parameter throws the second time. The descriptor is
  /// dynamically typed so that the value's own type does not decide whether
  /// it is accepted -- read_boxes decides that, and can say why.
  template<typename T>
  void declare_region_parameter(const std::string & name, const T & fallback);

  /// Read a region's boxes, from either a double or an integer array.
  bool read_boxes(const std::string & region, std::vector<double> & boxes);

  /// Read `regions` and everything declared under each region's name.
  bool read_regions();

  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  /// Classify every region against this grid and report what changed.
  void report(const nav_msgs::msg::OccupancyGrid & grid);

  /// Say one thing to the state. Returns false when the state did not take it,
  /// which leaves the region unreported so that the next grid tries again.
  bool tell(Watched & watched, const Emission & emission);

  std::vector<Watched> watched_;
  Thresholds thresholds_;
  std::chrono::nanoseconds call_timeout_{std::chrono::seconds(5)};
  bool active_{false};

  EpistemicStateClient::Ptr state_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
};

}  // namespace plansys2

#endif  // PLANSYS2_EPISTEMIC_PERCEPTION__EPISTEMICPERCEPTIONNODE_HPP_
