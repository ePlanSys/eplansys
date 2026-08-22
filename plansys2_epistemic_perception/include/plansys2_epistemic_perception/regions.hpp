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

#ifndef PLANSYS2_EPISTEMIC_PERCEPTION__REGIONS_HPP_
#define PLANSYS2_EPISTEMIC_PERCEPTION__REGIONS_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"

namespace plansys2
{

/// An axis-aligned box in the map frame, in metres.
///
/// Regions are given in map coordinates rather than in cells because a map is
/// re-gridded whenever it grows: slam_toolbox moves the origin and changes the
/// width as the robot explores, and a region written in cell indices would
/// quietly come to mean somewhere else. A box in metres means the same place
/// before and after.
struct Box
{
  double min_x{0.0};
  double min_y{0.0};
  double max_x{0.0};
  double max_y{0.0};
};

/// A named part of the world that a proposition can be about.
///
/// The union of the boxes, not their intersection: a corridor with a bend is
/// two boxes, and a cell in either of them is in the corridor.
struct Region
{
  std::string name;
  std::vector<Box> boxes;
};

/// Where the line falls between free, occupied and undecided.
///
/// The defaults are the convention nav2 and slam_toolbox use for a costmap
/// read back as an OccupancyGrid: below 25 is free enough to drive, above 65
/// is an obstacle, and the band between is a cell that has been seen without
/// being settled. A cell in that band counts as unobserved on purpose --
/// reporting it as free would hand an agent knowledge it does not have.
struct Thresholds
{
  std::int8_t free_below{25};
  std::int8_t occupied_above{65};
};

/// What a grid says about a region taken as a whole.
///
/// This is the part that cannot be read off a cell. An occupancy grid answers
/// a question per cell; a proposition is about a region, and the two are
/// bridged by a quantifier -- which one is a modelling decision, made here and
/// nowhere else. `Clear` is universal (every cell free), `Blocked` existential
/// (some cell occupied), and anything else is `Unknown`.
enum class RegionClass
{
  Unknown,
  Clear,
  Blocked,
};

/// The name of a class, for logs and errors.
const char * to_string(RegionClass region_class);

/// The cells of the grid that fall inside the region.
///
/// Cells are counted once however many boxes cover them, and boxes that fall
/// partly or wholly outside the grid contribute only the part that is on it.
/// An empty result means the region is not on this map at all, which is not
/// the same as the region being unobserved, and the caller has to tell them
/// apart: classify() reports both as Unknown, on_grid() distinguishes them.
std::vector<std::size_t> cells_of(
  const Region & region,
  const nav_msgs::msg::MapMetaData & info);

/// Whether any of the region falls on this map.
bool on_grid(const Region & region, const nav_msgs::msg::MapMetaData & info);

/// What the grid says about the region.
///
/// Occupied beats unobserved: a region with an obstacle in it is Blocked even
/// when part of it has never been seen, because no further observation can
/// make it passable. Unobserved beats free for the same reason in the other
/// direction -- a region is Clear only when every one of its cells has been
/// observed and settled as free.
RegionClass classify(
  const Region & region,
  const nav_msgs::msg::OccupancyGrid & grid,
  const Thresholds & thresholds = {});

}  // namespace plansys2

#endif  // PLANSYS2_EPISTEMIC_PERCEPTION__REGIONS_HPP_
