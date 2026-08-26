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

#include <cmath>
#include <vector>

#include "gtest/gtest.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "plansys2_epistemic_perception/regions.hpp"

namespace
{

/// A 4x4 grid of 1 m cells with its origin at the origin, every cell free.
///
/// Cell centres are therefore at 0.5, 1.5, 2.5 and 3.5 in both axes, which is
/// what the box arithmetic below is written against.
nav_msgs::msg::OccupancyGrid free_grid(std::uint32_t width = 4, std::uint32_t height = 4)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.info.resolution = 1.0;
  grid.info.width = width;
  grid.info.height = height;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(static_cast<std::size_t>(width) * height, 0);
  return grid;
}

plansys2::Region box_region(double min_x, double min_y, double max_x, double max_y)
{
  return plansys2::Region{"corridor", {plansys2::Box{min_x, min_y, max_x, max_y}}};
}

}  // namespace

TEST(regions, a_box_takes_the_cells_whose_centres_it_covers)
{
  const auto grid = free_grid();

  // Covers the centres at 0.5 and 1.5 in both axes: the bottom-left 2x2.
  const auto cells = plansys2::cells_of(box_region(0.0, 0.0, 2.0, 2.0), grid.info);

  EXPECT_EQ(cells, (std::vector<std::size_t>{0, 1, 4, 5}));
}

TEST(regions, a_box_smaller_than_a_cell_takes_the_cell_it_is_inside)
{
  const auto grid = free_grid();

  const auto cells = plansys2::cells_of(box_region(2.4, 2.4, 2.6, 2.6), grid.info);

  EXPECT_EQ(cells, (std::vector<std::size_t>{10}));
}

TEST(regions, a_centre_exactly_on_the_edge_is_inside)
{
  const auto grid = free_grid();

  // The box ends exactly on the centre of column 1, which is in.
  const auto cells = plansys2::cells_of(box_region(0.5, 0.5, 1.5, 0.5), grid.info);

  EXPECT_EQ(cells, (std::vector<std::size_t>{0, 1}));
}

TEST(regions, a_box_reaching_off_the_map_takes_the_part_that_is_on_it)
{
  const auto grid = free_grid();

  const auto cells = plansys2::cells_of(box_region(-10.0, -10.0, 0.6, 0.6), grid.info);

  EXPECT_EQ(cells, (std::vector<std::size_t>{0}));
}

TEST(regions, a_box_entirely_off_the_map_takes_nothing)
{
  const auto grid = free_grid();

  const plansys2::Region region{"elsewhere", {plansys2::Box{20.0, 20.0, 30.0, 30.0}}};

  EXPECT_TRUE(plansys2::cells_of(region, grid.info).empty());
  EXPECT_FALSE(plansys2::on_grid(region, grid.info));
}

TEST(regions, overlapping_boxes_count_a_cell_once)
{
  const auto grid = free_grid();

  const plansys2::Region region{"bend",
    {plansys2::Box{0.0, 0.0, 2.0, 1.0}, plansys2::Box{1.0, 0.0, 3.0, 1.0}}};

  const auto cells = plansys2::cells_of(region, grid.info);

  EXPECT_EQ(cells, (std::vector<std::size_t>{0, 1, 2}));
}

TEST(regions, a_rotated_origin_moves_the_cells_the_region_covers)
{
  auto grid = free_grid();

  // A quarter turn about z: the cell that was at (0.5, 0.5) in the map frame
  // is now at (-0.5, 0.5), and a box over the old place covers nothing.
  grid.info.origin.orientation.w = std::cos(M_PI / 4.0);
  grid.info.origin.orientation.z = std::sin(M_PI / 4.0);

  EXPECT_TRUE(plansys2::cells_of(box_region(0.1, 0.1, 0.9, 0.9), grid.info).empty());
  EXPECT_EQ(
    plansys2::cells_of(box_region(-0.9, 0.1, -0.1, 0.9), grid.info),
    (std::vector<std::size_t>{0}));
}

TEST(regions, a_region_of_free_cells_is_clear)
{
  const auto grid = free_grid();

  EXPECT_EQ(plansys2::classify(box_region(0.0, 0.0, 2.0, 2.0), grid), plansys2::RegionClass::Clear);
}

TEST(regions, one_occupied_cell_blocks_the_region)
{
  auto grid = free_grid();
  grid.data[5] = 100;

  EXPECT_EQ(
    plansys2::classify(box_region(0.0, 0.0, 2.0, 2.0), grid),
    plansys2::RegionClass::Blocked);
}

TEST(regions, one_unobserved_cell_leaves_the_region_undecided)
{
  auto grid = free_grid();
  grid.data[5] = -1;

  EXPECT_EQ(
    plansys2::classify(box_region(0.0, 0.0, 2.0, 2.0), grid),
    plansys2::RegionClass::Unknown);
}

TEST(regions, a_cell_seen_without_being_settled_counts_as_unobserved)
{
  auto grid = free_grid();
  grid.data[5] = 50;

  EXPECT_EQ(
    plansys2::classify(box_region(0.0, 0.0, 2.0, 2.0), grid),
    plansys2::RegionClass::Unknown);
}

TEST(regions, occupied_beats_unobserved)
{
  auto grid = free_grid();
  grid.data[1] = -1;
  grid.data[5] = 100;

  EXPECT_EQ(
    plansys2::classify(box_region(0.0, 0.0, 2.0, 2.0), grid),
    plansys2::RegionClass::Blocked);
}

// The four values around the two thresholds, pinned so the band cannot drift.
//
// Both bounds are strict: free is below free_below, occupied is above
// occupied_above, and [free_below, occupied_above] inclusive is the band of
// cells that have been seen without being settled. epistemic_slam's map fusion
// classifies cells by the same rule, and a disagreement between the two would
// mean one grid reading as two different situations.
TEST(regions, the_free_bound_is_strict)
{
  auto grid = free_grid();

  // Default thresholds: free_below 25, occupied_above 65.
  grid.data[5] = 24;
  EXPECT_EQ(
    plansys2::classify(box_region(0.0, 0.0, 2.0, 2.0), grid),
    plansys2::RegionClass::Clear);

  grid.data[5] = 25;
  EXPECT_EQ(
    plansys2::classify(box_region(0.0, 0.0, 2.0, 2.0), grid),
    plansys2::RegionClass::Unknown);
}

TEST(regions, the_occupied_bound_is_strict)
{
  auto grid = free_grid();

  // Exactly on the threshold the cell has been seen without being settled.
  // Reporting it as blocked would assert knowledge the map does not carry, and
  // unlike an undecided region -- which reports nothing -- a blocked region
  // fires an outcome the epistemic state cannot take back.
  grid.data[5] = 65;
  EXPECT_EQ(
    plansys2::classify(box_region(0.0, 0.0, 2.0, 2.0), grid),
    plansys2::RegionClass::Unknown);

  grid.data[5] = 66;
  EXPECT_EQ(
    plansys2::classify(box_region(0.0, 0.0, 2.0, 2.0), grid),
    plansys2::RegionClass::Blocked);
}

TEST(regions, the_bounds_stay_strict_when_the_thresholds_move)
{
  auto grid = free_grid();

  const plansys2::Thresholds moved{40, 80};

  grid.data[5] = 39;
  EXPECT_EQ(
    plansys2::classify(box_region(0.0, 0.0, 2.0, 2.0), grid, moved),
    plansys2::RegionClass::Clear);

  grid.data[5] = 40;
  EXPECT_EQ(
    plansys2::classify(box_region(0.0, 0.0, 2.0, 2.0), grid, moved),
    plansys2::RegionClass::Unknown);

  grid.data[5] = 80;
  EXPECT_EQ(
    plansys2::classify(box_region(0.0, 0.0, 2.0, 2.0), grid, moved),
    plansys2::RegionClass::Unknown);

  grid.data[5] = 81;
  EXPECT_EQ(
    plansys2::classify(box_region(0.0, 0.0, 2.0, 2.0), grid, moved),
    plansys2::RegionClass::Blocked);
}

TEST(regions, thresholds_move_the_line)
{
  auto grid = free_grid();
  grid.data[5] = 50;

  const plansys2::Thresholds generous{60, 90};

  EXPECT_EQ(
    plansys2::classify(box_region(0.0, 0.0, 2.0, 2.0), grid, generous),
    plansys2::RegionClass::Clear);
}

TEST(regions, a_region_that_is_not_on_the_map_is_undecided)
{
  const auto grid = free_grid();

  const plansys2::Region region{"elsewhere", {plansys2::Box{20.0, 20.0, 30.0, 30.0}}};

  EXPECT_EQ(plansys2::classify(region, grid), plansys2::RegionClass::Unknown);
}

TEST(regions, a_grid_whose_data_contradicts_its_header_decides_nothing)
{
  auto grid = free_grid();
  grid.data.resize(3);

  EXPECT_EQ(
    plansys2::classify(box_region(0.0, 0.0, 2.0, 2.0), grid),
    plansys2::RegionClass::Unknown);
}
