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

#include "plansys2_epistemic_perception/regions.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace plansys2
{

namespace
{

/// The map origin's rotation about z, in radians.
double origin_yaw(const nav_msgs::msg::MapMetaData & info)
{
  const auto & q = info.origin.orientation;
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

bool contains(const Box & box, double x, double y)
{
  const double min_x = std::min(box.min_x, box.max_x);
  const double max_x = std::max(box.min_x, box.max_x);
  const double min_y = std::min(box.min_y, box.max_y);
  const double max_y = std::max(box.min_y, box.max_y);

  return x >= min_x && x <= max_x && y >= min_y && y <= max_y;
}

/// The cells of one box, for a grid whose origin is not rotated.
///
/// The common case, and the only one that can be answered without walking the
/// whole map: the box maps to a rectangle of indices.
void cells_of_box_axis_aligned(
  const Box & box,
  const nav_msgs::msg::MapMetaData & info,
  std::vector<bool> & taken,
  std::vector<std::size_t> & cells)
{
  const double resolution = info.resolution;
  const double origin_x = info.origin.position.x;
  const double origin_y = info.origin.position.y;

  const double min_x = std::min(box.min_x, box.max_x);
  const double max_x = std::max(box.min_x, box.max_x);
  const double min_y = std::min(box.min_y, box.max_y);
  const double max_y = std::max(box.min_y, box.max_y);

  const auto width = static_cast<std::int64_t>(info.width);
  const auto height = static_cast<std::int64_t>(info.height);

  // A cell is in the box when its centre is, which is why the half cell is
  // there: it makes a box drawn along cell boundaries take the cells it covers
  // rather than the ones it touches. The tolerance is for the boundary case,
  // where a centre falls exactly on an edge and rounding decides the answer.
  constexpr double tolerance = 1e-9;
  auto first_at_or_after = [resolution](double distance) {
      return static_cast<std::int64_t>(std::ceil(distance / resolution - 0.5 - tolerance));
    };
  auto last_at_or_before = [resolution](double distance) {
      return static_cast<std::int64_t>(std::floor(distance / resolution - 0.5 + tolerance));
    };

  std::int64_t col_begin = first_at_or_after(min_x - origin_x);
  std::int64_t col_end = last_at_or_before(max_x - origin_x);
  std::int64_t row_begin = first_at_or_after(min_y - origin_y);
  std::int64_t row_end = last_at_or_before(max_y - origin_y);

  col_begin = std::max<std::int64_t>(col_begin, 0);
  row_begin = std::max<std::int64_t>(row_begin, 0);
  col_end = std::min<std::int64_t>(col_end, width - 1);
  row_end = std::min<std::int64_t>(row_end, height - 1);

  for (std::int64_t row = row_begin; row <= row_end; ++row) {
    for (std::int64_t col = col_begin; col <= col_end; ++col) {
      const auto index = static_cast<std::size_t>(row * width + col);
      if (!taken[index]) {
        taken[index] = true;
        cells.push_back(index);
      }
    }
  }
}

/// The cells of one box, for a grid whose origin is rotated.
///
/// A box that is axis-aligned in the map frame is not axis-aligned in index
/// space once the grid is turned, so there is no rectangle of indices to walk
/// and every cell has to be tested. slam_toolbox publishes an unrotated
/// origin, so this path is the exception; it is here so that a rotated map
/// gives a right answer slowly rather than a wrong one quickly.
void cells_of_box_rotated(
  const Box & box,
  const nav_msgs::msg::MapMetaData & info,
  double yaw,
  std::vector<bool> & taken,
  std::vector<std::size_t> & cells)
{
  const double resolution = info.resolution;
  const double origin_x = info.origin.position.x;
  const double origin_y = info.origin.position.y;
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  for (std::uint32_t row = 0; row < info.height; ++row) {
    for (std::uint32_t col = 0; col < info.width; ++col) {
      const double local_x = (col + 0.5) * resolution;
      const double local_y = (row + 0.5) * resolution;
      const double x = origin_x + cos_yaw * local_x - sin_yaw * local_y;
      const double y = origin_y + sin_yaw * local_x + cos_yaw * local_y;

      if (!contains(box, x, y)) {
        continue;
      }

      const auto index = static_cast<std::size_t>(row) * info.width + col;
      if (!taken[index]) {
        taken[index] = true;
        cells.push_back(index);
      }
    }
  }
}

}  // namespace

const char * to_string(RegionClass region_class)
{
  switch (region_class) {
    case RegionClass::Clear:
      return "clear";
    case RegionClass::Blocked:
      return "blocked";
    case RegionClass::Unknown:
    default:
      return "unknown";
  }
}

std::vector<std::size_t> cells_of(
  const Region & region,
  const nav_msgs::msg::MapMetaData & info)
{
  std::vector<std::size_t> cells;

  if (info.resolution <= 0.0 || info.width == 0 || info.height == 0) {
    return cells;
  }

  const auto size = static_cast<std::size_t>(info.width) * info.height;
  std::vector<bool> taken(size, false);
  const double yaw = origin_yaw(info);
  const bool rotated = std::abs(yaw) > 1e-6;

  for (const auto & box : region.boxes) {
    if (rotated) {
      cells_of_box_rotated(box, info, yaw, taken, cells);
    } else {
      cells_of_box_axis_aligned(box, info, taken, cells);
    }
  }

  return cells;
}

bool on_grid(const Region & region, const nav_msgs::msg::MapMetaData & info)
{
  return !cells_of(region, info).empty();
}

RegionClass classify(
  const Region & region,
  const nav_msgs::msg::OccupancyGrid & grid,
  const Thresholds & thresholds)
{
  const auto cells = cells_of(region, grid.info);
  if (cells.empty()) {
    return RegionClass::Unknown;
  }

  bool unobserved = false;

  for (const auto index : cells) {
    if (index >= grid.data.size()) {
      // The header says one size and the data another. Nothing can be
      // concluded from a grid that contradicts itself.
      return RegionClass::Unknown;
    }

    const std::int8_t value = grid.data[index];

    // Both bounds are strict, so the unsettled band is [free_below,
    // occupied_above] inclusive. A cell sitting exactly on occupied_above has
    // been seen without being settled, and calling it occupied would hand an
    // agent knowledge it does not have -- the same argument that keeps such a
    // cell from counting as free. The asymmetry matters because the two
    // answers are not equally recoverable: an unsettled cell reported as
    // blocked fires the sensing action's blocked outcome, and the state has no
    // operation for taking that back, while an undecided region reports
    // nothing at all.
    if (value > thresholds.occupied_above) {
      return RegionClass::Blocked;
    }
    if (value < 0 || value >= thresholds.free_below) {
      unobserved = true;
    }
  }

  return unobserved ? RegionClass::Unknown : RegionClass::Clear;
}

}  // namespace plansys2
