// Copyright 2024 Adam Morrissett
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

#include "wheeliebot_path_planner/detail/boost_graph_extensions.hpp"

#include <boost/graph/adjacency_list.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <range/v3/view/iota.hpp>
#include <range/v3/view/zip.hpp>
#include <rclcpp/rclcpp.hpp>

namespace wheeliebot_path_planner::detail
{
auto to_boost_graph(nav_msgs::msg::OccupancyGrid const & occupancy_grid) -> PositionGrid
{
  auto const grid_height{occupancy_grid.info.height};
  auto const grid_width{occupancy_grid.info.width};

  // The PositionGrid will have unconnected vertices if the OccupancyGrid has occupied
  // cells, but this approach keeps vertex management simple.
  PositionGrid position_grid(grid_width * grid_height);

  for (auto row{0U}; row < grid_height; ++row) {
    for (auto column{0U}; column < grid_width; ++column) {
      auto const flattened_index{row * grid_width + column};

      auto const resolution{occupancy_grid.info.resolution};

      // Columns increase along the x-axis, and rows increase along the y-axis
      position_grid[flattened_index].x = column * resolution + resolution / 2.0;
      position_grid[flattened_index].y = row * resolution + resolution / 2.0;

      if (occupancy_grid.data.at(flattened_index) != 0) {
        continue;
      }

      if (row < grid_height - 1 && occupancy_grid.data.at(flattened_index + grid_width) == 0) {
        boost::add_edge(flattened_index, flattened_index + grid_width, position_grid);
      }

      if (column < grid_width - 1 && occupancy_grid.data.at(flattened_index + 1) == 0) {
        boost::add_edge(flattened_index, flattened_index + 1, position_grid);
      }
    }
  }

  return position_grid;
}

}  // namespace wheeliebot_path_planner::detail
