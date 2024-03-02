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

#ifndef WHEELIEBOT_PATH_PLANNER__BOOST_GRAPH_EXTENSIONS_HPP_
#define WHEELIEBOT_PATH_PLANNER__BOOST_GRAPH_EXTENSIONS_HPP_

#include <boost/graph/adjacency_list.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace wheeliebot_path_planner
{
struct Point2d
{
  double x;
  double y;
};

using PositionGrid = boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, Point2d>;

auto to_boost_graph(nav_msgs::msg::OccupancyGrid const & occupancy_grid) -> PositionGrid;

}  // namespace wheeliebot_path_planner

#endif  // WHEELIEBOT_PATH_PLANNER__BOOST_GRAPH_EXTENSIONS_HPP_
