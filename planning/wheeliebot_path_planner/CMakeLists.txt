# Copyright 2024 Adam Morrissett
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.22)
project(wheeliebot_path_planner)

include(dependencies.cmake)

ament_auto_add_library(wheeliebot_path_planner SHARED
  src/a_star_path_planner_component.cpp
  src/bfs_path_planner_component.cpp
  src/detail/boost_graph_extensions.cpp
)

rclcpp_components_register_nodes(wheeliebot_path_planner
  "wheeliebot_path_planner::AStarPathPlannerComponent"
  "wheeliebot_path_planner::BfsPathPlannerComponent"
)

ament_auto_add_executable(wheeliebot_a_star_path_planner_node
  src/a_star_path_planner_node.cpp
)

ament_auto_add_executable(wheeliebot_bfs_path_planner_node
  src/bfs_path_planner_node.cpp
)

ament_auto_package()
