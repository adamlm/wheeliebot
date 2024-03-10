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

#include <rclcpp/rclcpp.hpp>

#include "wheeliebot_path_planner/a_star_path_planner_component.hpp"

auto main(int argc, char * argv[]) -> int
{
  rclcpp::init(argc, argv);
  auto node_ptr{std::make_shared<wheeliebot_path_planner::AStarPathPlannerComponent>()};

  try {
    rclcpp::spin(node_ptr);
  } catch (std::exception const & ex) {
    RCLCPP_FATAL_STREAM(node_ptr->get_logger(), "unhandled exception: " << ex.what());
  } catch (...) {
    RCLCPP_FATAL(node_ptr->get_logger(), "unknown error");
  }

  rclcpp::shutdown();

  return 0;
}
