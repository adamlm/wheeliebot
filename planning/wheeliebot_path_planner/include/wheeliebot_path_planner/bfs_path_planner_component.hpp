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

#ifndef WHEELIEBOT_PATH_PLANNER__BFS_PATH_PLANNER_COMPONENT_HPP_
#define WHEELIEBOT_PATH_PLANNER__BFS_PATH_PLANNER_COMPONENT_HPP_

#include <optional>

#include <boost/graph/adjacency_list.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#include "wheeliebot_path_planner/boost_graph_extensions.hpp"

namespace wheeliebot_path_planner
{
class BfsPathPlannerComponent : public rclcpp::Node
{
public:
  BfsPathPlannerComponent();
  explicit BfsPathPlannerComponent(rclcpp::NodeOptions const & options);

  auto make_path() -> std::optional<nav_msgs::msg::Path>;

  auto update_current_state(nav_msgs::msg::Odometry const & msg) -> void;
  auto update_target_pose(geometry_msgs::msg::PoseStamped const & msg) -> void;
  auto update_planning_grid(nav_msgs::msg::OccupancyGrid const & msg) -> void;

private:
  std::optional<nav_msgs::msg::Odometry> current_state_{std::nullopt};
  std::optional<geometry_msgs::msg::PoseStamped> target_pose_{std::nullopt};
  std::optional<PositionGrid> planning_grid_{std::nullopt};

  rclcpp::TimerBase::SharedPtr path_publication_timer_{nullptr};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_{nullptr};
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_{nullptr};
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_sub_{nullptr};

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_{nullptr};
};
}  // namespace wheeliebot_path_planner

#endif  // WHEELIEBOT_PATH_PLANNER__BFS_PATH_PLANNER_COMPONENT_HPP_
