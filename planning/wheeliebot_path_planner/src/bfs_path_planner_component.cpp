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

#include "wheeliebot_path_planner/bfs_path_planner_component.hpp"

#include <exception>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/copy.hpp>
#include <boost/graph/grid_graph.hpp>
#include <boost/range/iterator_range.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <range/v3/algorithm.hpp>
#include <range/v3/view/iota.hpp>
#include <range/v3/view/zip.hpp>
#include <rclcpp/rclcpp.hpp>

#include "wheeliebot_path_planner/detail/boost_graph_extensions.hpp"

namespace wheeliebot_path_planner
{
namespace
{
auto get_position_x(nav_msgs::msg::Odometry const & msg) { return msg.pose.pose.position.x; }

auto get_position_x(geometry_msgs::msg::PoseStamped const & msg) { return msg.pose.position.x; }

auto get_position_x(detail::Point2d const & point) { return point.x; }

auto get_position_y(nav_msgs::msg::Odometry const & msg) { return msg.pose.pose.position.y; }

auto get_position_y(geometry_msgs::msg::PoseStamped const & msg) { return msg.pose.position.y; }

auto get_position_y(detail::Point2d const & point) { return point.y; }

template <typename T, typename U>
auto pythagorean_distance(T const & a, U const & b)
{
  auto const delta_x{get_position_x(a) - get_position_x(b)};
  auto const delta_y{get_position_y(a) - get_position_y(b)};

  return std::hypot(delta_x, delta_y);
}
}  // namespace

BfsPathPlannerComponent::BfsPathPlannerComponent() : BfsPathPlannerComponent{rclcpp::NodeOptions{}}
{
}

BfsPathPlannerComponent::BfsPathPlannerComponent(rclcpp::NodeOptions const & options)
: Node{"wheeliebot_bfs_path_planner_node", options},
  odometry_sub_{create_subscription<nav_msgs::msg::Odometry>(
    "input/odom", 1, [this](nav_msgs::msg::Odometry const & msg) { update_current_state(msg); })},
  target_pose_sub_{create_subscription<geometry_msgs::msg::PoseStamped>(
    "input/target_pose", 1,
    [this](geometry_msgs::msg::PoseStamped const & msg) { update_target_pose(msg); })},
  occupancy_grid_sub_{create_subscription<nav_msgs::msg::OccupancyGrid>(
    "input/occupancy_grid", 1,
    [this](nav_msgs::msg::OccupancyGrid const & msg) { update_planning_grid(msg); })},
  path_pub_{create_publisher<nav_msgs::msg::Path>("output/path", 1)}
{
  using std::chrono_literals::operator""s;

  path_publication_timer_ = rclcpp::create_timer(this, this->get_clock(), 1s, [this] {
    if (auto const path{make_path()}) {
      path_pub_->publish(path.value());
    }
  });
}

auto BfsPathPlannerComponent::update_current_state(nav_msgs::msg::Odometry const & msg) -> void
{
  current_state_ = msg;
}

auto BfsPathPlannerComponent::update_target_pose(geometry_msgs::msg::PoseStamped const & msg)
  -> void
{
  target_pose_ = msg;
}

auto BfsPathPlannerComponent::update_planning_grid(nav_msgs::msg::OccupancyGrid const & msg) -> void
{
  planning_grid_ = detail::to_boost_graph(msg);
}

auto BfsPathPlannerComponent::make_path() -> std::optional<nav_msgs::msg::Path>
{
  if (!current_state_) {
    RCLCPP_DEBUG(get_logger(), "Not planning path: no current state");
    return std::nullopt;
  }

  if (!target_pose_) {
    RCLCPP_DEBUG(get_logger(), "Not planning path: no target pose");
    return std::nullopt;
  }

  if (!planning_grid_) {
    RCLCPP_DEBUG(get_logger(), "Not planning path: no planning grid");
    return std::nullopt;
  }

  auto const & current_state{current_state_.value()};
  auto const & target_pose{target_pose_.value()};
  auto const & planning_grid{planning_grid_.value()};

  auto const vertex_range{boost::make_iterator_range(boost::vertices(planning_grid))};

  auto const source_vertex{
    *ranges::min_element(vertex_range, {}, [&current_state, &planning_grid](auto const & v) {
      return pythagorean_distance(current_state, planning_grid[v]);
    })};

  auto const target_vertex{
    *ranges::min_element(vertex_range, {}, [&target_pose, &planning_grid](auto const & v) {
      return pythagorean_distance(target_pose, planning_grid[v]);
    })};

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";

  if (auto const vertex_path{
        detail::find_vertex_path(source_vertex, target_vertex, planning_grid)}) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";

    pose.pose.position.x = get_position_x(current_state);
    pose.pose.position.y = get_position_y(current_state);
    path.poses.push_back(pose);

    for (auto const & vertex : vertex_path.value()) {
      pose.pose.position.x = get_position_x(planning_grid[vertex]);
      pose.pose.position.y = get_position_y(planning_grid[vertex]);

      path.poses.push_back(pose);
    }

    pose.pose.position.x = get_position_x(target_pose);
    pose.pose.position.y = get_position_y(target_pose);
    path.poses.push_back(pose);
  }

  return path;
}

}  // namespace wheeliebot_path_planner
