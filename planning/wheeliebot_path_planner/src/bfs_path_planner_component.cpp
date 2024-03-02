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

#include "wheeliebot_path_planner/boost_graph_extensions.hpp"

namespace wheeliebot_path_planner
{
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
  planning_grid_ = to_boost_graph(msg);
}

template <typename Graph>
struct EarlySearchTermination
{
public:
  EarlySearchTermination(typename boost::graph_traits<Graph>::vertex_descriptor vertex)
  : vertex_{vertex}
  {
  }

  auto get_last_vertex() const noexcept { return vertex_; }

private:
  typename boost::graph_traits<Graph>::vertex_descriptor vertex_;
};

template <typename Graph>
struct is_target_vertex : public boost::base_visitor<is_target_vertex<Graph>>
{
public:
  using event_filter = boost::on_tree_edge;

  explicit is_target_vertex(typename boost::graph_traits<Graph>::vertex_descriptor target_vertex)
  : target_vertex_{target_vertex}
  {
  }

  auto operator()(typename boost::graph_traits<Graph>::edge_descriptor e, Graph const & g) const
    -> void
  {
    if (auto const v{boost::target(e, g)}; v == target_vertex_) {
      throw EarlySearchTermination<Graph>(v);
    }
  }

private:
  typename boost::graph_traits<Graph>::vertex_descriptor target_vertex_;
};

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
  auto & planning_grid{planning_grid_.value()};

  auto const vertex_range{boost::make_iterator_range(boost::vertices(planning_grid))};
  auto const closest_vertex{ranges::min_element(
    vertex_range, [&target_pose = std::as_const(target_pose),
                   &planning_grid = std::as_const(planning_grid)](auto const & a, auto const & b) {
      auto const delta_x_a{planning_grid[a].x - target_pose.pose.position.x};
      auto const delta_y_a{planning_grid[a].y - target_pose.pose.position.y};

      auto const delta_x_b{planning_grid[b].x - target_pose.pose.position.x};
      auto const delta_y_b{planning_grid[b].y - target_pose.pose.position.y};

      return std::hypot(delta_x_a, delta_y_a) < std::hypot(delta_x_b, delta_y_b);
    })};

  auto const target_pose_vertex{boost::add_vertex(
    Point2d{target_pose.pose.position.x, target_pose.pose.position.y}, planning_grid)};
  boost::add_edge(target_pose_vertex, *closest_vertex, planning_grid);

  auto const closest_current_vertex{ranges::min_element(
    vertex_range, [&current_state = std::as_const(current_state),
                   &planning_grid = std::as_const(planning_grid)](auto const & a, auto const & b) {
      auto const delta_x_a{planning_grid[a].x - current_state.pose.pose.position.x};
      auto const delta_y_a{planning_grid[a].y - current_state.pose.pose.position.y};

      auto const delta_x_b{planning_grid[b].x - current_state.pose.pose.position.x};
      auto const delta_y_b{planning_grid[b].y - current_state.pose.pose.position.y};

      return std::hypot(delta_x_a, delta_y_a) < std::hypot(delta_x_b, delta_y_b);
    })};

  RCLCPP_DEBUG_STREAM(get_logger(), "Closest vertex: " << std::to_string(*closest_current_vertex));
  RCLCPP_DEBUG_STREAM(
    get_logger(), "Vertex 25: x = " << planning_grid[25].x << ", y = " << planning_grid[25].y);

  auto const current_pose_vertex{boost::add_vertex(
    Point2d{current_state.pose.pose.position.x, current_state.pose.pose.position.y},
    planning_grid)};
  boost::add_edge(current_pose_vertex, *closest_current_vertex, planning_grid);

  using PredecessorList = std::vector<std::optional<int>>;
  PredecessorList predecessors(boost::num_vertices(planning_grid), std::nullopt);

  auto const predecessor_recorder{
    boost::record_predecessors(predecessors.data(), boost::on_tree_edge{})};
  auto const visitor{boost::make_bfs_visitor(
    std::pair{predecessor_recorder, is_target_vertex<PositionGrid>{target_pose_vertex}})};

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";

  try {
    RCLCPP_DEBUG_STREAM(
      get_logger(), "Current pose vertex: " << std::to_string(current_pose_vertex));
    RCLCPP_DEBUG_STREAM(get_logger(), "Target pose vertex: " << std::to_string(target_pose_vertex));
    boost::breadth_first_search(planning_grid, current_pose_vertex, boost::visitor(visitor));
  } catch (EarlySearchTermination<PositionGrid> const & result) {
    for (typename PredecessorList::value_type predecessor{result.get_last_vertex()};
         predecessor != std::nullopt; predecessor = predecessors.at(predecessor.value())) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.pose.position.x = planning_grid[predecessor.value()].x;
      pose.pose.position.y = planning_grid[predecessor.value()].y;

      path.poses.push_back(std::move(pose));
    }

    std::reverse(std::begin(path.poses), std::end(path.poses));
  }

  boost::remove_vertex(current_pose_vertex, planning_grid);
  boost::remove_vertex(target_pose_vertex, planning_grid);

  return path;
}

}  // namespace wheeliebot_path_planner
