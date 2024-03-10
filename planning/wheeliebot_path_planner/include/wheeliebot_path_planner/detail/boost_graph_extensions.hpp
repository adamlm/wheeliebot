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

#ifndef WHEELIEBOT_PATH_PLANNER__DETAIL__BOOST_GRAPH_EXTENSIONS_HPP_
#define WHEELIEBOT_PATH_PLANNER__DETAIL__BOOST_GRAPH_EXTENSIONS_HPP_

#include <vector>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

namespace wheeliebot_path_planner::detail
{
struct Point2d
{
  double x;
  double y;
};

using PositionGrid = boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, Point2d>;

auto to_boost_graph(nav_msgs::msg::OccupancyGrid const & occupancy_grid) -> PositionGrid;

template <typename Graph>
using vertex_descriptor_t = typename boost::graph_traits<Graph>::vertex_descriptor;

template <typename Graph>
using edge_descriptor_t = typename boost::graph_traits<Graph>::edge_descriptor;

template <typename Graph>
using vertex_bundle_t = typename boost::vertex_bundle_type<Graph>::type;

template <typename Graph>
struct EarlySearchTermination
{
public:
  explicit EarlySearchTermination(vertex_descriptor_t<Graph> vertex) : vertex_{vertex} {}

  auto get_terminal_vertex() const noexcept { return vertex_; }

private:
  vertex_descriptor_t<Graph> vertex_;
};

template <typename Graph>
struct TargetVertexChecker : public boost::base_visitor<TargetVertexChecker<Graph>>
{
public:
  using event_filter = boost::on_tree_edge;

  explicit TargetVertexChecker(vertex_descriptor_t<Graph> target) : target_{target} {}

  auto operator()(edge_descriptor_t<Graph> e, Graph const & g) const -> void
  {
    if (auto const v{boost::target(e, g)}; v == target_) {
      throw EarlySearchTermination<Graph>(v);
    }
  }

private:
  vertex_descriptor_t<Graph> target_;
};

template <typename Graph>
auto find_vertex_path(
  vertex_descriptor_t<Graph> source, vertex_descriptor_t<Graph> target, Graph const & graph)
  -> std::optional<std::vector<vertex_descriptor_t<Graph>>>
{
  std::vector<std::optional<vertex_descriptor_t<Graph>>> predecessors(
    boost::num_vertices(graph), std::nullopt);

  try {
    boost::breadth_first_search(
      graph, source,
      boost::visitor(boost::make_bfs_visitor(std::pair{
        boost::record_predecessors(predecessors.data(), boost::on_tree_edge{}),
        TargetVertexChecker<Graph>{target}})));
  } catch (EarlySearchTermination<Graph> const & result) {
    std::vector<vertex_descriptor_t<Graph>> path_vertices;

    for (std::optional<vertex_descriptor_t<Graph>> vertex{result.get_terminal_vertex()};
         vertex.has_value(); vertex = predecessors.at(vertex.value())) {
      path_vertices.push_back(vertex.value());
    }

    std::reverse(std::begin(path_vertices), std::end(path_vertices));
    return path_vertices;
  }

  return std::nullopt;
}

template <typename Graph>
struct AStarTargetVertexChecker : public boost::default_astar_visitor
{
public:
  explicit AStarTargetVertexChecker(vertex_descriptor_t<Graph> target) : target_{target} {}

  auto examine_vertex(vertex_descriptor_t<Graph> v, Graph const & g)
  {
    if (v == target_) {
      throw EarlySearchTermination<Graph>(v);
    }
  }

private:
  vertex_descriptor_t<Graph> target_;
};

template <class Graph>
class distance_heuristic : public boost::astar_heuristic<Graph, double>
{
public:
  distance_heuristic(vertex_descriptor_t<Graph> target, Graph const & graph)
  : target_{target}, graph_{graph}
  {
  }

  double operator()(vertex_descriptor_t<Graph> v)
  {
    auto const delta_x{graph_.get()[target_].x - graph_.get()[v].x};
    auto const delta_y{graph_.get()[target_].y - graph_.get()[v].y};

    return std::hypot(delta_x, delta_y);
  }

private:
  std::reference_wrapper<Graph const> graph_;
  vertex_descriptor_t<Graph> target_;
};

template <typename Graph>
auto find_vertex_path_a_star(
  vertex_descriptor_t<Graph> source, vertex_descriptor_t<Graph> target, Graph const & graph)
  -> std::optional<std::vector<vertex_descriptor_t<Graph>>>
{
  std::vector<vertex_descriptor_t<Graph>> predecessors(boost::num_vertices(graph));

  auto index = boost::get(boost::vertex_index, graph);
  auto pmap = boost::make_safe_iterator_property_map(
    std::begin(predecessors), std::size(predecessors), index);

  try {
    boost::astar_search(
      graph, source, distance_heuristic<Graph>(source, graph),
      boost::visitor(AStarTargetVertexChecker<Graph>(target))
        .weight_map(boost::static_property_map<double>{1.0})
        .predecessor_map(pmap));
  } catch (EarlySearchTermination<Graph> const & result) {
    std::vector<vertex_descriptor_t<Graph>> path_vertices;

    std::vector<vertex_descriptor_t<Graph>> p;
    for (auto cursor = target;;) {
      p.push_back(cursor);
      auto previous = std::exchange(cursor, predecessors.at(cursor));
      if (cursor == previous) {
        break;
      }
    }

    std::reverse(std::begin(p), std::end(p));
    return p;

    for (std::optional<vertex_descriptor_t<Graph>> vertex{result.get_terminal_vertex()};
         vertex.has_value(); vertex = predecessors.at(vertex.value())) {
      path_vertices.push_back(vertex.value());
    }

    std::reverse(std::begin(path_vertices), std::end(path_vertices));
    return path_vertices;
  }

  return std::nullopt;
}

}  // namespace wheeliebot_path_planner::detail

#endif  // WHEELIEBOT_PATH_PLANNER__DETAIL__BOOST_GRAPH_EXTENSIONS_HPP_
