// Copyright 2023 Adam Morrissett
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

#include <algorithm>
#include <chrono>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <libwheel/motion_planning/bound_range.hpp>
#include <libwheel/motion_planning/r2_space.hpp>
#include <libwheel/motion_planning/rapidly_exploring_random_trees.hpp>
#include <libwheel/motion_planning/uniform_sampler.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class PathPlannerNode : public rclcpp::Node
{
  static constexpr auto kMaxX{10.0};
  static constexpr auto kMaxY{10.0};

public:
  explicit PathPlannerNode()
  : Node{"planning_node"},
    path_publisher_{create_publisher<nav_msgs::msg::Path>("/plan", 1)},
    path_publish_timer_{create_wall_timer(1s, [this]() { this->publishPath(); })},
    goal_pose_subscription_{create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal", 1, [this](const geometry_msgs::msg::PoseStamped & msg) { update_goal_pose(msg); })},
    odometry_subscription_{create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 1, [this](const nav_msgs::msg::Odometry & msg) { this->update_current_pose(msg); })},
    space_sampler_{wheel::R2Space{
      wheel::BoundRange{wheel::LowerBound{0.0}, wheel::UpperBound{kMaxX}},
      wheel::BoundRange{wheel::LowerBound{0.0}, wheel::UpperBound{kMaxY}}}}
  {
  }

private:
  auto update_goal_pose(const geometry_msgs::msg::PoseStamped & msg) -> void
  {
    if (
      msg.pose.position.x < 0 || msg.pose.position.x > kMaxX || msg.pose.position.y < 0 ||
      msg.pose.position.y > kMaxY) {
      RCLCPP_ERROR(get_logger(), "Goal pose is outside of the planning boundary.");
      return;
    }

    RCLCPP_INFO(get_logger(), "Planning to (%lf, %lf).", msg.pose.position.x, msg.pose.position.y);
    goal_pose_ = msg;
  }

  auto update_current_pose(const nav_msgs::msg::Odometry & msg) -> void
  {
    current_pose_.header = msg.header;
    current_pose_.pose = msg.pose.pose;
  }

  auto publishPath() -> void
  {
    if (goal_pose_ == previous_goal_pose_) {
      return;
    }

    const wheel::R2Vector source{
      static_cast<float>(current_pose_.pose.position.x),
      static_cast<float>(current_pose_.pose.position.y)};

    const wheel::R2Vector goal{
      static_cast<float>(goal_pose_.pose.position.x),
      static_cast<float>(goal_pose_.pose.position.y)};

    const auto path = wheel::findRrtPath(space_sampler_, source, goal);

    if (path.has_value()) {
      nav_msgs::msg::Path path_msg;

      path_msg.header.frame_id = "base_link";
      path_msg.header.stamp = get_clock()->now();

      std::ranges::transform(
        std::as_const(path.value()), std::back_inserter(path_msg.poses), [](const auto & config) {
          geometry_msgs::msg::PoseStamped pose;

          pose.header.frame_id = "base_link";
          pose.pose.position.x = static_cast<double>(config.template get<wheel::X>());
          pose.pose.position.y = static_cast<double>(config.template get<wheel::Y>());

          return pose;
        });

      path_publisher_->publish(path_msg);
      previous_goal_pose_ = goal_pose_;
    }
  }

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::TimerBase::SharedPtr path_publish_timer_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;

  geometry_msgs::msg::PoseStamped current_pose_;
  geometry_msgs::msg::PoseStamped goal_pose_;
  geometry_msgs::msg::PoseStamped previous_goal_pose_{};

  wheel::UniformSampler<wheel::R2Space> space_sampler_;
};

auto main(int argc, char * argv[]) -> int
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPlannerNode>());
  rclcpp::shutdown();

  return 0;
}
