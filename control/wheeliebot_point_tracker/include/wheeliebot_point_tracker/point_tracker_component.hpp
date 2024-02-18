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

#ifndef WHEELIEBOT_POINT_TRACKER__POINT_TRACKER_COMPONENT_HPP_
#define WHEELIEBOT_POINT_TRACKER__POINT_TRACKER_COMPONENT_HPP_

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace wheeliebot_point_tracker
{
class PointTrackerComponent : public rclcpp::Node
{
public:
  PointTrackerComponent();
  explicit PointTrackerComponent(rclcpp::NodeOptions const & options);

  auto update_target_point(geometry_msgs::msg::PointStamped const & msg) -> void;

  auto update_current_state(nav_msgs::msg::Odometry const & msg) -> void;

  auto publish_twist_command() -> void;

private:
  std::optional<geometry_msgs::msg::PointStamped> target_point_{std::nullopt};
  std::optional<nav_msgs::msg::Odometry> current_state_{std::nullopt};

  rclcpp::TimerBase::SharedPtr twist_command_update_timer_{nullptr};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_{nullptr};
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_point_sub_{nullptr};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_command_pub_{nullptr};
};

}  // namespace wheeliebot_point_tracker

#endif  // WHEELIEBOT_POINT_TRACKER__POINT_TRACKER_COMPONENT_HPP_
