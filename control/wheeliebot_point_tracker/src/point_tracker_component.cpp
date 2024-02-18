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

#include "wheeliebot_point_tracker/point_tracker_component.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <cmath>

#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace wheeliebot_point_tracker
{
PointTrackerComponent::PointTrackerComponent() : PointTrackerComponent{rclcpp::NodeOptions{}} {}

PointTrackerComponent::PointTrackerComponent(rclcpp::NodeOptions const & options)
: Node("wheeliebot_point_tracker_node", options),
  odometry_sub_{create_subscription<nav_msgs::msg::Odometry>(
    "input/odom", 1, [this](nav_msgs::msg::Odometry const & msg) { update_current_state(msg); })},
  target_point_sub_{create_subscription<geometry_msgs::msg::PointStamped>(
    "input/target_point", 1,
    [this](geometry_msgs::msg::PointStamped const & msg) { update_target_point(msg); })},
  twist_command_pub_{create_publisher<geometry_msgs::msg::Twist>("output/cmd_vel", 1)}
{
  using std::chrono_literals::operator""s;

  twist_command_update_timer_ =
    rclcpp::create_timer(this, this->get_clock(), 1s, [this] { this->publish_twist_command(); });
}

auto PointTrackerComponent::update_target_point(geometry_msgs::msg::PointStamped const & msg)
  -> void
{
  target_point_ = msg;
}

auto PointTrackerComponent::update_current_state(nav_msgs::msg::Odometry const & msg) -> void
{
  current_state_ = msg;
}

auto PointTrackerComponent::publish_twist_command() -> void
{
  geometry_msgs::msg::Twist twist;
  twist.linear.x = 0.0;
  twist.angular.z = 0.0;

  if (current_state_ && target_point_) {
    auto const delta_x{target_point_.value().point.x - current_state_.value().pose.pose.position.x};
    auto const delta_y{target_point_.value().point.y - current_state_.value().pose.pose.position.y};

    auto const euclidean_distance{std::hypot(delta_x, delta_y)};
    auto const angle_to_target{std::atan2(delta_y, delta_x)};

    auto const current_yaw = [this] {
      tf2::Quaternion quaternion;
      tf2::fromMsg(current_state_.value().pose.pose.orientation, quaternion);

      tf2::Matrix3x3 matrix(quaternion);
      double roll, pitch, yaw;
      matrix.getRPY(roll, pitch, yaw);

      return yaw;
    }();

    auto const angular_distance{angle_to_target - current_yaw};

    twist.linear.x = 0.1 * euclidean_distance;
    twist.angular.z = 0.5 * angular_distance;
  } else if (!current_state_) {
    RCLCPP_DEBUG(get_logger(), "Sending zero twist command: current state unknown");
  } else if (!target_point_) {
    RCLCPP_DEBUG(get_logger(), "Sending zero twist command: no target point");
  }

  twist_command_pub_->publish(twist);
}

}  // namespace wheeliebot_point_tracker

RCLCPP_COMPONENTS_REGISTER_NODE(wheeliebot_point_tracker::PointTrackerComponent)
