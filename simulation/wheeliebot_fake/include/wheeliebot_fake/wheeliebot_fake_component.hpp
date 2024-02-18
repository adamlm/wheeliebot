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

#ifndef WHEELIEBOT_FAKE__WHEELIEBOT_FAKE_COMPONENT_HPP_
#define WHEELIEBOT_FAKE__WHEELIEBOT_FAKE_COMPONENT_HPP_

#include <rcl/time.h>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace wheeliebot_fake
{
class WheeliebotFakeComponent : public rclcpp::Node
{
public:
  explicit WheeliebotFakeComponent(rclcpp::NodeOptions const & options);

  auto update_target_speeds(geometry_msgs::msg::Twist const & msg) -> void;

  auto propagate_state() -> void;

  auto publish_transform() -> void;

  auto publish_odom() -> void;

private:
  rclcpp::Time current_state_stamp_{0, 0, RCL_ROS_TIME};
  std::array<double, 3> current_state_se2_{0.0, 0.0, 0.0};
  double target_linear_speed_{0.0};
  double target_angular_speed_{0.0};

  rclcpp::TimerBase::SharedPtr state_update_timer_{nullptr};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_{nullptr};

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_{nullptr};

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_{nullptr};
};

}  // namespace wheeliebot_fake

#endif  // WHEELIEBOT_FAKE__WHEELIEBOT_FAKE_COMPONENT_HPP_
