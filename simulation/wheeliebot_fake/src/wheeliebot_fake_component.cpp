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

#include "wheeliebot_fake/wheeliebot_fake_component.hpp"

#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <cmath>

#include <boost/numeric/odeint.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace wheeliebot_fake
{
WheeliebotFakeComponent::WheeliebotFakeComponent(rclcpp::NodeOptions const & options)
: Node{"wheeliebot_fake_node", options},
  cmd_vel_sub_{create_subscription<geometry_msgs::msg::Twist>(
    "input/cmd_vel", 1,
    [this](geometry_msgs::msg::Twist const & msg) { update_target_speeds(msg); })},
  odom_pub_{create_publisher<nav_msgs::msg::Odometry>("output/odom", 1)},
  tf_broadcaster_{std::make_unique<tf2_ros::TransformBroadcaster>(*this)}

{
  using std::chrono_literals::operator""s;

  state_update_timer_ = rclcpp::create_timer(this, this->get_clock(), 1s, [this] {
    RCLCPP_DEBUG(this->get_logger(), "Updating state");

    this->propagate_state();
    this->publish_transform();
    this->publish_odom();
  });

  current_state_stamp_ = get_clock()->now();
}

auto WheeliebotFakeComponent::update_target_speeds(geometry_msgs::msg::Twist const & msg) -> void
{
  RCLCPP_DEBUG(this->get_logger(), "Received cmd_vel msg");
  target_linear_speed_ = msg.linear.x;
  target_angular_speed_ = msg.angular.z;
}

auto WheeliebotFakeComponent::propagate_state() -> void
{
  auto const new_state_stamp{this->get_clock()->now()};
  auto const time_delta{
    (new_state_stamp - current_state_stamp_).to_chrono<std::chrono::duration<double>>().count()};

  boost::numeric::odeint::integrate(
    [this](auto const & state, auto & dstate_dt, double /* time */) {
      std::get<0>(dstate_dt) = target_linear_speed_ * std::cos(std::get<2>(state));
      std::get<1>(dstate_dt) = target_linear_speed_ * std::sin(std::get<2>(state));
      std::get<2>(dstate_dt) = target_angular_speed_;
    },
    current_state_se2_, 0.0, time_delta, 0.1);

  current_state_stamp_ = new_state_stamp;
}

auto WheeliebotFakeComponent::publish_transform() -> void
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = current_state_stamp_;
  transform.header.frame_id = "odom";
  transform.child_frame_id = "base_link";

  transform.transform.translation.x = std::get<0>(current_state_se2_);
  transform.transform.translation.y = std::get<1>(current_state_se2_);
  transform.transform.translation.z = 0.0;

  transform.transform.rotation = [this] {
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, std::get<2>(current_state_se2_));
    return tf2::toMsg(quaternion);
  }();

  tf_broadcaster_->sendTransform(transform);
}

auto WheeliebotFakeComponent::publish_odom() -> void
{
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = current_state_stamp_;
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_link";

  odom_msg.pose.pose.position.x = std::get<0>(current_state_se2_);
  odom_msg.pose.pose.position.y = std::get<1>(current_state_se2_);
  odom_msg.pose.pose.position.z = 0.0;

  odom_msg.pose.pose.orientation = [this] {
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, std::get<2>(current_state_se2_));
    return tf2::toMsg(quaternion);
  }();

  odom_msg.pose.covariance.at(0) = -1.0;

  odom_pub_->publish(odom_msg);
}

}  // namespace wheeliebot_fake

RCLCPP_COMPONENTS_REGISTER_NODE(wheeliebot_fake::WheeliebotFakeComponent)
