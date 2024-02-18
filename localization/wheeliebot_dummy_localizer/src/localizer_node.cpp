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

#include <tf2_ros/transform_broadcaster.h>

#include <chrono>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class DummyLocalizerNode : public rclcpp::Node
{
public:
  explicit DummyLocalizerNode()
  : rclcpp::Node("localizer"),
    publish_timer_{create_wall_timer(200ms, [this]() { this->timer_callback(); })},
    odom_publisher_{create_publisher<nav_msgs::msg::Odometry>("odom", 1)},
    accel_publisher_{create_publisher<geometry_msgs::msg::AccelStamped>("acceleration", 1)}
  {
    odom_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    map_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  auto timer_callback() -> void
  {
    const auto timestamp{get_clock()->now()};

    const auto x_offset{0.0};
    const auto y_offset{0.0};
    const auto z_offset{0.0};

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = timestamp;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    odom_msg.pose.pose.position.x = x_offset;
    odom_msg.pose.pose.position.y = y_offset;
    odom_msg.pose.pose.position.z = z_offset;
    odom_msg.pose.covariance.at(0) = -1;
    odom_msg.twist.covariance.at(0) = -1;

    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header.stamp = timestamp;
    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id = "base_link";
    odom_tf.transform.translation.x = x_offset;
    odom_tf.transform.translation.y = y_offset;
    odom_tf.transform.translation.z = z_offset;

    geometry_msgs::msg::AccelStamped accel_msg;
    accel_msg.header.stamp = timestamp;

    geometry_msgs::msg::TransformStamped map_tf;
    map_tf.header.stamp = timestamp;
    map_tf.header.frame_id = "map";
    map_tf.child_frame_id = "odom";
    map_tf.transform.translation.x = 0.0;
    map_tf.transform.translation.y = 0.0;
    map_tf.transform.translation.z = 0.0;

    odom_publisher_->publish(odom_msg);
    odom_broadcaster_->sendTransform(odom_tf);
    accel_publisher_->publish(accel_msg);
    map_broadcaster_->sendTransform(map_tf);
  }

private:
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;
  rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr accel_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> map_broadcaster_;
};

auto main(int argc, char * argv[]) -> int
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DummyLocalizerNode>());
  rclcpp::shutdown();

  return 0;
}
