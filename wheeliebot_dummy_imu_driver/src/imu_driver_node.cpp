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

#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

using namespace std::chrono_literals;

class DummyImuDriverNode : public rclcpp::Node {
  public:
    explicit DummyImuDriverNode()
        : rclcpp::Node("imu_driver"),
          publisher_timer_{create_wall_timer(20ms, [this]() { this->timer_callback(); })},
          data_raw_publisher_{create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 1)},
          data_publisher_{create_publisher<sensor_msgs::msg::Imu>("imu/data", 1)},
          mag_publisher_{create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", 1)} {}

    auto timer_callback() -> void {
        sensor_msgs::msg::Imu data_raw_msg;
        sensor_msgs::msg::Imu data_msg;
        sensor_msgs::msg::MagneticField mag_msg;

        const auto time{get_clock()->now()};
        data_raw_msg.header.stamp = time;
        data_msg.header.stamp = time;
        mag_msg.header.stamp = time;

        data_raw_msg.header.frame_id = "imu_link";
        data_msg.header.frame_id = "imu_link";
        mag_msg.header.frame_id = "imu_link";

        data_raw_msg.orientation_covariance[0] = -1;

        data_raw_publisher_->publish(data_raw_msg);
        data_publisher_->publish(data_msg);
        mag_publisher_->publish(mag_msg);
    }

  private:
    rclcpp::TimerBase::SharedPtr publisher_timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr data_raw_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr data_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_publisher_;
};

auto main(int argc, char *argv[]) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DummyImuDriverNode>());
    rclcpp::shutdown();

    return 0;
}
