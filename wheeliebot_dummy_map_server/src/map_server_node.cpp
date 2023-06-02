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
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class DummyMapServerNode : public rclcpp::Node
{
public:
  explicit DummyMapServerNode()
  : rclcpp::Node("map_server"),
    publish_timer_{create_wall_timer(200ms, [this]() { this->timer_callback(); })},
    map_publisher_{create_publisher<nav_msgs::msg::OccupancyGrid>("map", 1)}
  {
  }

  auto timer_callback() -> void
  {
    nav_msgs::msg::OccupancyGrid msg;

    msg.header.stamp = get_clock()->now();
    msg.header.frame_id = "map";

    msg.info.height = static_cast<std::size_t>(10U);
    msg.info.width = static_cast<std::size_t>(10U);
    msg.info.resolution = 1.0F;

    msg.data.resize(10U * 10U, 0U);

    map_publisher_->publish(msg);
  }

private:
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
};

auto main(int argc, char * argv[]) -> int
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DummyMapServerNode>());
  rclcpp::shutdown();

  return 0;
}
