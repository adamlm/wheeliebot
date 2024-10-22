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

#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include "wheeliebot_fake/wheeliebot_fake_component.hpp"

auto main(int argc, char * argv[]) -> int
{
  try {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<wheeliebot_fake::WheeliebotFakeComponent>(rclcpp::NodeOptions{}));
    rclcpp::shutdown();
  } catch (std::exception const & ex) {
    std::cerr << "wheeliebot_fake_node: " << ex.what() << '\n';
  } catch (...) {
    std::cerr << "wheeliebot_fake_node: unknown error\n";
    return 1;
  }

  return 0;
}
