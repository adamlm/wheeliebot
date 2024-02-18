# Copyright 2024 Adam Morrissett
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.22)
project(wheeliebot_fake)

include(dependencies.cmake)

ament_auto_add_library(wheeliebot_fake SHARED
  src/wheeliebot_fake_component.cpp
)

rclcpp_components_register_nodes(wheeliebot_fake
  "wheeliebot_fake::WheeliebotFakeComponent"
)

ament_auto_add_executable(wheeliebot_fake_node
  src/wheeliebot_fake_node.cpp
)

ament_auto_package()
