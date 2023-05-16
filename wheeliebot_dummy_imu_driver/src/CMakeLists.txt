# Copyright 2023 Adam Morrissett
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

include(GNUInstallDirs)

add_executable(imu_driver_node imu_driver_node.cpp)

ament_target_dependencies(imu_driver_node rclcpp sensor_msgs)

target_compile_features(imu_driver_node
  PRIVATE
    cxx_std_20
)

target_compile_options(imu_driver_node
  PRIVATE
    -Wall
    -Wextra
    -Wshadow
    -Wnon-virtual-dtor
    -Wpedantic
    -Wold-style-cast
    -Wunused
    -Wconversion
    -Wsign-conversion
    -Wcast-align
    -Woverloaded-virtual
    -Wdouble-promotion
    -Wformat=2
    -Wimplicit-fallthrough
    -Werror
)

install(TARGETS imu_driver_node
  RUNTIME
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/wheeliebot_dummy_imu_driver
)
