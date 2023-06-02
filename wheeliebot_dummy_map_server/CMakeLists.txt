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

cmake_minimum_required(VERSION 3.8)
project(wheeliebot_dummy_map_server)
enable_testing()

option(wheeliebot_dummy_map_server_EXPORT_COMPILE_COMMANDS "Export compile commands" ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ${wheeliebot_dummy_map_server_EXPORT_COMPILE_COMMANDS})

include(dependencies.cmake)

# If there is a `build` directory in the package, colcon will try to build the
# package(s) within that directory, causing an error.
file(TOUCH ${PROJECT_BINARY_DIR}/COLCON_IGNORE)

if(BUILD_TESTING)
  # Using clang-tidy instead of cpplint
  set(ament_cmake_cpplint_FOUND TRUE)

  # Using clang-format instead of uncrustify
  set(ament_cmake_uncrustify_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

add_subdirectory(src)

ament_package()
