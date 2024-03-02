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

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Using ClangFormat instead of uncrustify since the two are incompatible. This diverges from
  # upstream ROS 2 packages, which uses uncrustify instead of ClangFormat. See
  # https://github.com/ament/ament_lint/issues/146 for more context. ClangFormat seems more
  # popular for C++ projects beyond ROS.
  set(ament_cmake_uncrustify_FOUND TRUE)

  # The .clang-format file provided by the ament_clang_format package lets ClangFormat regroup
  # #include directives, which can violate the ROS 2 style guidelines.
  set(ament_cmake_clang_format_CONFIG_FILE ${PROJECT_SOURCE_DIR}/.clang-format)

  ament_lint_auto_find_test_dependencies()
endif()

# find_package(ament_cmake REQUIRED)
# find_package(nav_msgs REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(geometry_msgs REQUIRED)

# if(BUILD_TESTING)
#   find_package(ament_cmake_clang_format REQUIRED)
#   find_package(ament_cmake_clang_tidy REQUIRED)
#   find_package(ament_lint_auto REQUIRED)
# endif()

# include(FetchContent)

# FetchContent_Declare(libwheel_motion_planning
#   GIT_REPOSITORY git@github.com:adamlm/libwheel-motion_planning.git
#   GIT_TAG main
# )

# FetchContent_MakeAvailable(libwheel_motion_planning)
