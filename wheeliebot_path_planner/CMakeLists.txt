cmake_minimum_required(VERSION 3.22)
project(wheeliebot_path_planner)

option(
  wheeliebot_path_planner_EXPORT_COMPILE_COMMANDS
  "Export compile commands for the wheeliebot path planner package"
  ON
)

set(CMAKE_EXPORT_COMPILE_COMMANDS ${wheeliebot_path_planner_EXPORT_COMPILE_COMMANDS})

include(dependencies.cmake)

add_subdirectory(src)

if(BUILD_TESTING)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
