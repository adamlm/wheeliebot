cmake_minimum_required(VERSION 3.8)
project(wheeliebot_dummy_imu_driver)
enable_testing()

option(
  wheeliebot_dummy_imu_driver_EXPORT_COMPILE_COMMANDS
  "Export compile commands for the wheeliebot dummy IMU driver package"
  ON
)

set(CMAKE_EXPORT_COMPILE_COMMANDS ${wheeliebot_dummy_imu_driver_EXPORT_COMPILE_COMMANDS})

include(dependencies.cmake)

add_subdirectory(src)

ament_package()
