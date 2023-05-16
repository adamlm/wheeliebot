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
