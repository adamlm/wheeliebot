include(GNUInstallDirs)

add_executable(path_planner_node path_planner_node.cpp)

target_link_libraries(path_planner_node
  libwheel::motion_planning
)

ament_target_dependencies(path_planner_node rclcpp nav_msgs)

target_compile_features(path_planner_node
  PRIVATE
    cxx_std_20
)

target_compile_options(path_planner_node
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

install(TARGETS path_planner_node
  RUNTIME
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/wheeliebot_path_planner
)
