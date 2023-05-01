find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
endif()

include(FetchContent)

FetchContent_Declare(libwheel_motion_planning
  GIT_REPOSITORY git@github.com:adamlm/libwheel-motion_planning.git
  GIT_TAG main
)

FetchContent_MakeAvailable(libwheel_motion_planning)
