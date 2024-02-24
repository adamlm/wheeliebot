#include <rclcpp/rclcpp.hpp>

#include "wheeliebot_pure_pursuit_controller/pure_pursuit_controller_component.hpp"

auto main(int argc, char * argv[]) -> int
{
  rclcpp::init(argc, argv);
  auto node_ptr{
    std::make_shared<wheeliebot_pure_pursuit_controller::PurePursuitControllerComponent>()};

  try {
    rclcpp::spin(node_ptr);
  } catch (std::exception const & ex) {
    RCLCPP_FATAL_STREAM(node_ptr->get_logger(), "unhandled exception: " << ex.what());
  } catch (...) {
    RCLCPP_FATAL(node_ptr->get_logger(), "unknown error");
  }

  rclcpp::shutdown();

  return 0;
}
