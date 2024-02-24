#ifndef WHEELIEBOT_PURE_PURSUIT_CONTROLLER__PURE_PURSUIT_CONTROLLER_COMPONENT_HPP_
#define WHEELIEBOT_PURE_PURSUIT_CONTROLLER__PURE_PURSUIT_CONTROLLER_COMPONENT_HPP_

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

namespace wheeliebot_pure_pursuit_controller
{
class PurePursuitControllerComponent : public rclcpp::Node
{
public:
  explicit PurePursuitControllerComponent();
  explicit PurePursuitControllerComponent(rclcpp::NodeOptions const & options);

  auto update_reference_path(nav_msgs::msg::Path const & msg) -> void;

  auto update_current_state(nav_msgs::msg::Odometry const & msg) -> void;

  auto publish_twist_command() -> void;

private:
  std::optional<nav_msgs::msg::Path> reference_path_{std::nullopt};
  std::optional<nav_msgs::msg::Odometry> current_state_{std::nullopt};

  rclcpp::TimerBase::SharedPtr twist_command_update_timer_{nullptr};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_{nullptr};
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr reference_path_sub_{nullptr};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_command_pub_{nullptr};

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr lookahead_point_pub_{nullptr};
};
}  // namespace wheeliebot_pure_pursuit_controller

#endif  // WHEELIEBOT_PURE_PURSUIT_CONTROLLER__PURE_PURSUIT_CONTROLLER_COMPONENT_HPP_
