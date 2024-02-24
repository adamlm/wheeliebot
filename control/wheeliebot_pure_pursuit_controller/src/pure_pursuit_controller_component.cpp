#include "wheeliebot_pure_pursuit_controller/pure_pursuit_controller_component.hpp"

#include <rcl/time.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/pose.hpp>
#include <range/v3/algorithm/min_element.hpp>
#include <range/v3/view/sliding.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace wheeliebot_pure_pursuit_controller
{
PurePursuitControllerComponent::PurePursuitControllerComponent()
: PurePursuitControllerComponent{rclcpp::NodeOptions{}}
{
}

PurePursuitControllerComponent::PurePursuitControllerComponent(rclcpp::NodeOptions const & options)
: Node{"wheeliebot_pure_pursuit_controller_node", options},
  odometry_sub_{create_subscription<nav_msgs::msg::Odometry>(
    "input/odom", 1, [this](nav_msgs::msg::Odometry const & msg) { update_current_state(msg); })},
  reference_path_sub_{create_subscription<nav_msgs::msg::Path>(
    "input/reference_path", 1,
    [this](nav_msgs::msg::Path const & msg) { update_reference_path(msg); })},
  twist_command_pub_{create_publisher<geometry_msgs::msg::Twist>("output/cmd_vel", 1)},
  lookahead_point_pub_{
    create_publisher<geometry_msgs::msg::PoseStamped>("output/lookahead_point", 1)}
{
  using std::chrono_literals::operator""ms;

  twist_command_update_timer_ =
    rclcpp::create_timer(this, this->get_clock(), 200ms, [this] { this->publish_twist_command(); });
}

auto PurePursuitControllerComponent::update_reference_path(nav_msgs::msg::Path const & msg) -> void
{
  reference_path_ = msg;
}

auto PurePursuitControllerComponent::update_current_state(nav_msgs::msg::Odometry const & msg)
  -> void
{
  current_state_ = msg;
}

namespace
{
auto pythagorean_distance(geometry_msgs::msg::Pose const & a, geometry_msgs::msg::Pose const & b)
{
  return std::hypot(a.position.x - b.position.x, a.position.y - b.position.y);
}
}  // namespace

auto PurePursuitControllerComponent::publish_twist_command() -> void
{
  geometry_msgs::msg::Twist twist;
  twist.linear.x = 0.0;
  twist.angular.z = 0.0;

  const auto closest_path_pose =
    [this](nav_msgs::msg::Path const & path, nav_msgs::msg::Odometry const & state) {
      return ranges::min_element(path.poses, [this](auto const & a, auto const & b) {
        return pythagorean_distance(current_state_.value().pose.pose, a.pose) <
               pythagorean_distance(current_state_.value().pose.pose, b.pose);
      });
    };

  static constexpr auto look_ahead_at_most =
    [](auto const start, auto const stop, double max_distance) {
      double cumulative_distance{0.0};
      for (auto const [it, end] : ranges::subrange{start, stop} | ranges::views::sliding(2)) {
        cumulative_distance += pythagorean_distance(it[0].pose, it[1].pose);
        if (cumulative_distance > max_distance) {
          // it[1] is beyond the lookahead distance
          return it;
        }
      }

      return stop - 1;
    };

  static constexpr auto lookahead_distance{2.0};

  if (current_state_ && reference_path_) {
    if (
      pythagorean_distance(
        current_state_.value().pose.pose, reference_path_.value().poses.back().pose) < 0.1) {
      RCLCPP_DEBUG(get_logger(), "Stopping");
      twist_command_pub_->publish(twist);
      return;
    }

    auto const closest_path_pose_it{
      closest_path_pose(reference_path_.value(), current_state_.value())};
    auto const lookahead_pose_it{look_ahead_at_most(
      closest_path_pose_it, std::cend(reference_path_.value().poses), lookahead_distance)};

    RCLCPP_DEBUG_STREAM(
      get_logger(), "current state: x = " << current_state_.value().pose.pose.position.x << ", y = "
                                          << current_state_.value().pose.pose.position.y);

    RCLCPP_DEBUG_STREAM(
      get_logger(), "closest pose: x = " << closest_path_pose_it->pose.position.x
                                         << ", y = " << closest_path_pose_it->pose.position.y);

    RCLCPP_DEBUG_STREAM(
      get_logger(), "target pose: x = " << lookahead_pose_it->pose.position.x
                                        << ", y = " << lookahead_pose_it->pose.position.y);

    auto const target_point{lookahead_pose_it->pose};

    auto const delta_x{target_point.position.x - current_state_.value().pose.pose.position.x};
    auto const delta_y{target_point.position.y - current_state_.value().pose.pose.position.y};

    auto const euclidean_distance{std::hypot(delta_x, delta_y)};
    auto const angle_to_target{std::atan2(delta_y, delta_x)};

    auto const current_yaw = [this] {
      tf2::Quaternion quaternion;
      tf2::fromMsg(current_state_.value().pose.pose.orientation, quaternion);

      tf2::Matrix3x3 matrix(quaternion);
      double roll, pitch, yaw;
      matrix.getRPY(roll, pitch, yaw);

      return yaw;
    }();

    auto const angular_distance{angle_to_target - current_yaw};

    auto const distance_to_goal = [this] {
      auto const goal_pose{(std::cend(reference_path_.value().poses) - 1)->pose};
      auto const current_pose{current_state_.value().pose.pose};
      auto const delta_x_goal{goal_pose.position.x - current_pose.position.x};
      auto const delta_y_goal{goal_pose.position.y - current_pose.position.y};

      return std::hypot(delta_x_goal, delta_y_goal);
    }();

    twist.linear.x = std::min(0.1 * distance_to_goal, 0.25);
    twist.angular.z = 0.5 * angular_distance;

    geometry_msgs::msg::PoseStamped lookahead_point;
    lookahead_point.header.frame_id = "map";
    lookahead_point.pose = target_point;

    lookahead_point_pub_->publish(lookahead_point);
  } else if (!current_state_) {
    RCLCPP_DEBUG(get_logger(), "Sending zero twist command: current state unknown");
  } else if (!reference_path_) {
    RCLCPP_DEBUG(get_logger(), "Sending zero twist command: no reference path");
  }

  twist_command_pub_->publish(twist);
}
}  // namespace wheeliebot_pure_pursuit_controller

RCLCPP_COMPONENTS_REGISTER_NODE(wheeliebot_pure_pursuit_controller::PurePursuitControllerComponent)
