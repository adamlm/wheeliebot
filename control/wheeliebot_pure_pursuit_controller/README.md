# WheelieBot Pure Pursuit Controller

This package contains the pure pursuit controller for the WheelieBot

## Hard-coded constants

- lookahead distance: 2.0 m
- maximum linear speed: 0.25 m/s
- linear speed gain: 0.1
- linear angular speed gain: 0.5
- twist command publication rate: 5 Hz
- lookahead point publication rate: 5 Hz

## Interface

### Subscriptions

| Topic                    | Message Type                                | Description                                  |
| ------------------------ | ------------------------------------------- | -------------------------------------------- |
| `~/input/reference_path` | [`nav_msgs/msg/Path`][path_msg_url]         | Path to follow                               |
| `~/input/odom`           | [`nav_msgs/msg/Odometry`][odometry_msg_url] | WheelieBot's estimated position and velocity |

[path_msg_url]: https://docs.ros2.org/latest/api/nav_msgs/msg/Path.html
[odometry_msg_url]: https://docs.ros2.org/latest/api/nav_msgs/msg/Odometry.html

### Publishers

| Topic                      | Message Type                                              | Frequency | Description                      |
| -------------------------- | --------------------------------------------------------- | --------- | -------------------------------- |
| `~/output/cmd_vel`         | [`geometry_msgs/msg/Twist`][twist_msg_url]                | 5 Hz      | Twist needed to track path       |
| `~/output/lookahead_point` | [`geometry_msgs::msg::PoseStamped`][pose_stamped_msg_url] | 5 Hz      | Path pose used to generate twist |

[twist_msg_url]: https://docs.ros2.org/latest/api/geometry_msgs/msg/Twist.html
[pose_stamped_msg_url]: https://docs.ros2.org/latest/api/geometry_msgs/msg/PoseStamped.html

### Parameters

This package does not provide parameters.

### Services

This package does not provide services.

### Actions

This package does not provide actions.
