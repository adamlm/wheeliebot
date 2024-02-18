# WheelieBot Path Planner

This package contains the path planner for the WheelieBot

## Interface

### Subscribers

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `~/odom` | `nav_msgs/msg/Odometry` | WheelieBot's estimated position and velocity |
| `~/goal` | `geometry_msgs/msg/PoseStamped` | WheelieBot's goal pose |


### Publishers

| Topic | Message Type | Frequency | Description |
|-------|--------------|-----------|-------------|
| `~/path` | `nav_msgs/msg/Path` | 1 Hz | Path |


### Parameters
This package does not provide parameters.


### Services

This package does not provide services.


### Actions

This package does not provide actions.
