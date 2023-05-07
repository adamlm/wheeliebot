#!/bin/bash

# Reference: https://github.com/osrf/docker_images/blob/master/ros/humble/ubuntu/jammy/ros-core/ros_entrypoint.sh

set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --
exec "$@"
