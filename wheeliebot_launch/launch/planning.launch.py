from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():
    path_planner_node = Node(
        package="wheeliebot_path_planner",
        executable="path_planner_node",
        name="path_planner",
    )

    return LaunchDescription([path_planner_node])
