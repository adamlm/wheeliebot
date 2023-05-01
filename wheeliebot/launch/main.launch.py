from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_visualization_arg = DeclareLaunchArgument(
        "launch_visualization", default_value="True"
    )

    wheeliebot_dir = Path(get_package_share_directory("wheeliebot"))

    wheeliebot_visualization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [str(wheeliebot_dir / "launch" / "visualization.launch.py")]
        ),
        condition=IfCondition(LaunchConfiguration("launch_visualization")),
    )

    wheeliebot_planning = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [str(wheeliebot_dir / "launch" / "planning.launch.py")]
        )
    )

    return LaunchDescription(
        [launch_visualization_arg, wheeliebot_visualization, wheeliebot_planning]
    )
