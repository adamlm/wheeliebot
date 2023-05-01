from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource


def generate_launch_description():
    foxglove_bridge_dir = Path(get_package_share_directory("foxglove_bridge"))

    foxglove_bridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            [str(foxglove_bridge_dir / "foxglove_bridge_launch.xml")]
        )
    )

    return LaunchDescription([foxglove_bridge_launch])
