# Copyright 2023 Adam Morrissett
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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

    wheeliebot_launch_dir = Path(get_package_share_directory("wheeliebot_launch"))

    wheeliebot_visualization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [str(wheeliebot_launch_dir / "launch" / "visualization.launch.py")]
        ),
        condition=IfCondition(LaunchConfiguration("launch_visualization")),
    )

    wheeliebot_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [str(wheeliebot_launch_dir / "launch" / "localization.launch.py")]
        )
    )

    wheeliebot_planning = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [str(wheeliebot_launch_dir / "launch" / "planning.launch.py")]
        )
    )

    return LaunchDescription(
        [launch_visualization_arg, wheeliebot_visualization, wheeliebot_localization, wheeliebot_planning]
    )
