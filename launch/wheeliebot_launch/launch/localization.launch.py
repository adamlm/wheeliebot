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

from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():
    localizer_node = Node(
        package="wheeliebot_dummy_localizer",
        executable="dummy_localizer_node",
        name="localizer",
    )

    mapping_node = Node(
        package="wheeliebot_dummy_map_server",
        executable="dummy_map_server_node",
        name="map_server"
    )

    return LaunchDescription([localizer_node, mapping_node])
