# Copyright 2021 the Autoware Foundation
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
#
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.

"""
Example launch file for a new package.

Note: Does not work in ROS2 dashing!
"""

import ament_index_python
import launch
import launch_ros.actions


def generate_launch_description():
    """Launch wall_follower_node with default configuration."""
    wall_follower_node = launch_ros.actions.Node(
        package='wall_follower',
        executable='wall_follower_node_exe',
        name='wall_follower',
        namespace='laboratories',
        output='screen',
        parameters=[
            "{}/param/defaults.param.yaml".format(
                ament_index_python.get_package_share_directory(
                    "wall_follower"
                )
            ),
        ],
        remappings=[
            ('scan', '/scan'),
            ('ctrl_cmd', '/lgsvl/vehicle_control_cmd'),
        ]
    )

    ld = launch.LaunchDescription([
        wall_follower_node]
    )
    return ld

