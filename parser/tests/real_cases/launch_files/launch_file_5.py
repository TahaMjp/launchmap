# Copyright (c) 2025 Kodo Robotics
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    sim = use_sim_time.lower() in ["true", "1"]
    
    nodes = []

    nodes.append(Node(
        package="demo_camera",
        executable="camera_node",
        parameters=[{"use_sim_time": sim}],
        name="camera"
    ))

    nodes.append(Node(
        package="demo_robot",
        executable="robot_state_publisher",
        parameters=[{"use_sim_time": sim}],
        name="state_pub"
    ))

    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("sub_package"),
        GroupAction([
            Node(
                package="demo_bringup",
                executable="base_node",
                name="base"
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        LaunchConfiguration("sub_package").perform({}),
                        "launch",
                        "sub_launch.py"
                    )
                )
            )
        ]),

        OpaqueFunction(function=launch_setup)
    ])
