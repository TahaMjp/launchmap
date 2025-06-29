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
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, PushRosNamespace
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Declare a launch argument
        DeclareLaunchArgument("namespace", default_value="robot1", description="Robot namespace"),

        # Top-level Node
        Node(
            package="demo_nodes",
            executable="talker",
            name="talker_node",
            output="screen"
        ),

        # Include another launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("my_nav_package"),
                    "launch",
                    "navigation.launch.py"
                ])
            ),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time")
            }.items()
        ),

        # Group of nodes under a namespace
        GroupAction([
            PushRosNamespace(LaunchConfiguration("namespace")),
            Node(
                package="demo_nodes",
                executable="listener",
                name="listener_node",
                output="screen"
            )
        ])
    ])