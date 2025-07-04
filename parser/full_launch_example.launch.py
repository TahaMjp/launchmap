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

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    rviz_cfg_path = PathJoinSubstitution(
        [FindPackageShare("example_package"), "rviz/example_config.rviz"]
    ).perform(context)

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="visualizer",
        arguments=["-d", str(rviz_cfg_path)],
        condition=IfCondition(LaunchConfiguration("enable_rviz").perform(context)),
    )

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("example_package"),
                        "launch",
                        "components",
                        "robot.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={}.items(),
    )

    return [
    	rviz_node,
        robot_launch,
    ]


def generate_launch_description():
    declared_arguments = []

    def add_launch_arg(name: str, default_value: str = None):
        declared_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value)
        )

    add_launch_arg("enable_rviz", "False")

    return LaunchDescription(
        [*declared_arguments, OpaqueFunction(function=launch_setup)]
    )



