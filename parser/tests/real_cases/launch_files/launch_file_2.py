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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    param_path = LaunchConfiguration("driver_param_file").perform(context)
    if not param_path:
        param_path = PathJoinSubstitution(
            [
                FindPackageShare("generic_driver_pkg"),
                "params",
                "driver_config.param.yaml",
            ]
        ).perform(context)

    driver_node = Node(
        package="generic_driver_pkg",
        executable="generic_driver_node_exe",
        name="generic_driver_node",
        parameters=[param_path],
        output="screen",
        arguments=["--ros-args", "--log-level", "info", "--enable-stdout-logs"],
    )

    return [driver_node]


def generate_launch_description():
    declared_arguments = []

    def add_launch_arg(name: str, default_value: str = None, description: str = ""):
        declared_arguments.append(
            DeclareLaunchArgument(
                name, default_value=default_value, description=description
            )
        )

    add_launch_arg(
        "driver_param_file",
        default_value="",
        description="Path to the driver parameter YAML file",
    )

    return LaunchDescription(
        [*declared_arguments, OpaqueFunction(function=launch_setup)]
    )
