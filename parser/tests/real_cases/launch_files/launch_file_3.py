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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    driver1_param_path = PathJoinSubstitution(
        [
            FindPackageShare("anon_package_launch"),
            "params",
            "robot",
            "driver1_config.param.yaml",
        ]
    ).perform(context)

    driver1_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("generic_driver_pkg"),
                        "launch",
                        "generic_driver_pkg.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={"driver1_param_file": driver1_param_path}.items(),
    )

    return [driver1_launch]


def generate_launch_description():
    declared_arguments = []

    def add_launch_arg(name: str, default_value: str = None):
        declared_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    return LaunchDescription([*declared_arguments, OpaqueFunction(function=launch_setup)])
