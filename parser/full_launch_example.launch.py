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

from launch import LaunchDescription, OpaqueFunction
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    name = args[0]
    sim_flag = kwargs["sim"]
    enabled = sim_flag.lower() in ["true"]
    return [
        Node(
            package="pkg",
            executable="sim_node",
            name=name,
            parameters=[{"use_sim_time": enabled}]
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        OpaqueFunction(
            function=launch_setup,
            args=["simulator_bot"],
            kwargs={"sim": LaunchConfiguration("use_sim_time")}
        )
    ])