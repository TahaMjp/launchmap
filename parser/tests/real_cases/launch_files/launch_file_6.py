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
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    robot_description = f"<robot name='demo'>...</robot>"

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}]
        )
    ]


def generate_launch_description():
    # Declare arguments
    package_arg = DeclareLaunchArgument(
        'pkg_name',
        default_value='my_robot_pkg',
        description='Package containing the launch files'
    )
    use_sim_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    # Include
    included_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            LaunchConfiguration('pkg_name'),
            '/launch/other_launch_file.py'
        ])
    )

    # Group with condition
    conditional_group = GroupAction(
        actions=[
            Node(
                package='demo_nodes_cpp',
                executable='talker',
                name='conditional_talker',
                output='screen',
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
            )
        ],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('use_sim_time'), "' == 'true'"])),
        namespace='sim_ns'
    )

    # OpaqueFunction
    opaque = OpaqueFunction(function=launch_setup)

    # Composable Container + Nodes
    container = ComposableNodeContainer(
        name='my_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='demo_nodes_cpp',
                plugin='demo_nodes_cpp::Talker',
                name='talker_component',
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
            ),
            ComposableNode(
                package='demo_nodes_cpp',
                plugin='demo_nodes_cpp::Listener',
                name='listener_component',
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
            )
        ],
        output='screen'
    )

    return LaunchDescription([
        package_arg,
        use_sim_arg,
        included_launch,
        conditional_group,
        opaque,
        container
    ])
