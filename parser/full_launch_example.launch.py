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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    package_name = LaunchConfiguration('package_name').perform(context)
    urdf_file = LaunchConfiguration('urdf_file').perform(context)
    use_sim_time_str = LaunchConfiguration('use_sim_time').perform(context)

    package_directory = get_package_share_directory(package_name)
    robot_desc_path = os.path.join(package_directory, 'urdf', urdf_file)
    use_sim_time = use_sim_time_str.lower() in ['true', '1', 'yes']

    params = {
        'use_sim_time': use_sim_time,
        'robot_description': Command(['xacro ', robot_desc_path])
    }

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    return [node_robot_state_publisher]

def generate_launch_description():
    declare_pkg_name = DeclareLaunchArgument(
        'package_name',
        default_value='upao_robot_description',
        description='Name of the package containing the robot description'
    )
    declare_urdf_file = DeclareLaunchArgument(
        'urdf_file',
        default_value='robot_wrapper.urdf.xacro',
        description='URDF file to load for the robot description'
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )

    return LaunchDescription([
        declare_pkg_name,
        declare_urdf_file,
        declare_use_sim_time,
        OpaqueFunction(function=launch_setup),
    ])