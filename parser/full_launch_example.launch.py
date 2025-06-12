from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, PushRosNamespace
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    return LaunchDescription([

        # Declare arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false'
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value='robot1'
        ),

        # Top-level node
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='talker_main',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen'
        ),

        # Include another launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    'bringup',
                    'launch',
                    'sensors.launch.py'
                ])
            ),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
        ),

        # Grouped actions with namespace
        GroupAction([
            PushRosNamespace(LaunchConfiguration('robot_name')),
            Node(
                package='demo_nodes_cpp',
                executable='listener',
                name='listener_ns',
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                output='screen'
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join('bringup', 'launch', 'navigation.launch.py')
                )
            )
        ])
    ])