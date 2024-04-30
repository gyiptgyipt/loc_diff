#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    gazebo_pkg = get_package_share_directory('rom2109_gazebo')
    description_pkg = get_package_share_directory('rom2109_description')
    default_world_path = os.path.join(gazebo_pkg, 'worlds', 'cafe.world')
    
    bot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        description_pkg,'launch','description.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(gazebo_pkg, 'rviz2', 'display.rviz')],
        condition=IfCondition(LaunchConfiguration('open_rviz'))
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")),
        launch_arguments={
            "use_sim_time": "true",
            "robot_name": "rom2109",
            "world": default_world_path,
            "lite": "false",
            "world_init_x": "0.0",
            "world_init_y": "0.0",
            "world_init_heading": "0.0",
            "gui": "true",
            "close_loop_odom": "true",
        }.items(),
    )

    spawn_robot_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=["-database", "rom2109_tall", '-entity', 'rom2109',
        "-x", '0.0',
        "-y", '0.0',
        "-z", '0.1'],
        output='screen'
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('open_rviz', default_value='true', description='Open RViz.'),
            bot,
            gazebo_launch,
            rviz_node,
            spawn_robot_node,
        ]
    )