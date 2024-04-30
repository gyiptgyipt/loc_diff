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
    default_world_path = os.path.join(gazebo_pkg, 'worlds', 'cafe.world')
    
    urdf_pkg = get_package_share_directory('rom2109_description')
    urdf_path= os.path.join(urdf_pkg, 'urdf', "rom2109_bot.urdf")
    urdf = open(urdf_path).read()

    robot_state_publisher_node = Node(
        name="robot_state_publisher",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": urdf, 'use_sim_time': True}],
    )

    joint_state_node = Node(
        name="joint_state_publisher",
        package="joint_state_publisher",
        executable="joint_state_publisher",
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

    spawn_bot_robot_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-database", "rom2109_bot", "-entity", "rom2109_bot",
        "-x", '0.0',
        "-y", '0.0',
        "-z", '0.3'],
        output="screen"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('open_rviz', default_value='false', description='Open RViz.'),
            robot_state_publisher_node,
            #joint_state_node,
            rviz_node,
            gazebo_launch,
            spawn_bot_robot_node,
        ]
    )