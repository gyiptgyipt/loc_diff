#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

import xacro

def generate_launch_description():
    # gazebo_pkg = get_package_share_directory('rom2109_gazebo')
    # gz_ros_pkg = get_package_share_directory('gazebo_ros')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    urdf_pkg = get_package_share_directory('rom2109_description')
    #urdf_path= os.path.join(urdf_pkg, 'urdf', "urdf.urdf")
    #urdf = open(urdf_path).read()
    xacro_file = os.path.join(urdf_pkg,'urdf', 'robot_sim_ros2_control.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)

    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}

    robot_state_publisher_node = Node(
        name="robot_state_publisher",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output='screen',
        parameters=[params]
        #parameters=[{"robot_description": urdf, 'use_sim_time': use_sim_time}],
    )

    # joint_state_node = Node(
    #     name="joint_state_publisher",
    #     package="joint_state_publisher",
    #     executable="joint_state_publisher",
    # )

    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     arguments=['-d', os.path.join(urdf_pkg, 'rviz2', 'display.rviz')],
    #     condition=IfCondition(LaunchConfiguration('open_rviz'))
    # )

    return LaunchDescription(
        [
            DeclareLaunchArgument('use_sim_time', default_value='true', description='Use sim time if true'),
            robot_state_publisher_node,
            #joint_state_node,
            #rviz_node
        ]
    )
