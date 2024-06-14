#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro


def generate_launch_description():
    gazebo_pkg = get_package_share_directory('rlbd_gazebo')

    description_pkg = get_package_share_directory('rlbd_description')
    default_world_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'turtlebot3_house.world'
    )

    # default_world_path = os.path.join(
    #     get_package_share_directory('rldb_gazebo'),
    #     'worlds',
    #     'custom.world'
    # )
    
    bot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        description_pkg,'launch','description.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        gazebo_pkg,'launch','controller_spawner.launch.py'
        )]), 
        launch_arguments={'use_sim_time': 'true'}.items()
    )


    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(gazebo_pkg, 'rviz2', 'rldbmap_test.rviz')],
        condition=IfCondition(LaunchConfiguration('open_rviz'))
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")),
        launch_arguments={
            "use_sim_time": "true",
            "robot_name": "black_donut",
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
        arguments=['-entity', 'black_donut', '-topic', '/robot_description',
        #arguments=["-database", "rldb_tall", '-entity', 'rldb',
        "-x", '-1.0',
        "-y", '-1.0',
        "-z", '0.0'],
        output='screen'
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('open_rviz', default_value='false', description='Open RViz.'),
            bot,
            gazebo_launch,
            rviz_node,
            spawn_robot_node,
            controller,
        ]
    )