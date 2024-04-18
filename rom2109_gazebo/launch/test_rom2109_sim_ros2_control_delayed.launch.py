#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit, OnProcessStart

def generate_launch_description():
    gazebo_pkg = get_package_share_directory('rom2109_gazebo')
    description_pkg = get_package_share_directory('rom2109_description')
    default_world_path = os.path.join(gazebo_pkg, 'worlds', 'cafe.world')

    bot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            description_pkg, 'launch', 'description_xacro_sim_ros2_control.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(gazebo_pkg, 'rviz2', 'display.rviz')],
        condition=IfCondition(LaunchConfiguration('open_rviz'))
    )

    gazebo_params_file = os.path.join(get_package_share_directory(
        'rom2109_gazebo'), 'config', 'gazebo_params.yaml')

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
            "extra_gazebo_args": "--ros-args --params-file " + gazebo_params_file
        }.items(),
    )
    
    spawn_robot_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'rom2109',
                   "-x", '0.0',
                   "-y", '0.0',
                   "-z", '0.1'],
        output='screen'
    )
    delayed_spawn_robot_node = TimerAction(period=3.0, actions=[spawn_robot_node])

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )
    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=spawn_robot_node,
            on_start=[diff_drive_spawner],
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )
    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=spawn_robot_node,
            on_start=[joint_broad_spawner],
        )
    )

    twist_mux_params = os.path.join(gazebo_pkg, 'config', 'twist_mux.yaml')
    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {'use_sim_time': True}],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'open_rviz', default_value='true', description='Open RViz.'),
            bot,
            gazebo_launch,
            rviz_node,
            spawn_robot_node,
            delayed_diff_drive_spawner,
            delayed_joint_broad_spawner,
            twist_mux_node,
        ]
    )
