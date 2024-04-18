# # Copyright (c) 2023 ROM Robotics Ltd. Yangon
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
    urdf_path= os.path.join(urdf_pkg, 'urdf', "rom2109_tall.urdf")
    urdf = open(urdf_path).read()

    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    #------------------------------------------------------ SLAM ------------------------------------------------#
    namespace = LaunchConfiguration('namespace')
    declare_namespace_cmd = DeclareLaunchArgument('namespace', default_value='',description='Top-level namespace')

    use_namespace = LaunchConfiguration('use_namespace')
    declare_use_namespace_cmd = DeclareLaunchArgument('use_namespace', default_value='false',description='Whether to apply a namespace to the navigation stack')

    map_yaml_file = LaunchConfiguration('map')
    declare_map_yaml_cmd = DeclareLaunchArgument('map', default_value=os.path.join(bringup_dir, 'maps', 'turtlebot3_world.yaml'), description='Full path to map file to load')

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_cmd = DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true')

    params_file = LaunchConfiguration('params_file')
    declare_params_file_cmd = DeclareLaunchArgument('params_file', default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'), description='Full path to the ROS2 parameters file to use for all launched nodes')

    autostart = LaunchConfiguration('autostart')
    declare_autostart_cmd = DeclareLaunchArgument('autostart', default_value='true', description='Automatically startup the nav2 stack')

    use_composition = LaunchConfiguration('use_composition')
    declare_use_composition_cmd = DeclareLaunchArgument('use_composition', default_value='True', description='Whether to use composed bringup')

    slam = LaunchConfiguration('slam')
    declare_slam_cmd = DeclareLaunchArgument('slam', default_value='True',description='Whether run a SLAM')

    use_respawn = LaunchConfiguration('use_respawn')
    declare_use_respawn_cmd = DeclareLaunchArgument('use_respawn', default_value='False', description='Whether to respawn if a node crashes. Applied when composition is disabled.')
    #------------------------------------------------------ SLAM ------------------------------------------------#

    open_rviz = LaunchConfiguration('open_rviz')
    declare_open_rviz = DeclareLaunchArgument('open_rviz', default_value='True',description='Open RViz or not')

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
        # arguments=['-d', os.path.join(gazebo_pkg, 'rviz2', 'display.rviz')],
        arguments=['-d', os.path.join(bringup_dir, 'rviz', 'nav2_default_view.rviz')],
        condition=IfCondition(LaunchConfiguration('open_rviz'))
    )

    gazebo_launch_py = IncludeLaunchDescription(
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

    spawn_tall_robot_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-database", "rom2109_tall", "-entity", "rom2109_tall",
        "-x", '0.0',
        "-y", '0.0',
        "-z", '0.1'],
        output="screen"
    )

    bringup_launch_py = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'bringup_launch.py')),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace,
                          'slam': slam,
                          'map': map_yaml_file,
                          'use_sim_time': use_sim_time,
                          'params_file': params_file,
                          'autostart': autostart,
                          'use_composition': use_composition,
                          'use_respawn': use_respawn}.items())
    

    ld = LaunchDescription()
    ld.add_action(declare_open_rviz)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz_node)
    ld.add_action(gazebo_launch_py)
    ld.add_action(spawn_tall_robot_node)

    #-------- S L A M --------#
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    #ld.add_action(bringup_launch_py)
    #-------- S L A M --------#

    return ld

