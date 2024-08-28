from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
import os

def generate_launch_description():
    gps_wpf_dir = get_package_share_directory("rlnav2")
    
    # Specify the path to the YAML configuration file
    rl_params_file = os.path.join(gps_wpf_dir, "config", "test.yaml")
    
    return LaunchDescription([
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[rl_params_file, {"use_sim_time": True}],
        )
    ])
